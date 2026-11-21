using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    public enum RunPhase { Idle, Calibrating, Running, Parking, Done, Aborted }

    /// <summary>
    /// Runs a test: a slow calibration cycle (amplitude, preload, gas force), then every step of the sequence.
    /// Feed it samples, status and acks from the link, all from the same thread.
    /// </summary>
    public class TestRunner
    {
        readonly IDynoLink _link;
        readonly Settings _set;
        readonly TestSequence _seq;
        CycleDetector _detector;

        readonly List<Sample> _samples = new List<Sample>();
        readonly List<short> _stepOf = new List<short>();
        readonly List<short> _cycleOf = new List<short>();
        readonly List<double> _stepHz = new List<double>();
        readonly List<ProcessedCycle> _stepCycles = new List<ProcessedCycle>();
        readonly List<double> _stepPeriods = new List<double>();
        readonly List<double> _stepTemp = new List<double>();
        readonly Queue<int> _tempFor = new Queue<int>();   // what each pending thermometer reading is for: -1 = test start, else the step
        double _startTemp = double.NaN;

        readonly SampleClock _clock = new SampleClock();
        double _t;
        int _lastBdc = -1, _commandIndex, _fullCyclesSinceCommand, _consistentOffTarget;
        double _prevPeriod, _prevPeriod2, _prevPeriod3, _expectedPeriod, _commandTime, _lastProgressTime, _motorHz;
        double _speedFactor = 1;   // actual crank speed / nominal crank speed (motor slip, ratio error)

        bool _motorRunning;
        BdcParker _parker;                // return to bottom dead centre after the last step
        double _driveHz = double.NaN;     // last output frequency reported by the drive
        // Constant speed: the drive reports the commanded frequency (its acceleration ramp is over).
        // Only cycles that start after this (plus the polling delay) are measured or used for the speed factor.
        double _atSpeedSince = double.NaN;
        bool _driveRefLost;               // the drive never reached the commanded frequency: fall back on cycle periods
        const double DriveReportDelayS = 0.4;
        double _driveHzSettled = double.NaN, _driveHzSettledSince;   // to tell a slow ramp from a drive stuck off target

        public RunPhase Phase { get; private set; }
        public int CurrentStep { get; private set; }
        public CalibrationResult Calibration { get; private set; }
        public List<StepResult> Results { get; private set; }
        public DateTime Started { get; private set; }
        /// <summary>Motor frequency commanded for a step (0 if not reached or skipped).</summary>
        public double StepMotorHz(int step) { return step >= 0 && step < _stepHz.Count ? _stepHz[step] : 0; }
        public bool IsActive { get { return Phase == RunPhase.Calibrating || Phase == RunPhase.Running || Phase == RunPhase.Parking; } }

        /// <summary>A cycle ended. The flag says whether it counts as a measured cycle.</summary>
        public event Action<ProcessedCycle, bool> CycleCompleted;
        public event Action<CalibrationResult> Calibrated;
        public event Action<StepResult> StepCompleted;
        public event Action<string> Log;
        /// <summary>The test ended (true = completed normally).</summary>
        public event Action<bool, string> Finished;
        public event Action Progress;

        public TestRunner(IDynoLink link, Settings set, TestSequence seq)
        {
            _link = link; _set = set; _seq = seq;
            Results = new List<StepResult>();
            CurrentStep = -1;
        }

        public void Start()
        {
            if (IsActive) return;
            _samples.Clear(); _stepOf.Clear(); _cycleOf.Clear(); _stepHz.Clear(); Results.Clear();
            for (int i = 0; i < _seq.Steps.Count; i++) _stepHz.Add(0);
            _stepTemp.Clear(); for (int i = 0; i < _seq.Steps.Count; i++) _stepTemp.Add(double.NaN);
            _startTemp = double.NaN; _tempFor.Clear();
            ReadTemperature(-1);
            _detector = new CycleDetector(_set.CycleHysteresisMm);
            _clock.Reset(); _t = 0; _lastBdc = -1; _speedFactor = 1;
            Calibration = null;
            CurrentStep = -1;
            Started = DateTime.Now;
            _motorRunning = false;
            if (_set.SkipCalibrationCycle)
            {
                Say(string.Format("Calibration cycle skipped: amplitude, preload and gas force will come from the first steady cycle (speeds use {0:0.#} mm amplitude until then)", _set.NominalAmplitudeMm));
                NextStep();
                Raise(Progress);
                return;
            }
            Phase = RunPhase.Calibrating;
            Say("Calibration cycle: measuring amplitude, preload and gas force");
            Command(_set.CalibrationMotorHz, true);
            Raise(Progress);
        }

        /// <summary>Stops the motor and ends the test.</summary>
        public void Stop(string reason)
        {
            if (!IsActive) { _link.MotorStop(); return; }
            Abort(reason);
        }

        public void OnSamples(RawSample[] raw)
        {
            if (!IsActive) return;
            for (int i = 0; i < raw.Length && IsActive; i++)
            {
                if (!_clock.Advance(raw[i].TimeUs)) continue;   // out of order: skip
                _t = _clock.T;
                var s = new Sample { T = _t, X = _set.PositionMm(raw[i].Position), F = _set.ForceN(raw[i].Force) };
                if (Phase == RunPhase.Parking) { _parker.Feed(_t, s.X); if (_parker.Done) Finish(); continue; }   // not part of the record
                int index = _samples.Count;
                _samples.Add(s); _stepOf.Add((short)CurrentStep); _cycleOf.Add(0);

                int bdc = _detector.Feed(index, s.X);
                if (bdc >= 0) OnBdc(bdc);
                if (!IsActive) return;

                if (Phase == RunPhase.Running && _seq.Steps[CurrentStep].Type == StepType.Warmup
                    && _t - _commandTime >= _seq.Steps[CurrentStep].Count)
                    NextStep();
                else if (_t - _lastProgressTime > Math.Max(10, 3 * _expectedPeriod + 5))
                    Abort("No motion detected. Check that the safety circuit is reset and the drive is ready.");
            }
        }

        public void OnStatus(DynoStatus st)
        {
            if (IsActive && st.State == MachineState.Fault) { Abort(DynoStatus.Describe(st.Fault)); return; }
            if (IsActive && !st.AcqOn) { Abort("The ESP32 restarted during the test (acquisition off)"); return; }
            if (st.VfdOk)
            {
                _driveHz = st.VfdHz;
                if (double.IsNaN(_driveHzSettled) || Math.Abs(st.VfdHz - _driveHzSettled) > 0.05) { _driveHzSettled = st.VfdHz; _driveHzSettledSince = _t; }
                bool there = Math.Abs(st.VfdHz - _motorHz) <= 0.05 + 0.005 * _motorHz;
                if (!there) _atSpeedSince = double.NaN;
                else if (double.IsNaN(_atSpeedSince)) _atSpeedSince = _t;
            }
            if (Phase == RunPhase.Parking) _parker.OnStatus(st);
        }

        /// <summary>Asks for one thermometer reading, recorded for the test start (-1) or a step.</summary>
        void ReadTemperature(int forStep)
        {
            _tempFor.Enqueue(forStep);
            _link.RequestTemperature();
        }

        /// <summary>Damper temperature at the start of the test (NaN if not measured).</summary>
        public double StartTempC { get { return _startTemp; } }
        public double StepTempC(int step) { return step >= 0 && step < _stepTemp.Count ? _stepTemp[step] : double.NaN; }

        /// <summary>Feed every thermometer reading from the link. Readings nobody asked for (live display) are ignored.</summary>
        public void OnTemperature(TempReading t)
        {
            if (_tempFor.Count == 0) return;
            int k = _tempFor.Dequeue();
            if (!t.Ok) { Say("    damper temperature: no answer from the IR thermometer"); return; }
            if (k < 0) _startTemp = t.ObjectC; else if (k < _stepTemp.Count) _stepTemp[k] = t.ObjectC;
            Say(string.Format("    damper temperature {0:0.0} °C{1}", t.ObjectC, k < 0 ? string.Format(" at the start of the test (sensor body {0:0.0} °C)", t.BodyC) : ""));
        }

        public void OnAck(EventId id, CommandResult result)
        {
            if (IsActive && result != CommandResult.Ok && (id == EventId.MotorRun || id == EventId.MotorSetFreq))
                Abort("Motor command refused: " + DynoStatus.Describe(result));
        }

        /// <summary>
        /// True when the whole cycle starting at cycleStart ran at the commanded frequency. Uses the drive's
        /// reported output frequency, so cycles during the drive's acceleration ramp never count, however
        /// slow the ramp. Without a drive frequency (or if it never matches) it falls back on the cycle
        /// periods: three in a row within 1 %.
        /// </summary>
        bool AtConstantSpeed(double cycleStart, double period)
        {
            if (!_driveRefLost && !double.IsNaN(_driveHz))
            {
                if (!double.IsNaN(_atSpeedSince) && cycleStart >= _atSpeedSince + DriveReportDelayS) return true;
                // Still ramping (reported frequency moving) or only just settled: wait.
                if (_t - _driveHzSettledSince < 10 + 3 * _expectedPeriod) return false;
                _driveRefLost = true;
                Say(string.Format("Warning: the drive reports {0:0.00} Hz for {1:0.00} Hz commanded; using cycle periods to detect constant speed",
                    _driveHz, _motorHz));
            }
            // this cycle and the two before it all within 1 %
            return _fullCyclesSinceCommand >= 3 && Math.Abs(period - _prevPeriod2) / period < 0.01 && Math.Abs(period - _prevPeriod3) / period < 0.01;
        }

        // ------------------------------------------------------------------ return to bottom dead centre
        void StartParking()
        {
            Phase = RunPhase.Parking;
            CurrentStep = -1;
            _parker = new BdcParker(_link, _set);
            _parker.Log += Say;
            _parker.Start(_t, true, Calibration.BdcMm, Calibration.AmplitudeMm, _speedFactor);
            Raise(Progress);
        }

        void Finish()
        {
            Phase = RunPhase.Done;
            CurrentStep = -1;
            Raise(Progress);
            var h = Finished; if (h != null) h(true, "Test complete");
        }

        void OnBdc(int j)
        {
            int prev = _lastBdc;
            _lastBdc = j;
            _lastProgressTime = _t;
            if (prev < 0 || prev < _commandIndex) return;      // cycle straddles a speed change
            double period = _samples[j].T - _samples[prev].T;
            _fullCyclesSinceCommand++;
            bool nearTarget = Math.Abs(period - _expectedPeriod) / _expectedPeriod < _set.SpeedTolerancePct / 100.0;
            bool steady = _fullCyclesSinceCommand >= 2 && Math.Abs(period - _prevPeriod) / period < 0.02;
            _prevPeriod3 = _prevPeriod2; _prevPeriod2 = _prevPeriod; _prevPeriod = period;   // _prevPeriod is now this cycle
            steady = steady && AtConstantSpeed(_samples[prev].T, period);

            if (Phase == RunPhase.Calibrating)
            {
                if (!steady) return;                           // one settling cycle, then one measured cycle
                Calibration = Analyzer.Calibrate(_samples, prev, j, _set);
                _speedFactor = Clamp(Calibration.CrankHz / Kinematics.CrankHz(_motorHz, _set), 0.8, 1.2);
                for (int i = prev; i <= j; i++) _cycleOf[i] = 1;
                Say(string.Format("Amplitude {0:0.0} mm, preload {1:0} N, gas force {2:0} N, static slope {3:0.00} N/mm (dead centres alone: {4:0.00})",
                    Calibration.AmplitudeMm, Calibration.PreloadN, Calibration.GasForceN, Calibration.StaticSlopeNPerMm, Calibration.DeadCentreSlopeNPerMm));
                var h = Calibrated; if (h != null) h(Calibration);
                NextStep();
                return;
            }

            if (Calibration == null)                           // calibration cycle skipped: geometry from the first steady cycle
            {
                if (!steady) return;
                Calibration = Analyzer.Calibrate(_samples, prev, j, _set);
                _speedFactor = Clamp(Calibration.CrankHz / Kinematics.CrankHz(_motorHz, _set), 0.8, 1.2);
                Say(string.Format("From the first steady cycle: amplitude {0:0.0} mm, preload {1:0} N, gas force {2:0} N, static slope {3:0.00} N/mm (measured at speed)",
                    Calibration.AmplitudeMm, Calibration.PreloadN, Calibration.GasForceN, Calibration.StaticSlopeNPerMm));
                var hcal = Calibrated; if (hcal != null) hcal(Calibration);
                SequenceStep cur = _seq.Steps[CurrentStep];       // redo this step's speed with the measured amplitude
                double hz = Kinematics.MotorHzForSpeed(cur.SpeedMmS, Calibration.AmplitudeMm, _set) / _speedFactor;
                if (hz < _set.MinMotorHz || hz > _set.MaxMotorHz) { CurrentStep--; NextStep(); return; }   // NextStep reports the skip
                if (Math.Abs(hz / _motorHz - 1) > 0.01)
                {
                    Say(string.Format("    corrected to {0:0.00} Hz for {1:0.#} mm/s", hz, cur.SpeedMmS));
                    _stepHz[CurrentStep] = hz;
                    Command(hz, false);
                }
                return;
            }

            SequenceStep step = _seq.Steps[CurrentStep];
            ProcessedCycle c = Analyzer.Process(_samples, prev, j, _set, Calibration);
            c.Step = CurrentStep;
            bool accept = false;
            if (step.Type == StepType.Cycles && steady)
            {
                if (nearTarget) accept = true;
                else if (++_consistentOffTarget >= 2)
                {
                    accept = true;                             // steady but off target: measure it anyway
                    if (_consistentOffTarget == 2)
                        Say(string.Format("Warning: speed is {0:0.0} % off the {1:0.#} mm/s target",
                            100 * (_expectedPeriod / period - 1), step.SpeedMmS));
                }
            }
            if (accept)
            {
                _stepCycles.Add(c); _stepPeriods.Add(period);
                for (int i = prev; i < j; i++) _cycleOf[i] = (short)_stepCycles.Count;
            }
            var hc = CycleCompleted; if (hc != null) hc(c, accept);
            if (accept && _stepCycles.Count >= (int)step.Count)
            {
                double mean = 0;
                foreach (double p in _stepPeriods) mean += p;
                mean /= _stepPeriods.Count;
                _speedFactor = Clamp((1 / mean) / Kinematics.CrankHz(_motorHz, _set), 0.8, 1.2);
                StepResult r = Analyzer.Summarize(CurrentStep, step.SpeedMmS, _motorHz, new List<ProcessedCycle>(_stepCycles));
                r.TempC = _stepTemp[CurrentStep];
                Say(string.Format("    measured: peak speed {0:0.#} / {1:0.#} mm/s (comp / reb); drive {2:0.00} Hz commanded, {3} Hz reported",
                    r.CompPeakVel, -r.RebPeakVel, _motorHz, double.IsNaN(_driveHz) ? "-" : _driveHz.ToString("0.00")));
                Results.Add(r);
                var hs = StepCompleted; if (hs != null) hs(r);
                NextStep();
            }
        }

        void NextStep()
        {
            while (true)
            {
                CurrentStep++;
                if (CurrentStep >= _seq.Steps.Count)
                {
                    if (_set.ReturnToBdc && Calibration != null) { StartParking(); return; }
                    _link.MotorStop();
                    Finish();
                    return;
                }
                SequenceStep step = _seq.Steps[CurrentStep];
                double amp = Calibration != null ? Calibration.AmplitudeMm : _set.NominalAmplitudeMm;
                double hz = Kinematics.MotorHzForSpeed(step.SpeedMmS, amp, _set) / _speedFactor;
                if (hz < _set.MinMotorHz || hz > _set.MaxMotorHz)
                {
                    Say(string.Format("Skipped {0:0.#} mm/s: needs {1:0.00} Hz at this amplitude (allowed {2:0.##} to {3:0.##} Hz)",
                        step.SpeedMmS, hz, _set.MinMotorHz, _set.MaxMotorHz));
                    continue;
                }
                Phase = RunPhase.Running;
                _stepHz[CurrentStep] = hz;
                Say(step.Type == StepType.Warmup
                    ? string.Format("Warm-up: {0:0.#} mm/s for {1:0.#} s ({2:0.00} Hz)", step.SpeedMmS, step.Count, hz)
                    : string.Format("{0:0.#} mm/s x {1:0} cycles ({2:0.00} Hz)", step.SpeedMmS, step.Count, hz));
                Command(hz, false);
                ReadTemperature(CurrentStep);
                Raise(Progress);
                return;
            }
        }

        void Command(double hz, bool start)
        {
            _motorHz = hz;
            _expectedPeriod = 1 / (Kinematics.CrankHz(hz, _set) * _speedFactor);
            _commandIndex = _samples.Count;
            _commandTime = _t; _lastProgressTime = _t;
            _fullCyclesSinceCommand = 0; _consistentOffTarget = 0;
            _atSpeedSince = double.NaN;
            _stepCycles.Clear(); _stepPeriods.Clear();
            if (start || !_motorRunning) _link.MotorRun(hz); else _link.MotorSetFreq(hz);   // the first command starts the motor
            _motorRunning = true;
        }

        void Abort(string reason)
        {
            _link.MotorStop();
            if (Phase == RunPhase.Parking)              // all steps were measured: the test itself is complete
            {
                _parker.Cancel(reason);
                Finish();
                return;
            }
            Phase = RunPhase.Aborted;
            CurrentStep = -1;
            Say("Test stopped: " + reason);
            Raise(Progress);
            var h = Finished; if (h != null) h(false, reason);
        }

        /// <summary>Everything recorded so far, ready to save.</summary>
        public TestRecord BuildRecord(string name, string notes)
        {
            var r = new TestRecord { Name = name ?? "", Notes = notes ?? "", Date = Started };
            if (Calibration != null) r.Calibration = Calibration;
            r.Steps.AddRange(_seq.Steps);
            r.StepMotorHz.AddRange(_stepHz);
            r.StartTempC = _startTemp; r.StepTempC.AddRange(_stepTemp);
            r.Samples.AddRange(_samples); r.StepOf.AddRange(_stepOf); r.CycleOf.AddRange(_cycleOf);
            return r;
        }

        void Say(string msg) { var h = Log; if (h != null) h(msg); }
        static void Raise(Action a) { if (a != null) a(); }
        static double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
    }
}

using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    public enum RunPhase { Idle, Calibrating, Running, Done, Aborted }

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

        bool _haveTime;
        uint _lastUs;
        double _t;
        int _lastBdc = -1, _commandIndex, _fullCyclesSinceCommand, _consistentOffTarget;
        double _prevPeriod, _expectedPeriod, _commandTime, _lastProgressTime, _motorHz;
        double _speedFactor = 1;   // actual crank speed / nominal crank speed (motor slip, ratio error)

        public RunPhase Phase { get; private set; }
        public int CurrentStep { get; private set; }
        public CalibrationResult Calibration { get; private set; }
        public List<StepResult> Results { get; private set; }
        public DateTime Started { get; private set; }
        public bool IsActive { get { return Phase == RunPhase.Calibrating || Phase == RunPhase.Running; } }

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
            _detector = new CycleDetector(_set.CycleHysteresisMm);
            _haveTime = false; _t = 0; _lastBdc = -1; _speedFactor = 1;
            Calibration = null;
            CurrentStep = -1;
            Started = DateTime.Now;
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
                if (_haveTime) _t += unchecked(raw[i].TimeUs - _lastUs) / 1e6;
                _lastUs = raw[i].TimeUs; _haveTime = true;
                var s = new Sample { T = _t, X = _set.PositionMm(raw[i].Position), F = _set.ForceN(raw[i].Force) };
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
            if (IsActive && st.State == MachineState.Fault) Abort(DynoStatus.Describe(st.Fault));
        }

        public void OnAck(EventId id, CommandResult result)
        {
            if (IsActive && result != CommandResult.Ok && (id == EventId.MotorRun || id == EventId.MotorSetFreq))
                Abort("Motor command refused: " + DynoStatus.Describe(result));
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
            _prevPeriod = period;

            if (Phase == RunPhase.Calibrating)
            {
                if (!steady) return;                           // one settling cycle, then one measured cycle
                Calibration = Analyzer.Calibrate(_samples, prev, j, _set);
                _speedFactor = Clamp(Calibration.CrankHz / Kinematics.CrankHz(_motorHz, _set), 0.8, 1.2);
                for (int i = prev; i <= j; i++) _cycleOf[i] = 1;
                Say(string.Format("Amplitude {0:0.0} mm, preload {1:0} N, gas force {2:0} N",
                    Calibration.AmplitudeMm, Calibration.PreloadN, Calibration.GasForceN));
                var h = Calibrated; if (h != null) h(Calibration);
                NextStep();
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
                    _link.MotorStop();
                    Phase = RunPhase.Done;
                    CurrentStep = -1;
                    Raise(Progress);
                    var h = Finished; if (h != null) h(true, "Test complete");
                    return;
                }
                SequenceStep step = _seq.Steps[CurrentStep];
                double hz = Kinematics.MotorHzForSpeed(step.SpeedMmS, Calibration.AmplitudeMm, _set) / _speedFactor;
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
            _stepCycles.Clear(); _stepPeriods.Clear();
            if (start) _link.MotorRun(hz); else _link.MotorSetFreq(hz);
        }

        void Abort(string reason)
        {
            _link.MotorStop();
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
            r.Samples.AddRange(_samples); r.StepOf.AddRange(_stepOf); r.CycleOf.AddRange(_cycleOf);
            return r;
        }

        void Say(string msg) { var h = Log; if (h != null) h(msg); }
        static void Raise(Action a) { if (a != null) a(); }
        static double Clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
    }
}

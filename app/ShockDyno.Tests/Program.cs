using System.Linq;
// Console tests: run the whole chain (protocol, runner, analysis, storage) against the simulator.
using System;
using System.Collections.Generic;
using System.IO;
using ShockDyno.Core;

static class Program
{
    static int _failed;

    static void Check(bool ok, string what)
    {
        Console.WriteLine((ok ? "  ok   " : "  FAIL ") + what);
        if (!ok) _failed++;
    }

    static bool Near(double a, double b, double relTol) { return Math.Abs(a - b) <= relTol * Math.Abs(b); }

    static int Main()
    {
        TestProtocol();
        TestKinematics();
        TestSignal();
        TestSequenceFile();
        TestFullRun(25, 6);
        TestFullRun(75, 5);       // 15 mm/s needs 0.33 Hz at 75 mm: skipped
        TestFullRun(12.5, 5);     // 500 mm/s needs 65 Hz at 12.5 mm: skipped
        TestFullRun(25, 6, 0.5);
        TestAverageCycles();
        TestBushings(0.03);
        TestBushings(0.08);  // slow drive ramp: cycles during the ramp must not count
        TestSampleClock();
        TestUnplug();
        TestCalibrationFile();
        TestFindBdc();
        TestSafetyTrip();
        TestForceLimit();
        Console.WriteLine(_failed == 0 ? "ALL TESTS PASSED" : _failed + " TEST(S) FAILED");
        return _failed == 0 ? 0 : 1;
    }

    static void TestProtocol()
    {
        Console.WriteLine("Protocol");
        // Reference bytes produced by the firmware encoder (firmware/shock_dyno/protocol.cpp).
        byte[] fw = { 0xFF, 0x00, 0x55, 0x03, 0x05, 0x10, 0x27, 0x4F };
        byte[] pc = Packet.EncodeEvent(EventId.MotorRun, 0x10, 0x27);
        Check(BitConverter.ToString(fw) == BitConverter.ToString(pc), "PC encoder matches firmware bytes");

        var samples = new List<RawSample> {
            new RawSample { TimeUs = 4000000000, Force = -8388608, Position = 8388607 },
            new RawSample { TimeUs = 12, Force = -1, Position = 0 } };
        byte[] data = Packet.BuildData(samples, 0, 2);
        var stream = new List<byte> { 0x12, 0xFF, 0xFF, 0x00, 0x99 };     // garbage first
        stream.AddRange(data); stream.AddRange(pc);
        byte[] bad = (byte[])data.Clone(); bad[6] ^= 0x40; stream.AddRange(bad);   // corrupted packet
        stream.AddRange(pc);
        var dec = new PacketDecoder();
        int events = 0; RawSample[] got = null;
        dec.PacketReceived += (t, p) => { if (t == Packet.TypeData) got = Packet.ParseData(p); else events++; };
        foreach (byte b in stream) dec.Feed(b);                           // one byte at a time
        Check(got != null && got.Length == 2 && got[0].TimeUs == 4000000000 && got[0].Force == -8388608
              && got[0].Position == 8388607 && got[1].Force == -1, "24-bit samples survive a round trip");
        Check(events == 2 && dec.CrcErrors == 1, "decoder resynchronises after garbage and rejects a corrupted packet");
    }

    static void TestKinematics()
    {
        Console.WriteLine("Kinematics");
        var s = new Settings();
        Check(Near(Kinematics.MotorHzForSpeed(15, 25, s), 0.982, 0.002), "15 mm/s at 25 mm amplitude = 0.98 Hz");
        Check(Near(Kinematics.MotorHzForSpeed(500, 12.5, s), 65.48, 0.002), "500 mm/s at 12.5 mm amplitude = 65.5 Hz");
        Check(Near(Kinematics.SpeedForMotorHz(Kinematics.MotorHzForSpeed(123, 50, s), 50, s), 123, 1e-9), "speed <-> frequency round trip");
    }

    static void TestSignal()
    {
        Console.WriteLine("Signal");
        int n = 2000; double fs = 500, f0 = 6;
        var t = new double[n]; var y = new double[n];
        var rnd = new Random(3);
        for (int i = 0; i < n; i++) { t[i] = i / fs; y[i] = 25 * Math.Sin(2 * Math.PI * f0 * t[i]) + 0.01 * (rnd.NextDouble() - 0.5); }
        double[] yf = Signal.LowPassZeroPhase(y, 40, fs);
        double[] d = Signal.Derivative(t, yf, 3);
        double worst = 0, worstF = 0;
        for (int i = 0; i < n; i++)
        {
            worst = Math.Max(worst, Math.Abs(d[i] - 25 * 2 * Math.PI * f0 * Math.Cos(2 * Math.PI * f0 * t[i])));
            worstF = Math.Max(worstF, Math.Abs(yf[i] - 25 * Math.Sin(2 * Math.PI * f0 * t[i])));
        }
        Check(worstF < 0.05, "filter leaves a 6 Hz motion unchanged, ends included (max error " + worstF.ToString("0.000") + " mm)");
        Check(worst < 0.03 * 25 * 2 * Math.PI * f0, "velocity of a 6 Hz sine within 3 % of peak (max error " + worst.ToString("0.0") + " mm/s)");
    }

    static void TestSequenceFile()
    {
        Console.WriteLine("Sequence");
        string path = Path.Combine(Path.GetTempPath(), "seq_test.seq");
        TestSequence.Default().Save(path);
        TestSequence q = TestSequence.Load(path);
        Check(q.Steps.Count == 7 && q.Steps[0].Type == StepType.Warmup && q.Steps[6].SpeedMmS == 500, "default sequence saves and loads");
        bool threw = false;
        try { TestSequence.Parse(new[] { "cycles, fast, 3" }); } catch (FormatException) { threw = true; }
        Check(threw, "a malformed line is reported");
    }

    class Rig
    {
        public Settings Set = new Settings();
        public SimulatedDynoLink Sim;
        public TestRunner Runner;
        public bool? Success; public string Message;
        public List<string> Log = new List<string>();

        public Rig(double amplitude)
        {
            Set.ForceZeroCounts = 1234; Set.PositionZeroCounts = -5000;
            Sim = new SimulatedDynoLink(Set) { RealTime = false, AmplitudeMm = amplitude };
            Runner = new TestRunner(Sim, Set, TestSequence.Default());
            Sim.SamplesReceived += Runner.OnSamples;
            Sim.StatusReceived += Runner.OnStatus;
            Sim.AckReceived += Runner.OnAck;
            Runner.Finished += (ok, msg) => { Success = ok; Message = msg; };
            Runner.Log += Log.Add;
            Sim.Open();
            Sim.StartAcquisition();
            int lo, hi; Set.ForceLimitCounts(out lo, out hi);
            Sim.SetForceLimits(lo, hi);
        }

        public void Run(double maxSeconds, Action<double> each = null)
        {
            for (double t = 0; t < maxSeconds && Success == null; t += 0.02) { if (each != null) each(t); Sim.Step(0.02); }
        }
    }

    /// <summary>
    /// Real dampers: seal friction plus rubber bushings, so the force reverses a little after the crank.
    /// The dead centres alone then over-estimate the static slope; the mid-stroke fit must not.
    /// Also checks that the plotted loops (static force removed) are level, as on the real machine.
    /// </summary>
    static void TestBushings(double lagS)
    {
        Console.WriteLine("Static force with friction and rubber bushings (force lags " + lagS * 1000 + " ms)");
        var rig = new Rig(25);
        rig.Sim.FrictionN = 60; rig.Sim.BushingLagS = lagS;
        rig.Runner.Start();
        rig.Run(900);
        CalibrationResult c = rig.Runner.Calibration;
        if (c == null) { Check(false, "calibration produced (" + rig.Message + ")"); return; }
        double err = Math.Abs(c.StaticSlopeNPerMm - 0.4), errDc = Math.Abs(c.DeadCentreSlopeNPerMm - 0.4);
        Check(err < 0.1 + 1.5 * lagS && err < 0.35 * errDc, string.Format("static slope {0:0.00} N/mm (model 0.40; dead centres alone {1:0.00})", c.StaticSlopeNPerMm, c.DeadCentreSlopeNPerMm));
        Check(Near(c.PreloadN, 150, 0.06), "preload " + c.PreloadN.ToString("0.0") + " N (model 150)");
        Check(Near(c.GasForceN, 160, 0.03), "gas force " + c.GasForceN.ToString("0.0") + " N (model 160)");
        // raw view: the same cycles with the static force left in
        TestRecord rec = rig.Runner.BuildRecord("raw", "");
        rig.Set.RemoveStaticForce = true;
        double removed = MeanForce(rec.Analyze(rig.Set));
        rig.Set.RemoveStaticForce = false;
        double raw = MeanForce(rec.Analyze(rig.Set));
        rig.Set.RemoveStaticForce = true;
        Check(Math.Abs(raw - removed - 160) < 10, string.Format("raw view keeps the static force: mean force {0:0} N raw, {1:0} N with it removed", raw, removed));
    }

    static void TestAverageCycles()
    {
        Console.WriteLine("Average of the measured cycles");
        var rig = new Rig(25);
        rig.Runner.Start();
        rig.Run(900);
        foreach (StepResult r in rig.Runner.Results)
        {
            ProcessedCycle a = Analyzer.AverageCycles(r.Cycles);
            double vmax = a.V.Max(), fmax = a.F.Max();
            Check(r.Cycles.Count >= 2 && Near(vmax, r.CompPeakVel, 0.02) && Near(fmax, r.CompMaxForce, 0.05),
                string.Format("{0} mm/s: {1} cycles averaged, peak speed {2:0.0} (cycles {3:0.0}), max force {4:0} N (cycles {5:0})",
                    r.TargetSpeed, r.Cycles.Count, vmax, r.CompPeakVel, fmax, r.CompMaxForce));
        }
    }

    static double MeanForce(List<StepResult> res)
    {
        double sum = 0; int k = 0;
        foreach (StepResult r in res) foreach (ProcessedCycle c in r.Cycles) foreach (double f in c.F) { sum += f; k++; }
        return sum / Math.Max(1, k);
    }

    static void TestFullRun(double amplitude, int expectedSteps, double rampHzPerS = 10)
    {
        Console.WriteLine("Full test, amplitude " + amplitude + " mm, drive ramp " + rampHzPerS + " Hz/s");
        var rig = new Rig(amplitude);
        rig.Sim.RampHzPerS = rampHzPerS;
        rig.Runner.Start();
        rig.Run(900);
        Check(rig.Success == true, "test completes (" + rig.Message + ")");
        double parked = rig.Sim.CrankAboveBdcMm;
        Check(parked < 0.02 * amplitude + 0.3, "crank parked at bottom dead centre: " + parked.ToString("0.00") + " mm above BDC"
            + (parked < 0.02 * amplitude + 0.3 ? "" : "\n         " + string.Join("\n         ", rig.Log.GetRange(Math.Max(0, rig.Log.Count - 4), Math.Min(4, rig.Log.Count)))));
        CalibrationResult c = rig.Runner.Calibration;
        if (c == null) { Check(false, "calibration produced"); return; }
        Check(Near(c.AmplitudeMm, amplitude, 0.01), "amplitude measured: " + c.AmplitudeMm.ToString("0.00") + " mm");
        Check(Near(c.PreloadN, 150, 0.03), "preload measured: " + c.PreloadN.ToString("0.0") + " N (model 150)");
        Check(Near(c.GasForceN, 150 + 0.4 * amplitude, 0.03), "gas force measured: " + c.GasForceN.ToString("0.0") + " N (model " + (150 + 0.4 * amplitude) + ")");
        Check(rig.Runner.Results.Count == expectedSteps, rig.Runner.Results.Count + " speed steps measured (expected " + expectedSteps + ")");
        bool speedOk = true, forceOk = true; string detail = "";
        foreach (StepResult r in rig.Runner.Results)
        {
            double fc = Model(r.CompPeakVel, 4, 40, 1.2), fr = -Model(-r.RebPeakVel, 8, 40, 2.5);
            if (!Near(r.CompPeakVel, r.TargetSpeed, 0.04) || !Near(-r.RebPeakVel, r.TargetSpeed, 0.04)) speedOk = false;
            if (!Near(r.CompForceAtPeakVel, fc, 0.05) || !Near(r.RebForceAtPeakVel, fr, 0.05)) forceOk = false;
            detail += string.Format("\n         {0,4:0} mm/s: v {1,6:0.0} / {2,7:0.0}   F {3,7:0.0} / {4,8:0.0}   model {5,7:0.0} / {6,8:0.0}   n={7}",
                r.TargetSpeed, r.CompPeakVel, r.RebPeakVel, r.CompForceAtPeakVel, r.RebForceAtPeakVel, fc, fr, r.CycleCount);
        }
        Check(speedOk, "measured peak speeds within 4 % of target");
        Check(forceOk, "force at peak speed within 5 % of the damper model, static force removed" + detail);

        string path = Path.Combine(Path.GetTempPath(), "dyno_test_" + amplitude + ".csv");
        rig.Runner.BuildRecord("unit test", "note: with, comma").Save(path);
        TestRecord back = TestRecord.Load(path);
        List<StepResult> again = back.Analyze(rig.Set);
        bool same = again.Count == rig.Runner.Results.Count;
        for (int i = 0; same && i < again.Count; i++)
            same = Near(again[i].CompForceAtPeakVel, rig.Runner.Results[i].CompForceAtPeakVel, 0.01)
                && Near(again[i].RebPeakVel, rig.Runner.Results[i].RebPeakVel, 0.01) && again[i].CycleCount == 3;
        Check(same && back.Notes == "note: with, comma", "CSV saves, reloads and re-analyses to the same results");
        string sum = Path.Combine(Path.GetTempPath(), "dyno_summary.csv");
        TestRecord.ExportSummary(sum, new[] { new KeyValuePair<string, List<StepResult>>(back.Name, again) }, UnitSystem.Imperial);
        Check(File.ReadAllLines(sum).Length == again.Count + 1, "summary export has one row per speed");
    }

    static double Model(double v, double c1, double knee, double c2)
    {
        return (v <= knee ? c1 * v : c1 * knee + c2 * (v - knee)) + 15 * Math.Tanh(v / 2);
    }

    /// <summary>A port whose writes and close hang forever, like a USB serial port whose cable was pulled.</summary>
    class HungLink : DynoLinkBase
    {
        public bool Hang;
        public override bool IsOpen { get { return true; } }
        protected override bool UseHeartbeat { get { return false; } }
        protected override void OpenTransport() { }
        protected override void CloseTransport() { System.Threading.Thread.Sleep(System.Threading.Timeout.Infinite); }
        protected override void Write(byte[] packet) { if (Hang) System.Threading.Thread.Sleep(System.Threading.Timeout.Infinite); }
    }

    static void TestSampleClock()
    {
        Console.WriteLine("Sample clock");
        var c = new SampleClock();
        c.Advance(1000000); c.Advance(1002000);
        bool older = c.Advance(2000);                                   // a late packet from earlier
        c.Advance(1004000);
        Check(!older && Near(c.T, 0.004, 1e-9), "an older sample is skipped instead of jumping 4295 s ahead (t = " + c.T + " s)");
        for (uint k = 0; k < 100; k++) c.Advance(5 + 2000 * k);          // ESP32 restarted: its clock went back to 0
        Check(c.T > 0.004 + 0.09 && c.T < 0.004 + 0.21, "after an ESP32 restart, time carries on without a jump (t = " + c.T.ToString("0.000") + " s)");
        var w = new SampleClock(); w.Advance(4294966000u); w.Advance(704u);   // 32-bit microsecond counter wraps after 71 min
        Check(Near(w.T, 0.002, 1e-9), "the 32-bit microsecond wrap is handled");
    }

    static void TestUnplug()
    {
        Console.WriteLine("Serial link: cable pulled");
        var l = new HungLink();
        int raised = 0;
        l.Disconnected += r => System.Threading.Interlocked.Increment(ref raised);
        l.Open();
        l.Hang = true;
        new System.Threading.Thread(() => l.MotorStop()) { IsBackground = true }.Start();   // this write never returns
        System.Threading.Thread.Sleep(50);
        var sw = System.Diagnostics.Stopwatch.StartNew();
        for (int k = 0; k < 5; k++) l.MotorRun(10);                    // must not pile up behind the stuck write
        long sendMs = sw.ElapsedMilliseconds;
        Check(sendMs < 1000 && raised == 1, "calls after a stuck write return quickly (" + sendMs + " ms) and Disconnected is raised once");
        sw.Restart();
        l.Close();                                                      // CloseTransport hangs: Close must not
        Check(sw.ElapsedMilliseconds < 100, "Close returns at once even when closing the port hangs (" + sw.ElapsedMilliseconds + " ms)");
    }

    static void TestCalibrationFile()
    {
        Console.WriteLine("Sensor calibration file");
        string dir = Path.Combine(Path.GetTempPath(), "dyno_cal_test");
        if (Directory.Exists(dir)) Directory.Delete(dir, true);
        var a = new Settings { DataFolder = dir, ForceNPerCount = -0.0047123456789, ForceZeroCounts = 12345.5, PositionMmPerCount = 5.28e-5, PositionZeroCounts = -98765, ForceGainInfo = "2026-11-17, gain with 134.75 N" };
        a.SaveCalibration(a.CalibrationPath);
        a.AppendCalibrationHistory(a.CalibrationPath, "force gain");
        var b = new Settings { DataFolder = dir };
        bool ok = b.LoadCalibration(b.CalibrationPath);
        Check(ok && b.ForceNPerCount == a.ForceNPerCount && b.ForceZeroCounts == a.ForceZeroCounts && b.PositionMmPerCount == a.PositionMmPerCount
              && b.PositionZeroCounts == a.PositionZeroCounts && b.ForceGainInfo == a.ForceGainInfo, "calibration file saves and loads exactly");
        string ini = Path.Combine(dir, "settings.ini");
        a.Save(ini);
        Check(!File.ReadAllText(ini).Contains("ForceNPerCount"), "settings.ini no longer holds the calibration");
        File.AppendAllText(ini, "ForceNPerCount=0.0123\n");                       // an older settings.ini still loads its values
        Check(Settings.Load(ini).ForceNPerCount == 0.0123, "older settings.ini with calibration values still loads");
        Check(File.ReadAllLines(Path.Combine(dir, "sensor_calibration_history.csv")).Length == 2, "history file written");
    }

    static void TestFindBdc()
    {
        Console.WriteLine("Find bottom dead centre (button)");
        foreach (double start in new[] { 0.3, 2.0, 4.0, 5.9 })
        {
            var rig = new Rig(50);
            rig.Sim.SetCrankAngle(start);
            var parker = new BdcParker(rig.Sim, rig.Set);
            var log = new List<string>();
            parker.Log += log.Add;
            double t = 0; uint last = 0; bool have = false;
            rig.Sim.SamplesReceived += smp => { foreach (RawSample r in smp) { if (have) t += unchecked(r.TimeUs - last) / 1e6; last = r.TimeUs; have = true; parker.Feed(t, rig.Set.PositionMm(r.Position)); } };
            rig.Sim.StatusReceived += parker.OnStatus;
            parker.Start(0, false, double.NaN, double.NaN, 1);
            for (double s = 0; s < 120 && !parker.Done; s += 0.02) rig.Sim.Step(0.02);
            double x = rig.Sim.CrankAboveBdcMm;
            Check(parker.Done && x < 1.0, string.Format("from {0:0.0} rad: stopped {1:0.00} mm above BDC ({2})", start, x, log.Count > 0 ? log[log.Count - 1] : "no log"));
        }
    }

    static void TestSafetyTrip()
    {
        Console.WriteLine("Safety circuit opens during a test");
        var rig = new Rig(25);
        rig.Runner.Start();
        rig.Run(200, t => { if (t > 30) rig.Sim.SafetyOk = false; });
        Check(rig.Success == false && rig.Message.Contains("Safety"), "test aborts: " + rig.Message);
        rig.Runner.Start();
        rig.Run(5);
        Check(rig.Success == false, "a new test cannot start while the fault is latched");
    }

    static void TestForceLimit()
    {
        Console.WriteLine("Force above 3 kN");
        var rig = new Rig(25);
        rig.Sim.RebC2 = 12;      // a damper far too stiff: over 3 kN before 500 mm/s
        rig.Runner.Start();
        rig.Run(900);
        Check(rig.Success == false && rig.Message.Contains("Force limit"), "motor is stopped: " + rig.Message);
    }
}

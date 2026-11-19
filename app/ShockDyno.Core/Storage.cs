using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;

namespace ShockDyno.Core
{
    /// <summary>A complete test: raw calibrated samples plus what is needed to re-analyse them. Stored as one CSV file.</summary>
    public class TestRecord
    {
        public string Name = "";
        public string Notes = "";
        public DateTime Date = DateTime.Now;
        public CalibrationResult Calibration = new CalibrationResult();
        public List<SequenceStep> Steps = new List<SequenceStep>();
        public List<double> StepMotorHz = new List<double>();
        public List<Sample> Samples = new List<Sample>();
        public List<short> StepOf = new List<short>();    // -1 = calibration cycle
        public List<short> CycleOf = new List<short>();   // 0 = not a measured cycle
        public string FilePath;

        static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

        public void Save(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var w = new StreamWriter(path, false, new UTF8Encoding(false)))
            {
                w.WriteLine("# ShockDyno test v1");
                w.WriteLine("# name: " + OneLine(Name));
                w.WriteLine("# date: " + Date.ToString("s", Inv));
                w.WriteLine("# notes: " + OneLine(Notes));
                w.WriteLine("# amplitude_mm: " + Calibration.AmplitudeMm.ToString("R", Inv));
                w.WriteLine("# bdc_mm: " + Calibration.BdcMm.ToString("R", Inv));
                w.WriteLine("# preload_N: " + Calibration.PreloadN.ToString("R", Inv));
                w.WriteLine("# gas_force_N: " + Calibration.GasForceN.ToString("R", Inv));
                w.WriteLine("# static_slope_N_per_mm: " + Calibration.StaticSlopeNPerMm.ToString("R", Inv));
                w.WriteLine("# calibration_crank_hz: " + Calibration.CrankHz.ToString("R", Inv));
                for (int i = 0; i < Steps.Count; i++)
                    w.WriteLine("# step: " + i + ", " + Steps[i] + ", " + (i < StepMotorHz.Count ? StepMotorHz[i] : 0).ToString("0.###", Inv));
                w.WriteLine("# position and force are as measured: + = compression, static force (preload, gas) NOT removed");
                w.WriteLine("time_s,position_mm,force_N,step,cycle");
                for (int i = 0; i < Samples.Count; i++)
                {
                    Sample s = Samples[i];
                    w.Write(s.T.ToString("0.000000", Inv)); w.Write(',');
                    w.Write(s.X.ToString("0.0000", Inv)); w.Write(',');
                    w.Write(s.F.ToString("0.000", Inv)); w.Write(',');
                    w.Write(StepOf[i].ToString(Inv)); w.Write(',');
                    w.WriteLine(CycleOf[i].ToString(Inv));
                }
            }
            FilePath = path;
        }

        public static TestRecord Load(string path)
        {
            var r = new TestRecord { FilePath = path, Name = Path.GetFileNameWithoutExtension(path) };
            foreach (string line in File.ReadLines(path))
            {
                if (line.Length == 0) continue;
                if (line[0] == '#')
                {
                    int colon = line.IndexOf(':');
                    if (colon < 0) continue;
                    string key = line.Substring(1, colon - 1).Trim(), val = line.Substring(colon + 1).Trim();
                    switch (key)
                    {
                        case "name": r.Name = val; break;
                        case "notes": r.Notes = val; break;
                        case "date": DateTime.TryParse(val, Inv, DateTimeStyles.None, out r.Date); break;
                        case "amplitude_mm": r.Calibration.AmplitudeMm = D(val); break;
                        case "bdc_mm": r.Calibration.BdcMm = D(val); break;
                        case "preload_N": r.Calibration.PreloadN = D(val); break;
                        case "gas_force_N": r.Calibration.GasForceN = D(val); break;
                        case "static_slope_N_per_mm": r.Calibration.StaticSlopeNPerMm = D(val); break;
                        case "calibration_crank_hz": r.Calibration.CrankHz = D(val); break;
                        case "step":
                            string[] f = val.Split(',');
                            if (f.Length >= 4)
                            {
                                r.Steps.Add(new SequenceStep
                                {
                                    Type = (StepType)Enum.Parse(typeof(StepType), f[1].Trim(), true),
                                    SpeedMmS = D(f[2]), Count = D(f[3]),
                                });
                                r.StepMotorHz.Add(f.Length >= 5 ? D(f[4]) : 0);
                            }
                            break;
                    }
                    continue;
                }
                if (!char.IsDigit(line[0]) && line[0] != '-') continue;   // column header
                string[] c = line.Split(',');
                if (c.Length < 5) continue;
                r.Samples.Add(new Sample { T = D(c[0]), X = D(c[1]), F = D(c[2]) });
                r.StepOf.Add(short.Parse(c[3], Inv));
                r.CycleOf.Add(short.Parse(c[4], Inv));
            }
            return r;
        }

        /// <summary>Re-runs the analysis on the stored samples with the current analysis settings.</summary>
        public List<StepResult> Analyze(Settings set)
        {
            var results = new List<StepResult>();
            var cycles = new Dictionary<int, List<ProcessedCycle>>();
            int n = Samples.Count, i = 0;
            RecomputeCalibration(set);
            while (i < n)
            {
                if (CycleOf[i] == 0 || StepOf[i] < 0) { i++; continue; }
                int j = i;
                while (j + 1 < n && CycleOf[j + 1] == CycleOf[i] && StepOf[j + 1] == StepOf[i]) j++;
                ProcessedCycle c = Analyzer.Process(Samples, i, j, set, Calibration);
                c.Step = StepOf[i];
                if (!cycles.ContainsKey(c.Step)) cycles[c.Step] = new List<ProcessedCycle>();
                cycles[c.Step].Add(c);
                i = j + 1;
            }
            for (int k = 0; k < Steps.Count; k++)
                if (cycles.ContainsKey(k))
                    results.Add(Analyzer.Summarize(k, Steps[k].SpeedMmS, k < StepMotorHz.Count ? StepMotorHz[k] : 0, cycles[k]));
            return results;
        }

        /// <summary>
        /// Recomputes amplitude, preload, gas force and static slope from the saved calibration cycle
        /// (step -1, cycle 1), so older files benefit from improvements in the calculation.
        /// Keeps the stored values if the file has no calibration cycle.
        /// </summary>
        public void RecomputeCalibration(Settings set)
        {
            int i0 = -1, i1 = -1;
            for (int i = 0; i < Samples.Count; i++)
                if (StepOf[i] < 0 && CycleOf[i] == 1) { if (i0 < 0) i0 = i; i1 = i; }
                else if (i0 >= 0) break;
            if (i0 < 0 || i1 - i0 < 50) return;
            try { Calibration = Analyzer.Calibrate(Samples, i0, i1, set); }
            catch (Exception) { /* keep the stored values */ }
        }

        /// <summary>True if the file starts like a saved test (so summaries and other CSV files are not listed).</summary>
        public static bool IsTestFile(string path)
        {
            try
            {
                using (var r = new StreamReader(path)) { string l = r.ReadLine(); return l != null && l.StartsWith("# ShockDyno test"); }
            }
            catch (Exception) { return false; }
        }

        public static string MakeFileName(string folder, string name, DateTime date)
        {
            var sb = new StringBuilder();
            foreach (char ch in name ?? "") sb.Append(char.IsLetterOrDigit(ch) || ch == '-' || ch == '_' ? ch : '_');
            string safe = sb.Length > 0 ? "_" + sb : "";
            return Path.Combine(folder, date.ToString("yyyyMMdd_HHmmss", Inv) + safe + ".csv");
        }

        /// <summary>Summary export: one row per test speed with peak velocity and force, in the chosen units.</summary>
        public static void ExportSummary(string path, IEnumerable<KeyValuePair<string, List<StepResult>>> tests, UnitSystem u, bool staticForceRemoved = true)
        {
            string fu = Units.ForceUnit(u), su = Units.SpeedUnit(u).Replace("/", "_per_");
            using (var w = new StreamWriter(path, false, new UTF8Encoding(false)))
            {
                w.WriteLine(string.Join(",", new[] {
                    "test", "target_speed_" + su, "motor_frequency_Hz",
                    "compression_peak_velocity_" + su, "compression_force_at_peak_velocity_" + fu, "compression_max_force_" + fu,
                    "rebound_peak_velocity_" + su, "rebound_force_at_peak_velocity_" + fu, "rebound_max_force_" + fu, "cycles", "static_force_removed" }));
                foreach (var t in tests)
                    foreach (StepResult r in t.Value)
                        w.WriteLine(string.Join(",", new[] {
                            "\"" + t.Key.Replace("\"", "'") + "\"",
                            Units.Length(r.TargetSpeed, u).ToString("0.###", Inv),
                            r.MotorHz.ToString("0.00", Inv),
                            Units.Length(r.CompPeakVel, u).ToString("0.###", Inv),
                            Units.Force(r.CompForceAtPeakVel, u).ToString("0.##", Inv),
                            Units.Force(r.CompMaxForce, u).ToString("0.##", Inv),
                            Units.Length(r.RebPeakVel, u).ToString("0.###", Inv),
                            Units.Force(r.RebForceAtPeakVel, u).ToString("0.##", Inv),
                            Units.Force(r.RebMaxForce, u).ToString("0.##", Inv),
                            r.CycleCount.ToString(Inv), staticForceRemoved ? "yes" : "no" }));
            }
        }

        static double D(string s) { return double.Parse(s.Trim(), NumberStyles.Float, Inv); }
        static string OneLine(string s) { return (s ?? "").Replace("\r", " ").Replace("\n", " "); }
    }
}

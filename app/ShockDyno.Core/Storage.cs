using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Compression;
using System.Linq;
using System.Security;
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
        /// <summary>Static force readings (crank held at TDC / BDC), null if not measured.</summary>
        public StaticReading StaticTop, StaticBottom;
        /// <summary>Damper temperature (IR thermometer, C) at the start of the test and of each step; NaN = not measured.</summary>
        public double StartTempC = double.NaN;
        public List<double> StepTempC = new List<double>();
        public double StepTemp(int k) { return k >= 0 && k < StepTempC.Count ? StepTempC[k] : double.NaN; }
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
                if (StaticTop != null && StaticBottom != null)
                {
                    w.WriteLine("# static_tdc: " + StaticTop.XMm.ToString("R", Inv) + ", " + StaticTop.FN.ToString("R", Inv) + ", " + StaticTop.CreepMm.ToString("R", Inv) + ", " + StaticTop.Dir);
                    w.WriteLine("# static_bdc: " + StaticBottom.XMm.ToString("R", Inv) + ", " + StaticBottom.FN.ToString("R", Inv) + ", " + StaticBottom.CreepMm.ToString("R", Inv) + ", " + StaticBottom.Dir);
                }
                if (!double.IsNaN(StartTempC)) w.WriteLine("# start_temperature_C: " + StartTempC.ToString("0.00", Inv));
                for (int i = 0; i < Steps.Count; i++)
                    if (!double.IsNaN(StepTemp(i))) w.WriteLine("# step_temperature_C: " + i + ", " + StepTemp(i).ToString("0.00", Inv));
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

        /// <summary>Reads only the header (name, calibration, steps): fast, for listing a test's speeds.</summary>
        public static TestRecord LoadHeader(string path) { return Load(path, true); }

        public static TestRecord Load(string path) { return Load(path, false); }

        static TestRecord Load(string path, bool headerOnly)
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
                        case "static_tdc": r.StaticTop = Reading(val); break;
                        case "static_bdc": r.StaticBottom = Reading(val); break;
                        case "start_temperature_C": r.StartTempC = D(val); break;
                        case "step_temperature_C":
                            string[] tv = val.Split(',');
                            if (tv.Length >= 2)
                            {
                                int k = int.Parse(tv[0].Trim(), Inv);
                                while (r.StepTempC.Count <= k) r.StepTempC.Add(double.NaN);
                                r.StepTempC[k] = D(tv[1]);
                            }
                            break;
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
                if (!char.IsDigit(line[0]) && line[0] != '-') { if (headerOnly) break; continue; }   // column header
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
                {
                    StepResult r = Analyzer.Summarize(k, Steps[k].SpeedMmS, k < StepMotorHz.Count ? StepMotorHz[k] : 0, cycles[k]);
                    r.TempC = StepTemp(k);
                    results.Add(r);
                }
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
            try
            {
                CalibrationResult c = Analyzer.Calibrate(Samples, i0, i1, set);
                if (StaticTop != null && StaticBottom != null) Analyzer.ApplyStatic(c, StaticTop, StaticBottom);
                Calibration = c;
            }
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
        /// <summary>
        /// Summary table, one block per test (several tests: blocks one under the other with a blank row):
        ///   run name
        ///   Target Velocity | Compression Velocity | Compression Force | Rebound Velocity | Rebound Force
        ///   units
        ///   0 row, then one row per speed (force at peak velocity; rebound velocity as a positive speed, rebound force signed).
        /// Written as a formatted Excel file (.xlsx) or as plain CSV, by the file extension.
        /// </summary>
        public static void ExportSummary(string path, IEnumerable<KeyValuePair<string, List<StepResult>>> tests, UnitSystem u, bool staticForceRemoved = true)
        {
            var blocks = new List<SummaryBlock>();
            bool si = u == UnitSystem.SI;
            string su = si ? "mm/sec" : "in/sec", fu = Units.ForceUnit(u);
            foreach (var t in tests)
            {
                var b = new SummaryBlock { Title = t.Key, Units = new[] { su, su, fu, su, fu } };
                b.Rows.Add(new double[5]);
                foreach (StepResult r in t.Value)
                    b.Rows.Add(new[] {
                        Units.Length(r.TargetSpeed, u), Units.Length(r.CompPeakVel, u), Units.Force(r.CompForceAtPeakVel, u),
                        Units.Length(Math.Abs(r.RebPeakVel), u), Units.Force(r.RebForceAtPeakVel, u) });
                blocks.Add(b);
            }
            // SI: whole mm/s and N, as on the reference sheet; imperial: 0.01 in/s and 0.1 lbf
            string[] fmt = si ? new[] { "0", "0", "0", "0", "0" } : new[] { "0.00", "0.00", "0.0", "0.00", "0.0" };
            if (string.Equals(Path.GetExtension(path), ".xlsx", StringComparison.OrdinalIgnoreCase)) Xlsx.WriteSummary(path, blocks, fmt);
            else
                using (var w = new StreamWriter(path, false, new UTF8Encoding(true)))   // BOM: Excel then shows the ° and accents right
                {
                    for (int k = 0; k < blocks.Count; k++)
                    {
                        if (k > 0) w.WriteLine();
                        w.WriteLine(Csv(blocks[k].Title));
                        w.WriteLine(string.Join(",", SummaryBlock.Headers));
                        w.WriteLine(string.Join(",", blocks[k].Units));
                        foreach (double[] row in blocks[k].Rows)
                            w.WriteLine(string.Join(",", row.Select((v, i) => v.ToString(fmt[i], Inv))));
                    }
                }
        }

        static string Csv(string s) { return "\"" + (s ?? "").Replace("\"", "'") + "\""; }

        static StaticReading Reading(string val)
        {
            string[] p = val.Split(',');
            if (p.Length < 2) return null;
            return new StaticReading { XMm = D(p[0]), FN = D(p[1]), CreepMm = p.Length > 2 ? D(p[2]) : 0, Dir = p.Length > 3 ? Math.Sign((int)D(p[3])) : 0 };
        }

        static double D(string s) { return double.Parse(s.Trim(), NumberStyles.Float, Inv); }
        static string OneLine(string s) { return (s ?? "").Replace("\r", " ").Replace("\n", " "); }
    }

    public class SummaryBlock
    {
        public static readonly string[] Headers = { "Target Velocity", "Compression Velocity", "Compression Force", "Rebound Velocity", "Rebound Force" };
        public string Title;
        public string[] Units;
        public List<double[]> Rows = new List<double[]>();
    }

    /// <summary>Minimal .xlsx writer (one sheet, inline strings, borders, centred cells, merged title) - no extra packages.</summary>
    static class Xlsx
    {
        public static void WriteSummary(string path, List<SummaryBlock> blocks, string[] fmt)
        {
            var sheet = new StringBuilder();
            var merges = new List<string>();
            int row = 0;
            Func<int, int, string> refOf = (r, c) => (char)('A' + c) + (r + 1).ToString(CultureInfo.InvariantCulture);
            foreach (SummaryBlock b in blocks)
            {
                if (row > 0) row++;                                   // blank row between tests
                // title, merged across the 5 columns
                sheet.Append(RowXml(row, 30, Enumerable.Range(0, 5).Select(c => c == 0 ? Str(refOf(row, c), b.Title, 1) : Empty(refOf(row, c), 1))));
                merges.Add(refOf(row, 0) + ":" + refOf(row, 4)); row++;
                sheet.Append(RowXml(row, 36, Enumerable.Range(0, 5).Select(c => Str(refOf(row, c), SummaryBlock.Headers[c], 2)))); row++;
                sheet.Append(RowXml(row, 0, Enumerable.Range(0, 5).Select(c => Str(refOf(row, c), b.Units[c], 1)))); row++;
                foreach (double[] r in b.Rows)
                {
                    sheet.Append(RowXml(row, 0, Enumerable.Range(0, 5).Select(c => Num(refOf(row, c), Math.Round(r[c], fmt[c].Length > 2 ? fmt[c].Length - 2 : 0), StyleFor(fmt[c])))));
                    row++;
                }
            }
            string sheetXml = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                + "<worksheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
                + "<sheetPr><pageSetUpPr fitToPage=\"1\"/></sheetPr>"
                + "<cols><col min=\"1\" max=\"5\" width=\"20\" customWidth=\"1\"/></cols>"
                + "<sheetData>" + sheet + "</sheetData>"
                + (merges.Count > 0 ? "<mergeCells count=\"" + merges.Count + "\">" + string.Concat(merges.Select(m => "<mergeCell ref=\"" + m + "\"/>")) + "</mergeCells>" : "")
                + "<pageSetup orientation=\"portrait\" fitToWidth=\"1\" fitToHeight=\"0\"/></worksheet>";
            const string styles = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                + "<styleSheet xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\">"
                + "<numFmts count=\"1\"><numFmt numFmtId=\"164\" formatCode=\"0.0\"/></numFmts>"
                + "<fonts count=\"1\"><font><sz val=\"11\"/><name val=\"Calibri\"/></font></fonts>"
                + "<fills count=\"2\"><fill><patternFill patternType=\"none\"/></fill><fill><patternFill patternType=\"gray125\"/></fill></fills>"
                + "<borders count=\"2\"><border><left/><right/><top/><bottom/><diagonal/></border>"
                + "<border><left style=\"thin\"/><right style=\"thin\"/><top style=\"thin\"/><bottom style=\"thin\"/><diagonal/></border></borders>"
                + "<cellStyleXfs count=\"1\"><xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\"/></cellStyleXfs>"
                + "<cellXfs count=\"6\">"
                + "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"0\" xfId=\"0\"/>"
                + "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyBorder=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\" vertical=\"center\"/></xf>"
                + "<xf numFmtId=\"0\" fontId=\"0\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyBorder=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\" vertical=\"center\" wrapText=\"1\"/></xf>"
                + "<xf numFmtId=\"1\" fontId=\"0\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyNumberFormat=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\"/></xf>"
                + "<xf numFmtId=\"164\" fontId=\"0\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyNumberFormat=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\"/></xf>"
                + "<xf numFmtId=\"2\" fontId=\"0\" fillId=\"0\" borderId=\"1\" xfId=\"0\" applyNumberFormat=\"1\" applyBorder=\"1\" applyAlignment=\"1\"><alignment horizontal=\"center\"/></xf>"
                + "</cellXfs><cellStyles count=\"1\"><cellStyle name=\"Normal\" xfId=\"0\" builtinId=\"0\"/></cellStyles></styleSheet>";
            if (File.Exists(path)) File.Delete(path);
            using (ZipArchive z = ZipFile.Open(path, ZipArchiveMode.Create))
            {
                Add(z, "[Content_Types].xml", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                    + "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
                    + "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
                    + "<Default Extension=\"xml\" ContentType=\"application/xml\"/>"
                    + "<Override PartName=\"/xl/workbook.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml\"/>"
                    + "<Override PartName=\"/xl/worksheets/sheet1.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml\"/>"
                    + "<Override PartName=\"/xl/styles.xml\" ContentType=\"application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml\"/>"
                    + "</Types>");
                Add(z, "_rels/.rels", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                    + "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                    + "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument\" Target=\"xl/workbook.xml\"/>"
                    + "</Relationships>");
                Add(z, "xl/workbook.xml", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                    + "<workbook xmlns=\"http://schemas.openxmlformats.org/spreadsheetml/2006/main\" xmlns:r=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships\">"
                    + "<sheets><sheet name=\"Summary\" sheetId=\"1\" r:id=\"rId1\"/></sheets></workbook>");
                Add(z, "xl/_rels/workbook.xml.rels", "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>"
                    + "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
                    + "<Relationship Id=\"rId1\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet\" Target=\"worksheets/sheet1.xml\"/>"
                    + "<Relationship Id=\"rId2\" Type=\"http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles\" Target=\"styles.xml\"/>"
                    + "</Relationships>");
                Add(z, "xl/worksheets/sheet1.xml", sheetXml);
                Add(z, "xl/styles.xml", styles);
            }
        }

        static int StyleFor(string fmt) { return fmt == "0" ? 3 : fmt == "0.0" ? 4 : 5; }

        static string RowXml(int r, double height, IEnumerable<string> cells)
        {
            return "<row r=\"" + (r + 1) + "\"" + (height > 0 ? " ht=\"" + height.ToString(CultureInfo.InvariantCulture) + "\" customHeight=\"1\"" : "") + ">" + string.Concat(cells) + "</row>";
        }
        static string Str(string at, string text, int style)
        {
            return "<c r=\"" + at + "\" s=\"" + style + "\" t=\"inlineStr\"><is><t>" + SecurityElement.Escape(text ?? "") + "</t></is></c>";
        }
        static string Num(string at, double v, int style)
        {
            return "<c r=\"" + at + "\" s=\"" + style + "\"><v>" + v.ToString("R", CultureInfo.InvariantCulture) + "</v></c>";
        }
        static string Empty(string at, int style) { return "<c r=\"" + at + "\" s=\"" + style + "\"/>"; }

        static void Add(ZipArchive z, string name, string xml)
        {
            using (var w = new StreamWriter(z.CreateEntry(name).Open(), new UTF8Encoding(false))) w.Write(xml);
        }
    }
}

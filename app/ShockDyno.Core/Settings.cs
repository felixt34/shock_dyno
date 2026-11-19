using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Reflection;

namespace ShockDyno.Core
{
    public enum UnitSystem { SI, Imperial }

    /// <summary>All user settings. Saved as a plain key=value text file.</summary>
    public class Settings
    {
        [Category("1. General"), Description("SI: N, mm, mm/s.  Imperial: lbf, in, in/s.")]
        public UnitSystem Units { get; set; }
        [Category("1. General"), Description("Base folder: sensor calibration file, sequences, and the default place for project folders and runs.")]
        public string DataFolder { get; set; }
        [Category("1. General"), Description("Project folder: where test runs are saved, and which runs the History list shows. Empty = the data folder. Usually chosen with the Project box above the History list.")]
        public string ProjectFolder { get; set; }
        [Browsable(false)]
        public string RecentProjects { get; set; }      // '|'-separated, most recent first
        [Browsable(false)]
        public string LastPort { get; set; }

        /// <summary>Where runs are saved and listed: the project folder, or the data folder if none is chosen.</summary>
        [Browsable(false)]
        public string RunsFolder { get { return string.IsNullOrEmpty(ProjectFolder) ? DataFolder : ProjectFolder; } }

        [Category("2. Drive train"), Description("Belt reduction between motor and crank (motor turns per crank turn).")]
        public double BeltRatio { get; set; }
        [Category("2. Drive train"), Description("Motor nameplate speed (rpm) at the rated frequency.")]
        public double MotorRatedRpm { get; set; }
        [Category("2. Drive train"), Description("Motor rated frequency (Hz).")]
        public double MotorRatedHz { get; set; }
        [Category("2. Drive train"), Description("Lowest motor frequency the drive can hold steadily (Hz). Slower steps are skipped.")]
        public double MinMotorHz { get; set; }
        [Category("2. Drive train"), Description("Highest motor frequency allowed (Hz). Faster steps are skipped.")]
        public double MaxMotorHz { get; set; }

        [Category("3. Test"), Description("Motor frequency (Hz) used for the slow calibration cycle that measures amplitude, preload and gas force.")]
        public double CalibrationMotorHz { get; set; }
        [Category("3. Test"), Description("A cycle counts as measured when its speed is within this % of the target.")]
        public double SpeedTolerancePct { get; set; }
        [Category("3. Test"), Description("Position must reverse by this much (mm) for a dead centre to be detected.")]
        public double CycleHysteresisMm { get; set; }
        [Category("3. Test"), Description("At the end of a completed test, turn the crank slowly (calibration frequency) and stop it at bottom dead centre.")]
        public bool ReturnToBdc { get; set; }
        [Category("3. Test"), Description("Time (ms) from the stop command until the crank has stopped. The stop is sent this much before bottom dead centre. The log after each test says how far from BDC the crank stopped: raise this value if it stops past BDC (rising again), lower it if it stops before.")]
        public double StopLagMs { get; set; }

        [Category("4. Safety"), Description("The motor is stopped when the force magnitude exceeds this value (N).")]
        public double ForceLimitN { get; set; }

        [Category("5. Analysis"), Description("Zero-phase low-pass filter cut-off (Hz) applied to force and position.")]
        public double FilterCutoffHz { get; set; }
        [Category("5. Analysis"), Description("Fixed delay of the position sensor relative to the load cell (ms). Position is advanced by this much.")]
        public double LvitDelayMs { get; set; }
        [Category("5. Analysis"), Description("True: speed graphs use the speed magnitude, so compression and rebound are both at positive speed (force keeps its sign: compression above zero, rebound below). False: signed velocity, + compression.")]
        public bool AbsoluteSpeed { get; set; }
        [Category("5. Analysis"), Description("True: the static force (preload at bottom dead centre plus the gas force slope) is subtracted, so graphs show the damping force only. False: graphs show the force as measured.")]
        public bool RemoveStaticForce { get; set; }

        [SensorCalibration, Category("6. Sensor calibration"), Description("Newtons per ADC count. Negative if compression reads negative. Set from the Sensor calibration window.")]
        public double ForceNPerCount { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("ADC counts at zero force (set by Tare).")]
        public double ForceZeroCounts { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("Millimetres per ADC count. Negative if compression reads as a decreasing value.")]
        public double PositionMmPerCount { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("ADC counts at position 0 mm.")]
        public double PositionZeroCounts { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("When and how the force zero (Tare / Zero) was last set."), ReadOnly(true)]
        public string ForceZeroInfo { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("When and how the force gain was last set."), ReadOnly(true)]
        public string ForceGainInfo { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("When and how the position calibration was last set."), ReadOnly(true)]
        public string PositionInfo { get; set; }

        /// <summary>The sensor calibration file, in the data folder.</summary>
        [Browsable(false)]
        public string CalibrationPath { get { return Path.Combine(DataFolder ?? "", "sensor_calibration.ini"); } }

        public Settings()
        {
            Units = UnitSystem.SI;
            DataFolder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ShockDyno");
            LastPort = "";
            BeltRatio = 5; MotorRatedRpm = 1750; MotorRatedHz = 60; MinMotorHz = 0.5; MaxMotorHz = 60;
            CalibrationMotorHz = 1.0; SpeedTolerancePct = 4; CycleHysteresisMm = 2; ReturnToBdc = true; StopLagMs = 300;
            ForceLimitN = 3000;
            FilterCutoffHz = 40; LvitDelayMs = 0; AbsoluteSpeed = true; RemoveStaticForce = true;
            // Theoretical values: 500 kg / 2 mV/V cell on 5 V, PGA 64;  203.2 mm / 10 V LVIT through a /4 divider, PGA 1.
            ForceNPerCount = 0.0045665; ForceZeroCounts = 0;
            PositionMmPerCount = 4.8447e-5; PositionZeroCounts = 0;
        }

        public double ForceN(double counts) { return (counts - ForceZeroCounts) * ForceNPerCount; }
        public double PositionMm(double counts) { return (counts - PositionZeroCounts) * PositionMmPerCount; }
        public double ForceCounts(double newtons) { return ForceZeroCounts + newtons / ForceNPerCount; }
        public double PositionCounts(double mm) { return PositionZeroCounts + mm / PositionMmPerCount; }

        /// <summary>Force limits in raw counts for the ESP32 (lo &lt; hi whatever the sign of the scale).</summary>
        public void ForceLimitCounts(out int lo, out int hi)
        {
            double a = ForceCounts(-ForceLimitN), b = ForceCounts(ForceLimitN);
            lo = (int)Math.Round(Math.Min(a, b));
            hi = (int)Math.Round(Math.Max(a, b));
        }

        public static string DefaultPath
        {
            get
            {
                return Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), "ShockDyno", "settings.ini");
            }
        }

        public void Save(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var lines = new List<string>();
            foreach (PropertyInfo p in GetType().GetProperties())
            {
                if (!p.CanWrite || p.IsDefined(typeof(SensorCalibrationAttribute), false)) continue;   // in their own file
                lines.Add(p.Name + "=" + Convert.ToString(p.GetValue(this, null), CultureInfo.InvariantCulture));
            }
            File.WriteAllLines(path, lines.ToArray());
        }

        public static Settings Load(string path)
        {
            var s = new Settings();
            if (File.Exists(path)) s.Apply(File.ReadAllLines(path), false);   // older files also hold the sensor calibration
            return s;
        }

        /// <summary>Saves the sensor calibration (and when / how it was made) to its own file.</summary>
        public void SaveCalibration(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            var lines = new List<string> {
                "# DynamiX shock dyno - sensor calibration",
                "# Written by the application (Tools > Sensor calibration, Tare). Values in ADC counts.",
                "# force N  = (counts - ForceZeroCounts) * ForceNPerCount",
                "# position mm = (counts - PositionZeroCounts) * PositionMmPerCount",
                "# Saved " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture),
            };
            foreach (PropertyInfo p in GetType().GetProperties())
                if (p.CanWrite && p.IsDefined(typeof(SensorCalibrationAttribute), false))
                {
                    object v = p.GetValue(this, null);
                    lines.Add(p.Name + "=" + (v is double ? ((double)v).ToString("R", CultureInfo.InvariantCulture) : Convert.ToString(v, CultureInfo.InvariantCulture)));
                }
            File.WriteAllLines(path, lines.ToArray());
        }

        /// <summary>Reads a sensor calibration file. False if it does not exist.</summary>
        public bool LoadCalibration(string path)
        {
            if (!File.Exists(path)) return false;
            Apply(File.ReadAllLines(path), true);
            return true;
        }

        /// <summary>Appends the current calibration to sensor_calibration_history.csv next to the calibration file.</summary>
        public void AppendCalibrationHistory(string path, string what)
        {
            string h = Path.Combine(Path.GetDirectoryName(path), "sensor_calibration_history.csv");
            bool header = !File.Exists(h);
            using (var w = new StreamWriter(h, true))
            {
                if (header) w.WriteLine("date,change,ForceNPerCount,ForceZeroCounts,PositionMmPerCount,PositionZeroCounts");
                w.WriteLine(string.Join(",", new[] {
                    DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture), "\"" + (what ?? "").Replace("\"", "'") + "\"",
                    ForceNPerCount.ToString("R", CultureInfo.InvariantCulture), ForceZeroCounts.ToString("R", CultureInfo.InvariantCulture),
                    PositionMmPerCount.ToString("R", CultureInfo.InvariantCulture), PositionZeroCounts.ToString("R", CultureInfo.InvariantCulture) }));
            }
        }

        void Apply(string[] fileLines, bool calibrationOnly)
        {
            var s = this;
            foreach (string line in fileLines)
            {
                if (line.StartsWith("#")) continue;
                int eq = line.IndexOf('=');
                if (eq <= 0) continue;
                PropertyInfo p = typeof(Settings).GetProperty(line.Substring(0, eq).Trim());
                if (p == null || !p.CanWrite) continue;
                if (calibrationOnly && !p.IsDefined(typeof(SensorCalibrationAttribute), false)) continue;
                string v = line.Substring(eq + 1).Trim();
                try
                {
                    object val = p.PropertyType.IsEnum ? Enum.Parse(p.PropertyType, v)
                               : Convert.ChangeType(v, p.PropertyType, CultureInfo.InvariantCulture);
                    p.SetValue(s, val, null);
                }
                catch (Exception) { /* keep the default for an unreadable value */ }
            }
        }
    }

    /// <summary>Marks the settings stored in the sensor calibration file instead of settings.ini.</summary>
    [AttributeUsage(AttributeTargets.Property)]
    public class SensorCalibrationAttribute : Attribute { }

    /// <summary>Unit conversion for display and export. Internally everything is N, mm, mm/s.</summary>
    public static class Units
    {
        public const double NPerLbf = 4.4482216, MmPerIn = 25.4;
        public static double Force(double n, UnitSystem u) { return u == UnitSystem.SI ? n : n / NPerLbf; }
        public static double Length(double mm, UnitSystem u) { return u == UnitSystem.SI ? mm : mm / MmPerIn; }
        public static string ForceUnit(UnitSystem u) { return u == UnitSystem.SI ? "N" : "lbf"; }
        public static string LengthUnit(UnitSystem u) { return u == UnitSystem.SI ? "mm" : "in"; }
        public static string SpeedUnit(UnitSystem u) { return u == UnitSystem.SI ? "mm/s" : "in/s"; }
    }

    /// <summary>Crank (scotch yoke) kinematics: peak speed = amplitude x crank angular speed.</summary>
    public static class Kinematics
    {
        public static double CrankHz(double motorHz, Settings s)
        {
            return motorHz * (s.MotorRatedRpm / 60.0) / s.MotorRatedHz / s.BeltRatio;
        }

        public static double MotorHz(double crankHz, Settings s)
        {
            return crankHz * s.BeltRatio * s.MotorRatedHz / (s.MotorRatedRpm / 60.0);
        }

        public static double MotorHzForSpeed(double peakSpeedMmS, double amplitudeMm, Settings s)
        {
            return MotorHz(peakSpeedMmS / (2 * Math.PI * amplitudeMm), s);
        }

        public static double SpeedForMotorHz(double motorHz, double amplitudeMm, Settings s)
        {
            return 2 * Math.PI * amplitudeMm * CrankHz(motorHz, s);
        }
    }
}

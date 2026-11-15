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
        [Category("1. General"), Description("Folder where test CSV files are stored.")]
        public string DataFolder { get; set; }
        [Browsable(false)]
        public string LastPort { get; set; }

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

        [Category("4. Safety"), Description("The motor is stopped when the force magnitude exceeds this value (N).")]
        public double ForceLimitN { get; set; }

        [Category("5. Analysis"), Description("Zero-phase low-pass filter cut-off (Hz) applied to force and position.")]
        public double FilterCutoffHz { get; set; }
        [Category("5. Analysis"), Description("Fixed delay of the position sensor relative to the load cell (ms). Position is advanced by this much.")]
        public double LvitDelayMs { get; set; }

        [Category("6. Sensor calibration"), Description("Newtons per ADC count. Negative if compression reads negative. Set from the Sensor calibration window.")]
        public double ForceNPerCount { get; set; }
        [Category("6. Sensor calibration"), Description("ADC counts at zero force (set by Tare).")]
        public double ForceZeroCounts { get; set; }
        [Category("6. Sensor calibration"), Description("Millimetres per ADC count. Negative if compression reads as a decreasing value.")]
        public double PositionMmPerCount { get; set; }
        [Category("6. Sensor calibration"), Description("ADC counts at position 0 mm.")]
        public double PositionZeroCounts { get; set; }

        public Settings()
        {
            Units = UnitSystem.SI;
            DataFolder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ShockDyno");
            LastPort = "";
            BeltRatio = 5; MotorRatedRpm = 1750; MotorRatedHz = 60; MinMotorHz = 0.5; MaxMotorHz = 60;
            CalibrationMotorHz = 1.0; SpeedTolerancePct = 4; CycleHysteresisMm = 2;
            ForceLimitN = 3000;
            FilterCutoffHz = 40; LvitDelayMs = 0;
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
                if (!p.CanWrite) continue;
                lines.Add(p.Name + "=" + Convert.ToString(p.GetValue(this, null), CultureInfo.InvariantCulture));
            }
            File.WriteAllLines(path, lines.ToArray());
        }

        public static Settings Load(string path)
        {
            var s = new Settings();
            if (!File.Exists(path)) return s;
            foreach (string line in File.ReadAllLines(path))
            {
                int eq = line.IndexOf('=');
                if (eq <= 0) continue;
                PropertyInfo p = typeof(Settings).GetProperty(line.Substring(0, eq).Trim());
                if (p == null || !p.CanWrite) continue;
                string v = line.Substring(eq + 1).Trim();
                try
                {
                    object val = p.PropertyType.IsEnum ? Enum.Parse(p.PropertyType, v)
                               : Convert.ChangeType(v, p.PropertyType, CultureInfo.InvariantCulture);
                    p.SetValue(s, val, null);
                }
                catch (Exception) { /* keep the default for an unreadable value */ }
            }
            return s;
        }
    }

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

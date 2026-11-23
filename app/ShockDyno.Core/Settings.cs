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
        [Category("1. General"), Description("How often the screen is refreshed (ms, 50 to 1000). Changing it helps find whether noise at that rate comes from the PC (ground loop through the USB cable).")]
        public int ScreenRefreshMs { get; set; }
        [Category("1. General"), Description("Show the run names in the corner of the result graphs. Off by default: the colour squares in the History list say which run is which, and hovering over a trace shows its name.")]
        public bool ShowGraphLegend { get; set; }
        [Category("1. General"), Description("Damper temperature (IR thermometer) shown in the Machine box: read every this many seconds while no test runs. 0 = only at the start of a test and of each speed. During a test it is only read at those moments.")]
        public double TemperatureRefreshS { get; set; }
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
        [Category("3. Test"), Description("Drive frequency tolerance (Hz) for 'the drive has reached the commanded speed'. In sensorless vector (SVC) mode the drive adds slip compensation to its reported output frequency, which then moves with the load; this allowance (capped at 30 % of the commanded frequency) stops that from being taken for a ramp. In V/F mode the output frequency equals the command and this hardly matters.")]
        public double DriveSlipAllowanceHz { get; set; }
        [Category("3. Test"), Description("Cycles run and thrown away after each speed change (and in the calibration cycle) before measuring. 1 = the safe default: the measured cycles must also match the one before within 2 %. 0 = measure from the first full cycle that starts after the drive reports the new speed: saves one cycle per speed (about 10-16 s at the lowest speeds).")]
        public int SettlingCycles { get; set; }

        /// <summary>How far the drive's reported output frequency may be from the command and still count as at speed.</summary>
        public double DriveHzTolerance(double hz) { return 0.05 + 0.005 * Math.Abs(hz) + Math.Min(Math.Max(0, DriveSlipAllowanceHz), 0.3 * Math.Abs(hz)); }
        [Category("3. Test"), Description("Position must reverse by this much (mm) for a dead centre to be detected.")]
        public double CycleHysteresisMm { get; set; }
        [Category("3. Test"), Description("At the end of a completed test, turn the crank slowly (calibration frequency) and stop it at bottom dead centre.")]
        public bool ReturnToBdc { get; set; }
        [Category("3. Test"), Description("True: no slow calibration cycle at the start of a test. The amplitude, bottom dead centre and static force (preload, gas force) are then taken from the first steady cycle of the first step (usually the warm-up), and that step's speed is corrected once they are known. Faster, but the static force is measured at speed, so it is less accurate than with the slow calibration cycle.")]
        public bool SkipCalibrationCycle { get; set; }
        [Category("3. Test"), Description("Crank amplitude (mm, half stroke) set on the machine. Only used when the calibration cycle is skipped, for the speed of the first step until the amplitude has been measured.")]
        public double NominalAmplitudeMm { get; set; }
        [Category("3. Test"), Description("Time (ms) from the stop command until the crank has stopped. The stop is sent this much before bottom dead centre. The log after each test says how far from BDC the crank stopped: raise this value if it stops past BDC (rising again), lower it if it stops before.")]
        public double StopLagMs { get; set; }
        [Category("3. Test"), Description("Motor frequency (Hz) for returning to and finding bottom dead centre. After a test the drive first slows to this frequency, and the crank must have turned steadily at it for ParkSettleS before the stop point is armed, so it cannot overshoot whatever the test speed was.")]
        public double ParkMotorHz { get; set; }
        [Category("3. Test"), Description("Time (s) the crank must turn steadily at ParkMotorHz before the stop at bottom dead centre is armed.")]
        public double ParkSettleS { get; set; }
        [Category("3. Test"), Description("Static force measured with the crank stopped: after the calibration cycle the crank is held still at top and then bottom dead centre and the force averaged. The gas force at mid-stroke is the mean of the two (seal friction cancels), so it contains no damping force. The slope still comes from the moving cycle. Off = the moving calibration cycle alone (force at the dead centres while turning).")]
        public bool StaticCalibration { get; set; }
        [Category("3. Test"), Description("Two-speed approach to a dead centre: turn at this motor frequency (Hz) for most of the turn, then drop to ParkMotorHz for the last part (ParkSlowAngleDeg) and stop from there. Saves up to a slow crank turn (about 10 s at 1 Hz). 0 = the whole way at ParkMotorHz.")]
        public double ParkApproachHz { get; set; }
        [Category("3. Test"), Description("Two-speed approach: crank angle (degrees) before the dead centre where the speed drops to ParkMotorHz. It must leave time to slow down and turn steadily for ParkSettleS; if not, the crank goes round once more at the slow speed.")]
        public double ParkSlowAngleDeg { get; set; }
        [Category("3. Test"), Description("Static force: time (s) to wait after the crank stops before averaging.")]
        public double StaticSettleS { get; set; }
        [Category("3. Test"), Description("Static force: averaging time (s) at each dead centre.")]
        public double StaticAverageS { get; set; }
        [Category("3. Test"), Description("Static force: if the crank moves more than this (mm) while the force is averaged, the shock is back-driving it; the static values are then not used (moving-cycle values kept).")]
        public double StaticCreepLimitMm { get; set; }

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
        [SensorCalibration, Category("6. Sensor calibration"), Description("Force subtracted by Tare (N, absolute). Displayed and recorded force = absolute force - this. The force limit does not use it: it is always counted from the absolute zero (ForceZeroCounts), set with Zero in the Sensor calibration window.")]
        public double ForceTareN { get; set; }
        [SensorCalibration, Category("6. Sensor calibration"), Description("When the tare was last set."), ReadOnly(true)]
        public string ForceTareInfo { get; set; }
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
            CalibrationMotorHz = 1.0; DriveSlipAllowanceHz = 1.0; SettlingCycles = 1; SpeedTolerancePct = 4; CycleHysteresisMm = 2; ReturnToBdc = true; StopLagMs = 300; ParkMotorHz = 1.0; ParkSettleS = 1.0; StaticCalibration = true; ParkApproachHz = 5; ParkSlowAngleDeg = 100; StaticSettleS = 2; StaticAverageS = 1; StaticCreepLimitMm = 0.3; SkipCalibrationCycle = false; NominalAmplitudeMm = 25;
            ForceLimitN = 3000;
            FilterCutoffHz = 40; LvitDelayMs = 0; AbsoluteSpeed = true; RemoveStaticForce = true; ScreenRefreshMs = 200; TemperatureRefreshS = 2;
            // Theoretical values: 500 kg / 2 mV/V cell on 5 V, PGA 64;  203.2 mm / 10 V LVIT through a /4 divider, PGA 1.
            ForceNPerCount = 0.0045665; ForceZeroCounts = 0;
            PositionMmPerCount = 4.8447e-5; PositionZeroCounts = 0;
        }

        /// <summary>Force relative to the tare: what is displayed and recorded.</summary>
        public double ForceN(double counts) { return ForceAbsoluteN(counts) - ForceTareN; }
        /// <summary>Force relative to the absolute zero (no load on the cell), set in the Sensor calibration window.</summary>
        public double ForceAbsoluteN(double counts) { return (counts - ForceZeroCounts) * ForceNPerCount; }
        public double PositionMm(double counts) { return (counts - PositionZeroCounts) * PositionMmPerCount; }
        /// <summary>Raw counts for an absolute force (no tare).</summary>
        public double ForceCounts(double absoluteNewtons) { return ForceZeroCounts + absoluteNewtons / ForceNPerCount; }
        public double PositionCounts(double mm) { return PositionZeroCounts + mm / PositionMmPerCount; }

        /// <summary>
        /// Force limits in raw counts for the ESP32 (lo &lt; hi whatever the sign of the scale): +/- ForceLimitN
        /// from the ABSOLUTE zero, whatever the tare.
        /// </summary>
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

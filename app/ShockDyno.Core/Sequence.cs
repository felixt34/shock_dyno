using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace ShockDyno.Core
{
    public enum StepType { Warmup, Cycles }

    public class SequenceStep
    {
        public StepType Type { get; set; }
        /// <summary>Target peak speed, mm/s.</summary>
        public double SpeedMmS { get; set; }
        /// <summary>Warm-up: duration in seconds. Cycles: number of measured cycles.</summary>
        public double Count { get; set; }

        public override string ToString()
        {
            return Type == StepType.Warmup
                ? string.Format(CultureInfo.InvariantCulture, "warmup, {0}, {1}", SpeedMmS, Count)
                : string.Format(CultureInfo.InvariantCulture, "cycles, {0}, {1}", SpeedMmS, Count);
        }
    }

    /// <summary>
    /// A test protocol. Text format, one step per line:
    ///   warmup, speed mm/s, seconds        cycles, speed mm/s, number of cycles        # comment
    /// </summary>
    public class TestSequence
    {
        public List<SequenceStep> Steps = new List<SequenceStep>();

        public static TestSequence Default()
        {
            var q = new TestSequence();
            q.Steps.Add(new SequenceStep { Type = StepType.Warmup, SpeedMmS = 50, Count = 20 });
            foreach (double v in new double[] { 15, 25, 50, 100, 250, 500 })
                q.Steps.Add(new SequenceStep { Type = StepType.Cycles, SpeedMmS = v, Count = 3 });
            return q;
        }

        public static TestSequence Parse(IEnumerable<string> lines)
        {
            var q = new TestSequence();
            int n = 0;
            foreach (string raw in lines)
            {
                n++;
                string line = raw;
                int hash = line.IndexOf('#');
                if (hash >= 0) line = line.Substring(0, hash);
                line = line.Trim();
                if (line.Length == 0) continue;
                string[] f = line.Split(',');
                StepType type;
                double speed, count;
                if (f.Length != 3 || !Enum.TryParse(f[0].Trim(), true, out type)
                    || !double.TryParse(f[1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out speed)
                    || !double.TryParse(f[2].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out count)
                    || speed <= 0 || count <= 0)
                    throw new FormatException("Line " + n + ": expected \"warmup|cycles, speed, count\" but got \"" + raw + "\"");
                q.Steps.Add(new SequenceStep { Type = type, SpeedMmS = speed, Count = count });
            }
            return q;
        }

        public static TestSequence Load(string path) { return Parse(File.ReadAllLines(path)); }

        public void Save(string path)
        {
            var lines = new List<string> { "# Shock dyno test sequence", "# warmup, speed mm/s, seconds   |   cycles, speed mm/s, number of cycles" };
            foreach (SequenceStep s in Steps) lines.Add(s.ToString());
            File.WriteAllLines(path, lines.ToArray());
        }
    }
}

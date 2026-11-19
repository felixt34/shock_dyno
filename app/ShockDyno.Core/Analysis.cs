using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    /// <summary>One calibrated sample. + is compression for both position and force.</summary>
    public struct Sample
    {
        public double T;   // s
        public double X;   // mm (absolute sensor position)
        public double F;   // N (as measured, static force included)
    }

    /// <summary>Result of the slow calibration cycle run before every test.</summary>
    public class CalibrationResult
    {
        public double AmplitudeMm;       // half of the stroke
        public double BdcMm;             // absolute sensor position at bottom dead centre
        public double PreloadN;          // static force at bottom dead centre
        public double GasForceN;         // static force at mid-stroke
        public double StaticSlopeNPerMm; // static force change per mm of compression (gas spring / spring rate)
        public double DeadCentreSlopeNPerMm; // the same from the dead-centre forces only (for comparison; biased by bushings)
        public double CrankHz;           // measured crank frequency during the calibration cycle

        /// <summary>Static (speed-independent) force at xr mm above bottom dead centre.</summary>
        public double StaticForce(double xr) { return PreloadN + StaticSlopeNPerMm * xr; }
    }

    /// <summary>One processed cycle, bottom dead centre to bottom dead centre.</summary>
    public class ProcessedCycle
    {
        public int Step;
        public double[] T;   // s from the start of the cycle
        public double[] X;   // mm above bottom dead centre
        public double[] V;   // mm/s, + = compression
        public double[] F;   // N, static force removed, + = compression
        public double Period { get { return T[T.Length - 1]; } }
    }

    public class StepResult
    {
        public int Step;
        public double TargetSpeed;       // mm/s
        public double MotorHz;
        public int CycleCount;
        public double CompPeakVel, CompForceAtPeakVel, CompMaxForce;
        public double RebPeakVel, RebForceAtPeakVel, RebMaxForce;   // negative values
        public List<ProcessedCycle> Cycles = new List<ProcessedCycle>();
    }

    /// <summary>Finds bottom dead centres (position minima) in a live position stream.</summary>
    public class CycleDetector
    {
        readonly double _h;
        int _mode;           // 0 = waiting for a first maximum, 1 = looking for a minimum, 2 = looking for a maximum
        double _min, _max;
        int _minIdx;

        public CycleDetector(double hysteresisMm) { _h = hysteresisMm; _max = double.MinValue; }

        /// <summary>Feed one sample. Returns the index of a bottom dead centre when one is confirmed, otherwise -1.</summary>
        public int Feed(int index, double x)
        {
            if (_mode == 1)
            {
                if (x < _min) { _min = x; _minIdx = index; }
                else if (x > _min + _h) { _mode = 2; _max = x; return _minIdx; }
            }
            else
            {
                if (x > _max) _max = x;
                else if (x < _max - _h) { _mode = 1; _min = x; _minIdx = index; }
            }
            return -1;
        }
    }

    public static class Analyzer
    {
        const int Margin = 150;   // extra samples processed on each side so filters are settled inside the cycle

        /// <summary>
        /// Filters, differentiates and (when cal is given) removes the static force for samples [i0, i1].
        /// </summary>
        public static ProcessedCycle Process(IList<Sample> s, int i0, int i1, Settings set, CalibrationResult cal)
        {
            int a = Math.Max(0, i0 - Margin), b = Math.Min(s.Count - 1, i1 + Margin), n = b - a + 1;
            var t = new double[n]; var x = new double[n]; var f = new double[n];
            for (int i = 0; i < n; i++) { Sample p = s[a + i]; t[i] = p.T; x[i] = p.X; f[i] = p.F; }
            double fs = (n - 1) / Math.Max(1e-9, t[n - 1] - t[0]);
            x = Signal.LowPassZeroPhase(x, set.FilterCutoffHz, fs);
            f = Signal.LowPassZeroPhase(f, set.FilterCutoffHz, fs);
            if (set.LvitDelayMs != 0)
            {
                var xs = new double[n];
                int hint = 0;
                for (int i = 0; i < n; i++) xs[i] = Signal.Interp(t, x, t[i] + set.LvitDelayMs / 1000.0, ref hint);
                x = xs;
            }
            int hw = Math.Max(3, Math.Min(50, (int)Math.Round((i1 - i0) / 100.0)));
            double[] v = Signal.Derivative(t, x, hw);

            int m = i1 - i0 + 1, o = i0 - a;
            double bdc;
            if (cal != null) bdc = cal.BdcMm;
            else { bdc = double.MaxValue; for (int i = 0; i < m; i++) bdc = Math.Min(bdc, x[o + i]); }
            var c = new ProcessedCycle { T = new double[m], X = new double[m], V = new double[m], F = new double[m] };
            for (int i = 0; i < m; i++)
            {
                c.T[i] = t[o + i] - t[o];
                c.X[i] = x[o + i] - bdc;
                c.V[i] = v[o + i];
                c.F[i] = f[o + i] - (cal != null && set.RemoveStaticForce ? cal.StaticForce(c.X[i]) : 0);
            }
            return c;
        }

        /// <summary>
        /// Analyses the slow calibration cycle [i0, i1] (dead centre to dead centre). The static force is read
        /// at the two dead centres, where the speed is zero and the damper produces no damping force, so the
        /// result does not depend on how different compression and rebound damping are. Each reading is an
        /// average over a short window centred on the dead centre, which also cancels seal friction.
        /// </summary>
        public static CalibrationResult Calibrate(IList<Sample> s, int i0, int i1, Settings set)
        {
            ProcessedCycle c = Process(s, i0, i1, set, null);
            int n = c.X.Length, top = 0;
            for (int i = 1; i < n; i++) if (c.X[i] > c.X[top]) top = i;
            double stroke = c.X[top];
            var r = new CalibrationResult { AmplitudeMm = stroke / 2, CrankHz = 1 / c.Period };
            r.BdcMm = FilteredMin(s, i0, i1, set);

            // Level: force at the two dead centres (speed below 3 % of peak), averaged on both sides of each.
            // Seal friction flips sign there, and rubber bushings make the force reverse a little after the
            // crank does, so each dead-centre value is biased (TDC high, BDC low) - but equally and oppositely,
            // so their mean, the static force at mid-stroke, is right.
            int w = Math.Max(3, (int)Math.Round(0.005 * n));
            double fBdc = 0, fTdc = 0; int nb = 0, nt = 0;
            for (int i = 0; i <= w && i < n; i++) { fBdc += c.F[i] + c.F[n - 1 - i]; nb += 2; }
            for (int i = Math.Max(0, top - w); i <= Math.Min(n - 1, top + w); i++) { fTdc += c.F[i]; nt++; }
            fBdc /= nb; fTdc /= nt;
            r.GasForceN = (fBdc + fTdc) / 2;

            // Slope: from the middle of the stroke, away from the turnarounds. A straight line through the
            // compression branch and one through the rebound branch over 20-80 % of the stroke; the static
            // force is midway between them, so its slope is the mean of the two slopes. The speed profile is
            // symmetric about mid-stroke, so different compression / rebound damping does not tilt it.
            double bc, br;
            r.DeadCentreSlopeNPerMm = stroke > 0 ? (fTdc - fBdc) / stroke : 0;
            if (FitBranch(c, stroke, true, out bc) && FitBranch(c, stroke, false, out br)) r.StaticSlopeNPerMm = (bc + br) / 2;
            else r.StaticSlopeNPerMm = r.DeadCentreSlopeNPerMm;   // too few points: fall back on the dead centres
            r.PreloadN = r.GasForceN - r.StaticSlopeNPerMm * r.AmplitudeMm;
            return r;
        }

        /// <summary>Least-squares slope (N/mm) of force against position on one branch, over 20-80 % of the stroke.</summary>
        static bool FitBranch(ProcessedCycle c, double stroke, bool compression, out double slope)
        {
            double sx = 0, sy = 0, sxx = 0, sxy = 0; int k = 0;
            for (int i = 0; i < c.X.Length; i++)
            {
                if ((c.V[i] > 0) != compression || c.X[i] < 0.2 * stroke || c.X[i] > 0.8 * stroke) continue;
                sx += c.X[i]; sy += c.F[i]; sxx += c.X[i] * c.X[i]; sxy += c.X[i] * c.F[i]; k++;
            }
            double d = k * sxx - sx * sx;
            slope = k >= 20 && d > 1e-9 ? (k * sxy - sx * sy) / d : 0;
            return k >= 20 && d > 1e-9;
        }

        static double FilteredMin(IList<Sample> s, int i0, int i1, Settings set)
        {
            int a = Math.Max(0, i0 - Margin), b = Math.Min(s.Count - 1, i1 + Margin), n = b - a + 1;
            var t0 = s[a].T; var x = new double[n];
            for (int i = 0; i < n; i++) x[i] = s[a + i].X;
            double fs = (n - 1) / Math.Max(1e-9, s[b].T - t0);
            x = Signal.LowPassZeroPhase(x, set.FilterCutoffHz, fs);
            double min = double.MaxValue;
            for (int i = i0 - a; i <= i1 - a; i++) min = Math.Min(min, x[i]);
            return min;
        }

        /// <summary>Peak values of a step, averaged over its measured cycles.</summary>
        public static StepResult Summarize(int step, double targetSpeed, double motorHz, List<ProcessedCycle> cycles)
        {
            var r = new StepResult { Step = step, TargetSpeed = targetSpeed, MotorHz = motorHz, CycleCount = cycles.Count, Cycles = cycles };
            foreach (ProcessedCycle c in cycles)
            {
                double vmax = double.MinValue, vmin = double.MaxValue, fmax = double.MinValue, fmin = double.MaxValue;
                for (int i = 0; i < c.V.Length; i++)
                {
                    vmax = Math.Max(vmax, c.V[i]); vmin = Math.Min(vmin, c.V[i]);
                    fmax = Math.Max(fmax, c.F[i]); fmin = Math.Min(fmin, c.F[i]);
                }
                double sfc = 0, sfr = 0; int ncp = 0, nrb = 0;
                for (int i = 0; i < c.V.Length; i++)       // force averaged where speed is within 2 % of its peak
                {
                    if (c.V[i] >= 0.98 * vmax) { sfc += c.F[i]; ncp++; }
                    if (c.V[i] <= 0.98 * vmin) { sfr += c.F[i]; nrb++; }
                }
                r.CompPeakVel += vmax; r.RebPeakVel += vmin;
                r.CompMaxForce += fmax; r.RebMaxForce += fmin;
                r.CompForceAtPeakVel += sfc / Math.Max(1, ncp);
                r.RebForceAtPeakVel += sfr / Math.Max(1, nrb);
            }
            int k = Math.Max(1, cycles.Count);
            r.CompPeakVel /= k; r.RebPeakVel /= k; r.CompMaxForce /= k; r.RebMaxForce /= k;
            r.CompForceAtPeakVel /= k; r.RebForceAtPeakVel /= k;
            return r;
        }
    }
}

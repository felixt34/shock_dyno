using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    /// <summary>
    /// Turns the crank slowly (calibration frequency) and stops it at bottom dead centre.
    /// If the BDC position is not known yet, it first watches one full stroke to find it.
    /// Feed it every position sample and every status, from the same thread.
    /// </summary>
    public class BdcParker
    {
        readonly IDynoLink _link;
        readonly Settings _set;
        public event Action<string> Log;

        /// <summary>The crank is at rest (at BDC, or stopped early: see the log).</summary>
        public bool Done { get; private set; }
        public bool Seeking { get { return !_known; } }

        bool _known, _atSpeed, _approaching, _stopSent;
        double _bdc, _amp, _crankHz, _margin, _maxMove, _start, _parkStart, _stopTime, _lastMove;
        string _note;

        // finding BDC: one confirmed minimum, then one confirmed maximum
        CycleDetector _det;
        int _n, _seekMode;
        double _min = double.MaxValue, _max = double.MinValue;

        readonly Queue<double> _recent = new Queue<double>();      // last 50 ms of position (500 samples/s)

        public BdcParker(IDynoLink link, Settings set) { _link = link; _set = set; }

        /// <param name="t">current time (s), same clock as Feed</param>
        /// <param name="motorRunning">the motor is already turning (end of a test): only change its frequency</param>
        /// <param name="bdcMm">sensor position at BDC, or NaN to find it first</param>
        /// <param name="amplitudeMm">half stroke (ignored when bdcMm is NaN)</param>
        /// <param name="speedFactor">measured crank speed / nominal (1 if unknown)</param>
        public void Start(double t, bool motorRunning, double bdcMm, double amplitudeMm, double speedFactor)
        {
            _crankHz = Kinematics.CrankHz(_set.CalibrationMotorHz, _set) * speedFactor;
            _known = !double.IsNaN(bdcMm);
            if (_known) SetGeometry(bdcMm, amplitudeMm);
            else _det = new CycleDetector(_set.CycleHysteresisMm);
            _start = _parkStart = t;
            Say(_known ? "Returning to bottom dead centre" : "Finding bottom dead centre (one or two slow turns)");
            if (motorRunning) _link.MotorSetFreq(_set.CalibrationMotorHz);
            else _link.MotorRun(_set.CalibrationMotorHz);
        }

        void SetGeometry(double bdcMm, double amplitudeMm)
        {
            _bdc = bdcMm;
            _amp = Math.Max(1, amplitudeMm);
            // Near BDC the position is A (1 - cos wt): stop this far above BDC so the crank coasts onto it.
            double angle = 2 * Math.PI * _crankHz * Math.Max(0, _set.StopLagMs) / 1000.0;
            _margin = _amp * (1 - Math.Cos(Math.Min(angle, Math.PI / 2)));
            // Largest position change in 50 ms at the parking speed (x2): faster means the drive is still slowing down.
            _maxMove = 2 * _amp * 2 * Math.PI * _crankHz * 0.05 + 0.05;
        }

        public void OnStatus(DynoStatus st)
        {
            if (!Done && st.VfdOk && st.VfdHz <= 1.25 * _set.CalibrationMotorHz + 0.2) _atSpeed = true;
        }

        /// <summary>Stops the motor and ends (fault, STOP button, refused command).</summary>
        public void Cancel(string reason)
        {
            if (Done) return;
            _link.MotorStop();
            Done = true;
            Say("Bottom dead centre: stopped (" + reason + ")");
        }

        public void Feed(double t, double x)
        {
            if (Done) return;
            if (!_atSpeed && t - _start > 10) _atSpeed = true;    // no drive frequency reported: assume it has slowed down
            if (!_atSpeed) return;
            double period = 1 / _crankHz;

            if (!_known)
            {
                if (_seekMode == 0)
                {
                    _min = Math.Min(_min, x);
                    if (_det.Feed(_n++, x) >= 0) { _seekMode = 1; _max = x; }
                }
                else
                {
                    _max = Math.Max(_max, x);
                    if (x < _max - _set.CycleHysteresisMm)
                    {
                        SetGeometry(_min, (_max - _min) / 2);
                        _known = true; _parkStart = t;
                        Say(string.Format("Stroke found: amplitude {0:0.0} mm", _amp));
                    }
                }
                if (t - _start > 3 * period + 15) Cancel("no full stroke seen; is the crank turning and the LVIT working?");
                return;
            }

            double xr = x - _bdc;
            _recent.Enqueue(xr);
            if (_recent.Count > 25) _recent.Dequeue();
            double moved = xr - _recent.Peek();                   // over the last 50 ms
            if (!_stopSent)
            {
                bool full = _recent.Count == 25, slow = Math.Abs(moved) < _maxMove;
                bool falling = full && slow && moved < -0.01;
                if (falling && xr < 0.25 * _amp) _approaching = true;
                if (falling && xr <= _margin) SendStop(t, null);
                else if (_approaching && full && moved > 0.01) SendStop(t, "passed bottom dead centre before the stop");
                else if (t - _parkStart > 1.5 * period + 15) SendStop(t, "bottom dead centre not reached in time");
                return;
            }
            if (Math.Abs(moved) > 0.02) _lastMove = t;
            if (t - _lastMove > 0.5 || t - _stopTime > 5)          // crank at rest (or gave up waiting)
            {
                Done = true;
                string where = string.Format("Crank stopped {0:0.0} mm from bottom dead centre", Math.Abs(xr));
                Say(_note == null ? where : where + " (" + _note + ")");
            }
        }

        void SendStop(double t, string note)
        {
            _link.MotorStop();
            _stopSent = true; _stopTime = t; _lastMove = t; _note = note;
        }

        void Say(string m) { var h = Log; if (h != null) h(m); }
    }
}

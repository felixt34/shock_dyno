using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    /// <summary>
    /// Turns the crank slowly (ParkMotorHz) and stops it at bottom dead centre. At the end of a test the drive is
    /// first slowed to ParkMotorHz; the stop point is only armed once the drive reports that frequency AND the crank
    /// has turned steadily at the slow speed for ParkSettleS, so a fast test speed cannot carry it past BDC.
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

        bool _known, _atSpeed, _driveSlow, _approaching, _stopSent;
        double _bdc, _amp, _crankHz, _margin, _maxMove, _start, _parkStart, _stopTime, _lastMove, _slowSince = double.NaN;
        string _note;

        // finding BDC: one confirmed minimum, then one confirmed maximum
        CycleDetector _det;
        int _n, _seekMode;
        double _min = double.MaxValue, _max = double.MinValue;

        readonly Queue<double[]> _recent = new Queue<double[]>();  // last 50 ms of { time, position }, whatever the sample rate

        public BdcParker(IDynoLink link, Settings set) { _link = link; _set = set; }

        /// <param name="t">current time (s), same clock as Feed</param>
        /// <param name="motorRunning">the motor is already turning (end of a test): only change its frequency</param>
        /// <param name="bdcMm">sensor position at BDC, or NaN to find it first</param>
        /// <param name="amplitudeMm">half stroke (ignored when bdcMm is NaN)</param>
        /// <param name="speedFactor">measured crank speed / nominal (1 if unknown)</param>
        public void Start(double t, bool motorRunning, double bdcMm, double amplitudeMm, double speedFactor)
        {
            _crankHz = Kinematics.CrankHz(ParkHz, _set) * speedFactor;
            _known = !double.IsNaN(bdcMm);
            if (_known) SetGeometry(bdcMm, amplitudeMm);
            else _det = new CycleDetector(_set.CycleHysteresisMm);
            _start = _parkStart = t;
            Say(_known ? string.Format("Returning to bottom dead centre: slowing to {0:0.0#} Hz first", ParkHz) : "Finding bottom dead centre (one or two slow turns)");
            if (motorRunning) _link.MotorSetFreq(ParkHz);
            else _link.MotorRun(ParkHz);
        }

        double ParkHz { get { return Math.Max(_set.MinMotorHz, _set.ParkMotorHz > 0 ? _set.ParkMotorHz : _set.CalibrationMotorHz); } }

        void SetGeometry(double bdcMm, double amplitudeMm)
        {
            _bdc = bdcMm;
            _amp = Math.Max(1, amplitudeMm);
            // Near BDC the position is A (1 - cos wt): stop this far above BDC so the crank coasts onto it.
            double angle = 2 * Math.PI * _crankHz * Math.Max(0, _set.StopLagMs) / 1000.0;
            _margin = _amp * (1 - Math.Cos(Math.Min(angle, Math.PI / 2)));
            // Largest position change in 50 ms at the parking speed (+40 %): faster means the crank is still slowing down.
            _maxMove = 1.4 * _amp * 2 * Math.PI * _crankHz * 0.05 + 0.05;
        }

        public void OnStatus(DynoStatus st)
        {
            if (!Done && st.VfdOk && st.VfdHz <= ParkHz + 0.15) _driveSlow = true;
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
            if (!_driveSlow && t - _start > 30) _driveSlow = true;   // no drive frequency reported: rely on the motion check below
            double period = 1 / _crankHz;

            if (!_known)
            {
                if (!_driveSlow) return;
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
            _recent.Enqueue(new[] { t, xr });
            while (_recent.Count > 2 && t - _recent.Peek()[0] > 0.05) _recent.Dequeue();
            double moved = xr - _recent.Peek()[1];                // over the last 50 ms
            bool full = t - _recent.Peek()[0] > 0.04, slow = Math.Abs(moved) < _maxMove;
            if (!_atSpeed)
            {
                // Armed only after the drive is at the parking frequency and the crank has moved no faster than
                // the parking speed for ParkSettleS without a break.
                if (!_driveSlow || !full || !slow || double.IsNaN(_slowSince)) { _slowSince = t; return; }
                if (t - _slowSince < Math.Max(0.2, _set.ParkSettleS)) return;
                _atSpeed = true; _parkStart = t;
                Say(string.Format("Crank turning steadily at {0:0.0#} Hz: stopping at the next bottom dead centre", ParkHz));
            }
            if (!_stopSent)
            {
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

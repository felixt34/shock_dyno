using System;
using System.Collections.Generic;
using System.Threading;

namespace ShockDyno.Core
{
    /// <summary>
    /// A virtual ESP32 + machine + damper. It speaks the real packet protocol, so the whole application
    /// (and the tests) can run without hardware. Choose "Simulator" as the port in the app.
    /// </summary>
    public class SimulatedDynoLink : DynoLinkBase
    {
        readonly Settings _set;
        readonly PacketDecoder _rx = new PacketDecoder();
        readonly object _lock = new object(), _deliver = new object(), _tick = new object();
        readonly Random _rnd = new Random(1);
        readonly List<byte[]> _out = new List<byte[]>();
        Timer _timer;
        bool _open;

        // machine and damper model (edit freely for experiments)
        public double AmplitudeMm = 25, CenterMm = 100;
        public double GasForceN = 150, GasRateNPerMm = 0.4, FrictionN = 15;
        public double CompC1 = 4, CompKnee = 40, CompC2 = 1.2;     // N per mm/s below / above the knee speed
        public double RebC1 = 8, RebKnee = 40, RebC2 = 2.5;
        public double ForceNoiseN = 1.5, PosNoiseMm = 0.003;
        public double Slip = 0.025, RampHzPerS = 10;
        public double StopLagS = 0.25;
        /// <summary>Rubber bushings / play: the damping and friction force follows the motion with this time constant (s).</summary>
        public double BushingLagS = 0;
        double _dyn;                           // stop command -> drive starts to brake (USB, Modbus, drive)
        /// <summary>Crank position above bottom dead centre (mm), without noise.</summary>
        public void SetCrankAngle(double rad) { lock (_lock) _theta = rad; }
        public double CrankAboveBdcMm { get { lock (_lock) return AmplitudeMm * (1 - Math.Cos(_theta)); } }
        public bool SafetyOk = true;
        /// <summary>Virtual seconds per real second when the internal timer runs.</summary>
        public double TimeScale = 1;
        /// <summary>False: nothing happens until Step() is called (used by the tests).</summary>
        public bool RealTime = true;

        const double SampleHz = 500;
        MachineState _state = MachineState.Idle;
        FaultCode _fault = FaultCode.None;
        bool _acqOn, _limitsSet;
        int _limLo, _limHi, _overLimit;
        double _cmdHz, _hz, _theta, _frac;
        long _stopAtUs = -1;
        uint _us;

        public SimulatedDynoLink(Settings set)
        {
            _set = set;
            _rx.PacketReceived += OnCommand;
        }

        public override bool IsOpen { get { return _open; } }
        protected override bool UseHeartbeat { get { return false; } }

        protected override void OpenTransport()
        {
            _open = true;
            if (RealTime) _timer = new Timer(_ => Tick(), null, 20, 20);
        }

        protected override void CloseTransport()
        {
            _open = false;
            if (_timer != null) { _timer.Dispose(); _timer = null; }
        }

        protected override void Write(byte[] packet)
        {
            lock (_lock) _rx.Feed(packet, 0, packet.Length);
            Flush();
        }

        // Taking and delivering packets is one atomic step, so packets always arrive in order
        // even when the timer thread and the caller's thread both flush.
        void Flush()
        {
            lock (_deliver)
            {
                byte[][] packets;
                lock (_lock) { packets = _out.ToArray(); _out.Clear(); }
                foreach (byte[] p in packets) OnBytes(p, p.Length);
            }
        }

        void Tick()
        {
            if (!Monitor.TryEnter(_tick)) return;      // previous tick still running: skip this one
            try { Step(0.02 * TimeScale); }
            finally { Monitor.Exit(_tick); }
        }

        void Ack(EventId id, CommandResult r) { _out.Add(Packet.EncodeEvent(EventId.Ack, (byte)id, (byte)r)); }

        void OnCommand(byte type, byte[] p)
        {
            if (type != Packet.TypeEvent || p.Length == 0) return;
            var id = (EventId)p[0];
            switch (id)
            {
                case EventId.Heartbeat: break;
                case EventId.AcqStart: _acqOn = true; Ack(id, CommandResult.Ok); break;
                case EventId.AcqStop: _acqOn = false; StopMotor(); Ack(id, CommandResult.Ok); break;
                case EventId.SetForceLimits:
                    _limLo = BitConverter.ToInt32(p, 1); _limHi = BitConverter.ToInt32(p, 5); _limitsSet = true;
                    Ack(id, CommandResult.Ok); break;
                case EventId.MotorRun:
                    CommandResult r = _state == MachineState.Fault ? CommandResult.InFault
                        : !SafetyOk ? CommandResult.SafetyOpen
                        : !_limitsSet ? CommandResult.NoLimits
                        : !_acqOn ? CommandResult.AcqOff : CommandResult.Ok;
                    if (r == CommandResult.Ok) { _cmdHz = BitConverter.ToUInt16(p, 1) / 100.0; _state = MachineState.Running; _stopAtUs = -1; }
                    Ack(id, r); break;
                case EventId.MotorSetFreq:
                    if (_state == MachineState.Running) { _cmdHz = BitConverter.ToUInt16(p, 1) / 100.0; Ack(id, CommandResult.Ok); }
                    else Ack(id, CommandResult.NotRunning);
                    break;
                case EventId.MotorStop: StopMotor(); Ack(id, CommandResult.Ok); break;
                case EventId.ClearFault:
                    if (_state == MachineState.Fault) { _state = MachineState.Idle; _fault = FaultCode.None; }
                    Ack(id, CommandResult.Ok); break;
            }
        }

        void StopMotor()
        {
            if (_cmdHz > 0 && StopLagS > 0) _stopAtUs = _us + (long)(StopLagS * 1e6);   // the drive brakes a little later
            else _cmdHz = 0;
            if (_state == MachineState.Running) _state = MachineState.Idle;
        }

        void Trip(FaultCode f)
        {
            if (_state != MachineState.Fault) { _state = MachineState.Fault; _fault = f; }
            _cmdHz = 0; _stopAtUs = -1;
        }

        double Gauss()
        {
            double u1 = 1 - _rnd.NextDouble(), u2 = _rnd.NextDouble();
            return Math.Sqrt(-2 * Math.Log(u1)) * Math.Cos(2 * Math.PI * u2);
        }

        double DamperForce(double v)
        {
            double a = Math.Abs(v);
            double c1 = v >= 0 ? CompC1 : RebC1, knee = v >= 0 ? CompKnee : RebKnee, c2 = v >= 0 ? CompC2 : RebC2;
            double f = a <= knee ? c1 * a : c1 * knee + c2 * (a - knee);
            return Math.Sign(v) * f + FrictionN * Math.Tanh(v / 2);
        }

        /// <summary>Advances the simulation by dt virtual seconds and delivers the resulting packets.</summary>
        public void Step(double dt)
        {
            if (!_open) return;
            lock (_lock)
            {
                var pts = new List<RawSample>();
                _frac += dt * SampleHz;
                while (_frac >= 1)
                {
                    _frac -= 1;
                    double h = 1 / SampleHz;
                    if (_state == MachineState.Running && !SafetyOk) Trip(FaultCode.SafetyOpen);
                    if (_stopAtUs >= 0 && _us >= _stopAtUs) { _cmdHz = 0; _stopAtUs = -1; }
                    double step = RampHzPerS * h;
                    _hz += Math.Max(-step, Math.Min(step, _cmdHz - _hz));
                    double w = 2 * Math.PI * Kinematics.CrankHz(_hz, _set) * (1 - Slip);
                    _theta += w * h;
                    double xr = AmplitudeMm * (1 - Math.Cos(_theta));
                    double v = AmplitudeMm * w * Math.Sin(_theta);
                    double dyn = DamperForce(v);
                    _dyn = BushingLagS > 0 ? _dyn + (dyn - _dyn) * Math.Min(1, h / BushingLagS) : dyn;
                    double force = GasForceN + GasRateNPerMm * xr + _dyn + ForceNoiseN * Gauss();
                    double pos = CenterMm - AmplitudeMm + xr + PosNoiseMm * Gauss();
                    _us += (uint)(1e6 * h);
                    var s = new RawSample
                    {
                        TimeUs = _us,
                        Force = (int)Math.Round(_set.ForceCounts(force)),
                        Position = (int)Math.Round(_set.PositionCounts(pos)),
                    };
                    if (_state == MachineState.Running && _limitsSet && (s.Force < _limLo || s.Force > _limHi))
                    {
                        if (++_overLimit >= 3) Trip(FaultCode.ForceLimit);
                    }
                    else _overLimit = 0;
                    if (_acqOn) pts.Add(s);
                }
                for (int i = 0; i < pts.Count; i += 20)
                    _out.Add(Packet.BuildData(pts, i, Math.Min(20, pts.Count - i)));
                _out.Add(Packet.BuildStatus(new DynoStatus
                {
                    State = _state, Fault = _fault, SafetyOk = SafetyOk, AcqOn = _acqOn, LimitsSet = _limitsSet,
                    VfdOk = true, AdcOk = true, VfdHz = _hz, VfdStatus = _state == MachineState.Running ? 1 : 3,
                }));
            }
            Flush();
        }
    }
}

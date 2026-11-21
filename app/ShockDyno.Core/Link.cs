using System;
using System.IO.Ports;
using System.Threading;

namespace ShockDyno.Core
{
    /// <summary>Connection to the dyno controller. Events are raised on a background thread.</summary>
    public interface IDynoLink : IDisposable
    {
        bool IsOpen { get; }
        event Action<RawSample[]> SamplesReceived;
        event Action<DynoStatus> StatusReceived;
        event Action<EventId, CommandResult> AckReceived;
        /// <summary>Answer to RequestTemperature.</summary>
        event Action<TempReading> TemperatureReceived;
        event Action<string> Disconnected;
        void Open();
        void Close();
        void StartAcquisition();
        void StopAcquisition();
        void SetForceLimits(int loCounts, int hiCounts);
        void MotorRun(double hz);
        void MotorSetFreq(double hz);
        void MotorStop();
        void ClearFault();
        /// <summary>Asks the ESP32 to read the IR thermometer once (answered by TemperatureReceived).</summary>
        void RequestTemperature();
        /// <summary>
        /// Call regularly from the user-interface thread. Heartbeats are only sent while this keeps being
        /// called, so a frozen application stops the motor just like a pulled cable.
        /// </summary>
        void KeepAlive();
    }

    /// <summary>Packet handling common to the serial link and the simulator.</summary>
    public abstract class DynoLinkBase : IDynoLink
    {
        readonly PacketDecoder _decoder = new PacketDecoder();
        Timer _heartbeat;
        readonly object _writeLock = new object();
        volatile bool _dead;                      // the transport failed: every call returns at once
        int _disconnectRaised;
        int _aliveTick;                           // Environment.TickCount of the last KeepAlive()
        /// <summary>Heartbeats stop when the application has not called KeepAlive for this long.</summary>
        public const int AppFrozenMs = 1500;

        public event Action<RawSample[]> SamplesReceived;
        public event Action<DynoStatus> StatusReceived;
        public event Action<EventId, CommandResult> AckReceived;
        public event Action<TempReading> TemperatureReceived;
        public event Action<string> Disconnected;

        public abstract bool IsOpen { get; }
        protected abstract void OpenTransport();
        protected abstract void CloseTransport();
        protected abstract void Write(byte[] packet);
        /// <summary>The simulator sets this to false: it has no heartbeat watchdog.</summary>
        protected virtual bool UseHeartbeat { get { return true; } }

        protected DynoLinkBase()
        {
            _decoder.PacketReceived += OnPacket;
        }

        public void Open()
        {
            _dead = false; _disconnectRaised = 0;
            OpenTransport();
            _aliveTick = Environment.TickCount;
            if (UseHeartbeat)
                _heartbeat = new Timer(_ =>
                {
                    if (unchecked(Environment.TickCount - _aliveTick) < AppFrozenMs) Send(Packet.EncodeEvent(EventId.Heartbeat));
                }, null, 0, 100);
        }

        /// <summary>
        /// Never blocks: closing a USB serial port whose device was unplugged can hang, so the
        /// transport is closed on a background thread.
        /// </summary>
        public void Close()
        {
            _dead = true;
            if (_heartbeat != null) { _heartbeat.Dispose(); _heartbeat = null; }
            ThreadPool.QueueUserWorkItem(_ => { try { CloseTransport(); } catch (Exception) { } });
        }

        public void Dispose() { Close(); }

        public void KeepAlive() { _aliveTick = Environment.TickCount; }

        protected void OnBytes(byte[] data, int count)
        {
            lock (_decoder) _decoder.Feed(data, 0, count);
        }

        /// <summary>Marks the link dead (no more writes) and raises Disconnected once.</summary>
        protected void RaiseDisconnected(string reason)
        {
            _dead = true;
            var hb = _heartbeat;
            if (hb != null) hb.Change(Timeout.Infinite, Timeout.Infinite);
            if (Interlocked.Exchange(ref _disconnectRaised, 1) != 0) return;
            var h = Disconnected;
            if (h != null) h(reason);
        }

        void OnPacket(byte type, byte[] payload)
        {
            if (type == Packet.TypeData)
            {
                var h = SamplesReceived;
                if (h != null) h(Packet.ParseData(payload));
            }
            else if (payload.Length > 0)
            {
                if (payload[0] == (byte)EventId.Status)
                {
                    var st = Packet.ParseStatus(payload);
                    var h = StatusReceived;
                    if (st != null && h != null) h(st);
                }
                else if (payload[0] == (byte)EventId.Temp)
                {
                    var t = Packet.ParseTemp(payload);
                    var h = TemperatureReceived;
                    if (t.HasValue && h != null) h(t.Value);
                }
                else if (payload[0] == (byte)EventId.Ack && payload.Length >= 3)
                {
                    var h = AckReceived;
                    if (h != null) h((EventId)payload[1], (CommandResult)payload[2]);
                }
            }
        }

        void Send(byte[] packet)
        {
            if (_dead) return;
            // A write stuck on a vanished port must not pile up callers (or freeze the window).
            if (!Monitor.TryEnter(_writeLock, 300)) { RaiseDisconnected("the serial port stopped accepting data"); return; }
            try { if (!_dead && IsOpen) Write(packet); }
            catch (Exception ex) { RaiseDisconnected(ex.Message); }
            finally { Monitor.Exit(_writeLock); }
        }

        static byte[] Hz(double hz)
        {
            return BitConverter.GetBytes((ushort)Math.Max(0, Math.Min(65535, Math.Round(hz * 100))));
        }

        public void StartAcquisition() { Send(Packet.EncodeEvent(EventId.AcqStart)); }
        public void StopAcquisition() { Send(Packet.EncodeEvent(EventId.AcqStop)); }
        public void MotorRun(double hz) { Send(Packet.EncodeEvent(EventId.MotorRun, Hz(hz))); }
        public void MotorSetFreq(double hz) { Send(Packet.EncodeEvent(EventId.MotorSetFreq, Hz(hz))); }
        public void MotorStop() { Send(Packet.EncodeEvent(EventId.MotorStop)); }
        public void ClearFault() { Send(Packet.EncodeEvent(EventId.ClearFault)); }
        public void RequestTemperature() { Send(Packet.EncodeEvent(EventId.ReadTemp)); }
        public void SetForceLimits(int loCounts, int hiCounts)
        {
            var a = new byte[8];
            Buffer.BlockCopy(BitConverter.GetBytes(loCounts), 0, a, 0, 4);
            Buffer.BlockCopy(BitConverter.GetBytes(hiCounts), 0, a, 4, 4);
            Send(Packet.EncodeEvent(EventId.SetForceLimits, a));
        }
    }

    /// <summary>USB serial link to the ESP32.</summary>
    public class SerialDynoLink : DynoLinkBase
    {
        public const int Baud = 460800;
        readonly SerialPort _port;
        readonly string _name;
        readonly byte[] _rx = new byte[4096];
        Timer _watch;
        int _lastRxTick;
        /// <summary>The ESP32 sends a status every 100 ms: this much silence means the link is gone.</summary>
        public const int SilenceMs = 3000;

        public SerialDynoLink(string portName)
        {
            _name = portName;
            _port = new SerialPort(portName, Baud, Parity.None, 8, StopBits.One);
            _port.ReadBufferSize = 1 << 16;
            _port.WriteTimeout = 200;
            // Do not toggle DTR/RTS: on ESP32 dev boards they reset the chip.
            _port.DtrEnable = false;
            _port.RtsEnable = false;
            _port.DataReceived += (s, e) =>
            {
                try
                {
                    while (_port.IsOpen && _port.BytesToRead > 0)
                    {
                        int n = _port.Read(_rx, 0, _rx.Length);
                        if (n > 0) { _lastRxTick = Environment.TickCount; OnBytes(_rx, n); }
                    }
                }
                catch (Exception ex) { RaiseDisconnected(ex.Message); }
            };
        }

        public override bool IsOpen { get { return _port.IsOpen; } }
        protected override void OpenTransport()
        {
            _port.Open();
            // Known .NET issue: when the USB device disappears, the port's stream can throw from its
            // finalizer and crash the application. Keep the finalizer off while the port is in use.
            try { GC.SuppressFinalize(_port.BaseStream); } catch (Exception) { }
            _lastRxTick = Environment.TickCount;
            _watch = new Timer(_ => Watch(), null, 500, 500);
        }

        void Watch()
        {
            if (unchecked(Environment.TickCount - _lastRxTick) < 1000) return;
            bool present = true;
            try { present = Array.IndexOf(SerialPort.GetPortNames(), _name) >= 0; } catch (Exception) { }
            if (!present) RaiseDisconnected("USB cable unplugged (" + _name + " disappeared)");
            else if (unchecked(Environment.TickCount - _lastRxTick) > SilenceMs) RaiseDisconnected("no data from the ESP32 for " + SilenceMs / 1000 + " s");
        }

        protected override void CloseTransport()
        {
            var w = _watch; _watch = null;
            if (w != null) w.Dispose();
            try { GC.ReRegisterForFinalize(_port.BaseStream); } catch (Exception) { }
            try { if (_port.IsOpen) _port.Close(); } catch (Exception) { }
        }
        protected override void Write(byte[] packet) { _port.Write(packet, 0, packet.Length); }
    }
}

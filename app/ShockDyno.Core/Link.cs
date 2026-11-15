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
    }

    /// <summary>Packet handling common to the serial link and the simulator.</summary>
    public abstract class DynoLinkBase : IDynoLink
    {
        readonly PacketDecoder _decoder = new PacketDecoder();
        Timer _heartbeat;

        public event Action<RawSample[]> SamplesReceived;
        public event Action<DynoStatus> StatusReceived;
        public event Action<EventId, CommandResult> AckReceived;
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
            OpenTransport();
            if (UseHeartbeat)
                _heartbeat = new Timer(_ => Send(Packet.EncodeEvent(EventId.Heartbeat)), null, 0, 100);
        }

        public void Close()
        {
            if (_heartbeat != null) { _heartbeat.Dispose(); _heartbeat = null; }
            CloseTransport();
        }

        public void Dispose() { Close(); }

        protected void OnBytes(byte[] data, int count)
        {
            lock (_decoder) _decoder.Feed(data, 0, count);
        }

        protected void RaiseDisconnected(string reason)
        {
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
                else if (payload[0] == (byte)EventId.Ack && payload.Length >= 3)
                {
                    var h = AckReceived;
                    if (h != null) h((EventId)payload[1], (CommandResult)payload[2]);
                }
            }
        }

        void Send(byte[] packet)
        {
            try { if (IsOpen) lock (this) Write(packet); }
            catch (Exception ex) { RaiseDisconnected(ex.Message); }
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
        readonly byte[] _rx = new byte[4096];

        public SerialDynoLink(string portName)
        {
            _port = new SerialPort(portName, Baud, Parity.None, 8, StopBits.One);
            _port.ReadBufferSize = 1 << 16;
            _port.WriteTimeout = 500;
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
                        if (n > 0) OnBytes(_rx, n);
                    }
                }
                catch (Exception ex) { RaiseDisconnected(ex.Message); }
            };
        }

        public override bool IsOpen { get { return _port.IsOpen; } }
        protected override void OpenTransport() { _port.Open(); }
        protected override void CloseTransport() { try { if (_port.IsOpen) _port.Close(); } catch { } }
        protected override void Write(byte[] packet) { _port.Write(packet, 0, packet.Length); }
    }
}

// Packet protocol shared with the ESP32 firmware (see docs/protocol.md).
//   FF 00 TYPE LEN PAYLOAD[LEN] CRC8      CRC8 (poly 0x07) over TYPE, LEN, PAYLOAD
using System;
using System.Collections.Generic;

namespace ShockDyno.Core
{
    public enum EventId : byte
    {
        Heartbeat = 0x01, AcqStart = 0x02, AcqStop = 0x03, SetForceLimits = 0x04,
        MotorRun = 0x05, MotorSetFreq = 0x06, MotorStop = 0x07, ClearFault = 0x08,
        Status = 0x81, Ack = 0x82,
    }

    public enum MachineState : byte { Idle = 0, Running = 1, Fault = 2 }

    public enum FaultCode : byte
    {
        None = 0, HeartbeatLost = 1, ForceLimit = 2, SafetyOpen = 3, VfdComm = 4, VfdFault = 5, AdcTimeout = 6,
    }

    public enum CommandResult : byte
    {
        Ok = 0, BadArgs = 1, InFault = 2, SafetyOpen = 3, NoLimits = 4, AcqOff = 5, AdcNotReady = 6, NotRunning = 7, Unknown = 8,
    }

    /// <summary>One acquisition point as sent by the ESP32 (10 bytes on the wire).</summary>
    public struct RawSample
    {
        public uint TimeUs;
        public int Force;
        public int Position;
    }

    public class DynoStatus
    {
        public MachineState State;
        public FaultCode Fault;
        public bool SafetyOk, AcqOn, LimitsSet, VfdOk, AdcOk;
        public double VfdHz;
        public int VfdFault, Dropped, VfdStatus;

        public static string Describe(FaultCode f)
        {
            switch (f)
            {
                case FaultCode.None: return "No fault";
                case FaultCode.HeartbeatLost: return "Connection to the PC lost";
                case FaultCode.ForceLimit: return "Force limit exceeded";
                case FaultCode.SafetyOpen: return "Safety circuit open (door or emergency stop)";
                case FaultCode.VfdComm: return "No communication with the drive";
                case FaultCode.VfdFault: return "Drive fault";
                case FaultCode.AdcTimeout: return "Sensor readings stopped";
                default: return "Fault " + (int)f;
            }
        }

        public static string Describe(CommandResult r)
        {
            switch (r)
            {
                case CommandResult.Ok: return "OK";
                case CommandResult.BadArgs: return "Invalid command value";
                case CommandResult.InFault: return "Machine is in fault; clear the fault first";
                case CommandResult.SafetyOpen: return "Safety circuit is open; close the door and press the reset button";
                case CommandResult.NoLimits: return "Force limits not set";
                case CommandResult.AcqOff: return "Acquisition is off";
                case CommandResult.AdcNotReady: return "Sensors (ADC) not ready";
                case CommandResult.NotRunning: return "Motor is not running";
                default: return "Command rejected (" + (int)r + ")";
            }
        }
    }

    /// <summary>
    /// Turns the ESP32's 32-bit microsecond timestamps into seconds. A sample older than the previous one
    /// is rejected (it would otherwise wrap to a huge forward jump), and a gap over 1 s (ESP32 restarted,
    /// link paused) counts as one sample period instead of a jump.
    /// </summary>
    public class SampleClock
    {
        uint _last; bool _have; int _older;
        public double T { get; private set; }

        /// <summary>Advances to this sample. False if the sample is older than the previous one: skip it.</summary>
        public bool Advance(uint timeUs)
        {
            if (!_have) { _have = true; _last = timeUs; return true; }
            uint d = unchecked(timeUs - _last);
            if (d > 0x80000000u)                                 // older than the previous sample
            {
                if (++_older < 50) return false;                 // a late packet: skip it
                d = 2000;                                        // 0.1 s of "older" samples: the ESP32 clock restarted
            }
            _older = 0;
            _last = timeUs;
            T += d > 1000000u ? 0.002 : d / 1e6;
            return true;
        }

        public void Reset() { _have = false; T = 0; }
    }

    public static class Packet
    {
        public const byte Hdr0 = 0xFF, Hdr1 = 0x00, TypeEvent = 0x55, TypeData = 0xAA;
        public const int PointSize = 10;

        public static byte Crc8(byte[] data, int offset, int count, byte crc = 0)
        {
            for (int i = 0; i < count; i++)
            {
                crc ^= data[offset + i];
                for (int b = 0; b < 8; b++)
                    crc = (crc & 0x80) != 0 ? (byte)((crc << 1) ^ 0x07) : (byte)(crc << 1);
            }
            return crc;
        }

        public static byte[] Encode(byte type, byte[] payload)
        {
            int len = payload == null ? 0 : payload.Length;
            if (len > 255) throw new ArgumentException("Payload too long");
            var p = new byte[len + 5];
            p[0] = Hdr0; p[1] = Hdr1; p[2] = type; p[3] = (byte)len;
            if (len > 0) Buffer.BlockCopy(payload, 0, p, 4, len);
            p[4 + len] = Crc8(p, 2, len + 2);
            return p;
        }

        public static byte[] EncodeEvent(EventId id, params byte[] args)
        {
            var payload = new byte[1 + args.Length];
            payload[0] = (byte)id;
            Buffer.BlockCopy(args, 0, payload, 1, args.Length);
            return Encode(TypeEvent, payload);
        }

        public static void PutI24(byte[] b, int o, int v) { b[o] = (byte)v; b[o + 1] = (byte)(v >> 8); b[o + 2] = (byte)(v >> 16); }
        public static int GetI24(byte[] b, int o)
        {
            int v = b[o] | (b[o + 1] << 8) | (b[o + 2] << 16);
            if ((v & 0x800000) != 0) v |= unchecked((int)0xFF000000);
            return v;
        }

        public static RawSample[] ParseData(byte[] payload)
        {
            int n = payload.Length / PointSize;
            var s = new RawSample[n];
            for (int i = 0; i < n; i++)
            {
                int o = i * PointSize;
                s[i].TimeUs = BitConverter.ToUInt32(payload, o);
                s[i].Force = GetI24(payload, o + 4);
                s[i].Position = GetI24(payload, o + 7);
            }
            return s;
        }

        public static byte[] BuildData(IList<RawSample> samples, int start, int count)
        {
            var payload = new byte[count * PointSize];
            for (int i = 0; i < count; i++)
            {
                var s = samples[start + i];
                int o = i * PointSize;
                Buffer.BlockCopy(BitConverter.GetBytes(s.TimeUs), 0, payload, o, 4);
                PutI24(payload, o + 4, s.Force);
                PutI24(payload, o + 7, s.Position);
            }
            return Encode(TypeData, payload);
        }

        public static DynoStatus ParseStatus(byte[] p)   // p[0] is the event id
        {
            if (p.Length < 11) return null;
            return new DynoStatus
            {
                State = (MachineState)p[1],
                Fault = (FaultCode)p[2],
                SafetyOk = (p[3] & 0x01) != 0,
                AcqOn = (p[3] & 0x02) != 0,
                LimitsSet = (p[3] & 0x04) != 0,
                VfdOk = (p[3] & 0x08) != 0,
                AdcOk = (p[3] & 0x10) != 0,
                VfdHz = BitConverter.ToUInt16(p, 4) / 100.0,
                VfdFault = BitConverter.ToUInt16(p, 6),
                Dropped = BitConverter.ToUInt16(p, 8),
                VfdStatus = p[10],
            };
        }

        public static byte[] BuildStatus(DynoStatus s)
        {
            var p = new byte[11];
            p[0] = (byte)EventId.Status;
            p[1] = (byte)s.State; p[2] = (byte)s.Fault;
            p[3] = (byte)((s.SafetyOk ? 1 : 0) | (s.AcqOn ? 2 : 0) | (s.LimitsSet ? 4 : 0) | (s.VfdOk ? 8 : 0) | (s.AdcOk ? 16 : 0));
            Buffer.BlockCopy(BitConverter.GetBytes((ushort)Math.Round(s.VfdHz * 100)), 0, p, 4, 2);
            Buffer.BlockCopy(BitConverter.GetBytes((ushort)s.VfdFault), 0, p, 6, 2);
            Buffer.BlockCopy(BitConverter.GetBytes((ushort)s.Dropped), 0, p, 8, 2);
            p[10] = (byte)s.VfdStatus;
            return Encode(TypeEvent, p);
        }
    }

    /// <summary>Byte-wise packet decoder (finite state machine). Bytes may arrive in any chunking.</summary>
    public class PacketDecoder
    {
        enum S { Hdr0, Hdr1, Type, Len, Payload, Crc }
        S _st = S.Hdr0;
        byte _type, _len;
        int _pos;
        readonly byte[] _buf = new byte[255];

        public int CrcErrors { get; private set; }
        /// <summary>Raised with (type, payload) for every valid packet.</summary>
        public event Action<byte, byte[]> PacketReceived;

        public void Feed(byte[] data, int offset, int count)
        {
            for (int i = 0; i < count; i++) Feed(data[offset + i]);
        }

        public void Feed(byte b)
        {
            switch (_st)
            {
                case S.Hdr0: if (b == Packet.Hdr0) _st = S.Hdr1; break;
                case S.Hdr1: _st = b == Packet.Hdr1 ? S.Type : (b == Packet.Hdr0 ? S.Hdr1 : S.Hdr0); break;
                case S.Type:
                    if (b == Packet.TypeEvent || b == Packet.TypeData) { _type = b; _st = S.Len; }
                    else _st = b == Packet.Hdr0 ? S.Hdr1 : S.Hdr0;
                    break;
                case S.Len: _len = b; _pos = 0; _st = _len > 0 ? S.Payload : S.Crc; break;
                case S.Payload: _buf[_pos++] = b; if (_pos >= _len) _st = S.Crc; break;
                case S.Crc:
                    _st = S.Hdr0;
                    byte c = Packet.Crc8(_buf, 0, _len, Packet.Crc8(new[] { _type, _len }, 0, 2));
                    if (c != b) { CrcErrors++; break; }
                    var payload = new byte[_len];
                    Buffer.BlockCopy(_buf, 0, payload, 0, _len);
                    var h = PacketReceived;
                    if (h != null) h(_type, payload);
                    break;
            }
        }
    }
}

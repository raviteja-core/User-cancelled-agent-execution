using System;
using System.Buffers.Binary;

namespace PaperCutter.Hmi.Protocol
{
    public static class ProtocolConstants
    {
        public const byte SOF1 = 0xAA;
        public const byte SOF2 = 0x55;
        public const byte MinPayloadLen = 2;
        public const byte MaxPayloadLen = 250;
        public const int MaxPayloadDataLen = 248;
        public const int HeaderSize = 3;
        public const int CrcSize = 2;
        public const int InterByteTimeoutMs = 50;
    }

    public enum Command : byte
    {
        Hello = 0x01,
        Heartbeat = 0x02,
        LoadJob = 0x10,
        Start = 0x11,
        PauseResume = 0x12,
        Abort = 0x13,
        Estop = 0x20,
        ResetFault = 0x21,
        Telemetry = 0x30,
        Ack = 0x7E,
        Nack = 0x7F,
        SimInject = 0x90
    }

    public enum FaultCode : ushort
    {
        None = 0x0000,
        PaperJam = 0x0001,
        CommTimeout = 0x0002,
        PrematureLimit = 0x0003,
        Estop = 0x0004,
        CrcStorm = 0x0005,
        MotionOverrun = 0x0006,
        InvalidJob = 0x0007
    }

    public enum MachineState : byte
    {
        Idle = 0,
        Feeding = 1,
        Clamping = 2,
        Cutting = 3,
        ErrorJam = 4,
        ErrorFault = 5
    }

    [Flags]
    public enum LimitSwitchMask : byte
    {
        None = 0x00,
        BladeTdc = 0x01,
        BladeBdc = 0x02,
        ClampSwitch = 0x04,
        SafetyGuard = 0x08
    }

    public sealed class Packet
    {
        public byte Seq { get; set; }
        public byte Cmd { get; set; }
        public byte[] Payload { get; set; } = Array.Empty<byte>();

        public Packet() { }

        public Packet(byte seq, byte cmd, byte[]? payload = null)
        {
            Seq = seq;
            Cmd = cmd;
            Payload = payload ?? Array.Empty<byte>();
        }
    }

    public sealed class TelemetryData
    {
        public MachineState State { get; set; }
        public FaultCode FaultCode { get; set; }
        public int BackGaugePos { get; set; }
        public int BladePos { get; set; }
        public bool FeedSensor { get; set; }
        public LimitSwitchMask LimitSwitches { get; set; }
        public byte CutIndex { get; set; }
        public ushort CrcErrors { get; set; }
        public ushort DroppedPackets { get; set; }
        public ushort TimeoutErrors { get; set; }
        public bool ClampEngaged { get; set; }
        public ushort Reserved { get; set; }

        public static TelemetryData Parse(ReadOnlySpan<byte> payload)
        {
            if (payload.Length < 23)
            {
                throw new ArgumentException($"Payload length {payload.Length} too small for Telemetry");
            }

            return new TelemetryData
            {
                State = (MachineState)payload[0],
                FaultCode = (FaultCode)BinaryPrimitives.ReadUInt16BigEndian(payload.Slice(1, 2)),
                BackGaugePos = BinaryPrimitives.ReadInt32BigEndian(payload.Slice(3, 4)),
                BladePos = BinaryPrimitives.ReadInt32BigEndian(payload.Slice(7, 4)),
                FeedSensor = payload[11] != 0,
                LimitSwitches = (LimitSwitchMask)payload[12],
                CutIndex = payload[13],
                CrcErrors = BinaryPrimitives.ReadUInt16BigEndian(payload.Slice(14, 2)),
                DroppedPackets = BinaryPrimitives.ReadUInt16BigEndian(payload.Slice(16, 2)),
                TimeoutErrors = BinaryPrimitives.ReadUInt16BigEndian(payload.Slice(18, 2)),
                ClampEngaged = payload[20] != 0,
                Reserved = BinaryPrimitives.ReadUInt16BigEndian(payload.Slice(21, 2))
            };
        }
    }
}

using System;
using System.Buffers.Binary;

namespace PaperCutter.Hmi.Protocol
{
    public static class PacketEncoder
    {
        public static byte[] Encode(Packet packet)
        {
            if (packet == null) throw new ArgumentNullException(nameof(packet));
            byte[] payload = packet.Payload ?? Array.Empty<byte>();
            int length = payload.Length + 2;
            if (length < 2 || length > 250)
            {
                throw new ArgumentException($"Invalid packet payload length: {payload.Length}");
            }

            int totalSize = 5 + payload.Length + 2; // SOF1 (1) + SOF2 (1) + LEN (1) + SEQ (1) + CMD (1) + payload + CRC (2)
            byte[] buffer = new byte[totalSize];
            buffer[0] = 0xAA;
            buffer[1] = 0x55;
            buffer[2] = (byte)length;
            buffer[3] = packet.Seq;
            buffer[4] = packet.Cmd;

            if (payload.Length > 0)
            {
                Array.Copy(payload, 0, buffer, 5, payload.Length);
            }

            // CRC over LEN, SEQ, CMD, PAYLOAD (1 + length bytes starting at index 2)
            ushort crc = Crc16.Compute(buffer.AsSpan(2, 1 + length));
            BinaryPrimitives.WriteUInt16BigEndian(buffer.AsSpan(5 + payload.Length, 2), crc);

            return buffer;
        }

        public static byte[] EncodeHello(byte seq, byte protocolVersion = 1, ushort firmwareVersion = 100)
        {
            byte[] payload = new byte[3];
            payload[0] = protocolVersion;
            BinaryPrimitives.WriteUInt16BigEndian(payload.AsSpan(1, 2), firmwareVersion);
            return Encode(new Packet(seq, (byte)Command.Hello, payload));
        }

        public static byte[] EncodeHeartbeat(byte seq)
        {
            return Encode(new Packet(seq, (byte)Command.Heartbeat));
        }

        public static byte[] EncodeEstop(byte seq)
        {
            return Encode(new Packet(seq, (byte)Command.Estop));
        }

        public static byte[] EncodeResetFault(byte seq)
        {
            return Encode(new Packet(seq, (byte)Command.ResetFault));
        }

        public static byte[] EncodeStart(byte seq, ushort jobId)
        {
            byte[] payload = new byte[2];
            BinaryPrimitives.WriteUInt16BigEndian(payload, jobId);
            return Encode(new Packet(seq, (byte)Command.Start, payload));
        }

        public static byte[] EncodePauseResume(byte seq)
        {
            return Encode(new Packet(seq, (byte)Command.PauseResume));
        }

        public static byte[] EncodeAbort(byte seq)
        {
            return Encode(new Packet(seq, (byte)Command.Abort));
        }

        public static byte[] EncodeLoadJob(byte seq, ushort jobId, uint[] cutPositionsUnits)
        {
            if (cutPositionsUnits == null) throw new ArgumentNullException(nameof(cutPositionsUnits));
            if (cutPositionsUnits.Length > 60) throw new ArgumentException("Exceeds max cuts");

            byte[] payload = new byte[3 + cutPositionsUnits.Length * 4];
            BinaryPrimitives.WriteUInt16BigEndian(payload.AsSpan(0, 2), jobId);
            payload[2] = (byte)cutPositionsUnits.Length;
            for (int i = 0; i < cutPositionsUnits.Length; i++)
            {
                BinaryPrimitives.WriteUInt32BigEndian(payload.AsSpan(3 + i * 4, 4), cutPositionsUnits[i]);
            }

            return Encode(new Packet(seq, (byte)Command.LoadJob, payload));
        }
    }
}

using System;

namespace PaperCutter.Hmi.Protocol
{
    public static class Crc16
    {
        private static readonly ushort[] Table = new ushort[256];

        static Crc16()
        {
            for (ushort i = 0; i < 256; i++)
            {
                ushort curr = (ushort)(i << 8);
                for (int b = 0; b < 8; b++)
                {
                    if ((curr & 0x8000) != 0)
                    {
                        curr = (ushort)((curr << 1) ^ 0x1021);
                    }
                    else
                    {
                        curr = (ushort)(curr << 1);
                    }
                }
                Table[i] = curr;
            }
        }

        public static ushort Compute(ReadOnlySpan<byte> data, ushort initial = 0xFFFF)
        {
            ushort crc = initial;
            foreach (byte b in data)
            {
                crc = (ushort)((crc << 8) ^ Table[(crc >> 8) ^ b]);
            }
            return crc;
        }

        public static ushort Compute(byte[] data, int offset, int length, ushort initial = 0xFFFF)
        {
            return Compute(data.AsSpan(offset, length), initial);
        }
    }
}

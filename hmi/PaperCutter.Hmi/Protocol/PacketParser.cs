using System;
using System.Buffers.Binary;
using System.Collections.Generic;

namespace PaperCutter.Hmi.Protocol
{
    public enum ParserState : byte
    {
        WaitSof1 = 0,
        WaitSof2 = 1,
        ReadLen = 2,
        ReadBody = 3,
        ReadCrc1 = 4,
        ReadCrc2 = 5
    }

    public sealed class PacketParser
    {
        private ParserState _state = ParserState.WaitSof1;
        private byte _expectedLen = 0;
        private byte _bodyBytesRead = 0;
        private byte _crcHigh = 0;
        private byte _crcLow = 0;
        private int _timeSinceLastByteMs = 0;

        private readonly byte[] _bodyBuf = new byte[256];

        public ParserState State => _state;
        public ushort CrcErrors { get; private set; }
        public ushort DroppedPackets { get; private set; }
        public ushort TimeoutErrors { get; private set; }

        public void Reset()
        {
            _state = ParserState.WaitSof1;
            _expectedLen = 0;
            _bodyBytesRead = 0;
            _crcHigh = 0;
            _crcLow = 0;
            _timeSinceLastByteMs = 0;
        }

        public bool Tick(int deltaMs)
        {
            if (_state != ParserState.WaitSof1)
            {
                _timeSinceLastByteMs += deltaMs;
                if (_timeSinceLastByteMs >= 50)
                {
                    TimeoutErrors++;
                    Reset();
                    return true;
                }
            }
            return false;
        }

        public Packet? ProcessByte(byte b)
        {
            _timeSinceLastByteMs = 0;

            switch (_state)
            {
                case ParserState.WaitSof1:
                    if (b == 0xAA)
                    {
                        _state = ParserState.WaitSof2;
                    }
                    return null;

                case ParserState.WaitSof2:
                    if (b == 0x55)
                    {
                        _state = ParserState.ReadLen;
                    }
                    else if (b == 0xAA)
                    {
                        _state = ParserState.WaitSof2;
                    }
                    else
                    {
                        _state = ParserState.WaitSof1;
                    }
                    return null;

                case ParserState.ReadLen:
                    if (b >= 2 && b <= 250)
                    {
                        _expectedLen = b;
                        _bodyBytesRead = 0;
                        _bodyBuf[0] = b; // Store LEN for CRC check
                        _state = ParserState.ReadBody;
                    }
                    else
                    {
                        DroppedPackets++;
                        _state = ParserState.WaitSof1;
                    }
                    return null;

                case ParserState.ReadBody:
                    if (_bodyBytesRead < _expectedLen)
                    {
                        _bodyBuf[1 + _bodyBytesRead] = b;
                        _bodyBytesRead++;
                    }

                    if (_bodyBytesRead >= _expectedLen)
                    {
                        _state = ParserState.ReadCrc1;
                    }
                    return null;

                case ParserState.ReadCrc1:
                    _crcHigh = b;
                    _state = ParserState.ReadCrc2;
                    return null;

                case ParserState.ReadCrc2:
                    _crcLow = b;
                    ushort receivedCrc = (ushort)((_crcHigh << 8) | _crcLow);
                    ushort calculatedCrc = Crc16.Compute(_bodyBuf.AsSpan(0, 1 + _expectedLen));

                    if (receivedCrc == calculatedCrc)
                    {
                        byte seq = _bodyBuf[1];
                        byte cmd = _bodyBuf[2];
                        int payloadLen = _expectedLen - 2;
                        byte[] payload = new byte[payloadLen];
                        if (payloadLen > 0)
                        {
                            Array.Copy(_bodyBuf, 3, payload, 0, payloadLen);
                        }

                        Reset();
                        return new Packet(seq, cmd, payload);
                    }
                    else
                    {
                        CrcErrors++;
                        Reset();
                        return null;
                    }

                default:
                    Reset();
                    return null;
            }
        }

        public List<Packet> Parse(ReadOnlySpan<byte> data)
        {
            var result = new List<Packet>();
            foreach (byte b in data)
            {
                var pkt = ProcessByte(b);
                if (pkt != null)
                {
                    result.Add(pkt);
                }
            }
            return result;
        }
    }
}

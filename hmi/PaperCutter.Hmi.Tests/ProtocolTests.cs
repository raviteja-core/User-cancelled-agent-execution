using System;
using System.IO;
using System.Text.Json;
using PaperCutter.Hmi.Protocol;
using Xunit;

namespace PaperCutter.Hmi.Tests
{
    public class ProtocolTests
    {
        private static string FindGoldenVectorsPath()
        {
            string? dir = AppContext.BaseDirectory;
            while (dir != null)
            {
                string candidate = Path.Combine(dir, "tests", "test_vectors", "golden_vectors.json");
                if (File.Exists(candidate))
                {
                    return candidate;
                }
                dir = Directory.GetParent(dir)?.FullName;
            }
            throw new FileNotFoundException("golden_vectors.json not found");
        }

        [Fact]
        public void TestStandardCrcVector()
        {
            byte[] input = System.Text.Encoding.ASCII.GetBytes("123456789");
            ushort crc = Crc16.Compute(input);
            Assert.Equal(0x29B1, crc);
        }

        [Fact]
        public void TestGoldenVectorsEncodeAndDecode()
        {
            string jsonPath = FindGoldenVectorsPath();
            string jsonText = File.ReadAllText(jsonPath);
            using var doc = JsonDocument.Parse(jsonText);

            var packets = doc.RootElement.GetProperty("packets");
            foreach (var elem in packets.EnumerateArray())
            {
                string name = elem.GetProperty("name").GetString()!;
                byte seq = elem.GetProperty("seq").GetByte();
                byte cmd = elem.GetProperty("cmd").GetByte();
                string payloadHex = elem.GetProperty("payload_hex").GetString()!;
                string wireHex = elem.GetProperty("wire_hex").GetString()!;

                byte[] payload = Convert.FromHexString(payloadHex);
                byte[] expectedWire = Convert.FromHexString(wireHex);

                // Test Encode
                var pkt = new Packet(seq, cmd, payload);
                byte[] encoded = PacketEncoder.Encode(pkt);
                Assert.Equal(wireHex, Convert.ToHexString(encoded).ToLowerInvariant());

                // Test Decode streaming
                var parser = new PacketParser();
                var parsedList = parser.Parse(expectedWire);
                Assert.Single(parsedList);
                var decoded = parsedList[0];
                Assert.Equal(seq, decoded.Seq);
                Assert.Equal(cmd, decoded.Cmd);
                Assert.Equal(payloadHex, Convert.ToHexString(decoded.Payload).ToLowerInvariant());
                Assert.Equal(0, parser.CrcErrors);
                Assert.Equal(0, parser.DroppedPackets);
            }
        }

        [Fact]
        public void TestParserCorruptedCrc()
        {
            var parser = new PacketParser();
            byte[] hello = PacketEncoder.EncodeHello(1, 1, 100);
            hello[^1] ^= 0xFF; // Corrupt CRC LSB

            var result = parser.Parse(hello);
            Assert.Empty(result);
            Assert.Equal(1, parser.CrcErrors);
        }

        [Fact]
        public void TestParserInterByteTimeout()
        {
            var parser = new PacketParser();
            parser.ProcessByte(0xAA);
            parser.ProcessByte(0x55);
            parser.ProcessByte(0x05);

            Assert.NotEqual(ParserState.WaitSof1, parser.State);

            bool timedOut = parser.Tick(60);
            Assert.True(timedOut);
            Assert.Equal(ParserState.WaitSof1, parser.State);
            Assert.Equal(1, parser.TimeoutErrors);
        }
    }
}

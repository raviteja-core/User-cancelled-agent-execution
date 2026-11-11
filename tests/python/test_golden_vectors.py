import json
from pathlib import Path
import pytest
from papercutter_protocol import crc16_ccitt_false, PacketParser, Packet


def load_golden_vectors():
    vec_path = Path(__file__).resolve().parent.parent / "test_vectors" / "golden_vectors.json"
    with open(vec_path, "r") as f:
        return json.load(f)


def test_standard_crc_vector():
    vectors = load_golden_vectors()
    std = vectors["standard_check"]
    data = bytes.fromhex(std["input_hex"])
    computed = crc16_ccitt_false(data)
    assert computed == std["expected_crc_int"]
    assert f"0x{computed:04X}" == std["expected_crc"]


def test_golden_packets_encode():
    vectors = load_golden_vectors()
    for item in vectors["packets"]:
        payload = bytes.fromhex(item["payload_hex"])
        pkt = Packet(seq=item["seq"], cmd=item["cmd"], payload=payload)
        wire = pkt.encode()
        assert wire.hex() == item["wire_hex"], f"Encode mismatch for {item['name']}"


def test_golden_packets_decode_streaming():
    vectors = load_golden_vectors()
    for item in vectors["packets"]:
        wire = bytes.fromhex(item["wire_hex"])
        parser = PacketParser()
        packets = parser.parse_bytes(wire)
        assert len(packets) == 1, f"Failed to parse {item['name']}"
        pkt = packets[0]
        assert pkt.seq == item["seq"]
        assert pkt.cmd == item["cmd"]
        assert pkt.payload.hex() == item["payload_hex"]
        assert parser.crc_errors == 0
        assert parser.dropped_packets == 0


def test_parser_corrupted_crc():
    parser = PacketParser()
    vectors = load_golden_vectors()
    hello = bytes.fromhex(vectors["packets"][0]["wire_hex"])
    # Corrupt last byte (CRC LSB)
    corrupted = bytearray(hello)
    corrupted[-1] ^= 0xFF
    packets = parser.parse_bytes(bytes(corrupted))
    assert len(packets) == 0
    assert parser.crc_errors == 1


def test_parser_inter_byte_timeout():
    parser = PacketParser()
    # Send SOF and LEN
    parser.process_byte(0xAA)
    parser.process_byte(0x55)
    parser.process_byte(0x05)
    assert parser.state != 0

    # Advance time by 60 ms (> 50 ms timeout)
    timed_out = parser.tick(60)
    assert timed_out is True
    assert parser.state == 0
    assert parser.timeout_errors == 1

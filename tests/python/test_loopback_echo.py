"""
Integration test: Verifies loopback echo between running controller_app and Python test client.
Tests protocol handshake (HELLO) and command acknowledgement (ACK).
Supports both TCP (loopback) and Serial (COM11 when com0com configured).
"""

import socket
import time
import subprocess
import pytest
from pathlib import Path
from papercutter_protocol import Packet, PacketParser, Command


def run_controller_tcp(port: int = 9123):
    build_dir = Path(__file__).resolve().parent.parent.parent / "build" / "controller_app" / "Debug"
    exe_path = build_dir / "controller_app.exe"
    proc = subprocess.Popen(
        [str(exe_path), "--tcp", str(port)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(0.3)  # Give time for server socket to open
    return proc


def test_controller_tcp_echo_handshake():
    port = 9123
    proc = run_controller_tcp(port)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect(("127.0.0.1", port))

        parser = PacketParser()

        # Send HELLO
        hello = Packet(seq=1, cmd=Command.HELLO, payload=bytes([1, 0, 100]))
        sock.sendall(hello.encode())

        # Read responses (expect HELLO and ACK)
        received_packets = []
        start_time = time.time()
        while len(received_packets) < 2 and time.time() - start_time < 2.0:
            chunk = sock.recv(256)
            if not chunk:
                break
            pkts = parser.parse_bytes(chunk)
            received_packets.extend(pkts)

        assert len(received_packets) >= 1
        commands = [p.cmd for p in received_packets]
        assert Command.HELLO in commands or Command.ACK in commands

        # Send HEARTBEAT
        hb = Packet(seq=2, cmd=Command.HEARTBEAT)
        sock.sendall(hb.encode())

        start_time = time.time()
        ack_received = False
        while time.time() - start_time < 1.0:
            chunk = sock.recv(256)
            if not chunk:
                break
            for p in parser.parse_bytes(chunk):
                if p.cmd == Command.ACK and len(p.payload) >= 1 and p.payload[0] == 2:
                    ack_received = True
                    break
            if ack_received:
                break

        assert ack_received, "Failed to receive ACK for HEARTBEAT"
        sock.close()
    finally:
        proc.terminate()
        proc.wait(timeout=2.0)

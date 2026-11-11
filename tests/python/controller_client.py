"""
ControllerClient: Test harness client for communicating with the Paper Cutter Controller
over either TCP (loopback) or Serial COM port (com0com).
"""

import socket
import subprocess
import time
import threading
from pathlib import Path
from typing import List, Optional, Callable
import struct

from papercutter_protocol import (
    Packet,
    PacketParser,
    TelemetryData,
    Command,
    FaultCode,
    State,
)


class ControllerClient:
    def __init__(self, use_tcp: bool = True, port: int = 9150, serial_port: str = "COM11"):
        self.use_tcp = use_tcp
        self.port = port
        self.serial_port = serial_port
        self.sock: Optional[socket.socket] = None
        self.serial_conn = None
        self.proc: Optional[subprocess.Popen] = None
        self.parser = PacketParser()
        self.seq = 1

        self.received_packets: List[Packet] = []
        self.telemetry_history: List[TelemetryData] = []
        self.latest_telemetry: Optional[TelemetryData] = None

        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start_controller(self):
        build_dir = Path(__file__).resolve().parent.parent.parent / "build" / "controller_app" / "Debug"
        exe_path = build_dir / "controller_app.exe"
        if not exe_path.exists():
            raise FileNotFoundError(f"controller_app.exe not found at {exe_path}")

        if self.use_tcp:
            cmd = [str(exe_path), "--tcp", str(self.port)]
        else:
            # Controller opens COM10 by default
            cmd = [str(exe_path), "--serial", "COM10", "--baud", "115200"]

        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        time.sleep(0.4)  # Wait for controller socket/port to open

    def connect(self):
        if self.use_tcp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect(("127.0.0.1", self.port))
            self.sock.setblocking(False)
        else:
            import serial
            self.serial_conn = serial.Serial(self.serial_port, 115200, timeout=0.1)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self):
        while self._running:
            raw = b""
            try:
                if self.use_tcp and self.sock:
                    try:
                        raw = self.sock.recv(512)
                    except (BlockingIOError, socket.timeout):
                        pass
                elif self.serial_conn:
                    raw = self.serial_conn.read(512)
            except Exception:
                break

            if raw:
                with self._lock:
                    for b in raw:
                        pkt = self.parser.process_byte(b)
                        if pkt:
                            self.received_packets.append(pkt)
                            if pkt.cmd == Command.TELEMETRY:
                                try:
                                    t = TelemetryData.unpack(pkt.payload)
                                    self.telemetry_history.append(t)
                                    self.latest_telemetry = t
                                except Exception:
                                    pass
            else:
                time.sleep(0.005)

    def send_packet(self, cmd: int, payload: bytes = b"") -> int:
        current_seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        pkt = Packet(seq=current_seq, cmd=cmd, payload=payload)
        wire = pkt.encode()
        self.send_raw(wire)
        return current_seq

    def send_raw(self, data: bytes):
        if self.use_tcp and self.sock:
            self.sock.sendall(data)
        elif self.serial_conn:
            self.serial_conn.write(data)

    def send_hello(self) -> int:
        return self.send_packet(Command.HELLO, bytes([1, 0, 100]))

    def send_heartbeat(self) -> int:
        return self.send_packet(Command.HEARTBEAT, b"")

    def load_job(self, job_id: int, cut_positions: List[int]) -> int:
        payload = struct.pack(">HB", job_id, len(cut_positions))
        for pos in cut_positions:
            payload += struct.pack(">I", pos)
        return self.send_packet(Command.LOAD_JOB, payload)

    def start_job(self, job_id: int) -> int:
        return self.send_packet(Command.START, struct.pack(">H", job_id))

    def send_estop(self) -> int:
        return self.send_packet(Command.ESTOP, b"")

    def reset_fault(self) -> int:
        return self.send_packet(Command.RESET_FAULT, b"")

    def sim_inject(self, fault_type: int, param: int) -> int:
        payload = struct.pack(">BH", fault_type, param)
        return self.send_packet(Command.SIM_INJECT, payload)

    def wait_for_ack(self, expected_seq: int, timeout: float = 2.0) -> Optional[Packet]:
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                for p in self.received_packets:
                    if p.cmd == Command.ACK and len(p.payload) >= 1 and p.payload[0] == expected_seq:
                        return p
            time.sleep(0.01)
        return None

    def wait_for_nack(self, expected_seq: int, timeout: float = 2.0) -> Optional[Packet]:
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                for p in self.received_packets:
                    if p.cmd == Command.NACK and len(p.payload) >= 1 and p.payload[0] == expected_seq:
                        return p
            time.sleep(0.01)
        return None

    def wait_for_state(self, target_state: int, timeout: float = 5.0) -> Optional[TelemetryData]:
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if self.latest_telemetry and self.latest_telemetry.state == target_state:
                    return self.latest_telemetry
            time.sleep(0.01)
        return None

    def wait_for_predicate(self, predicate: Callable[[TelemetryData], bool], timeout: float = 5.0) -> Optional[TelemetryData]:
        start = time.time()
        while time.time() - start < timeout:
            with self._lock:
                if self.latest_telemetry and predicate(self.latest_telemetry):
                    return self.latest_telemetry
            time.sleep(0.01)
        return None

    def close(self):
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2.0)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass


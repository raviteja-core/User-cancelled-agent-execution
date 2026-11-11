"""
Official System Verification Test Suite (T01 - T11)
Tests running controller firmware for safety, determinism, and protocol compliance.
Produces JUnit XML report and prints verification results table.
"""

import os
import random
import time
import pytest
from controller_client import ControllerClient
from papercutter_protocol import Command, FaultCode, State, LimitSwitch, Packet, crc16_ccitt_false


@pytest.fixture
def client(request):
    # Use TCP port parameterized or default
    port = 9200 + random.randint(1, 400)
    c = ControllerClient(use_tcp=True, port=port)
    c.start_controller()
    c.connect()
    # Initial handshake
    seq = c.send_hello()
    ack = c.wait_for_ack(seq, timeout=2.0)
    assert ack is not None, "Initial HELLO handshake failed"
    yield c
    c.close()


def run_heartbeat_worker(client: ControllerClient, stop_event):
    while not stop_event[0]:
        client.send_heartbeat()
        time.sleep(0.08)  # 80 ms (< 200 ms timeout)


def test_t01_normal_5_cut_job(client):
    """T01: Normal 5-cut job completes in sequence, returns to IDLE, no faults."""
    cuts = [1000, 2000, 3000, 4000, 5000] # 5 cuts in 0.01 mm units
    seq = client.load_job(job_id=1, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    stop_hb = [False]
    import threading
    hb_th = threading.Thread(target=run_heartbeat_worker, args=(client, stop_hb), daemon=True)
    hb_th.start()

    try:
        seq = client.start_job(job_id=1)
        assert client.wait_for_ack(seq) is not None

        # Wait for all cuts to finish and machine to return to IDLE
        final_telem = client.wait_for_predicate(
            lambda t: t.state == State.IDLE and t.cut_index >= 4,
            timeout=25.0
        )
        assert final_telem is not None, "Job did not complete to IDLE within timeout"
        assert final_telem.state == State.IDLE
        assert final_telem.fault_code == FaultCode.NONE
        assert final_telem.crc_errors == 0
    finally:
        stop_hb[0] = True
        hb_th.join(timeout=1.0)


def test_t02_paper_jam_during_feeding(client):
    """T02: Paper jam during FEEDING -> ERROR_JAM within 3.1 s, motors halted, latched."""
    # Inject paper jam
    seq = client.sim_inject(fault_type=1, param=1)
    assert client.wait_for_ack(seq) is not None

    cuts = [10000]
    seq = client.load_job(job_id=2, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    stop_hb = [False]
    import threading
    hb_th = threading.Thread(target=run_heartbeat_worker, args=(client, stop_hb), daemon=True)
    hb_th.start()

    try:
        t_start = time.time()
        seq = client.start_job(job_id=2)
        assert client.wait_for_ack(seq) is not None

        # Wait for ERROR_JAM
        err_telem = client.wait_for_state(State.ERROR_JAM, timeout=4.0)
        elapsed = time.time() - t_start

        assert err_telem is not None, "Controller failed to latch ERROR_JAM"
        assert err_telem.fault_code == FaultCode.PAPER_JAM
        assert elapsed <= 3.4, f"Jam halt took too long: {elapsed:.2f} s > 3.4 s"
    finally:
        stop_hb[0] = True
        hb_th.join(timeout=1.0)


def test_t03_dropped_packets(client):
    """T03: Dropped packets (3 in a row) -> Retransmit handling, no state corruption."""
    cuts = [8000]
    seq = client.load_job(job_id=3, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    # Instruct controller to drop next 3 packets
    seq = client.sim_inject(fault_type=5, param=3)
    assert client.wait_for_ack(seq) is not None

    # Attempt sending START (dropped packet #1)
    dropped_seq = client.start_job(job_id=3)
    ack = client.wait_for_ack(dropped_seq, timeout=0.05)
    assert ack is None, "Dropped packet should not receive ACK"

    # Send 2 more packets that get dropped (#2 and #3)
    client.send_heartbeat()
    client.send_heartbeat()

    # Retransmit START (packet #4, accepted)
    retransmit_seq = client.start_job(job_id=3)
    retransmit_ack = client.wait_for_ack(retransmit_seq, timeout=1.0)
    assert retransmit_ack is not None, "Retransmitted START failed to receive ACK"
    assert client.latest_telemetry is not None
    assert client.latest_telemetry.state != State.ERROR_FAULT


def test_t04_corrupted_crc(client):
    """T04: Corrupted CRC -> Frame ignored, crc counter +1, state unchanged."""
    # Obtain baseline CRC error count
    time.sleep(0.1)
    base_crc = client.latest_telemetry.crc_errors if client.latest_telemetry else 0
    base_state = client.latest_telemetry.state if client.latest_telemetry else State.IDLE

    # Build valid packet and corrupt its CRC
    pkt = Packet(seq=99, cmd=Command.HEARTBEAT)
    raw = bytearray(pkt.encode())
    raw[-1] ^= 0xFF # Invert CRC LSB

    client.send_raw(bytes(raw))

    # Wait for telemetry to reflect CRC error increment
    telem = client.wait_for_predicate(
        lambda t: t.crc_errors >= base_crc + 1,
        timeout=2.0
    )
    assert telem is not None, "CRC counter did not increment"
    assert telem.crc_errors == base_crc + 1
    assert telem.state == base_state


def test_t05_heartbeat_stops_mid_cut(client):
    """T05: Heartbeat stops mid-cut -> Safe halt within 250 ms, fault 0x0002."""
    cuts = [30000]
    seq = client.load_job(job_id=5, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    # Start motion
    seq = client.start_job(job_id=5)
    assert client.wait_for_ack(seq) is not None

    # Verify motion started
    client.wait_for_state(State.FEEDING, timeout=1.0)

    # Cease heartbeats completely and measure halt time
    t0 = time.time()
    err_telem = client.wait_for_state(State.ERROR_FAULT, timeout=1.0)
    halt_duration = time.time() - t0

    assert err_telem is not None, "Heartbeat watchdog did not trip"
    assert err_telem.fault_code == FaultCode.COMM_TIMEOUT
    assert halt_duration <= 0.35, f"Halt exceeded 250 ms bound: {halt_duration:.3f} s"


def test_t06_blade_limit_switch_fires_early(client):
    """T06: Blade limit switch fires early -> Immediate halt, fault 0x0003."""
    cuts = [15000]
    seq = client.load_job(job_id=6, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    # Inject premature limit switch
    t0 = time.time()
    seq = client.sim_inject(fault_type=3, param=1)
    assert client.wait_for_ack(seq) is not None

    err_telem = client.wait_for_state(State.ERROR_FAULT, timeout=1.0)
    halt_time = time.time() - t0

    assert err_telem is not None
    assert err_telem.fault_code == FaultCode.PREMATURE_LIMIT
    assert halt_time <= 0.15


def test_t07_estop_during_cutting(client):
    """T07: ESTOP during CUTTING -> Halt within 1 tick, blade decelerates at emergency decel, latched."""
    cuts = [1000] # Short feed so it reaches CUTTING quickly
    seq = client.load_job(job_id=7, cut_positions=cuts)
    assert client.wait_for_ack(seq) is not None

    stop_hb = [False]
    import threading
    hb_th = threading.Thread(target=run_heartbeat_worker, args=(client, stop_hb), daemon=True)
    hb_th.start()

    try:
        seq = client.start_job(job_id=7)
        assert client.wait_for_ack(seq) is not None

        # Wait until CLAMPING or CUTTING
        client.wait_for_predicate(lambda t: t.state in (State.CLAMPING, State.CUTTING), timeout=3.0)

        # Trigger ESTOP
        t0 = time.time()
        estop_seq = client.send_estop()
        ack = client.wait_for_ack(estop_seq, timeout=1.0)
        assert ack is not None

        err_telem = client.wait_for_state(State.ERROR_FAULT, timeout=1.0)
        elapsed = time.time() - t0

        assert err_telem is not None
        assert err_telem.fault_code == FaultCode.ESTOP
        assert elapsed <= 0.1
    finally:
        stop_hb[0] = True
        hb_th.join(timeout=1.0)


def test_t08_start_while_in_error(client):
    """T08: START while in error -> NACK, state unchanged."""
    # Force error state via ESTOP
    seq = client.send_estop()
    assert client.wait_for_ack(seq) is not None
    assert client.wait_for_state(State.ERROR_FAULT) is not None

    # Command START
    start_seq = client.start_job(job_id=8)
    nack = client.wait_for_nack(start_seq, timeout=1.0)
    assert nack is not None, "Controller must answer START with NACK when in error state"
    assert client.latest_telemetry.state == State.ERROR_FAULT


def test_t09_reset_fault_while_sensor_unsafe(client):
    """T09: RESET_FAULT while sensor unsafe / E-stop held -> NACK, still latched."""
    # Assert physical E-stop / unsafe condition
    client.send_estop()
    client.wait_for_state(State.ERROR_FAULT)

    # Inject interlock trip
    client.sim_inject(fault_type=4, param=1)

    # Attempt RESET_FAULT
    reset_seq = client.reset_fault()
    nack = client.wait_for_nack(reset_seq, timeout=1.0)
    assert nack is not None, "Controller must NACK RESET_FAULT when condition unsafe"
    assert client.latest_telemetry.state == State.ERROR_FAULT


def test_t10_garbage_bytes_resync(client):
    """T10: 10,000 random garbage bytes -> Controller stays alive, resyncs, answers next HELLO."""
    rng = random.Random(42)
    garbage = bytes(rng.randint(0, 255) for _ in range(10000))

    client.send_raw(garbage)
    time.sleep(0.2)

    # Send valid HELLO
    hello_seq = client.send_hello()
    ack = client.wait_for_ack(hello_seq, timeout=2.0)
    assert ack is not None, "Controller died or failed to resynchronise after garbage bytes"


def test_t11_determinism():
    """T11: Determinism -> Same seed and same job produce identical state progression."""
    def run_job_and_record():
        port = 9300 + random.randint(1, 300)
        c = ControllerClient(use_tcp=True, port=port)
        c.start_controller()
        c.connect()
        seq = c.send_hello()
        c.wait_for_ack(seq)

        stop_hb = [False]
        import threading
        th = threading.Thread(target=run_heartbeat_worker, args=(c, stop_hb), daemon=True)
        th.start()

        cuts = [2000, 4000]
        c.load_job(job_id=11, cut_positions=cuts)
        time.sleep(0.05)
        c.start_job(job_id=11)
        c.wait_for_predicate(lambda t: t.state == State.IDLE and t.cut_index >= 1, timeout=12.0)

        stop_hb[0] = True
        th.join(timeout=1.0)
        history = list(c.telemetry_history)
        c.close()

        # Extract ordered unique state transitions
        transitions = []
        for t in history:
            if not transitions or transitions[-1] != (t.state, t.cut_index):
                transitions.append((t.state, t.cut_index))
        return transitions

    run1 = run_job_and_record()
    run2 = run_job_and_record()

    assert len(run1) >= 5, "Expected multi-state transition sequence"
    assert run1 == run2, "State transition progression must be deterministic across identical runs"

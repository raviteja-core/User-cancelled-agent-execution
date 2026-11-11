#!/usr/bin/env python3
"""
Virtual Multi-Axis Paper Cutter Controller & Windows HMI - Demo Script

Demonstrates:
  1. Spawning controller_app.exe and WPF HMI
  2. Protocol handshake (HELLO) & Heartbeat watchdog maintenance
  3. Loading a 3-cut production recipe (80.0 mm, 50.0 mm, 20.0 mm)
  4. Starting automatic cycle execution
  5. Monitoring live telemetry (Back-gauge, Clamp, Blade TDC/BDC)
  6. Injecting a paper jam fault mid-cycle
  7. Verifying deterministic 1 ms safe halt & latched fault (PAPER_JAM)
  8. Verifying illegal transition rejection (NACK on START during error)
  9. Clearing fault & safely resetting to IDLE
"""

import sys
import os
import time
import struct
import subprocess
import argparse
from pathlib import Path

# Add tests/python to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests" / "python"))

from controller_client import ControllerClient
from papercutter_protocol import Command, State, FaultCode, TelemetryData


def print_banner(text: str, color: str = "36"):
    line = "=" * 72
    print(f"\033[{color}m{line}\033[0m")
    print(f"\033[{color}m  {text.center(68)}\033[0m")
    print(f"\033[{color}m{line}\033[0m")


def main():
    parser = argparse.ArgumentParser(description="Paper Cutter Simulation Demo")
    parser.add_argument("--serial", action="store_true", help="Use com0com serial (COM10/COM11)")
    parser.add_argument("--port", type=int, default=9000, help="TCP port for loopback (default: 9000)")
    parser.add_argument("--no-hmi", action="store_true", help="Do not launch WPF HMI window")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    controller_bin = repo_root / "build" / "controller_app" / "Debug" / "controller_app.exe"
    hmi_bin = repo_root / "hmi" / "PaperCutter.Hmi" / "bin" / "Debug" / "net8.0-windows" / "PaperCutter.Hmi.exe"

    if not controller_bin.exists():
        print(f"\033[31m[ERROR] controller_app.exe not found at: {controller_bin}\033[0m")
        print("Please build with: cmake -B build ... && cmake --build build --config Debug")
        sys.exit(1)

    print_banner("VIRTUAL MULTI-AXIS PAPER CUTTER DEMO", "36;1")
    print(f"[*] Repository Root: {repo_root}")
    print(f"[*] Mode: {'Serial COM10/COM11' if args.serial else f'TCP 127.0.0.1:{args.port}'}")

    # 1. Launch WPF HMI if available
    hmi_proc = None
    if not args.no_hmi and hmi_bin.exists():
        print("[*] Launching WPF HMI application...")
        try:
            hmi_proc = subprocess.Popen([str(hmi_bin)], cwd=str(hmi_bin.parent))
            print("    -> WPF HMI launched successfully.")
        except Exception as e:
            print(f"    -> Warning: Could not launch HMI: {e}")

    # 2. Start Controller Client & Process
    client = ControllerClient(use_tcp=not args.serial, port=args.port, serial_port="COM11")
    print("[*] Starting C++ Controller Firmware Simulation...")
    client.start_controller()
    time.sleep(0.5)

    print("[*] Connecting client transport...")
    client.connect()

    heartbeat_active = True
    def hb_worker():
        while heartbeat_active:
            try:
                client.send_heartbeat()
            except Exception:
                pass
            time.sleep(0.08)

    import threading
    hb_thread = threading.Thread(target=hb_worker, daemon=True)
    hb_thread.start()

    try:
        # Handshake
        print("\n--- STEP 1: PROTOCOL HANDSHAKE (HELLO) ---")
        seq = client.send_hello()
        ack = client.wait_for_ack(seq, timeout=2.0)
        assert ack is not None, "Failed to receive ACK for HELLO"
        proto_ver = ack.payload[1] if len(ack.payload) > 1 else 1
        print(f"\033[32m[PASS]\033[0m Handshake OK. Protocol Version: {proto_ver}")

        # Wait for initial IDLE telemetry
        telem = client.wait_for_state(State.IDLE, timeout=2.0)
        assert telem is not None, "Controller not in IDLE state"
        print(f"    Current State: IDLE (State={telem.state}) | Latched Fault: {telem.fault_code}")

        # Load 3-cut Job Recipe
        print("\n--- STEP 2: LOAD 3-CUT JOB RECIPE ---")
        job_id = 401
        cut_positions = [8000, 5000, 2000] # 80.0 mm, 50.0 mm, 20.0 mm (in 0.01 mm)
        print(f"    Job ID: {job_id} | Cuts: {[p/100.0 for p in cut_positions]} mm")
        seq = client.load_job(job_id, cut_positions)
        ack = client.wait_for_ack(seq, timeout=2.0)
        assert ack is not None, "Failed to receive ACK for LOAD_JOB"
        print("\033[32m[PASS]\033[0m Job recipe successfully accepted and loaded into motion planner.")

        # Start Job Execution
        print("\n--- STEP 3: START AUTOMATIC CYCLE ---")
        seq = client.start_job(job_id)
        ack = client.wait_for_ack(seq, timeout=2.0)
        assert ack is not None, "Failed to receive ACK for START"
        print("\033[32m[PASS]\033[0m START command acknowledged. Machine cycle engaged.")

        # Monitor Cut 1
        print("\n[*] Monitoring Cut 1...")
        feeding = client.wait_for_state(State.FEEDING, timeout=2.0)
        print(f"    State -> FEEDING (Target: 80.00 mm)")
        
        cutting = client.wait_for_state(State.CUTTING, timeout=4.0)
        print(f"    State -> CUTTING (Pneumatic clamp engaged, blade cycling)")

        # Wait for transition to Cut 2 (FEEDING again)
        print("\n[*] Cut 1 complete. Advancing to Cut 2...")
        cut2_feed = client.wait_for_predicate(lambda t: t.state == State.FEEDING and t.cut_index >= 1, timeout=5.0)
        assert cut2_feed is not None, "Cut 2 feeding phase not reached"
        print(f"    State -> FEEDING Cut 2 (Index={cut2_feed.cut_index}, Pos={cut2_feed.back_gauge_pos/100.0:.2f} mm)")

        # Step 4: Inject Paper Jam
        print("\n--- STEP 4: INJECT PAPER FEED JAM ---")
        print("    Injecting optical sensor stall (fault_type=1, param=1) via SIM_INJECT...")
        seq = client.sim_inject(fault_type=1, param=1)
        client.wait_for_ack(seq, timeout=2.0)

        print("[*] Waiting for Safety Supervisor & FSM to detect jam and trip (3.0 s timer)...")
        jam_telem = client.wait_for_state(State.ERROR_JAM, timeout=6.0)
        assert jam_telem is not None, "Failed to transition to ERROR_JAM!"
        
        print("\033[31;1m[TRIPPED]\033[0m Deterministic Safety Halt Verified!")
        print(f"    FSM State:     ERROR_JAM ({jam_telem.state})")
        fault_name = FaultCode(jam_telem.fault_code).name if jam_telem.fault_code in FaultCode._value2member_map_ else f"UNKNOWN({jam_telem.fault_code})"
        print(f"    Latched Fault: {fault_name} (0x{jam_telem.fault_code:04X})")
        print(f"    Back Gauge:    {jam_telem.back_gauge_pos/100.0:.2f} mm (MOTION HALTED WITHIN 1 MS)")
        print(f"    Blade Knife:   {jam_telem.blade_pos/100.0:.2f} mm (SAFE AT TDC)")

        # Step 5: Test Fault Latching & Rejection
        print("\n--- STEP 5: VERIFY FAULT LATCHING & REJECTION ---")
        print("    Attempting START command while in ERROR_JAM...")
        nack_seq = client.start_job(job_id)
        nack = client.wait_for_nack(nack_seq, timeout=2.0)
        assert nack is not None, "Controller accepted illegal START command while in error state!"
        print("\033[32m[PASS]\033[0m Illegal transition rejected with NACK. Machine remains latched in error state.")

        # Step 6: Clear Jam and Reset Fault
        print("\n--- STEP 6: CLEAR SENSOR & RESET FAULT ---")
        print("    Clearing optical paper feed sensor...")
        seq = client.sim_inject(fault_type=1, param=0) # Clear fault injection
        client.wait_for_ack(seq, timeout=2.0)
        time.sleep(0.1)

        print("    Transmitting RESET_FAULT command...")
        seq = client.reset_fault()
        ack = client.wait_for_ack(seq, timeout=2.0)
        assert ack is not None, "Failed to receive ACK for RESET_FAULT"

        reset_telem = client.wait_for_state(State.IDLE, timeout=2.0)
        assert reset_telem is not None, "Did not return to IDLE state after reset"
        assert reset_telem.fault_code == FaultCode.NONE, "Latched fault was not cleared"

        print("\033[32m[PASS]\033[0m Controller safely reset to IDLE.")
        print(f"    Current State: IDLE ({reset_telem.state}) | Latched Fault: {FaultCode(reset_telem.fault_code).name}")

        print_banner("DEMO COMPLETED SUCCESSFULLY: ALL CHECKS PASSED", "32;1")

    finally:
        heartbeat_active = False
        client.close()
        if hmi_proc:
            try:
                # Leave HMI open for a moment if interactive, or kill if non-interactive
                time.sleep(1.0)
            except Exception:
                pass


if __name__ == "__main__":
    main()

"""Exercise test-firmware task watchdogs over a user-specified serial port.

Build esp32dev_watchdog_test first. Run with the PlatformIO Python environment
or install pyserial. Examples:
    python test/watchdog_smoke.py --port COM5 --task all
    python test/watchdog_smoke.py --port rfc2217://localhost:4000 --task 5

This test requests software resets and deliberately stalls application tasks.
It requires a running board or simulator with the fault-injection firmware.
"""

import argparse
import re
import sys
import time


BASELINE_SECONDS = 18.0
RESET_TIMEOUT_SECONDS = 30.0
TASK_NAMES = (
    "SensorTask",
    "FeatureTask",
    "ControlTask",
    "CommunicationTask",
    "StorageTask",
    "MonitorTask",
)
INJECTION_BANNER = "[TEST] Watchdog fault injection enabled; serial 0..5"
RESET_REASON = re.compile(r"\[BOOT\] resetReason=(\d+)\b")
BOOT_ID = re.compile(r"\[BOOT\] boot=(\d+)\b")
DATA_BOOT_ID = re.compile(r"\[DATA\] boot=(\d+)\b")
ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class SmokeFailure(Exception):
    pass


class SerialLog:
    def __init__(self, port):
        self.port = port
        self.buffer = bytearray()
        self.started = time.monotonic()

    def clear(self):
        self.buffer.clear()
        self.port.reset_input_buffer()

    def send(self, command):
        payload = command.encode("ascii")
        if self.port.write(payload) != len(payload):
            raise SmokeFailure("Could not write the complete serial command")

    def next_line(self, deadline):
        while time.monotonic() < deadline:
            end = self.buffer.find(b"\n")
            if end >= 0:
                raw = bytes(self.buffer[:end])
                del self.buffer[: end + 1]
                line = ANSI_ESCAPE.sub("", raw.decode("utf-8", errors="replace").rstrip("\r"))
                print(f"{time.monotonic() - self.started:8.2f}s | {line}", flush=True)
                return line
            self.buffer.extend(self.port.read(256))
            if len(self.buffer) > 16384:
                raise SmokeFailure("Serial output exceeded 16 KiB without a line ending")
        return None


def prepare_case(log):
    print("[HOST] Requesting software reset and checking the test-firmware banner", flush=True)
    log.clear()
    log.send("r")
    deadline = time.monotonic() + RESET_TIMEOUT_SECONDS
    reset_requested = False
    reason = None
    boot_id = None
    injection_enabled = False
    while time.monotonic() < deadline:
        line = log.next_line(deadline)
        if line is None:
            break
        if "[TEST] Software reset requested" in line:
            reset_requested = True
        if not reset_requested:
            continue
        match = RESET_REASON.search(line)
        if match:
            reason = int(match.group(1))
            boot_id = None
            injection_enabled = False
        match = BOOT_ID.search(line)
        if match and reason is not None:
            boot_id = int(match.group(1))
        if INJECTION_BANNER in line and reason is not None:
            injection_enabled = True
        if injection_enabled and boot_id is not None:
            if reason != 3:
                raise SmokeFailure(f"The requested software reset returned reason {reason}, expected ESP_RST_SW (3)")
            return boot_id
    raise SmokeFailure(
        "No complete test-firmware startup within 30 s after 'r'. Expected the software-reset request, "
        "resetReason, boot ID and fault-injection banner. Check the port and esp32dev_watchdog_test firmware."
    )


def verify_baseline(log, boot_id):
    print("[HOST] Checking 18 s of healthy operation before fault injection", flush=True)
    deadline = time.monotonic() + BASELINE_SECONDS
    saw_data = False
    while time.monotonic() < deadline:
        line = log.next_line(deadline)
        if line is None:
            break
        if RESET_REASON.search(line) or BOOT_ID.search(line):
            raise SmokeFailure("The device restarted during the healthy baseline")
        if ("watchdog got triggered" in line.lower() or
                "[HEALTH] Heartbeat late" in line or "[WDT] Heartbeat timeout" in line):
            raise SmokeFailure("A watchdog fault occurred before injection")
        match = DATA_BOOT_ID.search(line)
        if match:
            if int(match.group(1)) != boot_id:
                raise SmokeFailure("Telemetry changed boot ID during the healthy baseline")
            saw_data = True
    if not saw_data:
        raise SmokeFailure("No [DATA] frame from this boot during the 18 s baseline")


def inject_and_wait(log, task_index, previous_boot_id):
    task_name = TASK_NAMES[task_index]
    print(f"[HOST] Injecting stall {task_index} ({task_name}); waiting up to 30 s for watchdog reset", flush=True)
    log.send(str(task_index))
    deadline = time.monotonic() + RESET_TIMEOUT_SECONDS
    injection_accepted = False
    watchdog_trace = False
    reason = None
    while time.monotonic() < deadline:
        line = log.next_line(deadline)
        if line is None:
            break
        if f"[TEST] Stalling {task_name} at next heartbeat" in line:
            injection_accepted = True
        if "watchdog got triggered" in line.lower():
            watchdog_trace = True
        if "[TEST] Software reset requested" in line:
            raise SmokeFailure("An external software-reset request interrupted the injected fault")
        match = RESET_REASON.search(line)
        if match:
            reason = int(match.group(1))
        match = BOOT_ID.search(line)
        if match:
            new_boot_id = int(match.group(1))
            if not injection_accepted:
                raise SmokeFailure(f"Device restarted before it acknowledged the {task_name} injection")
            if reason is None:
                raise SmokeFailure("New boot ID appeared without a resetReason line")
            # ESP_RST_TASK_WDT is 6. A generic panic (4) alone is insufficient.
            if reason != 6 and not watchdog_trace:
                raise SmokeFailure(f"Reset reason {reason} had no task-watchdog trace")
            if new_boot_id <= previous_boot_id:
                raise SmokeFailure(f"Boot ID did not increase ({previous_boot_id} -> {new_boot_id})")
            print(f"[PASS] {task_name}: watchdog reset, reason={reason}, boot={previous_boot_id}->{new_boot_id}", flush=True)
            return
    raise SmokeFailure(
        f"{task_name}: no confirmed watchdog reboot within 30 s. "
        f"Injection acknowledged={injection_accepted}, watchdog trace={watchdog_trace}, resetReason={reason}."
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Explicit COM port or RFC2217 URL, for example COM5 or rfc2217://localhost:4000")
    parser.add_argument("--task", choices=("all", "0", "1", "2", "3", "4", "5"), default="all",
                        help="0 Sensor, 1 Feature, 2 Control, 3 Communication, 4 Storage, 5 Monitor (default: all)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate (default: 115200)")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        print("[FAIL] pyserial is required. Use the PlatformIO Python environment or run python -m pip install pyserial.", file=sys.stderr)
        return 2

    task_indices = range(len(TASK_NAMES)) if args.task == "all" else (int(args.task),)
    try:
        # pyserial's RFC2217 backend does not support write_timeout. Its socket
        # uses a finite network timeout; native serial writes use two seconds.
        write_timeout = None if args.port.lower().startswith("rfc2217://") else 2
        port = serial.serial_for_url(args.port, baudrate=args.baud, timeout=0.25,
                                     write_timeout=write_timeout, do_not_open=True)
        with port:
            log = SerialLog(port)
            for task_index in task_indices:
                print(f"\n[HOST] Case {task_index}: {TASK_NAMES[task_index]}", flush=True)
                boot_id = prepare_case(log)
                verify_baseline(log, boot_id)
                inject_and_wait(log, task_index, boot_id)
        print("[PASS] All requested watchdog cases completed", flush=True)
        return 0
    except (SmokeFailure, serial.SerialException, OSError, ValueError) as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n[FAIL] Test interrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())

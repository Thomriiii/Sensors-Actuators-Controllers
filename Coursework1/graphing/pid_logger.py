import csv
import os
import subprocess
import sys
import time
from pathlib import Path


BAUD = 115200
DURATION = 30
OUTPUT_DIR = "graphs"
CSV_FILE = "data.csv"
DEFAULT_PORT = None


def load_pyserial():
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "PySerial is not installed. Run:\n"
            f"  {sys.executable} -m pip install pyserial"
        ) from exc

    if not hasattr(serial, "Serial"):
        source = getattr(serial, "__file__", "<unknown>")
        raise SystemExit(
            "The imported 'serial' module is not PySerial.\n"
            f"Loaded from: {source}\n"
            "Fix it with:\n"
            f"  {sys.executable} -m pip uninstall -y serial\n"
            f"  {sys.executable} -m pip install pyserial"
        )

    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        list_ports = None

    return serial, list_ports


def detect_port(list_ports_module, configured_port):
    if configured_port:
        return configured_port

    if list_ports_module is None:
        raise SystemExit("No serial port configured and port auto-detection is unavailable.")

    preferred = []
    fallback = []
    for port in list_ports_module.comports():
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        hwid = (port.hwid or "").lower()
        device = port.device

        if any(token in description for token in ("arduino", "mkr", "samd")):
            preferred.append(device)
        elif any(token in manufacturer for token in ("arduino",)):
            preferred.append(device)
        elif "vid:pid=2341" in hwid:
            preferred.append(device)
        else:
            fallback.append(device)

    if preferred:
        return preferred[0]
    if len(fallback) == 1:
        return fallback[0]

    raise SystemExit(
        "Could not auto-detect the Arduino serial port.\n"
        "Set DEFAULT_PORT in pid_logger.py to the correct COM port."
    )


def parse_data_line(line):
    parts = line.split(",")
    if len(parts) != 5:
        return None
    try:
        return [float(value) for value in parts]
    except ValueError:
        return None


def wait_for_header(ser):
    deadline = time.time() + 12
    while time.time() < deadline:
        line = ser.readline().decode(errors="ignore").strip()
        if line == "raw,filtered,setpoint,error,output":
            return line.split(","), None

        values = parse_data_line(line)
        if values is not None:
            header = ["raw", "filtered", "setpoint", "error", "output"]
            return header, values

    raise SystemExit("Timed out waiting for CSV header or first valid data row from the board.")


def log_data(ser, header, duration_s, first_row=None):
    start_time = time.time()
    rows = []

    if first_row is not None:
        rows.append([0.0] + first_row)

    while time.time() - start_time < duration_s:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        values = parse_data_line(line)
        if values is None or len(values) != len(header):
            continue

        rows.append([time.time() - start_time] + values)

    return rows


def save_csv(csv_path, header, rows):
    with csv_path.open("w", newline="", encoding="ascii") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time"] + header)
        writer.writerows(rows)


def run_graphing(script_dir, csv_path):
    graphing_script = script_dir / "graphing.py"
    if not graphing_script.exists():
        print("graphing.py not found; skipping plot generation.", flush=True)
        return

    try:
        subprocess.run(
            [sys.executable, str(graphing_script), str(csv_path)],
            cwd=str(script_dir),
            check=True,
        )
    except subprocess.CalledProcessError as exc:
        print(f"graphing.py failed with exit code {exc.returncode}", flush=True)


def main():
    script_dir = Path(__file__).resolve().parent
    output_dir = script_dir / OUTPUT_DIR
    csv_path = output_dir / CSV_FILE
    output_dir.mkdir(exist_ok=True)

    serial, list_ports = load_pyserial()
    port = detect_port(list_ports, DEFAULT_PORT)

    print(f"Opening serial port {port}...", flush=True)
    ser = serial.Serial(port, BAUD, timeout=1)
    time.sleep(2)
    ser.reset_input_buffer()

    print("Waiting for CSV header...", flush=True)
    header, first_row = wait_for_header(ser)
    print("Header detected:", header, flush=True)

    print(f"Logging data for {DURATION} s...", flush=True)
    rows = log_data(ser, header, DURATION, first_row)
    ser.close()

    save_csv(csv_path, header, rows)
    print(f"CSV saved to {csv_path}", flush=True)

    run_graphing(script_dir, csv_path)


if __name__ == "__main__":
    main()

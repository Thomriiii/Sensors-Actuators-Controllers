import csv
import math
import statistics
import sys
from pathlib import Path


DATA_FILE = Path("data.csv")
CONTROL_PERIOD_S = 0.22
OUTPUT_MAX = 255.0
SETTLE_BAND = 0.05

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    plt = None


def load_rows(path: Path) -> tuple[str, list[dict[str, float]]]:
    rows: list[dict[str, float]] = []
    mode = "closed_loop"

    with path.open(newline="", encoding="ascii") as handle:
        reader = csv.reader(handle)
        for raw_row in reader:
            if not raw_row:
                continue

            first = raw_row[0].strip()
            if not first:
                continue
            if first.startswith("MaxContribution") or first.startswith("Setpoint"):
                continue

            lowered = [value.strip().lower() for value in raw_row]
            if lowered == ["raw", "filtered", "setpoint", "error", "output"]:
                mode = "closed_loop"
                continue
            if lowered == ["time", "raw", "filtered", "setpoint", "error", "output"]:
                mode = "closed_loop"
                continue
            if lowered == ["sensor", "output"]:
                mode = "open_loop"
                continue
            if lowered == ["time", "sensor", "output"]:
                mode = "open_loop"
                continue

            values = [float(value) for value in raw_row]
            if len(values) == 6:
                rows.append(
                    {
                        "time": values[0],
                        "raw": values[1],
                        "filtered": values[2],
                        "setpoint": values[3],
                        "error": values[4],
                        "output": values[5],
                    }
                )
                mode = "closed_loop"
            elif len(values) == 5:
                rows.append(
                    {
                        "raw": values[0],
                        "filtered": values[1],
                        "setpoint": values[2],
                        "error": values[3],
                        "output": values[4],
                    }
                )
                mode = "closed_loop"
            elif len(values) == 3:
                rows.append({"time": values[0], "sensor": values[1], "output": values[2]})
                mode = "open_loop"
            elif len(values) == 2:
                rows.append({"sensor": values[0], "output": values[1]})
                mode = "open_loop"
            else:
                raise ValueError(f"Unsupported CSV row: {raw_row}")

    if not rows:
        raise ValueError("No data rows found in CSV.")

    return mode, rows


def count_zero_crossings(errors: list[float]) -> int:
    zero_crossings = 0
    last_sign = 0
    for error in errors:
        sign = 0
        if error > 0.0:
            sign = 1
        elif error < 0.0:
            sign = -1

        if sign != 0 and last_sign != 0 and sign != last_sign:
            zero_crossings += 1
        if sign != 0:
            last_sign = sign

    return zero_crossings


def find_rise_time(filtered: list[float], setpoint: float) -> float:
    threshold = 0.9 * setpoint
    if filtered and filtered[0] >= threshold:
        return math.nan
    for index, value in enumerate(filtered):
        if value >= threshold:
            return index * CONTROL_PERIOD_S
    return math.nan


def find_settling_time(filtered: list[float], setpoint: float) -> float:
    tolerance = SETTLE_BAND * setpoint
    if filtered and abs(filtered[0] - setpoint) <= tolerance:
        return math.nan
    for index in range(len(filtered)):
        tail = filtered[index:]
        if all(abs(value - setpoint) <= tolerance for value in tail):
            return index * CONTROL_PERIOD_S
    return math.nan


def classify_closed_loop(rows: list[dict[str, float]]) -> tuple[str, dict[str, float]]:
    filtered = [row["filtered"] for row in rows]
    setpoint = rows[-1]["setpoint"]
    errors = [row["error"] for row in rows]
    output = [row["output"] for row in rows]

    steady_window = max(10, len(rows) // 5)
    final_error = statistics.fmean(abs(value) for value in errors[-20:])
    overshoot = max(0.0, max(filtered) - setpoint)
    overshoot_pct = 100.0 * overshoot / setpoint if setpoint > 0 else 0.0
    output_sat_fraction = sum(value >= OUTPUT_MAX - 1.0 for value in output) / len(output)
    steady_std = statistics.pstdev(filtered[-steady_window:]) if steady_window > 1 else 0.0
    zero_crossings = count_zero_crossings(errors)

    if output_sat_fraction > 0.30 and filtered[-1] < 0.9 * setpoint:
        state = "UNDERPOWERED"
    elif overshoot_pct > 10.0 or zero_crossings > max(6, len(rows) // 15) or steady_std > 0.08 * setpoint:
        state = "OSCILLATING"
    elif final_error > 0.08 * setpoint:
        state = "OVERDAMPED"
    else:
        state = "WELL-TUNED"

    metrics = {
        "steady_state_error": final_error,
        "overshoot_pct": overshoot_pct,
        "output_saturation_fraction": output_sat_fraction,
        "steady_state_std": steady_std,
        "zero_crossings": float(zero_crossings),
        "rise_time_s": find_rise_time(filtered, setpoint),
        "settling_time_s": find_settling_time(filtered, setpoint),
        "ise": sum(error * error for error in errors) * CONTROL_PERIOD_S,
    }
    return state, metrics


def save_metrics_report(state: str, metrics: dict[str, float]) -> None:
    lines = [f"classification,{state}"]
    for key, value in metrics.items():
        if math.isnan(value):
            lines.append(f"{key},nan")
        else:
            lines.append(f"{key},{value:.4f}")
    Path("metrics.txt").write_text("\n".join(lines) + "\n", encoding="ascii")


def print_metrics(state: str, metrics: dict[str, float]) -> None:
    print(f"classification: {state}")
    for key, value in metrics.items():
        if math.isnan(value):
            print(f"{key}: nan")
        else:
            print(f"{key}: {value:.4f}")


def maybe_plot(mode: str, rows: list[dict[str, float]]) -> None:
    if plt is None:
        print("plotting: skipped (matplotlib not installed)")
        return

    if mode == "open_loop":
        x_axis = [row.get("time", index) for index, row in enumerate(rows)]
        x_label = "Time (s)" if "time" in rows[0] else "Sample"
        sensor = [row["sensor"] for row in rows]
        output = [row["output"] for row in rows]

        plt.figure()
        plt.plot(x_axis, sensor, label="Sensor")
        plt.plot(x_axis, output, label="Output")
        plt.legend()
        plt.title("Open Loop Behaviour")
        plt.xlabel(x_label)
        plt.ylabel("Value")
        plt.savefig("open_loop.png")
    else:
        x_axis = [row.get("time", index) for index, row in enumerate(rows)]
        x_label = "Time (s)" if "time" in rows[0] else "Sample"
        raw = [row["raw"] for row in rows]
        filtered = [row["filtered"] for row in rows]
        setpoint = [row["setpoint"] for row in rows]
        error = [row["error"] for row in rows]
        output = [row["output"] for row in rows]

        plt.figure()
        plt.plot(x_axis, raw, label="Raw Sensor")
        plt.plot(x_axis, filtered, label="Filtered Sensor")
        plt.plot(x_axis, setpoint, label="Setpoint")
        plt.legend()
        plt.title("Closed Loop Sensor Response")
        plt.xlabel(x_label)
        plt.ylabel("ADC Value")
        plt.savefig("sensor_graph.png")

        plt.figure()
        plt.plot(x_axis, filtered, label="Measured")
        plt.plot(x_axis, setpoint, label="Setpoint")
        plt.legend()
        plt.title("Closed Loop Response")
        plt.xlabel(x_label)
        plt.ylabel("ADC Value")
        plt.savefig("closed_loop.png")

        plt.figure()
        plt.plot(x_axis, error, label="Error")
        plt.plot(x_axis, output, label="Output")
        plt.legend()
        plt.title("Error and Controller Output")
        plt.xlabel(x_label)
        plt.ylabel("Value")
        plt.savefig("disturbance.png")

    plt.show()


def main() -> None:
    data_file = Path(sys.argv[1]) if len(sys.argv) > 1 else DATA_FILE
    mode, rows = load_rows(data_file)
    if mode == "closed_loop":
        state, metrics = classify_closed_loop(rows)
        save_metrics_report(state, metrics)
        print_metrics(state, metrics)
    else:
        print("classification: open-loop capture")

    maybe_plot(mode, rows)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import base64
import csv
import importlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LOGS = ROOT / "logs"
NOTEBOOKS = (LOGS / "plot.ipynb",)
JUPYTER = ROOT / ".venv" / "bin" / "jupyter"
PROJECT_PYTHON = ROOT / ".venv" / "bin" / "python"
DEFAULT_ALPHAS = "0.02,0.04,0.06,0.10,0.15,0.20,0.30"


def find_platformio():
    executable = shutil.which("platformio")
    if executable:
        return executable
    executable = Path.home() / ".platformio" / "penv" / "bin" / "platformio"
    if executable.is_file():
        return str(executable)
    raise RuntimeError("PlatformIO was not found; install it or add it to PATH.")


def safe_stem(name):
    name = name.strip()
    if name.lower().endswith(".log"):
        name = name[:-4]
    name = re.sub(r"[^A-Za-z0-9._-]+", "-", name).strip(".-_")
    if not name:
        raise ValueError("Enter a log name containing letters or numbers.")
    return name


def allocate_run_paths(name):
    stem = safe_stem(name)
    suffix = 1
    while True:
        candidate = stem if suffix == 1 else f"{stem}_{suffix}"
        log_path = LOGS / f"{candidate}.log"
        plot_dir = LOGS / f"{candidate}_plots"
        if not log_path.exists() and not plot_dir.exists():
            return candidate, log_path, plot_dir
        suffix += 1


def find_run_artifacts(name, logs_dir=LOGS):
    stem = safe_stem(name)
    run_name = re.compile(rf"{re.escape(stem)}(?:_[2-9]|_[1-9][0-9]+)?")
    matches = []
    for path in logs_dir.iterdir():
        if path.is_file() and path.suffix == ".log" and run_name.fullmatch(path.stem):
            matches.append(path)
        elif path.is_dir() and path.name.endswith("_plots"):
            plot_stem = path.name[:-6]
            if run_name.fullmatch(plot_stem):
                matches.append(path)
    return sorted(matches)


def delete_run_artifacts(name, confirmed=False, logs_dir=LOGS):
    matches = find_run_artifacts(name, logs_dir)
    if not matches:
        print(f"No log or plot files found for '{safe_stem(name)}'.")
        return 0

    print("The following items will be deleted:")
    for path in matches:
        print(f"  {path}")
    if not confirmed:
        answer = input("Delete these items? [y/N] ").strip().lower()
        if answer not in ("y", "yes"):
            print("Cancelled; nothing was deleted.")
            return 1

    for path in matches:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    print(f"Deleted {len(matches)} items.")
    return 0


def run_build(platformio, alpha=None):
    command = [platformio, "run", "--environment", "teensy41"]
    print("Building Teensy 4.1 firmware", flush=True)
    env = os.environ.copy()
    if alpha is not None:
        env["BASEBOT_FILTER_ALPHA"] = f"{alpha:.8g}"
    subprocess.run(command, cwd=ROOT, env=env, check=True)


def upload_with_retry(platformio, alpha=None):
    command = [platformio, "run", "--target", "upload", "--environment", "teensy41"]
    env = os.environ.copy()
    if alpha is not None:
        env["BASEBOT_FILTER_ALPHA"] = f"{alpha:.8g}"
    for attempt in range(2):
        result = subprocess.run(command, cwd=ROOT, env=env)
        if result.returncode == 0:
            return
        if attempt == 0:
            print("Upload failed; retrying once.", flush=True)
    raise RuntimeError("Upload failed twice. Check the USB connection and Teensy bootloader.")


def choose_port(requested_port):
    if requested_port:
        return requested_port

    try:
        list_ports = importlib.import_module("serial.tools.list_ports")
    except ImportError as exc:
        raise RuntimeError("PySerial is unavailable in the PlatformIO Python environment.") from exc

    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        ports = [port for port in list_ports.comports() if port.vid == 0x16C0]
        if len(ports) == 1:
            return ports[0].device
        if len(ports) > 1:
            print("Teensy serial ports:")
            for index, port in enumerate(ports, 1):
                print(f"  {index}. {port.device} ({port.description})")
            selection = input("Select port number: ").strip()
            try:
                return ports[int(selection) - 1].device
            except (ValueError, IndexError) as exc:
                raise RuntimeError("Invalid serial port selection.") from exc
        print("Waiting for the Teensy serial port...", flush=True)
        time.sleep(0.5)
    raise RuntimeError("No Teensy USB serial port found. Connect the robot and try again.")


def capture_run(port, log_path, auto_start=False):
    try:
        serial = importlib.import_module("serial")
    except ImportError as exc:
        raise RuntimeError("PySerial is unavailable in the PlatformIO Python environment.") from exc

    print(f"Capturing to {log_path}", flush=True)
    if auto_start:
        print("Auto-start enabled; sending the firmware start command.", flush=True)
    else:
        print("Press the robot start button. Capture ends when the mission completes.", flush=True)
    with serial.Serial(port, baudrate=12000000, timeout=1) as connection:
        with log_path.open("x", encoding="utf-8") as output:
            if auto_start:
                connection.write(b"start\n")
                connection.flush()
                start_deadline = time.monotonic() + 10
                while time.monotonic() < start_deadline:
                    chunk = connection.readline()
                    if not chunk:
                        continue
                    line = chunk.decode("utf-8", errors="replace")
                    print(line, end="", flush=True)
                    output.write(line)
                    output.flush()
                    if "%% starting" in line:
                        break
                else:
                    raise RuntimeError("Robot did not acknowledge the automatic start command.")

            while True:
                chunk = connection.readline()
                if not chunk:
                    continue
                line = chunk.decode("utf-8", errors="replace")
                print(line, end="", flush=True)
                output.write(line)
                output.flush()
                if re.search(r"% sent \d+ log lines", line):
                    break

    numeric_rows = sum(
        bool(line.strip()) and not line.lstrip().startswith("%")
        for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    )
    if numeric_rows == 0:
        raise RuntimeError(f"No numeric log rows were captured in {log_path}.")


def export_notebook_plots(log_path, plot_dir):
    notebook_path = next((path for path in NOTEBOOKS if path.is_file()), None)
    if notebook_path is None:
        checked = ", ".join(str(path) for path in NOTEBOOKS)
        raise RuntimeError(f"Analysis notebook not found. Checked: {checked}")
    if not JUPYTER.is_file():
        raise RuntimeError(f"Jupyter executable not found: {JUPYTER}")

    notebook = json.loads(notebook_path.read_text(encoding="utf-8"))
    patched = False
    for cell in notebook.get("cells", []):
        if cell.get("cell_type") != "code":
            continue
        source = cell.get("source", [])
        lines = source.splitlines(keepends=True) if isinstance(source, str) else source[:]
        for index, line in enumerate(lines):
            if line.lstrip().startswith("log_path = Path("):
                lines[index] = f"log_path = Path({str(log_path.resolve())!r})\n"
                cell["source"] = lines
                patched = True
                break
        if patched:
            break
    if not patched:
        raise RuntimeError("Could not find the notebook's log_path assignment.")

    plot_dir.mkdir()
    with tempfile.TemporaryDirectory(prefix="robot-analysis-") as temp_name:
        temp_dir = Path(temp_name)
        input_notebook = temp_dir / "analysis_input.ipynb"
        output_notebook = temp_dir / "analysis_executed.ipynb"
        input_notebook.write_text(json.dumps(notebook), encoding="utf-8")

        subprocess.run(
            [
                str(JUPYTER), "nbconvert", "--to", "notebook", "--execute",
                "--ExecutePreprocessor.timeout=180", "--output", output_notebook.name,
                "--output-dir", str(temp_dir), str(input_notebook),
            ],
            cwd=LOGS,
            check=True,
        )

        executed = json.loads(output_notebook.read_text(encoding="utf-8"))
        image_count = 0
        for cell in executed.get("cells", []):
            for output in cell.get("outputs", []):
                image_data = output.get("data", {}).get("image/png")
                if not image_data:
                    continue
                if isinstance(image_data, list):
                    image_data = "".join(image_data)
                image_count += 1
                image_path = plot_dir / f"{log_path.stem}_plot_{image_count:02d}.png"
                image_path.write_bytes(base64.b64decode(image_data))

    if image_count == 0:
        raise RuntimeError("Notebook executed but produced no PNG plots.")
    return image_count


def parse_alphas(text):
    try:
        alphas = [float(value.strip()) for value in text.split(",") if value.strip()]
    except ValueError as exc:
        raise ValueError("Alphas must be comma-separated numbers in (0, 1].") from exc
    if len(alphas) < 2 or any(not math.isfinite(value) or not 0 < value <= 1 for value in alphas):
        raise ValueError("Provide at least two distinct alpha values in (0, 1].")
    if len(set(alphas)) != len(alphas):
        raise ValueError("Alpha values must be unique.")
    return alphas


def validate_sweep_config():
    config = (ROOT / "src" / "robot_config.h").read_text(encoding="utf-8")
    if not re.search(r"constexpr\s+ControlMode\s+controlMode\s*=\s*ControlMode::ClosedLoop\s*;", config):
        raise RuntimeError("Alpha sweep requires ControlMode::ClosedLoop in robot_config.h.")
    if not re.search(r"constexpr\s+bool\s+velocityFilterEnabled\s*=\s*true\s*;", config):
        raise RuntimeError("Alpha sweep requires velocityFilterEnabled = true.")
    gear_match = re.search(r"constexpr\s+float\s+gearRatio\s*=\s*([0-9.]+)f?\s*;", config)
    radius_match = re.search(r"constexpr\s+float\s+wheelRadius\s*=\s*([0-9.]+)f?\s*;", config)
    if not gear_match or not radius_match:
        raise RuntimeError("Could not read gear ratio and wheel radius from robot_config.h.")
    return float(gear_match.group(1)), float(radius_match.group(1))


def allocate_sweep_dir(name):
    base = safe_stem(name)
    suffix = 1
    while True:
        suffix_text = "" if suffix == 1 else f"_{suffix}"
        path = LOGS / f"{base}_alpha_sweep{suffix_text}"
        if not path.exists():
            path.mkdir()
            return path
        suffix += 1


def summarize_log(log_path, warmup_s, gear_ratio, wheel_radius):
    rows = []
    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            if line.startswith("%"):
                continue
            fields = line.split()
            if len(fields) < 23:
                continue
            try:
                values = [float(value) for value in fields[:23]]
            except ValueError:
                continue
            rows.append(values)

    driving = [row for row in rows if int(row[1]) == 2 and row[2] > 0]
    if not driving:
        raise RuntimeError(f"No positive-reference Driving samples in {log_path}.")
    drive_start = driving[0][0]
    scored = [row for row in driving if row[0] >= drive_start + warmup_s]
    if len(scored) < 2:
        raise RuntimeError(f"Not enough samples after the {warmup_s:g}s warmup in {log_path}.")

    def speeds(row):
        scale = wheel_radius / gear_ratio
        raw = (row[7] + row[8]) * 0.5 * scale
        filtered = (row[21] + row[22]) * 0.5 * scale
        return raw, filtered

    raw_errors = [speeds(row)[0] - row[2] for row in scored]
    filtered_speeds = [speeds(row)[1] for row in scored]
    raw_speeds = [speeds(row)[0] for row in scored]
    rmse = math.sqrt(sum(error * error for error in raw_errors) / len(raw_errors))
    mean_error = sum(raw_errors) / len(raw_errors)
    raw_mean = sum(raw_speeds) / len(raw_speeds)
    filtered_mean = sum(filtered_speeds) / len(filtered_speeds)
    raw_std = math.sqrt(sum((value - raw_mean) ** 2 for value in raw_speeds) / len(raw_speeds))
    filtered_std = math.sqrt(sum((value - filtered_mean) ** 2 for value in filtered_speeds) / len(filtered_speeds))
    voltage_steps = []
    for previous, current in zip(scored, scored[1:]):
        voltage_steps.extend((current[3] - previous[3], current[4] - previous[4]))
    voltage_step_rms = math.sqrt(sum(step * step for step in voltage_steps) / len(voltage_steps))

    settling_s = None
    settling_window_s = 0.1
    for index, row in enumerate(driving):
        window_end = row[0] + settling_window_s
        window = []
        for candidate in driving[index:]:
            if candidate[0] > window_end:
                break
            window.append(candidate)
        if not window or window[-1][0] < window_end - 0.002:
            continue
        if all(abs(speeds(candidate)[1] - candidate[2]) <= 0.1 * candidate[2] for candidate in window):
            settling_s = row[0] - drive_start
            break

    return {
        "raw_speed_rmse_mps": rmse,
        "mean_speed_error_mps": mean_error,
        "raw_speed_std_mps": raw_std,
        "filtered_speed_std_mps": filtered_std,
        "voltage_step_rms_v": voltage_step_rms,
        "settling_time_s": "" if settling_s is None else settling_s,
        "scored_samples": len(scored),
    }


def choose_sweep_winner(results, rmse_tolerance):
    best_rmse = min(result["raw_speed_rmse_mps"] for result in results)
    allowed_rmse = best_rmse * (1.0 + rmse_tolerance) + 1e-6
    eligible = [result for result in results if result["raw_speed_rmse_mps"] <= allowed_rmse]
    return min(eligible, key=lambda result: result["alpha"])


def write_sweep_summary(output_dir, results, rmse_tolerance):
    columns = ["alpha", "raw_speed_rmse_mps", "mean_speed_error_mps", "raw_speed_std_mps",
               "filtered_speed_std_mps", "voltage_step_rms_v", "settling_time_s",
               "scored_samples", "selected"]
    winner = choose_sweep_winner(results, rmse_tolerance)
    summary_path = output_dir / "summary.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as summary_file:
        writer = csv.DictWriter(summary_file, fieldnames=columns)
        writer.writeheader()
        rows = [
            {**result, "selected": result["alpha"] == winner["alpha"]}
            for result in sorted(results, key=lambda result: result["alpha"])
        ]
        writer.writerows(rows)

    if not PROJECT_PYTHON.is_file():
        raise RuntimeError(f"Project Python executable not found: {PROJECT_PYTHON}")
    plot_code = """
import csv, sys
import matplotlib.pyplot as plt
with open(sys.argv[1], newline='', encoding='utf-8') as source:
    rows = list(csv.DictReader(source))
alphas = [float(row['alpha']) for row in rows]
rmse = [float(row['raw_speed_rmse_mps']) for row in rows]
filtered_std = [float(row['filtered_speed_std_mps']) for row in rows]
voltage_steps = [float(row['voltage_step_rms_v']) for row in rows]
selected = [row['selected'].lower() == 'true' for row in rows]
selected_alpha = next(alpha for alpha, is_selected in zip(alphas, selected) if is_selected)
fig, axes = plt.subplots(2, 1, figsize=(8, 7), constrained_layout=True)
axes[0].plot(alphas, rmse, 'o-')
axes[0].set_ylabel('Raw speed RMSE (m/s)')
axes[0].set_title('Filter alpha sweep')
axes[0].axvline(selected_alpha, color='tab:green', linestyle=':', label=f'Selected alpha={selected_alpha:g}')
axes[0].legend()
axes[0].grid(True, alpha=0.3)
axes[1].plot(alphas, filtered_std, 'o-', label='Filtered speed std (m/s)')
axes[1].set_xlabel('EMA alpha')
axes[1].set_ylabel('Filtered speed std (m/s)')
axes[1].axvline(selected_alpha, color='tab:green', linestyle=':')
voltage_ax = axes[1].twinx()
voltage_ax.plot(alphas, voltage_steps, 's--', color='tab:red', label='Voltage step RMS (V)')
voltage_ax.set_ylabel('Voltage step RMS (V)')
lines = axes[1].get_lines() + voltage_ax.get_lines()
axes[1].legend(lines, [line.get_label() for line in lines])
axes[1].grid(True, alpha=0.3)
fig.savefig(sys.argv[2], dpi=160)
"""
    subprocess.run(
        [str(PROJECT_PYTHON), "-c", plot_code, str(summary_path), str(output_dir / "alpha_sweep_summary.png")],
        check=True,
    )
    return summary_path


def run_alpha_sweep(platformio, name, alphas, warmup_s, rmse_tolerance, requested_port=None):
    gear_ratio, wheel_radius = validate_sweep_config()
    output_dir = allocate_sweep_dir(name)
    results = []

    for index, alpha in enumerate(alphas, 1):
        label = f"{alpha:.8g}".replace(".", "p")
        log_path = output_dir / f"alpha_{label}.log"
        plot_dir = output_dir / f"alpha_{label}_plots"
        print(f"\n[{index}/{len(alphas)}] Testing alpha={alpha:g}", flush=True)
        run_build(platformio, alpha)
        upload_with_retry(platformio, alpha)
        port = choose_port(requested_port)
        capture_run(port, log_path, auto_start=True)
        metrics = summarize_log(log_path, warmup_s, gear_ratio, wheel_radius)
        result = {"alpha": alpha, **metrics}
        results.append(result)
        summary_path = write_sweep_summary(output_dir, results, rmse_tolerance)
        print(
            f"alpha={alpha:g}: raw-speed RMSE={metrics['raw_speed_rmse_mps']:.4f} m/s, "
            f"filtered std={metrics['filtered_speed_std_mps']:.4f} m/s, "
            f"voltage-step RMS={metrics['voltage_step_rms_v']:.3f} V, "
            f"settling={metrics['settling_time_s'] if metrics['settling_time_s'] != '' else 'not reached'}",
            flush=True,
        )
        export_notebook_plots(log_path, plot_dir)

    best = choose_sweep_winner(results, rmse_tolerance)
    summary_path = write_sweep_summary(output_dir, results, rmse_tolerance)
    print(
        f"\nSelected alpha={best['alpha']:g}: lowest alpha within "
        f"{rmse_tolerance:.1%} of minimum raw-speed RMSE "
        f"({best['raw_speed_rmse_mps']:.4f} m/s)."
    )
    print(f"Sweep results: {summary_path}")
    print(f"Comparison plot: {output_dir / 'alpha_sweep_summary.png'}")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Build, upload, capture one robot run, and export analysis plots."
    )
    parser.add_argument("name", nargs="?", help="log name; prompts if omitted")
    parser.add_argument("--port", help="USB serial port, e.g. /dev/ttyACM0")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="start the mission over USB instead of waiting for the physical button",
    )
    parser.add_argument("--sweep-alpha", action="store_true", help="run a closed-loop alpha sweep")
    parser.add_argument("--alphas", default=DEFAULT_ALPHAS, help="comma-separated sweep values")
    parser.add_argument("--warmup", type=float, default=0.25, help="seconds excluded from each driving segment")
    parser.add_argument("--rmse-tolerance", type=float, default=0.05,
                        help="choose the smallest alpha within this fraction of the best RMSE")
    parser.add_argument("--delete", action="store_true", help="delete matching logs and plot folders")
    parser.add_argument("--yes", action="store_true", help="skip the deletion confirmation prompt")
    args = parser.parse_args()

    LOGS.mkdir(exist_ok=True)
    if args.delete:
        if not args.name:
            parser.error("--delete requires a log name")
        return delete_run_artifacts(args.name, confirmed=args.yes)
    if args.yes:
        parser.error("--yes can only be used with --delete")

    if args.sweep_alpha:
        if args.auto_start:
            parser.error("--sweep-alpha already starts each run automatically; omit --auto-start")
        if not math.isfinite(args.warmup) or args.warmup < 0:
            parser.error("--warmup must be a finite, non-negative number")
        if not math.isfinite(args.rmse_tolerance) or args.rmse_tolerance < 0:
            parser.error("--rmse-tolerance must be a finite, non-negative number")
        try:
            alphas = parse_alphas(args.alphas)
            name = args.name or input("Alpha sweep name: ")
            return run_alpha_sweep(
                find_platformio(), name, alphas, args.warmup, args.rmse_tolerance,
                requested_port=args.port,
            )
        except KeyboardInterrupt:
            print("\nSweep interrupted; completed logs and results were preserved.")
            return 130
        except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1

    if args.alphas != DEFAULT_ALPHAS or args.warmup != 0.25 or args.rmse_tolerance != 0.05:
        parser.error("--alphas, --warmup, and --rmse-tolerance require --sweep-alpha")

    name = args.name or input("Log name (without .log): ")
    stem, log_path, plot_dir = allocate_run_paths(name)
    if stem != safe_stem(name):
        print(f"That name already exists; using '{stem}' instead.")

    try:
        platformio = find_platformio()
        run_build(platformio)
        upload_with_retry(platformio)
        port = choose_port(args.port)
        print(f"Using serial port {port}.", flush=True)
        capture_run(port, log_path, auto_start=args.auto_start)
        image_count = export_notebook_plots(log_path, plot_dir)
    except KeyboardInterrupt:
        print(f"\nStopped; any captured data remains at {log_path}.")
        return 130
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"\nLog saved: {log_path}")
    print(f"Exported {image_count} plots to: {plot_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

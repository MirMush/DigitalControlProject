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
NOTEBOOKS = (LOGS / "robot_log_viewer.ipynb",)


def find_jupyter():
    candidates = [
        ROOT / ".venv" / "Scripts" / "jupyter.exe",
        ROOT / ".venv" / "bin" / "jupyter",
    ]
    executable = shutil.which("jupyter")
    if executable:
        candidates.append(Path(executable))
    return next((path for path in candidates if path.is_file()), None)


JUPYTER = find_jupyter()


def find_platformio():
    for command in ("platformio", "pio"):
        executable = shutil.which(command)
        if executable:
            return executable

    platformio_env = Path.home() / ".platformio" / "penv"
    for executable in (
        platformio_env / "Scripts" / "platformio.exe",
        platformio_env / "Scripts" / "pio.exe",
        platformio_env / "bin" / "platformio",
        platformio_env / "bin" / "pio",
    ):
        if executable.is_file():
            return str(executable)
    raise RuntimeError("PlatformIO was not found; install it or add it to PATH.")


def add_platformio_python_packages(platformio):
    executable = Path(platformio).resolve()
    if executable.parent.name.lower() in ("scripts", "bin"):
        environment = executable.parent.parent
        site_packages = environment / "Lib" / "site-packages"
        if site_packages.is_dir() and str(site_packages) not in sys.path:
            sys.path.insert(0, str(site_packages))
            importlib.invalidate_caches()


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


def allocate_plot_dir(name):
    stem = safe_stem(name)
    suffix = 1
    while True:
        candidate = stem if suffix == 1 else f"{stem}_{suffix}"
        plot_dir = LOGS / f"{candidate}_plots"
        if not plot_dir.exists():
            return plot_dir
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


def run_build(platformio):
    command = [platformio, "run", "--environment", "teensy41"]
    print("Building Teensy 4.1 firmware", flush=True)
    subprocess.run(command, cwd=ROOT, check=True)


def upload_with_retry(platformio):
    command = [platformio, "run", "--target", "upload", "--environment", "teensy41"]
    for attempt in range(2):
        result = subprocess.run(command, cwd=ROOT)
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
        with log_path.open("x", encoding="utf-8", newline="") as output:
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


def capture_buffered_run(port, log_path):
    try:
        serial = importlib.import_module("serial")
    except ImportError as exc:
        raise RuntimeError("PySerial is unavailable in the PlatformIO Python environment.") from exc

    print(f"Downloading the robot's buffered run to {log_path}", flush=True)
    print("Keep the robot powered; this command does not build or upload firmware.", flush=True)
    received = bytearray()
    footer_match = None
    with serial.Serial(port, baudrate=12000000, timeout=1) as connection:
        time.sleep(0.2)
        connection.reset_input_buffer()
        connection.write(b"log\n")
        connection.flush()
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            chunk = connection.read(4096)
            if not chunk:
                continue
            received.extend(chunk)
            footer_match = re.search(rb"% sent (\d+) log lines\r?\n", received)
            if footer_match:
                break

    if footer_match is None:
        raise RuntimeError(
            "Timed out waiting for the buffered log. Check that the run finished "
            "and the robot is still powered."
        )

    log_bytes = bytes(received[:footer_match.end()])
    with log_path.open("xb") as output:
        output.write(log_bytes)

    expected_rows = int(footer_match.group(1))
    maximum_match = re.search(rb"% maximum log samples (\d+)", log_bytes)
    maximum_samples = int(maximum_match.group(1)) if maximum_match else None
    log_lines = log_bytes.decode("utf-8", errors="replace").splitlines()
    sample_lines = [
        line for line in log_lines
        if line.strip() and not line.lstrip().startswith("%")
    ]
    valid_rows = 0
    malformed_rows = 0
    for line in sample_lines:
        fields = line.split()
        if len(fields) != 22:
            malformed_rows += 1
            continue
        try:
            float(fields[0])
        except ValueError:
            continue
        valid_rows += 1

    if expected_rows == 0 or valid_rows == 0:
        raise RuntimeError(f"The robot returned no buffered samples; see {log_path}.")
    if valid_rows != expected_rows:
        raise RuntimeError(
            f"The robot reported {expected_rows} samples, but only {valid_rows} "
            f"valid rows were downloaded ({malformed_rows} malformed row(s)). "
            f"The file is at {log_path}; keep the robot powered and retry the download."
        )
    if maximum_samples is not None and expected_rows >= maximum_samples:
        print(
            "Warning: the log reached the robot's RAM buffer capacity; "
            "later samples may have been dropped.",
            flush=True,
        )
    print(f"Downloaded {valid_rows} samples to {log_path}.", flush=True)


def export_notebook_plots(log_path, plot_dir):
    notebook_path = next((path for path in NOTEBOOKS if path.is_file()), None)
    if notebook_path is None:
        checked = ", ".join(str(path) for path in NOTEBOOKS)
        raise RuntimeError(f"Analysis notebook not found. Checked: {checked}")
    if JUPYTER is None:
        raise RuntimeError(
            "Jupyter was not found. Open the notebook in VS Code to view plots, "
            "or install Jupyter in the project environment to export plots."
        )

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


def main():
    parser = argparse.ArgumentParser(
        description="Capture robot logs and export analysis plots."
    )
    parser.add_argument("name", nargs="?", help="log name; prompts if omitted")
    parser.add_argument(
        "--plot",
        action="store_true",
        help="plot an existing log by name instead of building and capturing a run",
    )
    parser.add_argument(
        "--download",
        action="store_true",
        help="download the robot's last RAM-buffered run without uploading firmware",
    )
    parser.add_argument("--port", help="USB serial port, e.g. COM9 or /dev/ttyACM0")
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="start the mission over USB instead of waiting for the physical button",
    )
    parser.add_argument("--delete", action="store_true", help="delete matching logs and plot folders")
    parser.add_argument("--yes", action="store_true", help="skip the deletion confirmation prompt")
    args = parser.parse_args()

    LOGS.mkdir(exist_ok=True)
    if args.download and (args.plot or args.auto_start or args.delete or args.yes):
        parser.error("--download cannot be combined with --plot, --auto-start, or --delete")
    if args.delete:
        if not args.name:
            parser.error("--delete requires a log name")
        return delete_run_artifacts(args.name, confirmed=args.yes)
    if args.yes:
        parser.error("--yes can only be used with --delete")

    if args.download:
        if not args.name:
            parser.error("--download requires a log name")
        stem, log_path, plot_dir = allocate_run_paths(args.name)
        if stem != safe_stem(args.name):
            print(f"That name already exists; using '{stem}' instead.")
        try:
            platformio = find_platformio()
            add_platformio_python_packages(platformio)
            port = choose_port(args.port)
            print(f"Using serial port {port}.", flush=True)
            capture_buffered_run(port, log_path)
            if JUPYTER is None:
                print(f"\nLog saved: {log_path}")
                print(
                    "Plot export skipped: Jupyter was not found. "
                    "Open the notebook in VS Code to view plots."
                )
                return 0
            image_count = export_notebook_plots(log_path, plot_dir)
        except KeyboardInterrupt:
            print(f"\nInterrupted; any downloaded data remains at {log_path}.")
            return 130
        except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1

        print(f"\nLog saved: {log_path}")
        print(f"Exported {image_count} plots to: {plot_dir}")
        return 0

    if args.plot:
        if not args.name:
            parser.error("--plot requires a log name")
        if args.port or args.auto_start:
            parser.error("--plot cannot be combined with capture options")
        try:
            stem = safe_stem(args.name)
            log_path = LOGS / f"{stem}.log"
            if not log_path.is_file():
                raise RuntimeError(f"Log file not found: {log_path}")
            plot_dir = allocate_plot_dir(stem)
            image_count = export_notebook_plots(log_path, plot_dir)
        except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1

        print(f"Exported {image_count} plots to: {plot_dir}")
        return 0

    name = args.name or input("Log name (without .log): ")
    stem, log_path, plot_dir = allocate_run_paths(name)
    if stem != safe_stem(name):
        print(f"That name already exists; using '{stem}' instead.")

    try:
        platformio = find_platformio()
        add_platformio_python_packages(platformio)
        run_build(platformio)
        upload_with_retry(platformio)
        port = choose_port(args.port)
        print(f"Using serial port {port}.", flush=True)
        capture_run(port, log_path, auto_start=args.auto_start)
        if JUPYTER is None:
            print(f"\nLog saved: {log_path}")
            print(
                "Plot export skipped: Jupyter was not found. "
                "Open the notebook in VS Code to view plots."
            )
            return 0
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

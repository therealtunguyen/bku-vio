#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def read_text(path: str) -> str:
    file_path = Path(path)
    if not file_path.is_file():
        return ""
    return file_path.read_text(encoding="utf-8", errors="replace").strip()


def run_command(command: list[str]) -> dict[str, object]:
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    return {
        "command": command,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }


def probe_module_version(module_name: str) -> dict[str, object]:
    return run_command(
        [
            sys.executable,
            "-c",
            (
                "import importlib; "
                "module = importlib.import_module(%r); "
                "print(getattr(module, '__version__', ''))"
            )
            % module_name,
        ]
    )


def read_temperatures() -> dict[str, float]:
    temperatures: dict[str, float] = {}
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        try:
            temperatures[str(path)] = float(
                path.read_text(encoding="utf-8", errors="replace").strip()
            ) / 1000.0
        except ValueError:
            continue
    return temperatures


def main() -> int:
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("rpi5_facts.json")
    output.parent.mkdir(parents=True, exist_ok=True)

    payload = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "hostname": platform.node(),
        "machine": platform.machine(),
        "platform": platform.platform(),
        "python": sys.version,
        "ros_distro": os.environ.get("ROS_DISTRO", ""),
        "os_release": read_text("/etc/os-release"),
        "cpuinfo": read_text("/proc/cpuinfo"),
        "meminfo": read_text("/proc/meminfo"),
        "temperatures_c": read_temperatures(),
        "numpy": probe_module_version("numpy"),
        "opencv": probe_module_version("cv2"),
    }
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(f"platform_facts={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

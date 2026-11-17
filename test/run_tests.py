"""Build and execute the firmware's portable model without Arduino or third-party packages."""

import os
from pathlib import Path
import shutil
import subprocess
import sys


def find_compiler():
    """Prefer CXX, then PATH, then the usual local toolchain locations."""
    candidate = os.environ.get("CXX")
    if candidate:
        return candidate
    found = shutil.which("g++") or shutil.which("clang++") or shutil.which("c++")
    if found:
        return found
    for path in (
        Path("C:/msys64/mingw64/bin/g++.exe"),
        Path("C:/MinGW/bin/g++.exe"),
    ):
        if path.is_file():
            return str(path)
    return None


def main():
    project = Path(__file__).resolve().parent.parent
    compiler = find_compiler()
    if compiler is None:
        print(
            "No C++11 compiler found. Install one, add it to PATH, "
            "or set CXX to the compiler executable.",
            file=sys.stderr,
        )
        return 1

    output = project / ".pio" / "host-tests"
    output.mkdir(parents=True, exist_ok=True)
    common = [compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic"]
    if os.name == "nt":
        common.append("-static")
    common += ["-I", str(project / "include")]
    targets = [
        ("monitor_model_tests", [str(project / "src" / "monitor_model.cpp"),
                                 str(project / "test" / "test_monitor_model.cpp")]),
        ("cache_ring_tests", [str(project / "test" / "test_cache_ring.cpp")]),
        ("vibration_synthetic_tests", ["-I", str(project / "test" / "vibration_source_stubs"),
                                       str(project / "src" / "vibration_source.cpp"),
                                       str(project / "test" / "test_vibration_source.cpp")]),
        ("vibration_mpu6050_tests", ["-DTERMINAL_USE_MPU6050=1", "-I", str(project / "test" / "vibration_source_stubs"),
                                     str(project / "src" / "vibration_source.cpp"),
                                     str(project / "test" / "test_vibration_source.cpp")]),
    ]
    for name, arguments in targets:
        executable = output / (name + (".exe" if os.name == "nt" else ""))
        print(f"Building {name}", flush=True)
        subprocess.run(common + arguments + ["-o", str(executable)], cwd=project, check=True)
        subprocess.run([str(executable)], cwd=project, check=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        sys.exit(1)

"""Is this laptop ready for the course?

    python tools/check_env.py

REQUIRED items must pass before week 2. OPTIONAL items are needed later, or only for
the simulator and the real arm; the line says which module first needs them.
Exit code 1 if anything REQUIRED is missing.
"""
import importlib
import os
import shutil
import socket
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT)

results = []


def report(kind, ok, name, detail):
    results.append((kind, ok))
    mark = "ok     " if ok else ("MISSING" if kind == "required" else "later  ")
    print("%s  %-9s %-22s %s" % (mark, kind, name, detail))


def module_version(name):
    try:
        module = importlib.import_module(name)
    except Exception as problem:                       # a broken install is also "missing"
        return None, type(problem).__name__
    return module, str(getattr(module, "__version__", "installed"))


def port_open(host, port, timeout=1.0):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def main():
    version = sys.version_info
    report("required", version >= (3, 9), "python", "%d.%d.%d (need 3.9 or newer)" % version[:3])

    module, detail = module_version("pytest")
    report("required", module is not None, "pytest", detail if module else "pip install pytest")

    try:
        from common.mock_robot import MockRobot
        from common.robot_io import connect             # noqa: F401
        pose = MockRobot().getActualTCPPose()
        report("required", len(pose) == 6, "course common/", "mock robot answers, TCP z = %.3f m" % pose[2])
    except Exception as problem:
        report("required", False, "course common/", "cannot import: %s. Run this from the course folder." % problem)

    for name, pip_name, first_needed in [
        ("rtde_control", "ur_rtde", "Module 2, simulator and real arm"),
        ("PyQt6", "PyQt6", "Week 4, windows with buttons"),
        ("PySide6", "PySide6-Essentials", "Week 4, the Qt Designer program"),
        ("cv2", "opencv-python", "Module 6"),
        ("torch", "torch", "Module 7"),
    ]:
        module, detail = module_version(name)
        report("optional", module is not None, pip_name,
               detail if module else "pip install %s   (first needed: %s)" % (pip_name, first_needed))

    docker = shutil.which("docker")
    report("optional", docker is not None, "docker", docker or "install Docker Desktop (first needed: Module 2, simulator)")

    running = port_open("127.0.0.1", 29999)
    report("optional", running, "simulator", "URSim answers on port 29999" if running
           else "not running. Start it: see twin/README.md")

    missing = [1 for kind, ok in results if kind == "required" and not ok]
    print()
    if missing:
        print("NOT READY: fix the MISSING lines above, then run this again.")
        return 1
    print("READY for the mock robot. Lines marked later are for coming modules.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

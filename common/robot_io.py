"""One way to connect, whatever is on the other end.

    from common.robot_io import connect
    with connect() as robot:
        print(robot.receive.getActualTCPPose())

The backend comes from the environment, so the same script runs everywhere:

    ROBOT=mock     (default) nothing installed, nothing moves
    (nothing)      the simulator if it is running on this PC, otherwise the mock
    ROBOT=ursim    Universal Robots' simulator, ROBOT_IP defaults to 127.0.0.1
    ROBOT=real     a real arm; ROBOT_IP is required

The real-robot path below is not tidy-looking for fun. Every step in it exists
because its absence cost us an afternoon on real hardware; the comments say which.
"""
import os
import socket
import threading
import time

from .mock_robot import MockRobot


def dashboard(ip, *commands, timeout=4.0):
    """Send commands to the UR dashboard server (port 29999); return the replies."""
    replies = []
    with socket.create_connection((ip, 29999), timeout=timeout) as sock:
        sock.settimeout(3.0)
        try:
            sock.recv(4096)                           # welcome banner
        except socket.timeout:
            pass
        for command in commands:
            sock.sendall((command + "\n").encode())
            time.sleep(0.35)
            try:
                replies.append(sock.recv(4096).decode(errors="replace").strip())
            except socket.timeout:
                replies.append("")
    return replies


def clear_stale_control_script(ip):
    """Stop an orphaned control script, if one is running. Returns True if it did.

    Closing an RTDE control connection does NOT stop the script it uploaded; the
    script keeps running on the controller with nobody attached. We measured what
    that does to the next connection: a leftover script on the default register
    range is harmless, one on the upper range makes every later connect hang.
    The fingerprint is "Program running: true" while the loaded program is
    STOPPED. Only that case is stopped, so a program genuinely playing from the
    pendant is left alone.
    """
    try:
        running, state = dashboard(ip, "running", "programState")
    except OSError as error:
        print("[robot_io] dashboard unavailable (%s); skipping stale-script check" % error)
        return False
    if "true" in running.lower() and state.upper().startswith("STOPPED"):
        dashboard(ip, "stop")
        time.sleep(0.8)
        print("[robot_io] stopped an orphaned control script before connecting")
        return True
    return False


def _construct_control(ip, flags, timeout_s=12.0):
    """RTDEControlInterface(...) with a hard timeout.

    The constructor can wait forever for its control script to report in, and it
    has no timeout of its own. So it runs on a helper thread; if it has not
    returned in time, the half-started script is stopped through the dashboard,
    which unblocks it, and a real error is raised instead of a frozen program.
    """
    from rtde_control import RTDEControlInterface
    box = {}

    def build():
        try:
            box["control"] = RTDEControlInterface(ip, flags=flags)
        except Exception as error:                    # noqa: BLE001 - re-raised below
            box["error"] = error

    worker = threading.Thread(target=build, daemon=True)
    worker.start()
    worker.join(timeout_s)
    if worker.is_alive():
        try:
            dashboard(ip, "stop")
        except OSError:
            pass
        worker.join(5.0)
        raise RuntimeError("the RTDE control script did not start within %.0f s; a stale "
                           "script was probably blocking it and has been stopped - try again"
                           % timeout_s)
    if "error" in box:
        raise box["error"]
    return box["control"]


def open_control(ip):
    """Open RTDE control: default register range first, upper range only if forced.

    The upper range exists for controllers where something else (a gripper URCap,
    a fieldbus) already owns the default registers. It is a fallback, not a
    preference: it is the range whose leftovers block later connections.
    """
    from rtde_control import RTDEControlInterface
    clear_stale_control_script(ip)
    try:
        return _construct_control(ip, RTDEControlInterface.FLAG_UPLOAD_SCRIPT)
    except Exception as first:                        # noqa: BLE001
        if "already in use" not in str(first).lower():
            raise
        print("[robot_io] default registers in use; trying the upper range")
        return _construct_control(ip, RTDEControlInterface.FLAG_UPLOAD_SCRIPT
                                  | RTDEControlInterface.FLAG_UPPER_RANGE_REGISTERS)


class Robot:
    """A connected robot: `.control` to command it, `.receive` to read it.

    `.lock` is here on purpose. The control interface is not thread-safe: two
    threads inside it at once crash the whole Python process with no traceback.
    Any code that uses `.control` from more than one thread must hold `.lock`.
    """

    def __init__(self, control, receive, backend, ip=None):
        self.control, self.receive = control, receive
        self.backend, self.ip = backend, ip
        self.lock = threading.Lock()

    def close(self):
        """Stop the control script, THEN disconnect - never just the socket."""
        with self.lock:
            for step in ("stopScript", "disconnect"):
                try:
                    getattr(self.control, step)()
                except Exception as error:            # noqa: BLE001 - closing must not raise
                    print("[robot_io] %s: %s" % (step, error))
        if self.receive is not self.control:
            try:
                self.receive.disconnect()
            except Exception as error:                # noqa: BLE001
                print("[robot_io] receive disconnect: %s" % error)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def simulator_is_running(ip="127.0.0.1", timeout=0.5):
    """True when the URSim dashboard answers on this PC."""
    try:
        with socket.create_connection((ip, 29999), timeout=timeout):
            return True
    except OSError:
        return False


def connect(backend=None, ip=None, model="UR5e"):
    """Connect to a robot.

    Nothing set: the simulator if it is running on this PC, otherwise the fake robot.
    ROBOT=mock forces the fake robot. ROBOT=real with ROBOT_IP is the lab arm.
    """
    backend = (backend or os.environ.get("ROBOT") or "").lower()
    if not backend:
        backend = "ursim" if simulator_is_running() else "mock"
    ip = ip or os.environ.get("ROBOT_IP") or ("127.0.0.1" if backend == "ursim" else None)

    if backend == "mock":
        mock = MockRobot(model=model)
        return Robot(mock, mock, "mock")
    if backend not in ("ursim", "real"):
        raise ValueError("ROBOT must be mock, ursim or real - got %r" % backend)
    if not ip:
        raise ValueError("ROBOT=real needs ROBOT_IP set to the arm's address")

    from rtde_receive import RTDEReceiveInterface
    control = open_control(ip)
    return Robot(control, RTDEReceiveInterface(ip), backend, ip)

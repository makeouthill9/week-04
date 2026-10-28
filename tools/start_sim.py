"""Start the UR simulator (URSim) for this course.

    python tools/start_sim.py            # start it (creates it the first time)
    python tools/start_sim.py stop       # stop it

What it runs for you, the first time:

    docker run -d --name ursim_course -e ROBOT_MODEL=UR5
        -p 5900:5900 -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004
        universalrobots/ursim_e-series:5.24.0

  -d                 run in the background
  --name             so we can find it again later
  ROBOT_MODEL=UR5    simulate a UR5e, like four of the lab arms
  -p 6080            the pendant screen, in your browser
  -p 29999           the dashboard (power on, play, stop)
  -p 30001-30004     the ports your Python code talks to
  :5.24.0            the same controller software version as the lab robots
"""
import shutil
import socket
import subprocess
import sys
import time

NAME = "ursim_course"
IMAGE = "universalrobots/ursim_e-series:5.24.0"
PENDANT = "http://localhost:6080/vnc.html"


def docker(*args):
    return subprocess.run(["docker"] + list(args), capture_output=True, text=True)


def port_open(port):
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1.0):
            return True
    except OSError:
        return False


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if shutil.which("docker") is None:
        print("Docker is not installed. Install Docker Desktop, start it, then run this again.")
        return 1
    if docker("info").returncode != 0:
        print("Docker is installed but not running. Start Docker Desktop, wait for it to say")
        print("'Engine running', then run this again.")
        return 1

    if args[:1] == ["stop"]:
        docker("stop", NAME)
        print("simulator stopped")
        return 0

    state = docker("inspect", "-f", "{{.State.Running}}", NAME)
    if state.returncode != 0:
        print("first start: downloading the simulator (about 2 GB, once) ...")
        made = subprocess.run(["docker", "run", "-d", "--name", NAME, "-e", "ROBOT_MODEL=UR5",
                               "-p", "5900:5900", "-p", "6080:6080", "-p", "29999:29999",
                               "-p", "30001-30004:30001-30004", IMAGE])
        if made.returncode != 0:
            print("docker could not start the simulator. Read the message above.")
            return 1
    elif state.stdout.strip() != "true":
        docker("start", NAME)
        print("simulator restarted")
    else:
        print("simulator is already running")

    print("waiting for the robot controller to boot ", end="", flush=True)
    for _ in range(90):
        if port_open(29999):
            break
        print(".", end="", flush=True)
        time.sleep(1)
    print()
    if not port_open(29999):
        print("the controller did not answer after 90 s. Try: python tools/start_sim.py stop, then start again.")
        return 1

    print("READY. Now do this by hand, in your browser:")
    print("  1. open  " + PENDANT + "   and press Connect")
    print("  2. if you see 'Confirmation of applied Safety Configuration' -> press Confirm")
    print("  3. red button bottom-left -> ON -> START, until it says Normal (green)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

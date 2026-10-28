"""The simple robot for the first weeks. One object, a few verbs, every move checked.

    from common.easy import connect
    robot = connect()        # simulator if it is running, otherwise the fake robot
    robot.describe()         # print where it is
    robot.home()             # go to the home pose
    robot.move_up(3)         # 3 cm up, straight line
    robot.move_down(3)       # 3 cm down
    robot.move_to(-133, -492, 400)                    # a point in mm from the base, like the pendant
    robot.path([(..), (..), (..)], blend_cm=2)        # through several points without stopping
    z = robot.touch_down()   # real arm: down slowly until it feels the table, returns the height
    robot.stop_test(10)      # how far the tool travels after STOP at 10 cm/s, measured in mm

For windows (week 4) three verbs start a move and return AT ONCE with a message, never an error:
    robot.jog(z_cm=1)        # "moving z +1 cm" or "outside the box: not moved"
    robot.go_home()          # "going home"
    robot.stop()             # "stopped"
    robot.close()            # stop the program on the robot, disconnect (also runs by itself at exit)

Every move does the four steps from the slides: safe? send. accepted? really there?
If any step fails it stops with a clear message. Distances are in CENTIMETRES here,
because that is what people say; the robot library underneath uses metres.
"""
import atexit
import math
import threading
import time

from . import robot_io
from .mock_robot import HOME_Q
from .safety import WorkspaceBox, verify_tcp

NORMAL = 1
SPEED = 0.05          # metres per second for straight moves
ACCEL = 0.3
CLOSE_ENOUGH_M = 0.002


class EasyRobot:
    def __init__(self, inner):
        self._r = inner
        self.name = inner.backend
        self._box = None
        self._speed = SPEED
        self._tol = CLOSE_ENOUGH_M
        atexit.register(self.close)

    # ---- eyes
    def position_mm(self):
        """Tool position [x, y, z] in millimetres from the robot base."""
        return [round(v * 1000) for v in self._r.receive.getActualTCPPose()[:3]]

    def joints_deg(self):
        return [round(math.degrees(q)) for q in self._r.receive.getActualQ()]

    def is_safe(self):
        return self._r.receive.getSafetyMode() == NORMAL

    def describe(self):
        print("robot     :", self.name)
        print("safety ok :", self.is_safe())
        print("joints deg:", self.joints_deg())
        print("tool mm   :", self.position_mm())

    # ---- hands
    def home(self):
        """Joint move to the home pose, then check the joints really are there."""
        self._must_be_safe()
        with self._r.lock:
            ok = self._r.control.moveJ(list(HOME_Q), 0.8, 0.8)
        if not ok:
            raise RuntimeError("STOPPED: the robot refused to move home")
        worst = max(abs(a - b) for a, b in zip(self._r.receive.getActualQ(), HOME_Q))
        if worst > 0.01:
            raise RuntimeError("STOPPED: the robot is not at home after the move")

    def move_up(self, cm):
        self._move_by(0, 0, cm)

    def move_down(self, cm):
        self._move_by(0, 0, -cm)

    def move_x(self, cm):
        self._move_by(cm, 0, 0)

    def move_y(self, cm):
        self._move_by(0, cm, 0)

    def move_to(self, x_mm, y_mm, z_mm):
        """Straight move to a point given in millimetres from the robot base, like the pendant shows."""
        target = self._r.receive.getActualTCPPose()
        target[0], target[1], target[2] = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
        self._move_to_pose(target, label=[x_mm, y_mm, z_mm])

    def path(self, points, blend_cm=2):
        """Flow through several points without stopping at each one.

        points: a list of (x, y, z) in millimetres from the base. blend_cm: how early the
        robot may start the next segment, so it rounds the corner instead of stopping in it.
        Every point is checked against the box BEFORE anything is sent; the whole path goes
        to the robot as ONE command; then the last point is checked like any move_to.
        """
        points = [tuple(p) for p in points]
        if len(points) < 2:
            raise RuntimeError("STOPPED: a path needs at least two points")
        if not 0 <= blend_cm <= 10:
            raise RuntimeError("STOPPED: blend must be between 0 and 10 cm")
        self._must_be_safe()                                             # 1. safe?
        here = self._r.receive.getActualTCPPose()
        rows, prev = [], here[:3]
        for k, (x_mm, y_mm, z_mm) in enumerate(points):
            pose = list(here)
            pose[0], pose[1], pose[2] = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
            if self._box is not None:                                    # 1b. every point inside the box?
                ok, why = self._box.check(pose)
                if not ok:
                    raise RuntimeError("STOPPED: point %d of the path, %s, is outside the box: %s" % (k + 1, [x_mm, y_mm, z_mm], why))
            seg = math.sqrt(sum((pose[i] - prev[i]) ** 2 for i in range(3)))
            blend = 0.0 if k == len(points) - 1 else min(blend_cm / 100.0, seg * 0.45)   # a corner cannot be rounder than its sides
            rows.append(pose + [self._speed, ACCEL, blend])
            prev = pose[:3]
        print("path: %d points, blend %.0f cm, %.0f cm/s" % (len(points), blend_cm, self._speed * 100))
        with self._r.lock:                                               # 2. send, all at once
            ok = self._r.control.moveL(rows)
        if not ok:                                                       # 3. accepted?
            raise RuntimeError("STOPPED: the robot refused the path")
        now = self._r.receive.getActualTCPPose()                         # 4. really at the last point?
        off = math.sqrt(sum((now[i] - rows[-1][i]) ** 2 for i in range(3)))
        if off > self._tol:
            raise RuntimeError("STOPPED: did not arrive at the end of the path, %.1f mm away" % (off * 1000))
        print("arrived: %.1f mm from the last point" % (off * 1000))

    # ---- for windows (week 4): start the move, do not wait, never crash, return a message
    def jog(self, x_cm=0, y_cm=0, z_cm=0):
        """Start a short straight move from where the tool is now, and return at once with a message.

        The window stays alive while the arm moves, so STOP can always be clicked.
        Nothing raises: a refused move comes back as a message for the window to show.
        """
        if not self.is_safe():
            return "not in Normal mode: not moved"
        target = self._r.receive.getActualTCPPose()
        last = getattr(self, "_jog_target", None)
        if last is not None and math.sqrt(sum((last[i] - target[i]) ** 2 for i in range(3))) > 0.001:
            target[:3] = last[:3]                      # the last click is not finished: add to its target
        target[0] += x_cm / 100.0
        target[1] += y_cm / 100.0
        target[2] += z_cm / 100.0
        if self._box is not None and not self._box.check(target)[0]:
            return "outside the box: not moved"
        with self._r.lock:
            ok = self._r.control.moveL(target, self._speed, ACCEL, True)   # True: start, do not wait
        self._last, self._jog_target = "L", target
        if not ok:
            return "the robot refused the move"
        moved = ["%s %+g cm" % (a, v) for a, v in (("x", x_cm), ("y", y_cm), ("z", z_cm)) if v]
        return "moving " + ", ".join(moved)

    def go_home(self):
        """Start the joint move to home and return at once with a message (home() waits, this does not)."""
        if not self.is_safe():
            return "not in Normal mode: not moved"
        with self._r.lock:
            ok = self._r.control.moveJ(list(HOME_Q), 0.8, 0.8, True)
        self._last, self._jog_target = "J", None
        return "going home" if ok else "the robot refused to go home"

    def stop(self):
        """Brake now, whatever is moving. Returns a message."""
        self._jog_target = None
        with self._r.lock:
            if getattr(self, "_last", "L") == "J":
                self._r.control.stopJ(2.0)      # a joint move is braked joint by joint
            else:
                self._r.control.stopL(2.0)      # a straight move is braked along its line
        return "stopped"

    def stop_test(self, cm_per_s, by_hand=False, brake=2.0):
        """How far does the tool travel after STOP? Measured, in mm.

        The arm moves sideways (towards -x) at this speed. After 6 cm, when it is at full speed:
          by_hand=False  the program sends the stop itself;
          by_hand=True   NOW is printed and YOU press Enter; your reaction time counts too.
        The stop is stopL(brake), brake in m/s^2. The distance is read from the robot, from the
        moment of NOW to the moment the arm stands still. Then the arm goes back to where it started.
        Measured on URSim 5.24 with brake=2: 5 cm/s 1.3 mm, 10 cm/s 5 mm, 20 cm/s 20 mm.
        """
        if not 0 < cm_per_s <= 25:
            raise RuntimeError("STOPPED: speed must be between 0 and 25 cm/s")
        self._must_be_safe()
        start = self._r.receive.getActualTCPPose()
        far = list(start)
        far[0] -= 0.25                                                   # the furthest it may ever go
        if self._box is not None and not self._box.check(far)[0]:
            raise RuntimeError("STOPPED: no room for the stop test, it needs 25 cm free towards -x inside the box")
        v = cm_per_s / 100.0

        def travelled(p, q):
            return math.sqrt(sum((p[i] - q[i]) ** 2 for i in range(3)))

        pressed = threading.Event()
        with self._r.lock:
            ok = self._r.control.moveL(far, v, 1.0, True)               # asynchronous: returns at once, the arm keeps going
        if not ok:
            raise RuntimeError("STOPPED: the robot refused to move")
        try:
            while travelled(self._r.receive.getActualTCPPose(), start) < 0.06:   # get up to speed
                if self.name != "mock":
                    time.sleep(0.002)
            now_pose = self._r.receive.getActualTCPPose()
            if by_hand:
                print("NOW")
                t_now = time.perf_counter()
                threading.Thread(target=lambda: (input(), pressed.set()), daemon=True).start()
                while not pressed.is_set():
                    if travelled(self._r.receive.getActualTCPPose(), start) >= 0.20:
                        break                                            # nobody pressed: the program stops it
                    time.sleep(getattr(self._r.control, "tick_s", 0.002))   # the fake robot: one tick of real time
                reaction_s = time.perf_counter() - t_now
                at_press = self._r.receive.getActualTCPPose()
        finally:
            with self._r.lock:
                self._r.control.stopL(brake)
        t0 = time.perf_counter()
        while math.sqrt(sum(c * c for c in self._r.receive.getActualTCPSpeed()[:3])) > 0.0005:
            if time.perf_counter() - t0 > 3:
                break
            if self.name != "mock":
                time.sleep(0.002)
        rest = self._r.receive.getActualTCPPose()
        total = travelled(rest, now_pose) * 1000
        if not by_hand:
            print("stop test %2.0f cm/s, the program stops it: %5.1f mm after the stop" % (cm_per_s, total))
        elif not pressed.is_set():
            print("stop test %2.0f cm/s: nobody pressed Enter, the program stopped it after 14 cm. Press Enter to go on." % cm_per_s)
            pressed.wait()                                               # the reader still waits for its Enter: give it one
        elif reaction_s < 0.08:
            print("stop test %2.0f cm/s: too early, you pressed before you could have seen NOW. Run it again." % cm_per_s)
        else:
            react = travelled(at_press, now_pose) * 1000
            print("stop test %2.0f cm/s, you stop it: %5.1f mm after NOW  (%.0f mm while you reacted in %.2f s, %.0f mm braking)"
                  % (cm_per_s, total, react, reaction_s, total - react))
        back = list(rest)
        back[:3] = start[:3]
        self._move_to_pose(back)
        return round(total, 1)

    def touch_down(self, max_cm=30, newtons=10):
        """Move straight down slowly until the tool FEELS something, then stop. Returns the height in mm.

        How: an asynchronous moveL down at 1 cm/s; while it runs, the wrist force sensor is read
        50 times a second; at `newtons` the move is stopped with stopL.

        Real arm only: the simulator cannot feel, so there it goes down until the box floor
        and stops with a message. Speed 1 cm/s, no matter what speed() was set to.
        """
        self._must_be_safe()
        start = self._r.receive.getActualTCPPose()
        print("touching down: %.0f cm at most, stop at %.0f N" % (max_cm, newtons))
        bottom = list(start)
        bottom[2] -= max_cm / 100.0
        with self._r.lock:
            self._r.control.zeroFtSensor()
            ok = self._r.control.moveL(bottom, 0.01, 0.25, True)          # asynchronous: 1 cm/s down, we watch
        if not ok:
            raise RuntimeError("STOPPED: the robot refused to move")
        felt, why = False, ""
        try:
            while True:
                now = self._r.receive.getActualTCPPose()
                force = self._r.receive.getActualTCPForce()
                down = (start[2] - now[2]) * 100
                if abs(force[2]) >= newtons:
                    felt = True
                    break
                if down >= max_cm - 0.05:
                    why = "went down %.0f cm and felt nothing" % max_cm
                    break
                if self._box is not None and not self._box.check(now)[0]:
                    why = "reached the box floor and felt nothing"
                    break
                if not self.is_safe():
                    why = "the robot left Normal mode"
                    break
                if self.name != "mock":
                    time.sleep(0.02)
        finally:
            with self._r.lock:
                self._r.control.stopL(10.0)
        if not felt:
            raise RuntimeError("STOPPED: " + why)
        z_mm = self.position_mm()[2]
        print("felt it: %.0f N at z = %d mm" % (abs(force[2]), z_mm))
        return z_mm

    # ---- teaching the robot about its world
    def set_tool(self, cm):
        """Tell the robot how long its tool is, and CHECK it listened: the tool tip must move by that much."""
        if cm == 0:
            self._r.control.setTcp([0, 0, 0, 0, 0, 0])
            print("tool length: 0 cm (no tool)")
            return
        ok, moved = verify_tcp(self._r.control, self._r.receive, cm / 100.0, settle_s=0.0 if self.name == "mock" else 0.15)
        if not ok:
            raise RuntimeError("STOPPED: the robot did not apply the tool length (tip moved %.0f mm, expected %.0f)" % (moved * 1000, cm * 10))
        print("tool length: %.1f cm (checked: the tool tip moved %.0f mm)" % (cm, moved * 1000))

    def set_box(self, x, y, z):
        """Where the tool tip MAY go, in millimetres from the base: x=(lo, hi), y=(lo, hi), z=(lo, hi)."""
        limits = {"x": (x[0] / 1000.0, x[1] / 1000.0), "y": (y[0] / 1000.0, y[1] / 1000.0), "z": (z[0] / 1000.0, z[1] / 1000.0)}
        self._box = WorkspaceBox([0, 0, 0, 0, 0, 0], limits)
        print("box set: x %s  y %s  z %s  (mm from the base)" % (list(x), list(y), list(z)))

    def inside_box(self, x_mm, y_mm, z_mm):
        """True when the point is inside the box (or no box is set)."""
        if self._box is None:
            return True
        return self._box.check([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0])[0]

    def speed(self, cm_per_s):
        """Straight-move speed. Never more than 25 cm/s in this course."""
        if not 0 < cm_per_s <= 25:
            raise RuntimeError("STOPPED: speed must be between 0 and 25 cm/s")
        self._speed = cm_per_s / 100.0
        print("speed: %.0f cm/s" % cm_per_s)

    def tolerance(self, mm):
        """How exact a move must be: it counts as arrived when the tool ends within this distance of the target."""
        if not 0.1 <= mm <= 20:
            raise RuntimeError("STOPPED: tolerance must be between 0.1 and 20 mm")
        self._tol = mm / 1000.0
        print("tolerance: %.1f mm (a move counts as arrived within this distance of the target)" % mm)

    def close(self):
        if self._r is not None:
            self._r.close()
            self._r = None

    # ---- inside: the four steps
    def _must_be_safe(self):
        if not self.is_safe():
            raise RuntimeError("STOPPED: the robot is not in Normal mode")

    def _move_by(self, dx_cm, dy_cm, dz_cm):
        target = self._r.receive.getActualTCPPose()
        target[0] += dx_cm / 100.0
        target[1] += dy_cm / 100.0
        target[2] += dz_cm / 100.0
        self._move_to_pose(target)

    def _move_to_pose(self, target, label=None):
        self._must_be_safe()                                             # 1. safe?
        if self._box is not None:                                        # 1b. inside the box?
            ok, why = self._box.check(target)
            if not ok:
                raise RuntimeError("STOPPED: %s is outside the box: %s" % ([round(v * 1000) for v in target[:3]], why))
        if label is not None:
            print("moving to", label, "mm")
        with self._r.lock:                                               # 2. send
            ok = self._r.control.moveL(target, self._speed, ACCEL)
        if not ok:                                                       # 3. accepted?
            raise RuntimeError("STOPPED: the robot refused the move")
        now = self._r.receive.getActualTCPPose()                         # 4. really there?
        off = math.sqrt(sum((now[i] - target[i]) ** 2 for i in range(3)))
        if off > self._tol:
            raise RuntimeError("STOPPED: did not arrive, %.1f mm away" % (off * 1000))
        if label is not None:
            print("arrived: %.1f mm from the target" % (off * 1000))


def connect():
    """The simulator if it is running on this PC, otherwise the fake robot. ROBOT=real for the arm."""
    return EasyRobot(robot_io.connect())

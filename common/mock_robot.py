"""A stand-in for a Universal Robots arm that needs nothing installed.

`MockRobot` offers the parts of the ur_rtde control AND receive interfaces that this
course uses, so the same student code runs against it, against URSim, and against
a real arm. It is deliberately honest about three things real robots do:

  * joint moves go through forward kinematics, so moveJ changes the pose you read back
  * setTcp really shifts the reported TCP along the tool axis (or, with
    honour_tcp=False, silently does not - the failure that put a pencil into our table)
  * it notices two threads calling it at once, which on the real library is a crash

It does not model dynamics, collisions or inverse kinematics.
"""
import math
import threading
import time

# Published UR DH parameters (a, d). Both sets are checked, not just copied:
#   UR3  - against our own UR3: predicted X/Y matched the live robot to under 1 mm.
#   UR5e - against URSim e-Series at HOME_Q: matched to 0.00 mm (see test_common.py).
DH = {
    "UR3":  ([0, -0.24365, -0.21325, 0, 0, 0], [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]),
    "UR5e": ([0, -0.425,   -0.3922,  0, 0, 0], [0.1625, 0, 0, 0.1333,  0.0997,  0.0996]),
}
ALPHA = [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0]
HOME_Q = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]

SAFETY_NORMAL, SAFETY_PROTECTIVE_STOP = 1, 3
ROBOT_MODE_RUNNING = 7


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def forward_kinematics(q, model="UR5e"):
    """Joint angles (rad) -> 4x4 flange pose in the robot base frame."""
    a, d = DH[model]
    t = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for i in range(6):
        ct, st = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(ALPHA[i]), math.sin(ALPHA[i])
        t = _matmul(t, [[ct, -st * ca, st * sa, a[i] * ct],
                        [st, ct * ca, -ct * sa, a[i] * st],
                        [0.0, sa, ca, d[i]],
                        [0.0, 0.0, 0.0, 1.0]])
    return t


def rotvec_from_matrix(r):
    """3x3 rotation -> UR-style rotation vector (axis * angle)."""
    angle = math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0)))
    if angle < 1e-9:
        return [0.0, 0.0, 0.0]
    if abs(angle - math.pi) < 1e-6:                      # 180 deg: read the axis off the diagonal
        ax = [math.sqrt(max(0.0, (r[i][i] + 1.0) / 2.0)) for i in range(3)]
        if r[0][1] < 0:
            ax[1] = -ax[1]
        if r[0][2] < 0:
            ax[2] = -ax[2]
        return [c * angle for c in ax]
    s = 2.0 * math.sin(angle)
    return [(r[2][1] - r[1][2]) / s * angle,
            (r[0][2] - r[2][0]) / s * angle,
            (r[1][0] - r[0][1]) / s * angle]


def matrix_from_rotvec(v):
    """UR-style rotation vector -> 3x3 rotation (Rodrigues formula)."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = (c / angle for c in v)
    c, s, k = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [[c + x * x * k, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, c + y * y * k, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, c + z * z * k]]


class MockRobot:
    """One object that plays both RTDEControlInterface and RTDEReceiveInterface."""

    def __init__(self, model="UR5e", honour_tcp=True, realtime=False):
        if model not in DH:
            raise ValueError("unknown model %r - choose from %s" % (model, sorted(DH)))
        self.model, self.honour_tcp, self.realtime = model, honour_tcp, realtime
        self.calls = []                      # every command, in order
        self.collisions = 0                  # > 0: two threads were inside the robot at once
        self.safety_mode, self.robot_mode = SAFETY_NORMAL, ROBOT_MODE_RUNNING
        self.fail_next_move = False
        self.stop_short = 0.0            # metres: pretend a protective stop halted the arm this far before the target
        self.table_z = 0.0               # metres: a table the tool tip cannot go below; touching it gives a force reading
        self.tick_s = 0.05               # simulated seconds per position read while a speedL is running
        self._vel = [0.0, 0.0, 0.0]
        self._brake = 0.0
        self._goal = None
        self._contact = False
        self._tcp = [0.0] * 6
        self._payload = 0.0
        self._connected, self._script_running = True, True
        self._busy, self._guard = 0, threading.Lock()
        self._q = list(HOME_Q)
        self._set_flange_from_q()

    # ------------------------------------------------------------ internals
    def _enter(self, name, *args):
        with self._guard:
            if self._busy:
                self.collisions += 1
            self._busy += 1
            self.calls.append((name,) + args)
        time.sleep(0.001)                    # hold the object briefly so a race is observable
        with self._guard:
            self._busy -= 1

    def _set_flange_from_q(self):
        t = forward_kinematics(self._q, self.model)
        self._flange_p = [t[0][3], t[1][3], t[2][3]]
        self._flange_r = [row[:3] for row in t[:3]]

    def _move_allowed(self):
        if self.fail_next_move:
            self.fail_next_move = False
            return False
        return self._connected and self.safety_mode == SAFETY_NORMAL

    def _tool_z(self):
        return [self._flange_r[i][2] for i in range(3)]

    def _tcp_offset(self):
        return self._tcp[2] if self.honour_tcp else 0.0

    # ------------------------------------------------------------ control side
    def moveJ(self, q, speed=1.05, acceleration=1.4, asynchronous=False):
        if q and isinstance(q[0], (list, tuple)):                 # path form
            self._enter("moveJ_path", [list(p) for p in q])
            target = list(q[-1][:6])
        else:
            self._enter("moveJ", list(q), speed, acceleration)
            if speed <= 0 or acceleration <= 0:
                raise ValueError("speed and acceleration must be positive")
            target = list(q)
        if not self._move_allowed():
            return False
        self._q = target
        self._set_flange_from_q()
        return True

    def moveL(self, pose, speed=0.25, acceleration=1.2, asynchronous=False):
        if pose and isinstance(pose[0], (list, tuple)):           # path form: rows of 9
            self._enter("moveL_path", [list(p) for p in pose])
            target = list(pose[-1][:6])
        else:
            self._enter("moveL", list(pose), speed, acceleration)
            if speed <= 0 or acceleration <= 0:
                raise ValueError("speed and acceleration must be positive")
            target = list(pose)
        if not self._move_allowed():
            return False
        self._vel, self._brake, self._goal = [0.0, 0.0, 0.0], 0.0, None
        if asynchronous and not (pose and isinstance(pose[0], (list, tuple))):
            here = self._pose()                                   # start moving; every read advances it
            d = [target[i] - here[i] for i in range(3)]
            n = math.sqrt(sum(c * c for c in d))
            if n > 1e-9:
                self._vel = [c / n * speed for c in d]
                self._goal = target[:3]
            return True
        if self.realtime:
            here = self.getActualTCPPose()
            dist = math.sqrt(sum((target[i] - here[i]) ** 2 for i in range(3)))
            time.sleep(min(2.0, dist / max(speed, 1e-6)))
        if self.stop_short:
            target = list(target)
            target[2] -= self.stop_short                          # stopped early, below the target
        self._flange_r = matrix_from_rotvec(target[3:6])
        z, off = self._tool_z(), self._tcp_offset()
        self._flange_p = [target[i] - z[i] * off for i in range(3)]
        return True

    def stopJ(self, a=2.0, asynchronous=False):
        self._enter("stopJ")

    def stopL(self, a=10.0, asynchronous=False):
        """Brake a running asynchronous moveL. Measured on URSim 5.24: after stopL(a) the tool
        travels v*v/a, twice the textbook v*v/2a, so the fake robot brakes at a/2 to match."""
        self._enter("stopL", a)
        if self._vel != [0.0, 0.0, 0.0]:
            self._brake = a / 2.0

    def speedL(self, xd, acceleration=0.25, time=0.0):
        """Start moving the tool at this velocity (m/s, base frame); it keeps going until speedStop."""
        self._enter("speedL", list(xd), acceleration, time)
        if not self._move_allowed():
            return False
        self._vel = list(xd[:3])
        self._brake = 0.0
        return True

    def speedStop(self, a=10.0):
        """Brake to a stop at a m/s^2. The arm keeps travelling while it brakes, like the real one."""
        self._enter("speedStop", a)
        if self._vel != [0.0, 0.0, 0.0]:
            self._brake = a

    def zeroFtSensor(self):
        self._enter("zeroFtSensor")
        return True

    def _tick(self):
        """Advance a running speedL by one tick of simulated time; the tool tip never goes below the table."""
        if self._vel == [0.0, 0.0, 0.0]:
            return
        dt = self.tick_s
        sp = math.sqrt(sum(v * v for v in self._vel))
        if self._brake:
            t = min(dt, sp / self._brake)                              # it may come to rest inside this tick
            new = sp - self._brake * t
            step = [v / sp * (sp + new) / 2 * t for v in self._vel]
            self._vel = [v / sp * new for v in self._vel] if new > 1e-9 else [0.0, 0.0, 0.0]
            if new <= 1e-9:
                self._brake = 0.0
        else:
            step = [v * dt for v in self._vel]
        if self._goal is not None:                                 # an asynchronous moveL ends at its target
            here = self._pose()
            left = math.sqrt(sum((self._goal[i] - here[i]) ** 2 for i in range(3)))
            if math.sqrt(sum(c * c for c in step)) >= left:
                step = [self._goal[i] - here[i] for i in range(3)]
                self._vel, self._brake, self._goal = [0.0, 0.0, 0.0], 0.0, None
        tip_z = self._pose()[2]
        for i in range(3):
            self._flange_p[i] += step[i]
        if tip_z + step[2] <= self.table_z:
            self._flange_p[2] += self.table_z - (tip_z + step[2])        # sit exactly on the table
            self._vel, self._brake, self._goal = [0.0, 0.0, 0.0], 0.0, None
            self._contact = True
        else:
            self._contact = False

    def setTcp(self, tcp):
        self._enter("setTcp", list(tcp))
        if self.honour_tcp:
            self._tcp = list(tcp)

    def setPayload(self, mass, cog=(0.0, 0.0, 0.0)):
        self._enter("setPayload", mass, list(cog))
        self._payload = mass

    def stopScript(self):
        self._enter("stopScript")
        self._script_running = False

    def reconnect(self):
        self._enter("reconnect")
        self._connected = self._script_running = True
        return True

    def disconnect(self):
        self._enter("disconnect")
        self._connected = False

    def isConnected(self):
        return self._connected

    def isProgramRunning(self):
        return self._script_running

    # ------------------------------------------------------------ receive side
    def getActualQ(self):
        return list(self._q)

    def getActualTCPSpeed(self):
        self._tick()
        return list(self._vel) + [0.0, 0.0, 0.0]

    def getSafetyMode(self):
        return self.safety_mode

    def getRobotMode(self):
        return self.robot_mode

    def _pose(self):
        z, off = self._tool_z(), self._tcp_offset()
        return ([self._flange_p[i] + z[i] * off for i in range(3)]
                + rotvec_from_matrix(self._flange_r))

    def getActualTCPPose(self):
        self._tick()                         # a running speedL moves on every read: simulated time
        return self._pose()

    def getActualTCPForce(self):
        """Force on the tool [Fx, Fy, Fz, Tx, Ty, Tz]; pushing on the table reads 30 N upward."""
        return [0.0, 0.0, 30.0 if self._contact else 0.0, 0.0, 0.0, 0.0]

    # ------------------------------------------------------------ test hooks
    def protective_stop(self):
        """Put the arm into protective stop; moves return False until clear_stop()."""
        self.safety_mode = SAFETY_PROTECTIVE_STOP

    def clear_stop(self):
        self.safety_mode = SAFETY_NORMAL

    def moves(self):
        """Just the motion commands, for assertions in tests."""
        return [c for c in self.calls if c[0].startswith("move")]

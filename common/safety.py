"""Checks that run BEFORE the robot moves.

Every function here is pure geometry or a read-back from the controller. None of
them commands motion, so all of them can be tested with no robot at all - which is
the point: the cheapest place to catch a bad move is before it is sent.
"""
import math
import time

# Conservative working limits for this course, as horizontal distance from the base
# column in metres: (inner keep-out, outer limit). The outer value is pulled in from
# the arm's nominal reach to stay clear of full extension, where the arm deflects
# most and where a straight-line move is most likely to be refused.
REACH = {
    "UR3":  (0.15, 0.46),
    "UR5e": (0.20, 0.80),
}


class WorkspaceBox:
    """An axis-aligned box of allowed offsets from a reference pose.

    limits = {"x": (lo, hi), "y": (lo, hi), "z": (lo, hi)} in metres, relative to
    `reference`. Note what this box is anchored to: if you re-capture the reference
    (for example every time the robot homes), the box moves with it. That is fine
    for jogging and wrong for anything fixed in the world, like a sheet of paper.
    """

    def __init__(self, reference, limits, margin=0.0):
        self.reference = list(reference)
        self.limits = dict(limits)
        self.margin = margin

    def check(self, pose):
        """Return (ok, reason). `reason` is a sentence a person can act on."""
        for i, axis in enumerate("xyz"):
            lo, hi = self.limits[axis]
            rel = pose[i] - self.reference[i]
            if not (lo - self.margin <= rel <= hi + self.margin):
                return False, ("%s offset %+.3f m is outside the allowed %+.3f .. %+.3f m"
                               % (axis.upper(), rel, lo, hi))
        return True, ""


def preflight_points(strokes, origin_xy, model="UR5e", area=(0.30, 0.20)):
    """Validate a whole drawing before any motion. Returns a list of problems.

    `strokes` is a list of polylines, each a list of (x, y) in metres relative to
    `origin_xy`. An empty list back means every point is reachable and inside the
    drawing area. The alternative is discovering an unreachable point half way
    through a stroke, with the pen down.
    """
    inner, outer = REACH[model]
    ox, oy = origin_xy
    area_w, area_h = area
    problems, far, near = [], None, None
    for stroke in strokes:
        for px, py in stroke:
            if not (-1e-9 <= px <= area_w + 1e-9) or not (-1e-9 <= py <= area_h + 1e-9):
                problems.append("point (%.3f, %.3f) falls outside the %.0f x %.0f mm drawing area"
                                % (px, py, area_w * 1000, area_h * 1000))
                break
            r = math.hypot(ox + px, oy + py)
            if r > outer and (far is None or r > far):
                far = r
            if r < inner and (near is None or r < near):
                near = r
    if far is not None:
        problems.append("point at %.3f m exceeds the %s limit of %.3f m - move the work closer"
                        % (far, model, outer))
    if near is not None:
        problems.append("point at %.3f m is inside the %.3f m base keep-out - move the work away"
                        % (near, inner))
    return problems


def safe_blend_radius(stroke, requested):
    """Clamp a blend radius to something the controller will accept.

    A blend radius at or above half the shortest segment makes the controller
    reject the ENTIRE path, not just that corner. So this is a hard clamp, and a
    stroke with no interior corner gets 0.
    """
    if len(stroke) < 3:
        return 0.0
    segs = [math.hypot(stroke[i + 1][0] - stroke[i][0], stroke[i + 1][1] - stroke[i][1])
            for i in range(len(stroke) - 1)]
    positive = [d for d in segs if d > 1e-9]
    if not positive:
        return 0.0
    return max(0.0, min(requested, min(positive) * 0.49))


def verify_tcp(control, receive, tool_length_m, payload_kg=0.0, settle_s=0.15, tolerance=0.25):
    """Set the tool centre point and PROVE the controller applied it.

    Sets the TCP to the flange, reads the pose, sets it to the tool, reads again.
    The reported point must move by about `tool_length_m`. Returns (ok, moved_m).

    Why bother: setTcp can be silently lost (on our arm, whenever the control script
    was re-uploaded). The robot then believes its tool tip is at the flange, and
    every Z it is sent is wrong by the full length of the tool - downwards.
    Comparing against "whatever the TCP was before" is not enough either: if the
    right TCP was already set, the shift is zero and a naive check reports failure.
    """
    control.setTcp([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(settle_s)
    flange = receive.getActualTCPPose()
    control.setTcp([0.0, 0.0, tool_length_m, 0.0, 0.0, 0.0])
    if payload_kg:
        control.setPayload(payload_kg, [0.0, 0.0, 0.0])
    time.sleep(settle_s)
    tip = receive.getActualTCPPose()
    moved = math.sqrt(sum((tip[i] - flange[i]) ** 2 for i in range(3)))
    ok = tool_length_m * (1 - tolerance) <= moved <= tool_length_m * (1 + tolerance)
    return ok, moved

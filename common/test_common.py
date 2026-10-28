"""Tests for the shared course library. Run from the course folder:  pytest common/"""
import math
import threading

import pytest

from common import safety
from common.mock_robot import (HOME_Q, MockRobot, forward_kinematics, matrix_from_rotvec,
                               rotvec_from_matrix)
from common.robot_io import Robot, connect


def dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# ------------------------------------------------------------------ kinematics
def test_rotvec_round_trip():
    for v in ([0.0, 0.0, 0.0], [0.3, -0.2, 0.9], [-1.58, 2.70, -0.01], [0.0, math.pi, 0.0]):
        back = rotvec_from_matrix(matrix_from_rotvec(v))
        r1, r2 = matrix_from_rotvec(v), matrix_from_rotvec(back)
        assert all(abs(r1[i][j] - r2[i][j]) < 1e-6 for i in range(3) for j in range(3))


def test_ur3_kinematics_match_the_real_arm():
    # Joints and TCP read from our UR3 on 2026-09-03; X and Y agreed to under 1 mm.
    q = [-0.6109, -1.607, -1.2479, -1.8445, 1.5817, -0.1035]
    t = forward_kinematics(q, "UR3")
    assert abs(t[0][3] - 0.1808) < 0.002 and abs(t[1][3] - (-0.2639)) < 0.002


def test_ur5e_kinematics_match_ursim():
    # TCP reported by URSim e-Series at HOME_Q on 2026-09-22; our FK agreed to 0.00 mm.
    t = forward_kinematics(HOME_Q, "UR5e")
    for got, want in zip((t[0][3], t[1][3], t[2][3]), (-0.1330, -0.4919, 0.4888)):
        assert abs(got - want) < 0.0005


# ------------------------------------------------------------------ mock robot
def test_move_j_changes_the_pose_you_read_back():
    robot = MockRobot()
    before = robot.getActualTCPPose()
    q = list(HOME_Q); q[0] += 0.5
    assert robot.moveJ(q, 0.5, 0.5) is True
    assert robot.getActualQ() == q
    assert dist(before, robot.getActualTCPPose()) > 0.05


def test_move_l_lands_on_the_target():
    robot = MockRobot()
    target = robot.getActualTCPPose(); target[2] += 0.05
    assert robot.moveL(target, 0.05, 0.3) is True
    assert dist(robot.getActualTCPPose(), target) < 1e-9


def test_move_l_accepts_a_blended_path():
    robot = MockRobot()
    p = robot.getActualTCPPose()
    path = [[p[0] + 0.01 * i, p[1], p[2], p[3], p[4], p[5], 0.05, 0.3, 0.0] for i in range(1, 4)]
    assert robot.moveL(path) is True
    assert abs(robot.getActualTCPPose()[0] - (p[0] + 0.03)) < 1e-9


def test_set_tcp_shifts_the_reported_point_by_the_tool_length():
    robot = MockRobot()
    flange = robot.getActualTCPPose()
    robot.setTcp([0, 0, 0.275, 0, 0, 0])
    assert abs(dist(flange, robot.getActualTCPPose()) - 0.275) < 1e-9


def test_a_controller_that_ignores_set_tcp_is_detectable():
    robot = MockRobot(honour_tcp=False)
    flange = robot.getActualTCPPose()
    robot.setTcp([0, 0, 0.275, 0, 0, 0])
    assert dist(flange, robot.getActualTCPPose()) < 1e-9


def test_protective_stop_refuses_moves_until_cleared():
    robot = MockRobot()
    target = robot.getActualTCPPose(); target[2] += 0.01
    robot.protective_stop()
    assert robot.moveL(target, 0.05, 0.3) is False
    robot.clear_stop()
    assert robot.moveL(target, 0.05, 0.3) is True


def test_bad_speed_is_rejected():
    robot = MockRobot()
    with pytest.raises(ValueError):
        robot.moveL(robot.getActualTCPPose(), 0.0, 0.3)


def _hammer(robot, lock):
    def work():
        for _ in range(40):
            if lock:
                with lock:
                    robot.moveJ(HOME_Q, 0.5, 0.5)
            else:
                robot.moveJ(HOME_Q, 0.5, 0.5)
    threads = [threading.Thread(target=work) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


def test_two_threads_without_a_lock_collide():
    robot = MockRobot()
    _hammer(robot, lock=None)
    assert robot.collisions > 0


def test_two_threads_with_the_lock_never_collide():
    robot = MockRobot()
    _hammer(robot, lock=threading.Lock())
    assert robot.collisions == 0


# ------------------------------------------------------------------ safety
def test_workspace_box_explains_a_refusal():
    box = safety.WorkspaceBox([0.3, -0.2, 0.4, 0, 3.14, 0],
                              {"x": (-0.25, 0.25), "y": (-0.25, 0.25), "z": (-0.20, 0.20)})
    assert box.check([0.4, -0.2, 0.4, 0, 3.14, 0]) == (True, "")
    ok, reason = box.check([0.3, -0.2, 0.7, 0, 3.14, 0])
    assert ok is False and reason.startswith("Z offset")


def test_preflight_catches_reach_keepout_and_area():
    square = [[(0.0, 0.0), (0.05, 0.0), (0.05, 0.05), (0.0, 0.05)]]
    assert safety.preflight_points(square, (0.30, -0.20), model="UR3") == []
    assert any("exceeds" in p for p in safety.preflight_points(square, (0.45, 0.30), model="UR3"))
    assert any("keep-out" in p for p in safety.preflight_points(square, (0.01, 0.01), model="UR3"))
    too_wide = [[(0.0, 0.0), (0.40, 0.0)]]
    assert any("drawing area" in p for p in safety.preflight_points(too_wide, (0.2, 0.0)))


def test_blend_radius_stays_under_half_the_shortest_segment():
    stroke = [(0, 0), (0.002, 0), (0.002, 0.010)]
    assert safety.safe_blend_radius(stroke, 0.005) < 0.001
    assert safety.safe_blend_radius([(0, 0), (1, 0)], 0.005) == 0.0


def test_verify_tcp_accepts_an_honest_controller():
    robot = MockRobot()
    ok, moved = safety.verify_tcp(robot, robot, 0.275, settle_s=0)
    assert ok and abs(moved - 0.275) < 1e-9


def test_verify_tcp_rejects_a_controller_that_ignored_it():
    robot = MockRobot(honour_tcp=False)
    ok, moved = safety.verify_tcp(robot, robot, 0.275, settle_s=0)
    assert not ok and moved < 1e-9


def test_verify_tcp_still_passes_when_the_tcp_was_already_right():
    robot = MockRobot()
    robot.setTcp([0, 0, 0.275, 0, 0, 0])             # the case a naive before/after check fails
    ok, _ = safety.verify_tcp(robot, robot, 0.275, settle_s=0)
    assert ok


# ------------------------------------------------------------------ robot_io
def test_connect_defaults_to_the_mock(monkeypatch):
    monkeypatch.delenv("ROBOT", raising=False)
    monkeypatch.setattr("common.robot_io.simulator_is_running", lambda *a, **k: False)
    with connect() as robot:
        assert robot.backend == "mock" and len(robot.receive.getActualTCPPose()) == 6


def test_connect_rejects_nonsense(monkeypatch):
    monkeypatch.delenv("ROBOT_IP", raising=False)
    with pytest.raises(ValueError):
        connect(backend="toaster")
    with pytest.raises(ValueError):
        connect(backend="real")


def test_close_stops_the_script_before_disconnecting():
    mock = MockRobot()
    Robot(mock, mock, "mock").close()
    names = [c[0] for c in mock.calls]
    assert names.index("stopScript") < names.index("disconnect")


def test_easy_robot_moves_and_checks(monkeypatch):
    monkeypatch.setenv("ROBOT", "mock")
    from common.easy import connect as easy_connect
    robot = easy_connect()
    z0 = robot.position_mm()[2]
    robot.home()
    robot.move_up(3)
    assert robot.position_mm()[2] == z0 + 30
    robot.move_down(3)
    assert robot.position_mm()[2] == z0
    robot._r.control.stop_short = 0.02
    with pytest.raises(RuntimeError):
        robot.move_up(3)
    robot._r.control.stop_short = 0.0
    with pytest.raises(RuntimeError):
        robot.speed(30)
    with pytest.raises(RuntimeError):
        robot.tolerance(mm=50)
    robot.tolerance(mm=5)
    robot.speed(10)
    robot.move_up(2)
    robot.close()


def test_easy_path_goes_through_every_point_as_one_command(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT", "mock")
    from common.easy import connect as easy_connect
    robot = easy_connect()
    robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))
    robot.home()
    robot.path([(-133, -492, 400), (-233, -492, 400), (-233, -392, 400)], blend_cm=2)
    assert robot.position_mm() == [-233, -392, 400]
    paths = [c for c in robot._r.control.calls if c[0] == "moveL_path"]
    assert len(paths) == 1 and len(paths[0][1]) == 3
    assert paths[0][1][0][8] == 0.02 and paths[0][1][-1][8] == 0.0      # blend on the corners, none at the end
    out = capsys.readouterr().out
    assert "path: 3 points, blend 2 cm" in out and "arrived:" in out
    with pytest.raises(RuntimeError, match="point 2 of the path"):
        robot.path([(-133, -492, 400), (-133, -492, 50)])
    assert len([c for c in robot._r.control.calls if c[0] == "moveL_path"]) == 1   # nothing was sent


def test_easy_touch_down_stops_on_the_table(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT", "mock")
    from common.easy import connect as easy_connect
    robot = easy_connect()
    robot.set_tool(cm=20)
    robot.set_box(x=(-400, 100), y=(-700, -300), z=(-20, 700))
    robot.home()
    robot._r.control.table_z = 0.012                                   # the fake table is 12 mm above the base plate
    z = robot.touch_down()
    assert z == 12 and robot.position_mm()[2] == 12
    assert "felt it: 30 N at z = 12 mm" in capsys.readouterr().out
    assert robot._r.control.calls[-1][0] == "stopL"
    robot.home()
    robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))        # the floor is above the table: it must give up
    with pytest.raises(RuntimeError, match="box floor"):
        robot.touch_down()


def test_easy_stop_test_measures_the_braking_distance(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT", "mock")
    from common.easy import connect as easy_connect
    robot = easy_connect()
    robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))
    robot.home()
    start = robot.position_mm()
    d = [robot.stop_test(v) for v in (5, 10, 20)]
    assert d == [1.2, 5.0, 20.0] or d == [1.3, 5.0, 20.0]             # v*v/a with a = 2: twice the speed, four times the distance
    assert robot.position_mm() == start                               # it goes back where it started
    assert "stop test 20 cm/s, the program stops it:  20.0 mm after the stop" in capsys.readouterr().out
    with pytest.raises(RuntimeError, match="no room"):
        robot.set_box(x=(-200, 100), y=(-700, -300), z=(100, 700))
        robot.stop_test(10)


def test_easy_stop_test_by_hand_counts_the_reaction(monkeypatch, capsys):
    monkeypatch.setenv("ROBOT", "mock")
    import builtins
    import time as _t
    from common.easy import connect as easy_connect
    robot = easy_connect()
    robot.home()
    monkeypatch.setattr(builtins, "input", lambda *a: _t.sleep(0.2))   # reacts in 0.2 s
    assert robot.stop_test(10, by_hand=True) > 5.0                   # more than the program stop alone
    out = capsys.readouterr().out
    assert "NOW" in out and "you stop it" in out and "while you reacted" in out


def test_easy_window_verbs_never_wait_and_never_raise(monkeypatch):
    monkeypatch.setenv("ROBOT", "mock")
    from common.easy import connect as easy_connect
    robot = easy_connect()
    robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))
    robot.home()
    x, y, z = robot.position_mm()
    assert robot.jog(z_cm=1) == "moving z +1 cm"
    assert robot._r.control.calls[-1][0] == "moveL"                    # started, asynchronous
    assert robot.jog(z_cm=1) == "moving z +1 cm"                       # a quick second click adds to the first
    for _ in range(50):
        robot.position_mm()
    assert robot.position_mm() == [x, y, z + 20]
    assert robot.jog(z_cm=-50) == "outside the box: not moved"
    assert robot.stop() == "stopped" and robot._r.control.calls[-1][0] == "stopL"
    assert robot.go_home() == "going home"
    assert robot.stop() == "stopped" and robot._r.control.calls[-1][0] == "stopJ"
    robot._r.control.protective_stop()
    assert robot.jog(z_cm=1) == "not in Normal mode: not moved"

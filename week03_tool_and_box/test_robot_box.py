"""Week 3 tests. Run from the course folder:  pytest week03_tool_and_box/

They run robot_box.py on the fake robot and read what it prints. One test per TODO.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COURSE = os.path.abspath(os.path.join(HERE, ".."))
STUDENT_FILE = os.path.join(HERE, "robot_box.py")
SOLUTION = os.path.join(COURSE, "instructor", "solutions", "week03_robot_box.py")
TARGET = SOLUTION if os.path.exists(SOLUTION) and os.environ.get("TEST_STUDENT") != "1" else STUDENT_FILE


def run(path=TARGET):
    code = open(path, encoding="utf-8").read()
    env = dict(os.environ, ROBOT="mock", PYTHONIOENCODING="utf-8")
    result = subprocess.run([sys.executable, "-c", code], cwd=COURSE, env=env, capture_output=True, text=True)
    return result.stdout + result.stderr, result.returncode


def test_it_runs_and_prints_done():
    out, code = run()
    assert "done" in out and code == 0
    for line in ("box set:", "speed: 10 cm/s", "tolerance: 2.0 mm", "arrived:"):
        assert line in out, "the four settings must print: " + line


def test_todo1_tool_length_is_set_and_checked():
    out, _ = run()
    assert "checked" in out and "no tool" not in out, "set_tool must get the measured gripper length"


def test_todo2_moves_to_the_second_point():
    out, _ = run()
    assert "moving to [-133, -400, 400]" in out


def test_todo3_a_point_outside_the_box_is_skipped_not_sent():
    out, _ = run()
    assert "skipping" in out
    assert "moving to [-133, -492, 50]" not in out


def test_todo4_visits_three_points_in_one_smooth_path():
    out, _ = run()
    assert "path: 3 points, blend 2 cm" in out and "arrived:" in out.split("path: 3 points")[1]


def test_the_file_runs_as_given():
    out, code = run(STUDENT_FILE)
    assert "done" in out and code == 0

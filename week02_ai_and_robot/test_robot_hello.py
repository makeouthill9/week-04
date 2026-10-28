"""Week 2 tests. Run from the course folder:  pytest week02_ai_and_robot/

They run robot_hello.py on the fake robot and read what it prints. One test per TODO.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COURSE = os.path.abspath(os.path.join(HERE, ".."))
STUDENT_FILE = os.path.join(HERE, "robot_hello.py")
SOLUTION = os.path.join(COURSE, "instructor", "solutions", "week02_robot_hello.py")
TARGET = SOLUTION if os.path.exists(SOLUTION) and os.environ.get("TEST_STUDENT") != "1" else STUDENT_FILE


def run(path=TARGET, lift=None):
    """Run the program on the fake robot and return what it printed. lift=N swaps the LIFT_CM line."""
    code = open(path, encoding="utf-8").read()
    if lift is not None:
        code = "\n".join("LIFT_CM = %d" % lift if line.startswith("LIFT_CM = ") else line for line in code.splitlines())
    env = dict(os.environ, ROBOT="mock", PYTHONIOENCODING="utf-8")
    result = subprocess.run([sys.executable, "-c", code], cwd=COURSE, env=env, capture_output=True, text=True)
    return result.stdout + result.stderr, result.returncode


def test_it_runs_and_prints_done():
    out, code = run()
    assert "done" in out and code == 0


def test_todo1_lift_is_5cm():
    out, _ = run()
    assert "lifting 5 cm" in out


def test_todo2_prints_the_position_again_at_the_end():
    out, _ = run()
    assert out.count("tool mm") == 2, "robot.describe() should run twice: at the start and after coming back"


def test_todo3_silly_lift_is_refused_before_moving():
    out, code = run(lift=50)
    assert "STOPPED" in out and code == 1
    assert "going" not in out and "lifting" not in out


def test_todo4_lifts_three_times():
    out, _ = run()
    assert out.count("lifting") == 3


def test_the_file_runs_as_given():
    out, code = run(STUDENT_FILE)
    assert "done" in out and code == 0

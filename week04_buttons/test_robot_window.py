"""Week 4 tests. Run from the course folder:  pytest week04_buttons/

They open the window without showing it, click the buttons from code, and check what the
fake robot did. One test per TODO.
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
COURSE = os.path.abspath(os.path.join(HERE, ".."))
STUDENT_FILE = os.path.join(HERE, "robot_window.py")
SOLUTION = os.path.join(COURSE, "instructor", "solutions", "week04_robot_window.py")
TARGET = SOLUTION if os.path.exists(SOLUTION) and os.environ.get("TEST_STUDENT") != "1" else STUDENT_FILE

DRIVER = r'''
import json, runpy, sys
from PyQt6.QtWidgets import QApplication, QPushButton, QSlider, QLabel
app = QApplication([])
ns = runpy.run_path(sys.argv[1], run_name="robot_window")     # runs the file, but not its window loop
robot, w = ns["robot"], ns["MainWindow"]()
out = {}

def settle():
    last = None
    for _ in range(400):                                       # the fake robot moves a little on every read
        now = robot.position_mm()
        if now == last:
            return now
        last = now
    return last

def click(name):
    b = w.findChild(QPushButton, name)
    if b is None:
        return None
    b.click()
    msg = w.findChild(QLabel, "message").text()
    return msg, settle()

out["start"] = settle()
out["up"] = click("up")
out["down"] = click("down")
w.show_position(); out["label"] = w.findChild(QLabel, "position").text()
robot.jog(z_cm=-10); w.findChild(QPushButton, "stop").click()
out["stop"] = [w.findChild(QLabel, "message").text(), settle()]
robot.go_home(); settle()
out["outside"] = [robot.jog(z_cm=-40), settle()]
out["buttons"] = [n for n in ("x_minus", "x_plus", "y_minus", "y_plus") if w.findChild(QPushButton, n) is not None]
out["x_plus"] = click("x_plus")
out["y_minus"] = click("y_minus")
out["home"] = click("home")
out["joints"] = robot.joints_deg()
s = w.findChild(QSlider, "speed")
if s is not None:
    s.setValue(20)
out["speed"] = [round(robot._speed * 100), w.findChild(QLabel, "speed_value").text()]
print("RESULT" + json.dumps(out))
'''


def run(path=TARGET):
    env = dict(os.environ, ROBOT="mock", QT_QPA_PLATFORM="offscreen", PYTHONIOENCODING="utf-8")
    result = subprocess.run([sys.executable, "-c", DRIVER, path], cwd=COURSE, env=env, capture_output=True, text=True)
    lines = [l for l in result.stdout.splitlines() if l.startswith("RESULT")]
    assert lines, "the window did not open:\n" + result.stdout + result.stderr
    return json.loads(lines[-1][6:])


def test_window_opens_and_the_given_buttons_work():
    out = run()
    x, y, z = out["start"]
    assert out["up"] == ["moving z +1 cm", [x, y, z + 10]]
    assert out["down"][1] == [x, y, z]
    assert out["label"] == "x %d   y %d   z %d mm" % (x, y, z)
    assert out["stop"][0] == "stopped" and z - 100 < out["stop"][1][2] < z, "STOP must brake a move half way"
    assert out["outside"] == ["outside the box: not moved", [x, y, z]]


def test_todo1_four_buttons_drawn_in_qt_designer():
    assert run()["buttons"] == ["x_minus", "x_plus", "y_minus", "y_plus"]


def test_todo2_the_four_buttons_move_the_robot():
    out = run()
    x, y, z = out["start"]
    assert out["x_plus"] == ["moving x +1 cm", [x + 10, y, z]]
    assert out["y_minus"] == ["moving y -1 cm", [x + 10, y - 10, z]]


def test_todo3_home_button():
    out = run()
    assert out["home"] and out["home"][0] == "going home"
    assert out["joints"] == [-90, -90, -90, -90, 90, 0]


def test_todo4_speed_slider():
    assert run()["speed"] == [20, "20 cm/s"]


def test_the_file_runs_as_given():
    out = run(STUDENT_FILE)
    x, y, z = out["start"]
    assert out["up"] == ["moving z +1 cm", [x, y, z + 10]]

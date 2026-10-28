# Week 4 - buttons for the robot. The window comes from Qt Designer: robot_window.ui
#
# Run it:          python week04_buttons/robot_window.py
# Edit the window: pyside6-designer week04_buttons/robot_window.ui
#
# A window program does not run from top to bottom. It waits. A click calls one of your functions.

import sys
sys.path.insert(0, ".")
from PyQt6 import uic
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QLabel, QSlider
from common.easy import connect

robot = connect()
robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))   # week 3's box, behind every button
robot.speed(5)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("week04_buttons/robot_window.ui", self)        # the window you drew in Qt Designer

        # find each widget by the name you gave it in Qt Designer
        self.position = self.findChild(QLabel, "position")
        self.message = self.findChild(QLabel, "message")
        self.speed_value = self.findChild(QLabel, "speed_value")

        # a click on a button calls a function
        self.findChild(QPushButton, "up").clicked.connect(self.on_up)
        self.findChild(QPushButton, "down").clicked.connect(self.on_down)
        self.findChild(QPushButton, "stop").clicked.connect(self.on_stop)
        # TODO 2 (by hand): connect x_minus, x_plus, y_minus, y_plus, like the three lines above
        # TODO 3 (with the agent): connect the Home button to a function that calls robot.go_home()
        # TODO 4 (with the agent): when the speed slider moves, call robot.speed() and show "N cm/s" in speed_value

        # every 200 ms, ask the robot where it is and show it
        self.timer = QTimer()
        self.timer.timeout.connect(self.show_position)
        self.timer.start(200)

    def on_up(self):
        self.message.setText(robot.jog(z_cm=1))

    def on_down(self):
        self.message.setText(robot.jog(z_cm=-1))

    def on_stop(self):
        self.message.setText(robot.stop())

    def show_position(self):
        x, y, z = robot.position_mm()
        self.position.setText(f"x {x}   y {y}   z {z} mm")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

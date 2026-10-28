# Week 3 experiment: how far does the arm travel after STOP?   Not a TODO: run it and fill the worksheet.
#
# Run it:   python week03_tool_and_box/stop_distance.py
# The arm moves sideways 6 cm to get up to speed, then it is stopped, and the program measures
# how far the tool went AFTER the stop. Then it goes back and does the next speed.

import sys
sys.path.insert(0, ".")
from common.easy import connect

robot = connect()
robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))
robot.home()

print("Part 1: the program stops the arm. Only the braking counts.")
for speed in (5, 10, 20):
    robot.stop_test(speed)

print()
print("Part 2: YOU stop the arm. Watch the terminal; the moment you read NOW, press Enter.")
for speed in (5, 10, 20):
    input("speed %d cm/s: press Enter to start, then wait for NOW ..." % speed)
    robot.stop_test(speed, by_hand=True)

robot.home()
print("done")

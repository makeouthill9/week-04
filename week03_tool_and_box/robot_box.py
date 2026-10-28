# Week 3 - four things you tell the robot: what it holds, where it may go, how fast, how exact.
#
# Run it:   python week03_tool_and_box/robot_box.py
# Numbers here are in MILLIMETRES from the robot base, exactly like the robot screen shows.

import sys
sys.path.insert(0, ".")
from common.easy import connect

robot = connect()

GRIPPER_CM = 00              # TODO 1: measure the RG2 gripper with a ruler, flange to fingertips, in cm. Type it here.
robot.set_tool(cm=GRIPPER_CM)

robot.set_box(x=(-400, 100), y=(-700, -300), z=(100, 700))   # where the tool tip MAY go
robot.speed(10)                                        # how fast: centimetres per second. The course limit is 25.
robot.tolerance(mm=2)                                  # how exact: a move counts as arrived within 2 mm of the target

robot.home()
robot.describe()

robot.move_to(-133, -492, 400)                         # a point inside the box
# TODO 2: a second point, 10 cm closer to the robot:  robot.move_to(-133, -400, 400)

# TODO 3 (with the agent): before moving to a point, ask the robot if it is inside the box.
#     if robot.inside_box(-133, -492, 50):
#         robot.move_to(-133, -492, 50)
#     else:
#         print("skipping: that point is outside the box")

# TODO 4 (with the agent): visit these three points in ONE smooth path, no stop at the corners:
#     points = [(-133, -492, 400), (-233, -492, 400), (-233, -392, 400)]
#     robot.path(points, blend_cm=2)

robot.home()
print("done")

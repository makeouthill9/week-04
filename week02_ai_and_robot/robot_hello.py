# Week 2 - my first robot program.
#
# Run it:   python week02_ai_and_robot/robot_hello.py
# If the simulator is running, the robot in the browser moves. If not, a fake robot is used.

import sys
sys.path.insert(0, ".")
from common.easy import connect

LIFT_CM = 3                 # TODO 1: make it 5

robot = connect()           # simulator if it is running, otherwise the fake robot

robot.describe()            # where is the robot? (eyes)
robot.home()                # go to the home pose (hands)

# TODO 3: refuse a silly lift BEFORE moving. Two lines:
#     if LIFT_CM > 10:
#         print("STOPPED: lift too big"); sys.exit(1)

print("lifting", LIFT_CM, "cm and coming back ...")
robot.move_up(LIFT_CM)      # straight up, checked
robot.move_down(LIFT_CM)    # and back, checked

# TODO 2: print where the robot is now. One line: robot.describe()

# TODO 4 (with the agent): do the lift 3 times using a for loop.

print("done")

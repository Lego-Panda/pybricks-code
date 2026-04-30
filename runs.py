from rob_conf import *
from functions import Move, Turn, Shell, Calibration, battery_percent

# ---

# EXAMPLE OF MAKING A RUN
def example_run():
    # Start by calibrating the movement, turn and the shell.
    # You can calibrate via running the calibrate run below.
    m = Move(kp=0, ki=0, kd=0, 
             kp_curve=0, ki_curve=0, kd_curve=0)
    t = Turn(kp=0, ki=0, kd=0)
    s = Shell(kp=0, ki=0, kd=0)

    m.move(500, -50, 10) # 500mm/50cm to run | -50% of the motor | 10ms to wait after the task.
    t.turn(90, 50, 10) # 90 deg to turn | 50% of the motor speed | 10ms to wait after the task.

    # After the run is done, you can reset the shell's position:
    s.turn_until_color("yellow", 100) # Runs until sees yellow.
    # you can choose 4 colors. to select in the function make sure its in ""
    # "red", "blue", "green", "yellow". you can also start with an Uppercase.

# To run what you made, do this:
# example_run()
# !! example_run is just a name. if your run is named run1, do run1()

def cal():
    c = Calibration()

    # c.auto_tune_turn()
    c.auto_tune_straight_precision()

cal()

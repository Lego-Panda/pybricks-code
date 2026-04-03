from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *

hub = PrimeHub(Axis.Y, Axis.Z)
hub.display.orientation(Side.BOTTOM)

def run1():
    # rob = Robot(kp=5, ki=0, kd=10, turnKp=0.08, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=5, turn_wait_time=1)
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=10, turnKi=0, turnKd=25, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=2, armKp= 5, armKi=0, armKd=10, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    # rob.pid(68, -50)
    rob.accelDecel(67, -70)
    wait(10)
    rob.turn(-18, 50)
    wait(10)

    arm.run(-700)
    wait(400)
    arm.brake()
    wait(1000)

    # rob.pid(2, -30)
    wait(10)
    rob.turn(-40, 50)
    wait(10)
    arm.run(1000)
    wait(2500)
    arm.brake()
    wait(300)
    rob.pid(6, 30)
    wait(10)
    rob.turn(45, 50)
    wait(300)
    rob.pid(9, -50)
    wait(10)
    shell.run(1000)
    wait(1000)
    shell.stop()
    wait(300)
    rob.pid(10, 30)
    wait(10)
    rob.turn(50, 50)
    wait(300)
    rob.pid(75, 75)
    wait(10)
    rob.shellButton(-180)

####

def run2():
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6, turnKi=0, turnKd=15, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(40)
    hub.speaker.beep()
    wait(100)

    rob.pid(44, -50)
    wait(200)
    for i in range(4):
        arm.run_time(-700, 700)
        wait(300)
        arm.run_time(700, 700)
    wait(200)
    rob.pid(50, 50)

####

def run3():
    rob = Robot(kp=1, ki=0, kd=2, turnKp=0, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)

    # rob.accelDecel(42, -525)
    rob.pid_distance(45, -50, 1, 0, 5)
    wait(10)
    arm.run(-1000)
    wait(1000)
    arm.stop()
    wait(10)
    rob.pid(13, 50)
    wait(100)
    rob.pid(7, -30)
    wait(10)
    arm.run(1000)
    wait(1300)
    arm.stop()
    wait(100)
    rob.pid(3, -30)
    wait(10)
    shell.run_time(1100, 2000)
    wait(10)
    rob.pid(35, 50)
    wait(10)
    arm.run(700)
    wait(10)
    arm.stop()
    # rob.stopColor("stopYellow", 180)

####

def run4():
    rob = Robot(kp=2.34, ki=0, kd=12.256, turnKp=5.94, turnKi=0.636, turnKd=13.86, shellKp=50.4, shellKi=15.508, shellKd=100.95, shellTol=2, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    # rob.pid(50, 50)
    # rob.turn(90, 50)
    # rob.shellTurn(90)

    # rob.pid_distance(22, -50, 2, 0 , 10)
    # wait(100)
    # rob.turn(-90, 50)
    rob.arc(35, 85, -50)
    wait(100)
    rob.accelDecel(45, -70)
    wait(100)
    rob.turnWhileShell(-270, 90, 100, 50)
    wait(100)
    rightwheel.dc(-30)
    leftwheel.dc(-30)
    wait(1200)
    wheels.brake()
    wait(100)
    rob.shellTurn(20)
    # rob.shellTurnTime(350, 100)
    wait(100)
    arm.run_time(-660, 3000)
    wait(10)
    rob.shellTurn(-20)
    wait(10)
    rob.pid(10, 30)
    wait(10)
    rob.turn(-50, 40)
    wait(10)
    rob.pid(11, 30)
    wait(600)
    # rob.turn(-20, 50)
    # wait(10)
    rob.pid(15, -60)
    wait(10)
    rob.turn(-45, 50)
    wait(10)
    rob.pid(20, 50)
    wait(10)
    rob.turn(30, 50)
    wait(10)
    rob.pid(70, 70)
    wait(10)
    # rob.stopColor("stopYellow", 180, 100r)
    # rob.shellTurn(90 )


###

def run5():
    rob = Robot(kp=4.32, ki=0.145, kd=32, turnKp=9.3, turnKi=1.185, turnKd=18.242, shellKp=24.0, shellKi=2.133, shellKd=67.5, shellTol=5, turnTol=5, armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(500)

    rob.pid(30, -50)

    
###

def run6():
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6.5, turnKi=0, turnKd=20, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)


    arm.run_time(-1000, 1100)
    wait(10)
    arm.run_time(1000, 700)
    wait(10)
    rob.pid(10, -30)
    wait(10)
    rob.turn(-22, 40)
    wait(10)
    rob.pid(58, -50)
    wait(10)
    rob.turn(68, 40)
    wait(300)
    rob.pid(40, -100)
    wait(40)
    arm.run_time(1000, 2000)
    wait(300)
    rob.pid(18, 30)
    wait(10)
    rob.turn(-70, 40)
    wait(10)
    rob.pid(30, 50)
    wait(10)
    rob.turn(25, 50)
    wait(10)
    rob.pid(15, -40)
    wait(300)
    rob.turn(-55, 50)
    wait(10)
    rob.turn(55, 40)
    wait(10)
    rob.pid(50, 60)

###

def run7():
    rob = Robot(kp=2.5, ki=0, kd=16)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)

    rob.pid(60,-70)
    wait(400)
    rob.pid(3,50)
    wait(400)
    arm.run_time(660, 1000)
    wait(400)
    rob.pid(4,50)
    wait(200)
    # rob.pid(5, -50)
    # wait(200)
    rob.pid(5, -100)
    wait(10)
    rob.pid(45, 100)

###

def run8():
    # rob = Robot(kp=1, ki=0, kd=0.1, turnKp=9, turnKi=0, turnKd=18, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10,armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=100)
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6.12, turnKi=0.93, turnKd=12.073, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=2,armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=100)
    
    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100) 

    


###

def test():
    rob = Robot(kp=1.98, ki=0, kd=8.915, turnKp=6.12, turnKi=0.93, turnKd=10.073, shellKp=36.0, shellKi=10.105, shellKd=50.062, shellTol=2, turnTol=2, turn_wait_time=1) # Base Robot
    
    # rob.auto_tune_straight_precision(60, 50)
    # rob.auto_tune_shell(90)

    # rob.pid(20,40)
    # rob.turn(180,40)
    # rob.pid(20,40)
    # rob.shellTurn(90)
    # rob.turn_one_wheel(90,)
    # rob.arc(30, 90, 50)

    # rob.pid_distance(50, 50, 1.98, 0, 30)
    # rob.accelDecel(50, 70)

    # wheels.drive(1000, 0)
    arm.run_time(700, 1000)
    # wait(100000)
    # arm.run_time(-700,800)
    # wait(400)
    # rob.pid(60, 50)

###
 
def calibration():
    from auto_calibration import RobotCalibration

    rob_c = RobotCalibration()

    hub.display.orientation(Side.BOTTOM)
    selected_calibration = hub_menu("F", "T", "C")
    if selected_calibration == "F":
        hub.system.set_stop_button(None)
        hub.speaker.beep()
        rob_c.auto_tune_straight_precision(60, 50)
    elif selected_calibration == "T":
        hub.system.set_stop_button(None)
        hub.speaker.beep()
        rob_c.auto_tune_turn()
    elif selected_calibration == "C":
        hub.system.set_stop_button(None)
        hub.speaker.beep()
        rob_c.auto_tune_shell()


###

def battery():
    rob = Robot()
    rob.battery_percent()
    # for i in range(10):
    #     print(hub.battery.voltage())
    #     wait(100)
    # hub.display.icon(Icon.HAPPY)
    


######

# run1()

# run2()

# run3()

# run4()

#run5()

# run6()

# run7()

# run8()

# test()

calibration()

#battery()

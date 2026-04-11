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
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=10, turnKi=0, turnKd=25, shellKp=50, shellKi=0, shellKd=100, shellTol=2, turnTol=2, armKp= 5, armKi=0, armKd=10, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    # rob.pid(68, -50)
    rob.accelDecel(65, -70)
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

    # arm.run(1000)
    # wait(1500)
    # arm.brake()
    # wait(300)
    arm.run_time(speed=1000, time=1500, wait=False)
    rob.pid(6, 30)

    wait(10)

    rob.turn(45, 50)
    wait(300)
    rob.pid(9, -50)
    wait(300)

    # shell.dc(100)
    # wait(3000)
    # shell.stop()
    rob.turn_stuck(30, 80)

    # rob.shellTurn(20)
    # rob.turn_stuck(40, 80)

    wait(400)
    # rob.pid(10, 30)
    # wait(10)
    # rob.turn(50, 50)
    # wait(300)
    rob.pid(75, 75)
    wait(10)

    # rob.shellTurn(-40)

####

def run2():
    rob = Robot(kp=1.98, ki=0.092, kd=10.628, turnKp=6.36, turnKi=0.687, turnKd=14.717, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(40)
    hub.speaker.beep()
    wait(100)

    rob.pid_distance(25, -50)
    wait(100)
    rob.turn(25, 40)
    wait(100)
    rob.pid_distance(40, -50)
    wait(100)
    rob.turn(-100, 50)
    wait(100)
    rob.pid_distance(9, -40)
    
    for i in range(3):
        arm.dc(-100)
        wait(800)
        arm.brake()
        arm.dc(100)
        wait(800)
        arm.brake()

    arm.dc(100)
    wait(800)
    arm.brake()
        
    rob.pid_distance(12,-100)
    wait(30)
    rob.pid_distance(2,60)
    wait(30)
    # rob.arc(25, 120, 60)
    rob.pid(20, 50)
    rob.turn(120,40)    
    wait(30)
    rob.pid_distance(70,100)

    

####

def run2point5():
    rob = Robot(kp=1.98, ki=0.092, kd=10.628, turnKp=6.36, turnKi=0.687, turnKd=14.717, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(40)
    hub.speaker.beep()
    wait(100)

    # rob.pid_distance(45, -60)
    rob.accelDecel(48, -70)
    wait(10)
    arm.run_time(1000, 400)

    for i in range(4):
        arm.dc(-100)
        wait(800)
        arm.brake()
        arm.dc(100)
        wait(800)
        arm.brake()

    arm.dc(-100)
    wait(800)
    arm.brake()

    # arm.run_time(1000, 500)

    rob.pid(60, 100)


def run3():
    rob = Robot(kp=1, ki=0, kd=2, turnKp=6.36, turnKi=0.687, turnKd=14, shellKp=76, shellKi=0, shellKd=30, shellTol=2, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)

    rob.pid_distance(45, -60)
    wait(10)
    arm.run(-1000)
    wait(1000)
    arm.stop()
    wait(10)
    rob.pid(13, 60)
    wait(100)
    rob.pid(7, -40)
    wait(10)
    arm.run(1000)
    wait(2000)
    arm.stop()
    wait(100)
    rob.pid(3, -40)
    wait(10)
    # shell.run_time(1100, 300)
    rob.shellTurn(90)
    wait(10)
    rob.turn(30, 60)
    wait(10)
    rob.pid(40, 100)
    # wait(10)
    # arm.run(700)
    # wait(10)
    # arm.brake()
    # arm.run_time()

    # shell.run_time(-1100, 1350)
    rob.shellTurn(-90)



####

def run4():
    rob = Robot(kp=2.34, ki=0, kd=12.256, turnKp=5.94, turnKi=0.636, turnKd=13.86, shellKp=50.4, shellKi=15.508, shellKd=100.95, shellTol=2, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    rob.arc(38, 92, -50)
    wait(10)
    rob.accelDecel(45, -70)
    wait(100)
    rob.turnWhileShell(-270, 90, 100, 50)
    wait(100)
    rob.pid_distance(7, -30)
    wait(100)
    rob.shellTurn(20)
    wait(100)
    arm.run_time(-660, 3000)
    wait(10)
    rob.shellTurn(-20)
    wait(10)
    rob.arc(11, -47, 40)
    wait(10)
    rob.pid(9, 30)
    wait(300)
    rob.pid(2, -30)
    wait(1000)
    rob.turn(-30, 60)
    wait(300)
    rob.pid(100, -100)

    rob.shellTurn(-90)


###

def run5():
    rob = Robot(kp=4.32, ki=0.145, kd=32, turnKp=9.3, turnKi=1.185, turnKd=28.242, shellKp=24.0, shellKi=2.133, shellKd=67.5, shellTol=2, turnTol=2, armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(500)

    rob.battery_percent()

    rob.accelDecel(75, -60)
    wait(200)
    rob.turn(-90, 40)
    wait(200)
    rob.pid_distance(23, -50)
    wait(200)
    rob.turn(-32, 30)
    wait(200)
    rob.pid_distance(10, -50)

    for i in range(2):
        rob.turn(25, 80)
        wait(10)
        rob.turn(-25, 80)
        wait(60)

    rob.pid_distance(38, 50)
    wait(10)
    rob.turn(40, 50)

    wait(10)
    rob.pid_distance(10, -30)
    wait(10)
    arm.dc(100)
    wait(1500)
    arm.brake()
    # wait(300)
    # arm.dc(-100)
    # wait(1000)
    # arm.brake()
    # wait(600)
    # rob.pid_distance(2, 50)
    # wait(10)
    # # arm.dc(100)
    # # wait(1500)
    # # arm.brake()
    # # wait(10)
    # rob.turn(70, -50)
    # wait(10)
    # rob.pid(100, -70)

    
###

def run6():
    # rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6.5, turnKi=0, turnKd=20, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10, turn_wait_time=1)
    rob = Robot(kp=2.46, ki=0, kd=14.157, turnKp=7.14, turnKi=0.867, turnKd=14.677, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)

    arm.run_time(-1000, 1100)
    wait(10)
    arm.run_time(1000, 700)
    wait(10)

    rob.pid(10, -30)
    wait(10)
    rob.turn(-17, 40)
    wait(10)
    rob.pid(58, -50)
    wait(10)
    rob.turn(58, 40)
    wait(300)

    rob.pid(40, -100)
    wait(40)
    arm.run_time(1000, 2000)
    wait(300)
    rob.pid(18, 30)
    wait(10)

    rob.turn(-60, 40)
    wait(10)
    rob.pid(30, 50)
    wait(10)
    
    rob.turn(25, 50)
    wait(10)
    rob.pid(15, -40)
    wait(300)
    rob.turn(-55, 50)
    wait(500)
    rob.turn(55, 70)
    wait(10)
    rob.pid(50, 60)

def run6point5():
    rob = Robot(kp=2.46, ki=0, kd=14.157, turnKp=7.14, turnKi=0.867, turnKd=14.677, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=2, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)

    # rob.pid_distance(70, -70)
    rob.accelDecel(57, -70)
    wait(100)
    rob.turn(-45, 65)
    wait(100)
    rob.turn(45, 65)
    wait(100)
    arm.run_time(speed=-1000, time=1000, wait=True)
    # rob.pid_distance(5, -50)
    # wait(10)
    # rob.turn(-35, 50)
    # wait(10)
    # rob.pid_distance(10, -50)
    # wait(10)
    # rob.turn(75, 50)
    # wait(10)
    # rob.pid_distance(10, -50)
    # arm.run_time(speed=1000, time=1000, wait=True)
    # wait(10)
    # rob.pid(15, 50)
    # # rob.turn_one_wheel(-50, 70, "right")
    # rob.turn(-50, 70)
    # wait(10)
    rob.pid(100, 70)


###

def run7():
    rob = Robot(kp=2.5, ki=0, kd=16)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)

    rob.pid(60, -90)
    wait(400)
    rob.pid(3, 50)
    wait(400)
    arm.run_time(660, 1000)
    wait(400)
    rob.pid(5,50)
    wait(10)
    rob.pid(6, -100)
    wait(10)
    rob.pid(20, 100)
    wait(10)
    rob.pid(8, -50)
    wait(10)
    rob.pid(55, 100)
    wait(10)

###

def run8():
    rob = Robot(kp=1.8, ki=0, kd=11.493, turnKp=6.12, turnKi=0.604, turnKd=15.491, shellKp=21.6, shellKi=1.399, shellKd=83.362, shellTol=2, turnTol=2,armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=100)
    
    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)

    rob.battery_percent()

    # arm.run_time(1000, 500)
    # wait(100)
    # rob.pid_distance(25, -50)
    # wait(100)
    # rob.turnWhileShell(90, -50, 100,  50)
    # wait(100)
    # rob.pid_distance(48, -70)
    rob.accelDecel(42, -70)
    wait(500)
    rob.pid_distance(50, 100)
    wait(100)
    rob.shellTurn(-90, 100)
    
def run8point5():
    rob = Robot(kp=1.8, ki=0, kd=11.493, turnKp=6.12, turnKi=0.604, turnKd=15.491, shellKp=21.6, shellKi=1.399, shellKd=83.362, shellTol=2, turnTol=2,armKp=4.5, armKi=0, armKd=6.75, armTol=2, turn_wait_time=100)
    
    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)

    rob.battery_percent()

    arm.run_time(1000, 500)
    wait(10)
    rob.pid_distance(20, -50)
    wait(10)
    rob.shellTurn(90)
    wait(10)
    arm.run_time(1000, 500)
    wait(10)
    rob.pid_distance(20, -50)
    wait(10)
    rob.turn(-40, 50)
    wait(10)
    rob.pid_distance(36, -50)
    wait(10)
    rob.turn(-40, 50)
    wait(10)
    rob.pid_distance(10, -50)
    wait(10)
    arm.run_time(-1000, 1000)
    wait(10)
    rob.accelDecel(70, -60)
    wait(10)
    rob.shellTurn(-70)

###

def test():
    rob = Robot(kp=1.98, ki=0, kd=8.915, turnKp=6.12, turnKi=0.93, turnKd=10.073, shellKp=36.0, shellKi=10.105, shellKd=50.062, shellTol=2, turnTol=2, turn_wait_time=1) # Base Robot
    
    # rob.auto_tune_straight_precision(60, 50)
    # rob.auto_tune_shell(90)

    # rob.pid(30, -70)
    # wait(10)
    # rob.pid(30, 70)

    # rob.turn(180,40)
    # rob.pid(20,40)
    # rob.shellTurn(90)
    # rob.turn_one_wheel(90,)
    # rob.arc(30, 90, 50)

    # rob.pid_distance(50, 50, 1.98, 0, 30)
    # rob.accelDecel(50, 70)

    # wheels.drive(1000, 0);

    # arm.run_time(speed=1000, time=1500, wait=False)
    # arm
    
    # rob.pid_distance(20,-40)
    # wait(30)
    # rob.shellTurn(90)
    # wait(30)
    # rob.pid_distance(70,-40)
    # wait(30)
    # arm.run_time(-500,800)
    # wait(30)
    # rob.shellTurn(90)
    # while True:
    #     arm.dc(100)
    arm.run_time(-1000, 500)
    # wait(30)
    # rob.moveTime(1000,-30)
    # wait(30)
    # rob.pid_distance(2,30)
    # wait(30)
    # arm.run_time(-800,1000)
    # wait(30)
    # rob.turnWhileShell(90,-90.,100,50)
    # wait(30)
    # rob.pid_distance(10,-40)
    # wait(30)
    # rob.moveTime(1500,-25)
    # wait(30)
    # arm.run_time(1000,800)
    # wait(30)
    # arm.run_time(-1000,510)



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

def clean():
    while True:
        wheels.drive(1000, 0)

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

# run2point5()

# run3()

# run4()

# run5()

# run6()

# run6point5()

# run7()

# run8()

# run8point5()

# test()

# calibration()

# clean()

# battery()

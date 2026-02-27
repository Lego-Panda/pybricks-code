from robot_conf import CIRCUMFERENCE
from robot_conf import leftwheel
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import *

hub = PrimeHub(Axis.Y, Axis.Z)

def singnum(value):
    return value / abs(value)

class Robot:
    def __init__(self,kp=0, ki=0, kd=0,shellKp=0,shellKi=0,shellKd=0, shellTol=0, tol=0, wait_time=0, ks=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tol = tol
        self.wait_time = wait_time
        self.ks = ks
        self.shellKp = shellKp
        self.shellKi = shellKi
        self.shellKd = shellKd
        self.shellTol= shellTol

        self.errorSum = 0
        self.lastError = 0

    def pid(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        while abs(leftwheel.angle()) < distance / CIRCUMFERENCE * 360:
            error = 0 - hub.imu.heading()

            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            rightwheel.run(int(speed + pidValue))
            leftwheel.run(int(-speed + pidValue))

            self.lastError = error
            self.errorSum += error

            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def accelDecel(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        min_speed = 100 if speed >= 0 else -100

        thirty_percent_dist = (distance / CIRCUMFERENCE * 360) * 0.3

        while abs(leftwheel.angle()) < distance / CIRCUMFERENCE * 360:

            remaining_distance = (distance / CIRCUMFERENCE * 360) - abs(leftwheel.angle())

            error = 0  - hub.imu.heading()

            if abs(leftwheel.angle()) <= thirty_percent_dist and thirty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (abs(leftwheel.angle()) / thirty_percent_dist)

            elif remaining_distance <= thirty_percent_dist and thirty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (remaining_distance / thirty_percent_dist)

            else:
                current_speed = speed
                
            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            rightwheel.run(int(current_speed + pidValue))
            leftwheel.run(int(-current_speed + pidValue))

            self.lastError = error
            self.errorSum += error

            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def turn(self, degrees, speed):
        hub.imu.reset_heading(-degrees)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        on_setpoint = True
        time_at_setpoint = 0
        wait(100)

        while time_at_setpoint < self.wait_time:

            error = hub.imu.heading()

            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            rightwheel.run(int(speed * singnum(degrees) - pidValue))
            leftwheel.run(int(speed * singnum(degrees) - pidValue))

            self.lastError = error
            self.errorSum += error

            if not on_setpoint: time_at_setpoint += 0.02
            else: time_at_setpoint = 0

            on_setpoint = abs(hub.imu.heading()) >= self.tol

        leftwheel.stop()
        rightwheel.stop()

    def shellTurn(self, degrees, speed=800):
        
        self.errorSum = 0
        self.lastError = 0

        shell.reset_angle(0)

        while abs(shell.angle() * SHELL_RATIO) <= abs(degrees):

            error = shell.angle() * SHELL_RATIO

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell.run(int(speed * singnum(degrees) - pidValue))

            self.lastError = error
            self.errorSum += error

            on_setpoint = True
            time_at_setpoint = 0

            if not on_setpoint: time_at_setpoint += 0.02
            else: time_at_setpoint = 0

            on_setpoint = abs(shell.angle() * SHELL_RATIO) >= self.shellTol

            wait(100)

        shell.stop()

    def turnWhileShell(self, shellDegrees, turnDegrees, shellSpeed=500, turnSpeed=200):
        hub.imu.reset_heading(-turnDegrees)
        shell.reset_angle(0)
        self.errorSum = 0
        turnLastError = 0
        turnErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        turnAtSetPoint = False
        shellAtSetPoint = False
        turn_on_setpoint = True
        turn_time_at_setpoint = 0
        shell_on_setpoint = True
        shell_time_at_setpoint = 0
        wait(100)

        while not turnAtSetPoint and not shellAtSetPoint:

            if not turnAtSetPoint:
                turnError = hub.imu.heading()

                turnPidValue = self.kp * turnError + self.ki * turnErrorSum + self.kd * (turnError - turnLastError)

                rightwheel.run(int(turnSpeed * singnum(turnDegrees) - turnPidValue))
                leftwheel.run(int(turnSpeed * singnum(turnDegrees) - turnPidValue))

                turnLastError = turnError

                if not turn_on_setpoint: turn_time_at_setpoint += 0.02
                else: turn_time_at_setpoint = 0
                
                if not turn_on_setpoint:
                    turnAtSetPoint = True

                turn_on_setpoint = abs(hub.imu.heading()) >= self.tol

            if turnAtSetPoint:
                leftwheel.stop()
                rightwheel.stop()
                turnAtSetPoint = True

            if not shellAtSetPoint:
                shellError = shell.angle() * SHELL_RATIO

                shellPidValue = self.kp * shellError + self.ki * shellErrorSum + self.kd * (shellError - shellLastError)

                shell.run(int(shellSpeed * singnum(shellDegrees) - shellPidValue))

                shellLastError = shellError

                shell_on_setpoint = True
                shell_time_at_setpoint = 0

                if not shell_on_setpoint: shell_time_at_setpoint += 0.02
                else: shell_time_at_setpoint = 0

                if not shell_on_setpoint:
                    shellAtSetPoint = True

                shell_on_setpoint = abs(hub.imu.heading()) >= self.tol

                wait(100)

            if shellAtSetPoint:
                shell.stop()
                shellAtSetPoint = True
        
        shell.stop()
        leftwheel.stop()
        rightwheel.stop()

    def stopColor(self, preset, degrees=365, speed=500):
        self.errorSum = 0
        self.lastError = 0

        stop_color, slow_color = COLOR_PRESETS[preset]

        shell.reset_angle(0)

        while abs(shell.angle() * SHELL_RATIO) <= abs(degrees):

            error = shell.angle() * SHELL_RATIO

            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            shell.run(int(speed * singnum(degrees) - pidValue))

            self.lastError = error

            on_setpoint = True
            time_at_setpoint = 0

            current_color = colorS.color()

            if current_color == stop_color:
                wait(100)
                shell.stop()
                break
            elif current_color == slow_color:
                speed = 400

            if not on_setpoint:
                time_at_setpoint += 0.02
            else:
                time_at_setpoint = 0

            on_setpoint = (shell.angle() * SHELL_RATIO) >= self.tol 

            wait(100)

        shell.stop()

    def shellButton(self, degrees=365):
        while True:
            pressed = hub.buttons.pressed()
            if Button.RIGHT in pressed or Button.LEFT in pressed:
                self.stopColor("stopYellow", degrees)
                break
            

    def shellPitch(self):
        while True:
            if hub.imu.tilt()[0] < 90:
                Robot.stopColor(self, "stopYellow")
                break

    def battery_percent(self):
        voltage = hub.battery.voltage()

        min_v = 6000
        max_v = 8300

        percent = (voltage - min_v) / (max_v - min_v) * 100
        percent = max(0, min(100, percent))  # clamp 0–100

        print(percent)

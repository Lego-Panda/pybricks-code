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

    def decel(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        min_speed = 50
        decel_start = (distance / CIRCUMFERENCE * 360) * 0.3

        while abs(leftwheel.angle()) < distance / CIRCUMFERENCE * 360:

            remaining_distance = (distance / CIRCUMFERENCE * 360) - abs(leftwheel.angle())
            print(remaining_distance)
            if remaining_distance <= 40:
                break

            error = 0  - hub.imu.heading()

            if remaining_distance <= decel_start and decel_start > 0:
                current_speed = min_speed + (speed - min_speed) * (remaining_distance / decel_start)
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
        SHELL_RATIO = 1/3.78

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
                leftwheel.run(int(-turnSpeed * singnum(turnDegrees) - turnPidValue))

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

    def battery(self):
        voltage = hub.battery.voltage()
        
        # 2. Define the absolute physical limits of the battery
        MIN_VOLTAGE = 6500
        MAX_VOLTAGE = 8400
        
        # 3. Map the voltage to a 100-point scale
        percentage = (voltage - MIN_VOLTAGE) * 100 / (MAX_VOLTAGE - MIN_VOLTAGE)
        
        # 4. "Clamp" the value so it cannot exceed 100% or drop below 0%
        # (Sometimes a fresh off-the-charger battery spikes to 8450mV for a few seconds)
        percentage = max(0, min(100, percentage))
        
        return round(percentage, 1)

        # --- Execution ---
        print("Battery Level:", get_battery_percentage(), "%")
        print("Raw Voltage:", hub.battery.voltage(), "mV")
        print("Raw Current:", hub.battery.current(), "mA")

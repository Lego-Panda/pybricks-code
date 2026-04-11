from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import *

hub = PrimeHub(Axis.Y, Axis.Z)

class Robot:
    def __init__(self,kp=0.0, ki=0.0, kd=0.0, turnKp=0.0, turnKi=0.0, turnKd=0.0, shellKp=0.0, shellKi=0.0, shellKd=0.0, shellTol=0.0, turnTol=2, armKp=0.0, armKi=0.0, armKd=0.0, armTol=0.0, turn_wait_time=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.turnKp = turnKp
        self.turnKi = turnKi
        self.turnKd = turnKd
        self.turnTol = turnTol
        self.turn_wait_time = turn_wait_time
        self.shellKp = shellKp
        self.shellKi = shellKi
        self.shellKd = shellKd
        self.shellTol= shellTol
        self.armKp = armKp
        self.armKi = armKi
        self.armKd = armKd
        self.armTol = armTol

        self.errorSum = 0
        self.lastError = 0

    def pid(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        target_angle = (distance / CIRCUMFERENCE) * 360

        self.errorSum = 0
        self.lastError = 0

        while (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2 < target_angle:

            error = -hub.imu.heading()

            self.errorSum = max(-50, min(50, self.errorSum + error))
            
            pidValue = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * (error - self.lastError))

            leftwheel.dc(max(-100, min(100, speed - pidValue)))
            rightwheel.dc(max(-100, min(100, speed + pidValue)))

            self.lastError = error
            wait(10)

            if leftwheel.stalled() or rightwheel.stalled():
                wait(500)
                break

        leftwheel.brake()
        rightwheel.brake()

    def moveTime(self, duration, speed):
        timer = StopWatch()
        
        while True:
            if timer.time() < duration:
                leftwheel.dc(speed)
                rightwheel.dc(speed)
            else:
                leftwheel.brake()
                rightwheel.brake()
                break
            wait(10)

    def pid_distance(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        target_spins = distance / CIRCUMFERENCE

        self.errorSum = 0
        self.lastError = 0

        while True:
            average_angle = (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2
            current_spins = average_angle / 360

            distance_left = target_spins - current_spins

            if distance_left < 0.01:
                break

            error = -hub.imu.heading()

            self.errorSum = max(-50, min(50, self.errorSum + error))
            
            pidValue = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * (error - self.lastError))

            leftwheel.dc(max(-100, min(100, speed - pidValue)))
            rightwheel.dc(max(-100, min(100, speed + pidValue)))

            self.lastError = error
            wait(10)

        leftwheel.dc(0)
        rightwheel.dc(0)

    def pid_distance_stuck(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        target_spins = distance / CIRCUMFERENCE

        self.errorSum = 0
        self.lastError = 0

        while True:
            average_angle = (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2
            current_spins = average_angle / 360

            distance_left = target_spins - current_spins

            if distance_left < 0.01:
                break

            error = -hub.imu.heading()

            self.errorSum = max(-50, min(50, self.errorSum + error))
            
            pidValue = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * (error - self.lastError))

            leftwheel.dc(max(-100, min(100, speed - pidValue)))
            rightwheel.dc(max(-100, min(100, speed + pidValue)))

            if rightwheel.stalled() or leftwheel.stalled():
                break

            self.lastError = error
            wait(10)

        leftwheel.dc(0)
        rightwheel.dc(0)

    def accelDecel(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        min_speed = 25 if speed >= 0 else -25
        target_angle = (distance / CIRCUMFERENCE) * 360

        twenty_percent_dist = (distance / CIRCUMFERENCE * 360) * 0.2

        while (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2 < target_angle:

            remaining_distance = (distance / CIRCUMFERENCE * 360) - ((abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2)

            error = -hub.imu.heading()

            self.errorSum = max(-50, min(50, self.errorSum + error))

            if abs(leftwheel.angle()) <= twenty_percent_dist and twenty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (abs(leftwheel.angle()) / twenty_percent_dist)

            elif remaining_distance <= twenty_percent_dist and twenty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (remaining_distance / twenty_percent_dist)

            else:
                current_speed = speed
                
            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            leftwheel.dc(max(-100, min(100, current_speed - pidValue)))
            rightwheel.dc(max(-100, min(100, current_speed + pidValue)))

            self.lastError = error

            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def arc(self, distance, target_angle, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        target_distance_degrees = (distance / CIRCUMFERENCE) * 360

        self.errorSum = 0
        self.lastError = 0

        while True:
            current_distance_degrees = (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2

            if current_distance_degrees >= target_distance_degrees:
                break

            progress = current_distance_degrees / target_distance_degrees
            current_target_heading = target_angle * progress
            error = current_target_heading - hub.imu.heading()

            # I-Zone and Clamp
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0

            pidValue = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * (error - self.lastError))

            leftwheel.dc(int(max(-100, min(100, speed - pidValue))))
            rightwheel.dc(int(max(-100, min(100, speed + pidValue))))

            self.lastError = error
            wait(10)

        leftwheel.brake()
        rightwheel.brake()


    # --------------------------------
    # ------------- TURN -------------
    # --------------------------------
    
    def turn(self, degrees, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.turn_wait_time:

            current_heading = hub.imu.heading()
            error = degrees - current_heading
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.turnKp * error + self.turnKi * self.errorSum + self.turnKd * (error - self.lastError)

            left_power = int(max(-speed, min(speed, -pidValue)))
            right_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| PID:", pidValue, "| L_PWR:", left_power, "| R_PWR:", right_power)

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            self.lastError = error

            if abs(error) <= self.turnTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            wait(20)

        leftwheel.brake()
        rightwheel.brake()

    def turn_stuck(self, degrees, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.turn_wait_time:

            current_heading = hub.imu.heading()
            error = degrees - current_heading
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.turnKp * error + self.turnKi * self.errorSum + self.turnKd * (error - self.lastError)

            left_power = int(max(-speed, min(speed, -pidValue)))
            right_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| PID:", pidValue, "| L_PWR:", left_power, "| R_PWR:", right_power)

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            self.lastError = error

            if abs(error) <= self.turnTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0

            if rightwheel.stalled() or leftwheel.stalled():
                break
            
            wait(20)

        leftwheel.brake()
        rightwheel.brake()


    def turn_one_wheel(self, degrees, speed, active_wheel="left"):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.turn_wait_time:

            current_heading = hub.imu.heading()
            error = degrees - current_heading
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.turnKp * error + self.turnKi * self.errorSum + self.turnKd * (error - self.lastError)

            if active_wheel == "left":
                left_power = int(max(-speed, min(speed, -pidValue)))
                print("Error:", error, "| PID:", pidValue, "| L_PWR:", left_power, "| Pivot: Right")
                leftwheel.dc(left_power)
                rightwheel.brake()
                
            elif active_wheel == "right":
                right_power = int(max(-speed, min(speed, pidValue)))
                print("Error:", error, "| PID:", pidValue, "| R_PWR:", right_power, "| Pivot: Left")
                rightwheel.dc(right_power)
                leftwheel.brake()
                
            else:
                print("Invalid wheel. Use 'left' or 'right'.")
                break

            self.lastError = error

            if abs(error) <= self.turnTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            wait(20)

        leftwheel.brake()
        rightwheel.brake()

    # --------------------------------
    # ------------- SHELL ------------
    # --------------------------------
    
    def shellTurn(self, degrees, speed=100):
        
        self.errorSum = 0
        self.lastError = 0
        
        shell.reset_angle(0)
        
        time_at_setpoint = 0
        wait(100) # Let the physical mechanism settle before moving

        while True:

            current_angle = shell.angle() * SHELL_RATIO
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| Pwr:", shell_power, "| Timer:", time_at_setpoint)

            shell.dc(shell_power)

            self.lastError = error

            if shell.stalled():
                break

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            wait(20)

        shell.brake()

    def shellTurnTime(self, duration, speed):
        timer = StopWatch()
        
        while True:
            if timer.time() < duration:
                shell.dc(speed)
            else:
                shell.brake()
                break
            wait(10)

    def turnWhileShell(self, shellDegrees, turnDegrees, shellSpeed=50, turnSpeed=50):
        hub.imu.reset_heading(0)
        shell.reset_angle(0)

        self.errorSum = 0
        turnLastError = 0
        turnErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        turn_time_at_setpoint = 0
        shell_time_at_setpoint = 0
        wait(100)

        while turn_time_at_setpoint < 100 or shell_time_at_setpoint < 100:

            # ==========================================
            #              Robot Base PID
            # ==========================================
            turn_error = turnDegrees - hub.imu.heading()
            
            if abs(turn_error) < 15:
                turnErrorSum = max(-50, min(50, turnErrorSum + turn_error))
            else:
                turnErrorSum = 0 

            turnPid = self.turnKp * turn_error + self.turnKi * turnErrorSum + self.turnKd * (turn_error - turnLastError)

            left_power = int(max(-turnSpeed, min(turnSpeed, -turnPid)))
            right_power = int(max(-turnSpeed, min(turnSpeed, turnPid)))

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            turnLastError = turn_error

            if abs(turn_error) <= self.turnTol: 
                turn_time_at_setpoint += 20
            else: 
                turn_time_at_setpoint = 0

            # ==========================================
            #                 Shell PID
            # ==========================================
            shell_error = shellDegrees - (shell.angle() * SHELL_RATIO)
            
            if abs(shell_error) < 15:
                shellErrorSum = max(-50, min(50, shellErrorSum + shell_error))
            else:
                shellErrorSum = 0 

            shellPid = self.shellKp * shell_error + self.shellKi * shellErrorSum + self.shellKd * (shell_error - shellLastError)

            shell_power = int(max(-shellSpeed, min(shellSpeed, shellPid)))

            shell.dc(shell_power)

            shellLastError = shell_error

            if abs(shell_error) <= self.shellTol: 
                shell_time_at_setpoint += 20
            else: 
                shell_time_at_setpoint = 0
            
            print("TurnErr:", turn_error, "TurnPwr:", left_power, "|| ShellErr:", shell_error, "ShellPwr:", shell_power)

            wait(20)
        
        shell.brake()
        leftwheel.brake()
        rightwheel.brake()

    
    def stopColor(self, preset, degrees, speed=100):
        
        stop_color, slow_color = COLOR_PRESETS[preset]

        self.errorSum = 0
        self.lastError = 0
        
        shell.reset_angle(0)
        
        time_at_setpoint = 0
        wait(100) 

        while True:

            current_angle = shell.angle() * SHELL_RATIO
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| Pwr:", shell_power, "| Timer:", time_at_setpoint)

            shell.dc(shell_power)

            current_color = colorS.color()

            if current_color == stop_color:
                wait(100)
                shell.brake()
                break
            elif current_color == slow_color:
                speed = 40

            self.lastError = error

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20 
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            wait(20)

        shell.brake()

    def shellButton(self, degrees=365):
        while True:
            pressed = hub.buttons.pressed()
            if Button.RIGHT in pressed or Button.LEFT in pressed:
                self.stopColor("stopYellow", degrees)
                break

    def battery_percent(self):
        voltage = hub.battery.voltage()

        min_v = 6000
        max_v = 8300

        percent = (voltage - min_v) / (max_v - min_v) * 100
        percent = max(0, min(100, percent))

        print(percent)

    def armPID(self, degrees, speed):
        self.errorSum = 0
        self.lastError = 0
        
        stuck_timer = 0
        time_at_setpoint = 0
        
        wait(100) 

        while True:
            current_angle = arm.angle()
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.armKp * error + self.armKi * self.errorSum + self.armKd * (error - self.lastError)
            arm_power = int(max(-speed, min(speed, pidValue)))

            arm.dc(arm_power)

            if abs(error - self.lastError) < 0.5:
                stuck_timer += 20 
            else:
                stuck_timer = 0 

            if stuck_timer >= 1500:
                print("Arm stuck for 1.5s - Stopping.")
                break

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            self.lastError = error
            wait(20)

        arm.stop() 

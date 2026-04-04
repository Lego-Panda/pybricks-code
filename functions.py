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

            if rightwheel.stalled() or leftwheel.stalled():
                break

            self.lastError = error
            wait(10)

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

            if rightwheel.stalled() or leftwheel.stalled():
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

        # THE FIX: The loop continues until BOTH timers reach 100ms at the exact same time.
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
        percent = max(0, min(100, percent))  # clamp 0–100

        print(percent)

    def moveWhileShell(self, shellDegrees, moveDistance, shellSpeed=500, moveSpeed=150):

        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)
        shell.reset_angle(0)
        self.errorSum = 0
        moveLastError = 0
        moveErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        moveAtSetPoint = False
        shellAtSetPoint = False
        shell_on_setpoint = True
        shell_time_at_setpoint = 0
        wait(10)

        while not moveAtSetPoint or not shellAtSetPoint:
            if not moveAtSetPoint:
                if abs(leftwheel.angle()) >= moveDistance / CIRCUMFERENCE * 360:
                    moveAtSetPoint = True
                    wheels.brake()
                else:
                    moveError = 0 - hub.imu.heading() 
                    movePidValue = self.kp * moveError + self.ki * moveErrorSum + self.kd * (moveError - moveLastError)

                    rightwheel.run(int(moveSpeed + movePidValue))
                    leftwheel.run(int(moveSpeed - movePidValue))

                    moveLastError = moveError
                    moveErrorSum += moveError

            if not shellAtSetPoint:
                currentShellAngle = shell.angle() * SHELL_RATIO
                
                if abs(currentShellAngle) >= abs(shellDegrees):
                    shellAtSetPoint = True
                    shell.brake()
                else:
                    direction = 1 if shellDegrees > 0 else -1
                    shell.run(shellSpeed * direction)

            wait(10)

    def armPID(self, degrees, speed):
        
        self.errorSum = 0
        self.lastError = 0
        
        arm.reset_angle(0)
        
        time_at_setpoint = 0
        wait(100) # Let the physical mechanism settle before moving

        while True:

            current_angle = arm.angle() * 1
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.armKp * error + self.armKi * self.errorSum + self.armKd * (error - self.lastError)

            arm_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| Pwr:", arm_power, "| Timer:", time_at_setpoint)

            arm.dc(arm_power)

            self.lastError = error

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            wait(20)

        arm.brake()

    def arm(self, target_angle):
        arm.run_target(500, target_angle, wait=False)
        
        while not arm.done():
            current_angle = arm.angle()
            remaining = target_angle - current_angle
            
            print("Current: {}° | Remaining: {}°".format(current_angle, remaining))
            
            if current_angle > 100:
                print("Warning: High angle!")
                
            # Short pause to prevent the output window from flickering
            wait(100) 
            
        print("Target reached.")

    # --------------------------------
    # ------------ TUNING ------------
    # --------------------------------

    def auto_tune_turn(self, target_degrees=90, speed=50):
        test_kp = 3
        ku = 0.0
        tu = 0.0
        dt = 0.02

        while True:
            hub.imu.reset_heading(0)
            leftwheel.  reset_angle(0)
            rightwheel.reset_angle(0)
            wait(200)

            timer = StopWatch()
            zero_crossings = 0
            last_error = target_degrees
            cross_times = []

            # Run the test for exactly 3 seconds
            while timer.time() < 3000:
                current_heading = hub.imu.heading()
                error = target_degrees - current_heading

                # Detect if the robot crosses the target line
                if (last_error > 0 and error <= 0) or (last_error < 0 and error >= 0):
                    # Ignore the very first split-second initialization
                    if timer.time() > 50: 
                        zero_crossings += 1
                        cross_times.append(timer.time())

                # Proportional-only control for the test
                pidValue = test_kp * error

                left_power = int(max(-speed, min(speed, -pidValue)))
                right_power = int(max(-speed, min(speed, pidValue)))

                leftwheel.dc(left_power)
                rightwheel.dc(right_power)

                last_error = error
                wait(20)

            leftwheel.brake()
            rightwheel.brake()

            print("Tested Kp:", test_kp, "| Crossings:", zero_crossings)

            # 10 crossings indicates sustained oscillation (wobble) without damping
            if zero_crossings >= 10:
                ku = test_kp
                
                # A full period (Tu) is the time between crossing 1 and crossing 3
                if len(cross_times) >= 10:
                    periods = []
                    for i in range(len(cross_times) - 2):
                        periods.append(cross_times[i+2] - cross_times[i])
                        avg_tu_ms = sum(periods) / len(periods)
                        tu = avg_tu_ms / 1000.0
                        print("Average Tu calculated from", len(periods), "samples is:",tu)
                    
                else:
                    tu = 0.5 # Failsafe period if array is too small

                break

            else:
                # If it didn't wobble enough, increase Kp and repeat
                test_kp += 0.1
                wait(1000)

        # Ziegler-Nichols Calculation mapped to a static 20ms loop without dt integration
        final_kp = 0.60 * ku
        final_ki = (1.20 * ku / tu) * dt
        final_kd = (0.075 * ku * tu) / dt

        print("--- AUTO-TUNE COMPLETE ---")
        print("Ultimate Gain (Ku):", ku)
        print("Ultimate Period (Tu):", round(tu, 3), "sec")
        print("--------------------------")
        print("Set self.turnKp =", round(final_kp, 3))
        print("Set self.turnKi =", round(final_ki, 3))
        print("Set self.turnKd =", round(final_kd, 3))

        return final_kp, final_ki, final_kd
    
    def auto_tune_straight_precision(self, max_distance_cm=60.0, speed=40):
        test_kp = 0.5
        dt = 0.01 
        
        target_crossings = 12 
        
        # Calculation now uses the variable passed into the function
        max_degrees = (max_distance_cm / CIRCUMFERENCE) * 360

        print("--- STARTING STRAIGHT AUTO-TUNE ---")
        print("Robot will only drive FORWARD up to", max_distance_cm, "cm.")

        while True:
            print("\nPlace robot at the START line.")
            print("Press ANY button on the hub to start run with Kp:", round(test_kp, 2))
            
            while not hub.buttons.pressed():
                wait(10)
            
            while hub.buttons.pressed():
                wait(10)
            
            wait(500) 

            hub.imu.reset_heading(0)
            leftwheel.reset_angle(0)
            rightwheel.reset_angle(0)

            timer = StopWatch()
            zero_crossings = 0
            last_error = 0
            cross_times = []

            while zero_crossings < target_crossings:
                
                abs_position = (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2
                
                # Dynamic boundary check
                if abs_position >= max_degrees:
                    print("--> Reached", max_distance_cm, "cm limit! Stopping.")
                    break 
                
                error = 0 - hub.imu.heading()

                if (last_error > 0 and error <= 0) or (last_error < 0 and error >= 0):
                    if timer.time() > 100:
                        zero_crossings += 1
                        cross_times.append(timer.time())
                        print("Wobble detected! Crossing", zero_crossings, "out of", target_crossings)

                pidValue = test_kp * error

                left_power = int(max(-100, min(100, speed - pidValue)))
                right_power = int(max(-100, min(100, speed + pidValue)))

                leftwheel.dc(left_power)
                rightwheel.dc(right_power)

                last_error = error
                wait(10)

            leftwheel.brake()
            rightwheel.brake()

            if zero_crossings >= target_crossings:
                ku = test_kp
                
                periods = []
                for i in range(len(cross_times) - 2):
                    periods.append(cross_times[i+2] - cross_times[i])
                
                avg_tu_ms = sum(periods) / len(periods)
                tu = avg_tu_ms / 1000.0
                
                print("Tested Kp:", round(test_kp, 2), "| Crossings:", zero_crossings)
                print("Average Tu calculated from", len(periods), "samples is:", round(tu, 3))
                break 
                
            else:
                print("Kp:", round(test_kp, 2), "failed (Not enough wobbles:",zero_crossings,").")
                print("Increasing Kp by 0.1 for the next attempt...")
                test_kp += 0.1 

        final_kp = 0.60 * ku
        final_ki = (1.20 * ku / tu) * dt
        final_kd = (0.075 * ku * tu) / dt

        print("\n--- STRAIGHT PRECISION TUNE COMPLETE ---")
        print("Ultimate Gain (Ku):", round(ku, 2))
        print("Average Period (Tu):", round(tu, 3), "sec")
        print("--------------------------")
        print("Set self.straightKp =", round(final_kp, 3))
        print("Set self.straightKi =", round(final_ki, 3))
        print("Set self.straightKd =", round(final_kd, 3))
        
        self.straightKp = final_kp
        self.straightKi = final_ki
        self.straightKd = final_kd

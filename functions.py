from rob_conf import *
import umath as math

# ---

class Move:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, kp_curve=0.0, ki_curve=0.0, kd_curve=0.0):

        self.kp, self.ki, self.kd = kp, ki, kd
        self.kp_curve, self.ki_curve, self.kd_curve = kp_curve, ki_curve, kd_curve
        self.errorSum = self.lastError = 0

    def move(self, distance, speed, wait_time):
        gyro.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)
        timer = StopWatch()

        target_distance = (distance / WHEEL_CIRCUMFEFRENCE) * 360
        self.errorSum = self.lastError = last_time = 0
        
        wait(100)

        while (rightwheel.angle() + leftwheel.angle()) / 2 < target_distance:
            current_time = timer.time() / 1000.0 
            dt = current_time - last_time
            
            if dt <= 0: 
                wait(1)
                continue

            error = -gyro.heading()

            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error * dt))
            else:
                self.errorSum = 0 

            derivative = (error - self.lastError) / dt

            correction = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * derivative)

            left_p = int(max(-100, min(100, speed - correction)))
            right_p = int(max(-100, min(100, speed + correction)))
            
            leftwheel.dc(left_p)
            rightwheel.dc(right_p)

            self.lastError = error
            last_time = current_time

            wait(10)

        leftwheel.brake()
        rightwheel.brake()
        wait(wait_time)

    def move(self, distance, speed, accel_distance=0, decel_distance=0, wait_time=10):
        gyro.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)
        timer = StopWatch()

        target_deg = (distance / WHEEL_CIRCUMFEFRENCE) * 360
        accel_deg = (accel_distance / WHEEL_CIRCUMFEFRENCE) * 360
        decel_deg = (decel_distance / WHEEL_CIRCUMFEFRENCE) * 360

        min_speed=25
        
        self.errorSum = self.lastError = last_time = 0
        wait(100)

        current_deg = 0
        while current_deg < target_deg:
            current_time = timer.time() / 1000.0 
            dt = current_time - last_time
            
            if dt <= 0: 
                wait(1)
                continue

            current_deg = (rightwheel.angle() + leftwheel.angle()) / 2
            remaining_deg = target_deg - current_deg

            current_target_speed = speed

            if current_deg < accel_deg:
                current_target_speed = min_speed + (speed - min_speed) * (current_deg / accel_deg)
            
            if remaining_deg < decel_deg:
                decel_speed = min_speed + (speed - min_speed) * (remaining_deg / decel_deg)
                current_target_speed = min(current_target_speed, decel_speed)

            current_target_speed = max(min_speed, current_target_speed)

            error = -gyro.heading()

            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error * dt))
            else:
                self.errorSum = 0 

            derivative = (error - self.lastError) / dt
            correction = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * derivative)

            leftwheel.dc(int(max(-100, min(100, current_target_speed - correction))))
            rightwheel.dc(int(max(-100, min(100, current_target_speed + correction))))

            self.lastError = error
            last_time = current_time
            wait(10)

        leftwheel.brake()
        rightwheel.brake()
        wait(wait_time)

    def curve(self, radius, angle, speed):
        gyro.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        dist = (math.pi * abs(radius) * abs(angle)) / 180
        target_distance = (dist / WHEEL_CIRCUMFEFRENCE) * 360

        r_left = radius + 50
        r_right = radius - 50

        max_r = max(abs(r_left), abs(r_right))
        base_v_left = speed * (r_left / max_r)
        base_v_right = speed * (r_right / max_r)

        self.errorSum = self.lastError = last_time = 0
        timer = StopWatch()

        while (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2 < target_distance:
            curr_time = timer.time()
            dt = (curr_time - last_time) / 1000.0
            if dt <= 0:
                wait(1)
                continue
            last_time = curr_time

            progress = ((abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2) / target_distance
            target_heading = progress * angle

            error = target_heading - gyro.heading()
            self.errorSum = max(-50, min(50, self.errorSum + (error * dt)))
            derivative = (error - self.lastError) / dt

            correction = (self.kp_curve * error) + (self.ki_curve * self.errorSum) + (self.kd_curve * derivative)

            leftwheel.dc(max(-100, min(100, base_v_left - correction)))
            rightwheel.dc(max(-100, min(100, base_v_right + correction)))

            self.lastError = error

            wait(20)

        leftwheel.brake()
        rightwheel.brake()

class Turn:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, tol=2, wait_time=1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.tol = tol
        self.wait_time = wait_time
        self.lastError = 0
        self.errorSum = 0

    def turn(self, deg, speed, wait_time_after=10):
        gyro.reset_heading(0)
        timer = StopWatch()
        self.lastError = self.errorSum = last_time = time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.wait_time:
            current_time = timer.time() / 1000.0
            dt = current_time - last_time
            if dt <= 0:
                wait(1)
                continue

            current_heading = hub.imu.heading()
            error = deg - current_heading
            
            if abs(error) < 10:
                self.errorSum += error * dt
                self.errorSum = max(-20, min(20, self.errorSum))
            else:
                self.errorSum = 0

            derivative = (error - self.lastError) / dt

            print(error)

            correction = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * derivative)

            leftwheel.dc(int(max(-speed, min(speed, -correction))))
            rightwheel.dc(int(max(-speed, min(speed, correction))))

            self.lastError = error
            last_time = current_time

            if abs(error) <= self.tol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            wait(20)

        leftwheel.brake()
        rightwheel.brake()
        wait(wait_time_after)

    def turn_one_wheel(self, degrees, speed, wheel, wait_time_after=10):
        gyro.reset_heading(0)
        timer = StopWatch()
        self.lastError = self.errorSum = last_time = time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.wait_time:
            current_time = timer.time() / 1000.0 
            dt = current_time - last_time

            if dt <= 0:
                wait(1)
                continue

            current_heading = gyro.heading()
            error = degrees - current_heading

            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error * dt))
            else:
                self.errorSum = 0 

            derivative = (error - self.lastError) / dt

            correction = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * derivative)

            # wheel picking
            if wheel == "left" or "l":
                leftwheel.dc(max(-speed, min(speed, -correction)))
            elif wheel == "right" or "r":
                rightwheel.dc(max(-speed, min(speed, correction)))

            self.lastError = error
            last_time = current_time

            print("Error: ", error)

            if abs(error) <= 3:
                time_at_setpoint += 20
            else:
                time_at_setpoint = 0

            wait(10)

        leftwheel.brake()
        rightwheel.brake()
        wait(wait_time_after)

    def lqr(self, deg, speed, k1, k2):
        gyro.reset_heading(0)
        time_at_setpoint = 0

        while time_at_setpoint < self.wait_time:
            current_heading = gyro.heading()
            error = deg - current_heading
            
            current_rate = hub.imu.angular_velocity(Axis.Z)

            correction = (k1 * error) - (k2 * current_rate) 

            l_power = max(-speed, min(speed, -correction))
            r_power = max(-speed, min(speed, correction))

            leftwheel.dc(int(l_power))
            rightwheel.dc(int(r_power))

            print(error)

            if abs(error) <= 3:
                time_at_setpoint += 20
            else:
                time_at_setpoint = 0

            wait(20)

        leftwheel.brake()
        rightwheel.brake()


class Shell:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, tol=2.0, wait_time=100):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.tol, self.wait_time = tol, wait_time
        self.lastError = self.errorSum = 0

    def shellTurn(self, deg, speed, wait_time_after=10):
        shell.reset_angle(0)
        timer = StopWatch()
        self.lastError = self.errorSum = last_time = time_at_setpoint = 0
        wait(10)

        while time_at_setpoint < self.wait_time:
            current_time = timer.time() / 1000.0
            dt = current_time - last_time
            if dt <= 0:
                wait(1)
                continue

            curr_angle = shell.angle() * SHELL_RATIO
            error = deg - curr_angle
            
            if abs(error) < 10:
                self.errorSum += error * dt
                self.errorSum = max(-20, min(20, self.errorSum))
            else:
                self.errorSum = 0

            derivative = (error - self.lastError) / dt

            print(error)

            correction = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * derivative)

            shell.dc(max(-speed, min(speed, correction)))

            self.lastError = error
            last_time = current_time

            if abs(error) <= self.tol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            wait(20)

        shell.brake()
        wait(wait_time_after)

    def turn_until_color(self, target_color, speed, wait_after=10):

        shell.dc(speed)

        while True:
            h, s, v = colorSens.hsv()

            if s > 40:
                if target_color == "red" or "Red":
                    if h < 15 or h > 345:
                        shell.hold()
                        break
                elif target_color == "yellow" or "Yellow":
                    if 40 < h < 70:
                        shell.hold()
                        break
                elif target_color == "green" or "Green":
                    if 100 < h < 150:
                        shell.hold()
                        break
                elif target_color == "blue" or "Blue":
                    if 200 < h < 250:
                        shell.hold()
                        break

        wait(wait_after)


    def turnWhileShell(self, shellDegrees, turnDegrees, shellSpeed=100, turnSpeed=50, wait_time_after=10):
        gyro.reset_heading(0)
        shell.reset_angle(0)
        timer = StopWatch()

        turnLastError = turnErrorSum = turn_time_at_setpoint = 0
        shellLastError = shellErrorSum = shell_time_at_setpoint = 0
        
        last_time = 0
        wait(100)

        while turn_time_at_setpoint < Turn.wait_time or shell_time_at_setpoint < self.wait_time:
            current_time = timer.time() / 1000.0
            dt = current_time - last_time
            if dt <= 0:
                wait(1)
                continue

            current_heading = hub.imu.heading()
            turn_error = turnDegrees - current_heading
            
            if abs(turn_error) < 10:
                turnErrorSum += turn_error * dt
                turnErrorSum = max(-20, min(20, turnErrorSum))
            else:
                turnErrorSum = 0

            turn_derivative = (turn_error - turnLastError) / dt
            turn_correction = (Turn.kp * turn_error) + (Turn.ki * turnErrorSum) + (Turn.kd * turn_derivative)

            if turn_time_at_setpoint < Turn.wait_time:
                leftwheel.dc(int(max(-turnSpeed, min(turnSpeed, -turn_correction))))
                rightwheel.dc(int(max(-turnSpeed, min(turnSpeed, turn_correction))))
            else:
                leftwheel.brake()
                rightwheel.brake()

            if abs(turn_error) <= Turn.tol: 
                turn_time_at_setpoint += 20
            else: 
                turn_time_at_setpoint = 0

            curr_shell_angle = shell.angle() * SHELL_RATIO
            shell_error = shellDegrees - curr_shell_angle
            
            if abs(shell_error) < 10:
                shellErrorSum += shell_error * dt
                shellErrorSum = max(-20, min(20, shellErrorSum))
            else:
                shellErrorSum = 0

            shell_derivative = (shell_error - shellLastError) / dt
            shell_correction = (self.kp * shell_error) + (self.ki * shellErrorSum) + (self.kd * shell_derivative)

            if shell_time_at_setpoint < self.wait_time:
                shell.dc(int(max(-shellSpeed, min(shellSpeed, shell_correction))))
            else:
                shell.brake()

            if abs(shell_error) <= self.tol: 
                shell_time_at_setpoint += 20
            else: 
                shell_time_at_setpoint = 0

            turnLastError = turn_error
            shellLastError = shell_error
            last_time = current_time
            
            wait(20)

        shell.brake()
        leftwheel.brake()
        rightwheel.brake()
        wait(wait_time_after)

    def moveWhileShell(self, distance, shellDegrees, moveSpeed=50, shellSpeed=100, wait_time_after=10):
        gyro.reset_heading(0)
        shell.reset_angle(0)

        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        target_distance = (distance / WHEEL_CIRCUMFEFRENCE) * 360

        self.errorSum = 0
        moveLastError = 0
        moveErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        shell_time_at_setpoint = 0
        wait(100)

        while shell_time_at_setpoint < 100 and leftwheel.angle() + rightwheel.angle() / 2 < target_distance:
            moveError = -gyro.heading()

            moveCorrection = (Move.kp * moveError) + (Move.ki * moveErrorSum) + (Move.kd * (moveError - moveLastError))

            leftwheel.dc(max(-100, min(100, moveSpeed - moveCorrection)))
            rightwheel.dc(max(-100, min(100, moveSpeed + moveCorrection)))

            moveErrorSum += moveError
            moveLastError = moveError

            # ---

            shell_error = shellDegrees - (shell.angle() * SHELL_RATIO)

            if abs(shell_error) < 15:
                shellErrorSum = max(-50, min(50, shellErrorSum + shell_error))
            else:
                shellErrorSum = 0 

            shellCorrection = (self.kp * shell_error) + (self.ki * shellErrorSum) + (self.kd * (shell_error - shellLastError))

            shell.dc(max(-shellSpeed, min(shellSpeed, shellCorrection)))

            shellLastError = shell_error

            if abs(shell_error) <= self.tol: 
                shell_time_at_setpoint += 20
            else: 
                shell_time_at_setpoint = 0

            wait(10)

        shell.brake()
        rightwheel.brake()
        leftwheel.brake()
        wait(wait_time_after)

class Calibration:
    def __init__(self, tol=2, wait_time=100):
        self.tol, self.wait_time= tol, wait_time

        self.errorSum = self.lastError = 0

    def auto_tune_turn(self, target_degrees=90, speed=50):
        test_kp = 3.0
        ku = 0.0
        tu = 0.0

        while True:
            hub.imu.reset_heading(0)
            leftwheel.reset_angle(0)
            rightwheel.reset_angle(0)
            wait(200)

            timer = StopWatch()
            zero_crossings = 0
            last_error = target_degrees
            cross_times = []
            last_time = 0

            while timer.time() < 3000:
                current_time = timer.time() / 1000.0
                dt = current_time - last_time
                
                current_heading = hub.imu.heading()
                error = target_degrees - current_heading

                if (last_error > 0 and error <= 0) or (last_error < 0 and error >= 0):
                    if timer.time() > 50: 
                        zero_crossings += 1
                        cross_times.append(current_time)

                correction = test_kp * error

                left_power = int(max(-speed, min(speed, -correction)))
                right_power = int(max(-speed, min(speed, correction)))

                leftwheel.dc(left_power)
                rightwheel.dc(right_power)

                last_error = error
                last_time = current_time
                wait(10)

            leftwheel.brake()
            rightwheel.brake()

            print("Tested Kp:", test_kp, "| Crossings:", zero_crossings)

            if zero_crossings >= 10:
                ku = test_kp
                
                if len(cross_times) >= 10:
                    periods = []
                    for i in range(len(cross_times) - 2):
                        periods.append(cross_times[i+2] - cross_times[i])
                    tu = sum(periods) / len(periods)
                    print("Average Tu calculated:", round(tu, 3), "s")
                else:
                    tu = 0.5

                break
            else:
                test_kp += 0.2
                wait(1000)

        final_kp = 0.60 * ku
        final_ki = (1.20 * ku / tu)
        final_kd = (0.075 * ku * tu)

        print("--- AUTO-TUNE COMPLETE ---")
        print("Ultimate Gain (Ku):", ku)
        print("Ultimate Period (Tu):", round(tu, 3), "sec")
        print("--------------------------")
        print("Set self.kp =", round(final_kp, 3))
        print("Set self.ki =", round(final_ki, 6))
        print("Set self.kd =", round(final_kd, 3))

        return final_kp, final_ki, final_kd
    
    def auto_tune_straight(self, max_distance=600.0, speed=50):
        test_kp = 0.6
        target_crossings = 12 
        
        max_degrees = (max_distance / WHEEL_CIRCUMFEFRENCE) * 360

        print("--- STARTING STRAIGHT AUTO-TUNE ---")
        print("Robot will only drive FORWARD up to ", max_distance, "mm.")

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
            last_time = 0

            while zero_crossings < target_crossings:
                current_time = timer.time() / 1000.0 
                dt = current_time - last_time
                
                if dt <= 0: 
                    wait(1)
                    continue

                abs_position = (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2
                
                if abs_position >= max_degrees:
                    print("--> Reached", max_distance, "mm limit! Stopping.")
                    break 
                
                error = -hub.imu.heading()

                if (last_error > 0 and error <= 0) or (last_error < 0 and error >= 0):
                    if timer.time() > 100:
                        zero_crossings += 1
                        cross_times.append(current_time)
                        print("Wobble detected! Crossing", zero_crossings)

                correction = test_kp * error

                left_p = int(max(-100, min(100, speed - correction)))
                right_p = int(max(-100, min(100, speed + correction)))

                leftwheel.dc(left_p)
                rightwheel.dc(right_p)

                last_error = error
                last_time = current_time
                wait(10)

            leftwheel.brake()
            rightwheel.brake()

            if zero_crossings >= target_crossings:
                ku = test_kp
                
                periods = []
                for i in range(len(cross_times) - 2):
                    periods.append(cross_times[i+2] - cross_times[i])
                
                avg_tu_ms = sum(periods) / len(periods)
                tu = avg_tu_ms 
                
                print("Tested Kp:", round(test_kp, 2), "| Crossings:", zero_crossings)
                print("Average Tu calculated:", round(tu, 3), "sec")
                break 
                
            else:
                print("Kp:", round(test_kp, 2), "failed (Not enough wobbles).")
                test_kp += 0.1 

        target_dt = 0.01
        
        final_kp = 0.60 * ku
        final_ki = (1.20 * ku / tu) * target_dt
        final_kd = (0.075 * ku * tu)

        print("\n--- STRAIGHT PRECISION TUNE COMPLETE ---")
        print("Ultimate Gain (Ku):", round(ku, 2))
        print("Average Period (Tu):", round(tu, 3), "sec")
        print("--------------------------")
        print("Set self.straightKp =", round(final_kp, 3))
        print("Set self.straightKi =", round(final_ki, 6))
        print("Set self.straightKd =", round(final_kd, 3))

        return final_kp, final_ki, final_kd

# ---

def battery_percent():
        voltage = hub.battery.voltage()

        min_v = 6000
        max_v = 8300

        percent = (voltage - min_v) / (max_v - min_v) * 100
        percent = max(0, min(100, percent))

        print(percent)

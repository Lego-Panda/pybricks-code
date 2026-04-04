from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import *

hub = PrimeHub(Axis.Y, Axis.Z)

class RobotCalibration:
    def __init__(self,kp=0.0, ki=0.0, kd=0.0, turnKp=0.0, turnKi=0.0, turnKd=0.0, shellKp=0.0, shellKi=0.0, shellKd=0.0, shellTol=2, turnTol=2, turn_wait_time=0.0):
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

        self.errorSum = 0
        self.lastError = 0

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
    
    def auto_tune_straight_precision(self, max_distance_cm=60.0, speed=50):
        test_kp = 0.6
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

    def auto_tune_shell(self, target_degrees=90):
        test_kp = 20.0 
        dt = 0.02 
        target_crossings = 10 
        
        # SAFETY LIMIT
        safety_limit_degrees = target_degrees + 60 

        print("--- STARTING SHELL AUTO-TUNE ---")
        print("WARNING: Ensure the shell has physical room to swing past", target_degrees, "degrees!")
        print("\n1. Move the shell manually to the ZERO (starting) position.")
        print("2. Press ANY button on the hub to START the automated process.")
        
        while not hub.buttons.pressed():
            wait(10)
        while hub.buttons.pressed():
            wait(10)
        wait(500) 

        while True:
            print("\n--- Testing Kp:", round(test_kp, 2), "---")
            shell.reset_angle(0)
            timer = StopWatch()
            zero_crossings = 0
            
            last_error = target_degrees 
            cross_times = []

            while zero_crossings < target_crossings and timer.time() < 8000:
                
                current_angle = shell.angle() * SHELL_RATIO
                
                if current_angle > safety_limit_degrees or current_angle < -30:
                    print("--> SAFETY ABORT! Shell swung too far. Stopping to prevent damage.")
                    break 
                
                error = target_degrees - current_angle

                if (last_error > 0 and error <= 0) or (last_error < 0 and error >= 0):
                    if timer.time() > 50:
                        zero_crossings += 1
                        cross_times.append(timer.time())
                        print("Wobble detected! Crossing", zero_crossings, "out of", target_crossings)

                pidValue = test_kp * error

                power = int(max(-100, min(100, pidValue)))
                shell.dc(power)

                last_error = error
                wait(20)

            shell.brake()

            if zero_crossings >= target_crossings:
                ku = test_kp
                
                periods = []
                for i in range(len(cross_times) - 2):
                    periods.append(cross_times[i+2] - cross_times[i])
                
                avg_tu_ms = sum(periods) / len(periods)
                tu = avg_tu_ms / 1000.0
                
                print("Tested Kp:", round(test_kp, 2), "SUCCESS!")
                print("Average Tu calculated from", len(periods), "samples is:", round(tu, 3))
                break 
                
            else:
                print(" Kp:", round(test_kp, 2), "failed (Not enough wobbles).")
                print("Increasing Kp by 2.0. Next attempt in 3 seconds...")
                print("--> PLEASE MANUALLY RETURN SHELL TO 0 NOW <--")
                test_kp += 2.0 
                wait(3000) 

        final_kp = 0.60 * ku
        final_ki = (1.20 * ku / tu) * dt
        final_kd = (0.075 * ku * tu) / dt

        print("\n--- SHELL PRECISION TUNE COMPLETE ---")
        print("Ultimate Gain (Ku):", round(ku, 2))
        print("Average Period (Tu):", round(tu, 3), "sec")
        print("--------------------------")
        print("Set self.shellKp =", round(final_kp, 3))
        print("Set self.shellKi =", round(final_ki, 3))
        print("Set self.shellKd =", round(final_kd, 3))
        
        self.shellKp = final_kp
        self.shellKi = final_ki
        self.shellKd = final_kd
        
        return final_kp, final_ki, final_kd

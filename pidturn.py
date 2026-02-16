#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
left_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_18_1, False)
left_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor_group = MotorGroup(left_motor_a, left_motor_b)
right_motor_a = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
right_motor_b = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
right_motor_group = MotorGroup(right_motor_a, right_motor_b)
drivetrain = DriveTrain(left_motor_group, right_motor_group, 319.19, 295, 40, MM, 0.25)
inertial_15 = Inertial(Ports.PORT15)


# Library imports
from vex import *

# Begin project code
#Autonomous
def inertial_turn(target_heading):
    actual_heading = inertial_15.heading()
    error = target_heading - actual_heading
    Kp = 1 #Tune these values for precise turns and reduces oscillation, set it to these values at first and then tune it
    Ki = 0 #Tune these values for precise turns and reduces oscillation, set it to these values at first and then tune it
    Kd = 0 #Tune these values for precise turns and reduces oscillation, set it to these values at first and then tune it
    tolerance = 2
    integral = 0
    previous_error = error


    while abs(error) > tolerance:
        actual_heading = inertial_15.heading(DEGREES)

        error = target_heading - actual_heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        integral += error
        integral = max(min(integral, 100), -100)  

        derivative = error - previous_error
        motor_speed = Kp * error + Ki * integral + Kd * derivative
        motor_speed = motor_speed * 0.5
        right_motor_a.spin(FORWARD, motor_speed, PERCENT)
        right_motor_b.spin(FORWARD, motor_speed, PERCENT)
        left_motor_a.spin(REVERSE, motor_speed, PERCENT)
        left_motor_b.spin(REVERSE, motor_speed, PERCENT)

        # Display error on the Brain's screen
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Error: %.2f" % error)
        brain.screen.set_cursor(2, 1)
        brain.screen.print("Heading: %.2f" % actual_heading)
        brain.screen.set_cursor(3, 1)
        brain.screen.print("Motor Speed: %.2f" % motor_speed)
        wait(20,MSEC)
        previous_error = error

    right_motor_a.stop()
    right_motor_b.stop()
    left_motor_a.stop()
    left_motor_b.stop()

def inertial_calibration():
    brain.screen.print("Calibration Start")
    inertial_15.calibrate()
    wait(1,SECONDS)
    brain.screen.next_row()
    brain.screen.print("Calibration Complete")

inertial_calibration()
wait(1,SECONDS) #This must be here so the inertial fully calibrates
inertial_turn(90) #Set this value to the target heading

#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import TouchSensor
#import numpy as np
import math
from time import sleep
from color_tracking import Tracker
from server import Server
from uvs import UVS
from client import Client


# Lengths of the links
link1 = 16.8  # length of link 1 in cm
link2 = 10  # length of link 2 in cm

# Initialize motors and sensors
motor1 = LargeMotor(OUTPUT_C)
motor2 = LargeMotor(OUTPUT_D)
touch_sensor = TouchSensor()

#  Function to move robot arm given link angles
def move_to_angles(theta1, theta2):
    """
    Moves the robot arm to the specified joint angles.
    """
    motor1.on_for_degrees(SpeedPercent(5), theta1)
    sleep(2)
    motor2.on_for_degrees(SpeedPercent(5), theta2)


#  Analytic method to implement inverse kinematics
def inverse_kinematics_analytic(x, y):
    """
    Given an (x, y) position in cm, this function calculates the joint angles required
    to reach that position.
    """
    # Calculate the distance from the origin to the point
    r = math.sqrt(x**2 + y**2)
    
    # Check if the target is reachable, otherwise return current position
    if r > link1 + link2:
        print("Target is not reachable")
        return motor1.position, motor2.position

    # Calculate joint angles using inverse kinematics formulas
    cos_theta2 = (r**2 - link1**2 - link2**2) / (2 * link1 * link2)
    
    # Ensure cos_theta2 is within the valid range for acos
    cos_theta2 = max(-1, min(1, cos_theta2))
    
    # Two possible solutions for theta2 (elbow up and elbow down)
    theta2_up = math.acos(cos_theta2)
    theta2_down = -math.acos(cos_theta2)

    # Calculate theta1 for both solutions
    sin_theta1_up = link2*math.sin(theta2_up)/math.sqrt(x**2+y**2)
    sin_theta1_down = link2*math.sin(theta2_down)/math.sqrt(x**2+y**2)

    # Ensure sin_theta1_up and sin_theta1_down are within the valid range for asin
    sin_theta1_up = max(-1, min(1, sin_theta1_up))
    sin_theta1_down = max(-1, min(1, sin_theta1_down))

    theta1_up = math.asin(sin_theta1_up) + math.atan2(y, x)
    theta1_down = math.asin(sin_theta1_down) + math.atan2(y, x)

    # Convert joint angles from radians to degrees
    theta1_up = math.degrees(theta1_up)
    theta2_up = math.degrees(theta2_up)
    theta1_down = math.degrees(theta1_down)
    theta2_down = math.degrees(theta2_down)

    # Choose solution based on smallest total joint angle
    if abs(theta1_up) + abs(theta2_up) <= abs(theta1_down) + abs(theta2_down):
        return theta1_up, theta2_up
    else:
        return theta1_down, theta2_down


# Function to draw a straight line defined by two points
def draw_straight_line(point1, point2, num_steps):
    # Ensure num_steps is a positive integer
    if num_steps <= 0:
        print("Invalid number of steps.")
        return

    # Extract the coordinates of the two points
    x1, y1 = point1
    x2, y2 = point2

    # Calculate the step increments for x and y
    dx = (x2 - x1) / num_steps
    dy = (y2 - y1) / num_steps

    # Initialize the current position to the starting point
    current_x = 0
    current_y = 0

    initial_theta1 = 1
    initial_theta2 = 1

    # Move the robot arm through the intermediate points
    for step in range(num_steps + 1):
        new_x = current_x + dx
        new_y = current_y + dy
        # Move the robot arm to the current position (x, y)
        calculated_theta1, calculated_theta2 = inverse_kinematics_analytic(new_x, new_y)

        # Print or log the current position 
        print("Step", step, "Position:", current_x, current_y)

        change_theta_1 = calculated_theta1 - initial_theta1
        change_theta_2 = calculated_theta2 - initial_theta2

        move_to_angles(change_theta_1,change_theta_2)

        initial_theta1 = change_theta_1
        initial_theta2 = change_theta_2

def main():


    motor1.reset()
    motor2.reset()


    point1 = (15, -10)  # Starting point
    point2 = (15, 15) # Ending point
    num_steps = 10  # Number of steps to divide the line

    draw_straight_line(point1, point2, num_steps)



    # Initialize tracker, server, and UVS
    tracker = Tracker('g', 'r')
    server = Server('192.168.0.2', 9999)
    client = Client('192.168.0.2', 999)
    uvs = UVS(tracker, server, client)

    # Run UVS
    print("Running UVS")
    uvs.run()

if __name__ == "__main__":
    main()
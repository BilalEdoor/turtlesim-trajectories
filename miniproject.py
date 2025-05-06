#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

pose = Pose()

def pose_callback(data):
    global pose
    pose = data

def move(distance, speed, is_forward):
    vel_msg = Twist()
    vel_msg.linear.x = speed if is_forward else -speed
    vel_msg.angular.z = 0

    distance_moved = 0.0
    t0 = rospy.Time.now().to_sec()

    while distance_moved < distance:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        distance_moved = speed * (t1 - t0)
        rate.sleep()

    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)

def rotate(angle_deg, angular_speed_deg, clockwise):
    vel_msg = Twist()
    angular_speed = math.radians(angular_speed_deg)
    vel_msg.angular.z = -angular_speed if clockwise else angular_speed

    angle_rad = math.radians(angle_deg)
    angle_moved = 0.0
    t0 = rospy.Time.now().to_sec()

    while angle_moved < angle_rad:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        angle_moved = angular_speed * (t1 - t0)
        rate.sleep()

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def square(edge_length):
    for _ in range(4):
        move(edge_length, 1.0, True)
        rotate(90, 90, False)

def triangle(side_length):
    for _ in range(3):
        move(side_length, 1.0, True)
        rotate(120, 90, False)

def circle(radius):
    vel_msg = Twist()
    vel_msg.linear.x = 1.0
    vel_msg.angular.z = 1.0 / radius

    duration = 2 * math.pi * radius / vel_msg.linear.x
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < duration:
        velocity_publisher.publish(vel_msg)
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def spiral(change_radius=0.1, max_radius=10):
    vel_msg = Twist()
    r = 0.1
    while r < max_radius:
        vel_msg.linear.x = r
        vel_msg.angular.z = 1.0
        velocity_publisher.publish(vel_msg)
        r += change_radius
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def point_to_point(x_goal, y_goal):
    vel_msg = Twist()
    while True:
        k_linear = 1.5
        distance = math.sqrt((x_goal - pose.x)**2 + (y_goal - pose.y)**2)
        linear_speed = k_linear * distance

        k_angular = 4
        angle_to_goal = math.atan2(y_goal - pose.y, x_goal - pose.x)
        angular_speed = k_angular * (angle_to_goal - pose.theta)

        vel_msg.linear.x = linear_speed
        vel_msg.angular.z = angular_speed

        velocity_publisher.publish(vel_msg)

        if distance < 0.1:
            break
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def wave_motion(amplitude=1.0, wavelength=2.0, cycles=2):
    vel_msg = Twist()
    t = 0
    while t < 2 * math.pi * cycles:
        vel_msg.linear.x = 1.0
        vel_msg.angular.z = amplitude * math.cos(t / wavelength)
        velocity_publisher.publish(vel_msg)
        t += 0.1
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    rospy.init_node('turtle_trajectory_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)

    while True:
        print("\nSelect one of the following motion trajectories for turtle robot:")
        print("1. Square")
        print("2. Triangle")
        print("3. Circular")
        print("4. Spiral")
        print("5. Point to Point")
        print("6. Hexagon")
        print("7. Wave motion")
        print("8. Exit")

        try:
            choice = int(input("Enter your choice: "))
        except:
            print("Please enter a valid number.")
            continue

        if choice == 1:
            length = float(input("Enter edge length of square: "))
            square(length)
        elif choice == 2:
            length = float(input("Enter side length of triangle: "))
            triangle(length)
        elif choice == 3:
            radius = float(input("Enter radius of circle: "))
            circle(radius)
        elif choice == 4:
            step = float(input("Enter spiral step increment (e.g., 0.1): "))
            spiral(step)
        elif choice == 5:
            x = float(input("Enter goal x: "))
            y = float(input("Enter goal y: "))
            point_to_point(x, y)
        elif choice == 6:
            length = float(input("Enter edge length of hexagon: "))
            for _ in range(6):
                move(length, 1.0, True)
                rotate(60, 90, False)
        elif choice == 7:
            amplitude = float(input("Enter amplitude of wave: "))
            wavelength = float(input("Enter wavelength: "))
            cycles = int(input("Enter number of cycles: "))
            wave_motion(amplitude, wavelength, cycles)
        elif choice == 8:
            print("Exiting...")
            break
        else:
            print("Invalid choice. Try again.")

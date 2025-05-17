#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
import math
import sys

pose = Pose()

def update_pose(data):
    global pose
    pose = data

def is_within_bounds(x, y, margin=1.0):
    return margin <= x <= 11 - margin and margin <= y <= 11 - margin

def move_linear(velocity_publisher, speed, distance, is_forward):
    #Create a new Twist message: This is the message that contains the movement commands (linear and angular).
    vel_msg = Twist()
    #Linear velocity (on the x-axis), Absolute value of velocity (always positive), The velocity is positive (forward), The speed is negative (backward).
    vel_msg.linear.x = abs(speed) if is_forward else -abs(speed)
    #There is no rotation during movement, only straight movement.
    vel_msg.angular.z = 0
    #Returns the current time as the reference point for calculating elapsed time.
    t0 = rospy.Time.now().to_sec()
    #Initialize a variable to track the distance traveled so far.
    current_distance = 0
    #Store the turtle's starting position.
    start_x = pose.x
    start_y = pose.y
    
    rate = rospy.Rate(10)
    #The loop continues as long as we haven't covered the required distance yet.
    while current_distance < distance:
        future_x = pose.x + math.cos(pose.theta) * 0.1
        future_y = pose.y + math.sin(pose.theta) * 0.1
        #Check: Is the turtle about to go out the window (out of bounds)?
        if not is_within_bounds(future_x, future_y):
            print("Robot stopped: reached boundary of the turtlesim window.")
            break
        #Sends the current speed command to the turtle to move.
        velocity_publisher.publish(vel_msg)
        #Current time.
        t1 = rospy.Time.now().to_sec()
        #How many meters have you traveled? Using speed and time: Distance = Speed × Time.
        current_distance = speed * (t1 - t0)
        rate.sleep()
    #Stop the turtle from moving completely. Send a Twist message at 0 velocity to stop it.
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    
# velocity_publisher: The same publisher that sends movement commands.
# angular_speed_degree: The angular velocity, but in degrees.
# angle_degree: The angle the turtle must turn (in degrees).
# clockwise: Rotation type: If True → clockwise.
def rotate(velocity_publisher, angular_speed_degree, angle_degree, clockwise):
    vel_msg = Twist()

    #abs(angular_speed_degree): Takes the absolute value of the speed (make sure it's positive).
    #math.radians(): Converts the speed from degrees to radians, because ROS uses radians for rotation.
    #Example:
    #ex.  0° → 𝜋/2 ≈ 1.57 radians
    angular_speed = math.radians(abs(angular_speed_degree))

    # vel_msg.angular.z: The angular velocity about the z-axis (i.e., the turtle rotates).
    # If clockwise: Make it negative ➝ -angular_speed.
    # If counter-clockwise: Keep it positive.
    vel_msg.angular.z = -angular_speed if clockwise else angular_speed

    t0 = rospy.Time.now().to_sec()

    #How many angles has the turtle turned so far?
    current_angle = 0
    rate = rospy.Rate(10)

    # We convert the angle from degrees to radians because all angular operations in ROS are based on radians.
    # We continue repeating as long as the turtle has rotated the desired angle.
    while current_angle < math.radians(angle_degree):

        #We send the current rotation command to the turtle.
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        
        # We calculate the angle it has turned so far using:
        # Angle = Angular Velocity × Time
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()

    # top the rotation by setting the angular velocity to 0.
    # We send the message to make the turtle stop moving.
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def draw_square(velocity_publisher):
    length = float(input("Enter the edge length of the square (<= 4.0): "))
    if length > 4.0:
        print("Too large! Reducing to 5.0")
        length = 4.0
    # We repeat 4 times (because the square has 4 sides).
    for _ in range(4):

        # Calls the move_linear function.
        move_linear(velocity_publisher, 1.0, length, True) 

        #Call the rotate function to rotate the turtle 90 degrees
        rotate(velocity_publisher, 20, 90, False)

def draw_triangle(velocity_publisher):
    side = float(input("Enter triangle side length (<= 4.0): "))
    if side > 6.0:
        print("Too large! Reducing to 4.0")
        side = 4.0

    if side <= 0:
        print("Length must be positive! Setting to 1.0")
        side = 1.0


    #We repeat the two steps (move + rotate) 3 times because the triangle has 3 sides.
    for _ in range(3):
        #Straight line motion is required.
        move_linear(velocity_publisher, 1.0, side, True)
        rotate(velocity_publisher, 20, 120, False)

def draw_circle(velocity_publisher):
    radius = float(input("Enter circle radius (<= 4.0): "))
    if radius > 4.0:
        print("Too large! Reducing to 4.0")
        radius = 4.0
    if radius <= 0:
        print("Radius must be positive! Setting to 1.0")
        radius = 1.0

    # The user is asked to specify the direction
    # 1 = clockwise.
    # 2 = counterclockwise.
    # strip() removes extra spaces (if the user presses Space by mistake).
    direction = input("Choose direction: 1 for clockwise, 2 for counter-clockwise: ").strip()

    # If the user enters anything other than 1 or 2.
    # Prints an alert.
    # Resets the orientation to the default (counterclockwise).
    if direction not in ["1", "2"]:
        print("Invalid input! Defaulting to counter-clockwise.")
        direction = "2"

    vel_msg = Twist()

    #Specifies the forward linear velocity (1 meter per second).
    vel_msg.linear.x = 1.0

    # Angular velocity = v / r = 1 / radius (law of circular motion).
    vel_msg.angular.z = -1.0 / radius if direction == "1" else 1.0 / radius

    # We calculate the time for a complete revolution using the relationship:
    # Circumference = 2πr
    # Time = Distance / Linear Velocity
    # Here, linear velocity = 1 → time = 2πr / 1 = 2πr
    time = 2 * math.pi * radius / vel_msg.linear.x

    #Takes the present time as a starting point.
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)

    #The loop continues as long as the elapsed time is less than the time required to draw the circle.
    while rospy.Time.now().to_sec() - t0 < time:

        # Checks that the turtle doesn't go outside the window.
        # If it does go outside, it prints a message and stops.
        if not is_within_bounds(pose.x, pose.y):
            print("Robot stopped! reached boundary of the turtlesim window.")
            break
        velocity_publisher.publish(vel_msg)
        rate.sleep()
    velocity_publisher.publish(Twist())

 
def draw_spiral(velocity_publisher):

    # The user is asked to enter the starting radius of the spiral.
    # If it's too small (< 0.1), it's set to 0.1.
    # If it's too large (> 5.0), it's reduced to 5.0 to stay within the visible window.
    start_radius = float(input("Enter starting radius: "))
    if start_radius < 0.1:
        print("Too small! Using 0.1")
        start_radius = 0.1
    elif start_radius > 1.0:
        print("Too large! Reducing to 1.0")
        start_radius = 1.0

    # The variable r represents the initial radius of the helical motion.
    # It starts at the user-defined value, controlling how tight the spiral begins.
    vel_msg = Twist()
    r = start_radius

    # Set the loop repetition rate to 10 times per second (10Hz).
    # This regulates the refresh rate to be smooth and regular.
    rate = rospy.Rate(10)

    # The loop continues as long as:
    # - The turtle is inside the turtlesim window.
    # - The radius r does not exceed 5.5 (so it doesn't go outside the window).
    while is_within_bounds(pose.x, pose.y) and r < 5.5:

        # Set the linear velocity so that it equals the current radius r.
        # This makes the turtle move faster as the circle gets larger.
        vel_msg.linear.x = r

        # Set the angular velocity to 1.0 (constant).
        # This causes the turtle to rotate at the same rate as the curve, but expand over time.
        vel_msg.angular.z = 1.0
        velocity_publisher.publish(vel_msg)

        # A very small increase in radius.
        # This gradual increase creates a spiral shape.
        # If it increases too much, the shape becomes unsmooth.
        r += 0.01
        rate.sleep()

    # After exiting the loop, send a zero-velocity message to stop the turtle.
    velocity_publisher.publish(Twist())

    # If the turtle exited the window, show a message.
    if not is_within_bounds(pose.x, pose.y):
        print("Robot stopped: reached boundary of the turtlesim window")

def draw_point_to_point(velocity_publisher):
    x = float(input("Enter destination x (1-10): "))
    y = float(input("Enter destination y (1-10): "))
    if not is_within_bounds(x, y):
        print("Coordinates out of bounds. Aborting.")
        return
    if x <= 0 or y <= 0:
        print("Coordinates must be positive.")
        return

    # Calculates the angle between the turtle's current position (pose.x, pose.y) and the target point (x, y).
    # Use atan2 to obtain the correct angle in all directions (from -π to +π).
    angle_to_goal = math.atan2(y - pose.y, x - pose.x)

    # Call the rotate function to direct the turtle toward the target point.
    # math.degrees(...): Convert the difference between the two angles from radians to degrees.
    # angle_to_goal - pose.theta < 0: If the angle is negative, we need to rotate clockwise.
    rotate(velocity_publisher, 30, math.degrees(angle_to_goal - pose.theta), angle_to_goal - pose.theta < 0)

    # Calculate the distance between the current location of the point and the target location using the Pythagorean Theorem.
    distance = math.sqrt((x - pose.x)**2 + (y - pose.y)**2)

    #Call the move_linear function to move the turtle straight towards the point.
    move_linear(velocity_publisher, 1.0, distance, True)

def draw_hexagon(velocity_publisher):
    length = float(input("enter the edge length of hexagon (<=3:)"))
    if length > 3:
       print("Too large! reducing to 3")
       length = 3.0
    
    for _ in range(6): 
        # Move the turtle forward a distance equal to the length of the side
        move_linear(velocity_publisher, 1.0, length, True)
        #After each side, the turtle rotates 60 degrees.
        rotate(velocity_publisher, 20, 60, False)

def draw_sine_wave(velocity_publisher):
    amplitude = float(input("Enter amplitude (e.g., 2.0): "))
    frequency = float(input("Enter frequency (e.g., 1.0): "))
    speed = float(input("Enter forward speed (e.g., 1.0): "))
    if amplitude <= 0 or frequency <= 0 or speed <= 0:
        print("Invalid input values. All values must be positive.")
        return

    rate = rospy.Rate(30)
    twist = Twist()

    #The variable t represents time (it increases gradually with each iteration).
    # It is used in the  sine wave equation.
    t = 0.0
    while not rospy.is_shutdown():

        # The mathematical equation for the instantaneous deflection angle is:
        # Taken from the derivative of the function sin(fx) = f * cos(fx
        # We use atan to obtain the angle representing the slope of the curve at each instant.
        # This causes the turtle to follow the waveform.
        angle = math.atan(amplitude * frequency * math.cos(frequency * t))

        twist.linear.x = speed
        twist.angular.z = angle

        if not is_within_bounds(pose.x, pose.y):
            print("Robot stopped: reached boundary of the turtlesim window.")
            break

        velocity_publisher.publish(twist)
        # Gradually increase the time (approximately every 0.05 seconds).
        # Each new value of t represents a new point on the sine curve.
        t += 0.05
        rate.sleep()

    velocity_publisher.publish(Twist())

def reset_turtle(_unused=None):
    rospy.wait_for_service("/reset")
    try:
        reset = rospy.ServiceProxy('/reset', Empty)
        reset()
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def main():
    rospy.init_node('turtle_motion_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, update_pose)

    rospy.sleep(1) 

    while True:
        print("\nSelect one of the following motion trajectories for turtle robot:")
        print("0. Exit turtle\n1. Square\n2. Triangle\n3. Circular\n4. Spiral\n5. Point to Point\n6. hexagon\n7. Sine Wave\n8. Reset turtl")
        choice = input("Enter your choice (0-8): ")
        if choice == '0':
            print("Exiting the program. Goodbye!")
            sys.exit()
        elif choice == '1':
            draw_square(velocity_publisher)
        elif choice == '2':
            draw_triangle(velocity_publisher)
        elif choice == '3':
            draw_circle(velocity_publisher)
        elif choice == '4':
            draw_spiral(velocity_publisher)
        elif choice == '5':
            draw_point_to_point(velocity_publisher)
        elif choice == '6':
            draw_hexagon(velocity_publisher)
        elif choice == '7':
            draw_sine_wave(velocity_publisher)
        elif choice == '8':
            reset_turtle()
        else:
            print("Invalid choice!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
# AGV-using-ROS-and-Adruino
Creating an Automated Guided Vehicle (AGV) involves integrating Robot Operating System (ROS) with Arduino for the physical components and ROS communications to coordinate the different parts of the system (like sensors, motors, etc.). You can use ROS for high-level logic and communications, and Arduino for low-level control of hardware (e.g., motors, sensors).

I'll provide an outline and the key steps for developing an AGV with ROS and Arduino, as well as Python code snippets for interfacing between them.
Prerequisites

    ROS Installation: Install ROS on your development machine (you can use ROS Noetic or any other version suitable for your AGV).
    Arduino Setup: Have an Arduino board (e.g., Arduino Uno, Mega) connected and ready to interface with ROS.
    ROS-Serial: This allows communication between ROS and Arduino. You can use the rosserial package for this.

Steps to Set Up AGV with ROS and Arduino

    Arduino Hardware Setup
        Motors for movement (DC motors, stepper motors, etc.)
        Sensors (Ultrasonic, LIDAR, etc.) for navigation and obstacle detection
        Motor Driver (e.g., L298N for controlling the motors)
        Encoder for tracking wheel rotation
        ROS Serial Communication for sending data to/from ROS

    Arduino Code (to control motors, read sensors, and send data to ROS)

Here is an example Arduino sketch to control the motors and send sensor data to ROS via rosserial:

#include <ros.h>
#include <std_msgs/String.h>
#include <Arduino.h>

// Define pin numbers for motors (change according to your setup)
#define motorA1 3
#define motorA2 4
#define motorB1 5
#define motorB2 6

ros::NodeHandle nh;

// Create ROS Publisher for sensor data
std_msgs::String msg;
ros::Publisher chatter("sensor_data", &msg);

void setup() {
  // Initialize motors as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize serial communication for ROS
  nh.initNode();
  nh.advertise(chatter);

  // Initialize motor control (stop initially)
  stopMotors();
}

void loop() {
  // Move forward
  moveForward();
  delay(1000);  // Move forward for 1 second

  // Stop motors
  stopMotors();
  delay(1000);  // Stop for 1 second

  // Read sensors (example: ultrasonic distance sensor)
  int sensor_value = analogRead(A0);
  
  // Send sensor data to ROS
  msg.data = "Sensor Value: " + String(sensor_value);
  chatter.publish(&msg);

  // Handle ROS communication
  nh.spinOnce();
}

void moveForward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

Explanation:

    The Arduino code sets up two DC motors for movement (controlled by motor driver pins).
    The rosserial library is used to send data to a ROS topic called sensor_data, which could represent any sensor data such as distance or speed.
    The motors move forward for 1 second, then stop for 1 second.
    The sensor value is read and published as a message on the sensor_data topic.

3. Setting Up ROS to Communicate with Arduino

On the ROS side, we need a Python script that subscribes to the sensor data from the Arduino and publishes commands to control the robot’s movement (e.g., forward, turn, stop).

    Install ROS Serial: First, ensure you have the rosserial package installed in your ROS environment:

    sudo apt-get install ros-noetic-rosserial

    ROS Python Node (to subscribe to sensor data and publish motor control commands)

Here’s a basic Python ROS node that subscribes to sensor_data and publishes movement commands:

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

# Callback function to handle incoming sensor data from Arduino
def sensor_data_callback(msg):
    rospy.loginfo(f"Received sensor data: {msg.data}")
    # Implement logic based on sensor data (e.g., obstacle avoidance)
    # Example: Stop the robot if the sensor value exceeds a threshold
    if "Sensor Value" in msg.data:
        sensor_value = int(msg.data.split(":")[1].strip())
        if sensor_value < 100:  # Threshold for proximity (e.g., distance)
            move_forward()
        else:
            stop_robot()

# Function to move robot forward
def move_forward():
    move_cmd = Twist()
    move_cmd.linear.x = 0.2  # Move forward with speed 0.2 m/s
    pub.publish(move_cmd)

# Function to stop robot
def stop_robot():
    move_cmd = Twist()
    move_cmd.linear.x = 0.0  # Stop movement
    pub.publish(move_cmd)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('agv_control_node')

    # Publisher to control the robot's movement (geometry_msgs::Twist)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscriber to get sensor data from Arduino
    rospy.Subscriber('sensor_data', String, sensor_data_callback)

    # Spin ROS node
    rospy.spin()

Explanation:

    The Python ROS node subscribes to the sensor_data topic and processes the incoming data from the Arduino.
    It checks if the sensor value is below a certain threshold (e.g., an obstacle is near), and publishes a movement command (Twist message) to the /cmd_vel topic, which is a standard topic used for controlling robot motion in ROS.
    If an obstacle is detected (sensor value exceeds the threshold), the robot will stop by publishing a zero velocity command.

4. Running the System

    Launch ROS Serial: First, you'll need to establish a serial connection with the Arduino. Run this command in the terminal:

rosrun rosserial_python serial_node.py /dev/ttyACM0

(Replace /dev/ttyACM0 with the correct serial port for your Arduino.)

Run ROS Node: Start the Python node:

    rosrun agv_control agv_control_node.py

    Testing: If everything is set up correctly, the robot will move forward, stop, and react based on sensor data sent from Arduino.

5. AGV Simulation with ROS

For simulating your AGV in a virtual environment (before testing on real hardware), you can use Gazebo with ROS.

    Install Gazebo: sudo apt-get install ros-noetic-gazebo-ros-pkgs
    Create a robot model in URDF or SDF format, and launch the simulation environment.

You can write a ROS launch file to integrate Gazebo with your control node, where you simulate the robot's movement and sensor feedback.
Conclusion:

The code provided will help you get started with building an AGV using ROS and Arduino. You can extend this with more sensors (e.g., LIDAR, cameras) and more advanced control algorithms (e.g., PID controllers, path planning). The setup allows for real-time communication between ROS and Arduino, and can easily be tested on hardware or in a simulated environment using Gazebo.

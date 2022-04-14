#!/usr/bin/env python

import rospy
from std_msgs.msg import String, UInt32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf_conversions
from tf import transformations
import tf2_ros
import geometry_msgs.msg
import math

x_pressed=False


def pub_zero_cmd_vel():
    move = Twist()
    move.linear.x=0
    move.linear.y=0
    move.linear.z=0
    move.angular.x=0
    move.angular.y=0
    move.angular.z=0
    pub2 = rospy.Publisher('/mcu/command/manual_twist', Twist, queue_size=10)
    i=0
    while i<10     : 
        pub2.publish(move)
        i+=1
        rospy.sleep(0.1)


def joy_callback(data):
    global x_pressed
    pub_controller_data = rospy.Publisher(
        '/command/setAction', UInt32, queue_size=10)
    # Command vision60 to stand
    if data.axes[7] == 1:
        pub_controller_data.publish(1)
        print("Stand Ghost")
    # Command vision60 to sit
    if data.axes[7] == -1:
        pub_controller_data.publish(0)
        print("Sit Ghost")
    # Command vision60 to walk
    if data.buttons[0] == 1:
        pub_controller_data.publish(2)
        print("Walk Ghost")
   
    #if x button is released after pressed, pub vel 0 to cmd vel
    if data.buttons[4]==1:
        x_pressed=True
    elif data.buttons[4]==0 and x_pressed==True:
        x_pressed=False
        pub_zero_cmd_vel()

def main():
    global x_pressed
    rospy.init_node('velocity_converter_vision60_1', anonymous=True)
    # rospy.Subscriber("/cmd_vel", Twist, velocity_callback)
    rospy.Subscriber("/joy", Joy, joy_callback)
    x_pressed=False
    rate = rospy.Rate(10)  # 10hz
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

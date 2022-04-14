#!/usr/bin/env python
# license removed for brevity

from turtle import dot
import rospy
import time
import math
import numpy as np
from tf import transformations
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

x_pressed = False
base_vel = None

def pub_zero_cmd_vel():
    move = Twist()
    move.linear.x=0
    move.linear.y=0
    move.linear.z=0
    move.angular.x=0
    move.angular.y=0
    move.angular.z=0
    
    pub2 = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    i=0
    while i<10     : 
        pub2.publish(move)
        i+=1
        rospy.sleep(0.1)


def joyCallback(data):
    global x_pressed

    #if x button is released after pressed, pub vel 0 to cmd vel
    if data.buttons[4]==1:
        x_pressed=True
    elif data.buttons[4]==0 and x_pressed==True:
        x_pressed=False
        pub_zero_cmd_vel()


def baseVelCallback(data):
    global base_vel

    base_vel = Twist()
    base_vel = data


def wheelVel():
    global base_vel

    mat_A = np.array([[-1, 1, 0.382], 
                      [1, 1, -0.382], 
                      [-1, 1, -0.382], 
                      [1, 1, 0.382]])

    theta = 0
 
    mat_B = np.array([[math.cos(math.pi + theta), math.sin(math.pi + theta), 0], 
                      [-1*math.sin(math.pi + theta), math.cos(math.pi + theta), 0],
                      [0, 0, 1]])

    mat_C = np.array([[base_vel.linear.x], 
                      [base_vel.linear.y], 
                      [base_vel.angular.z]])

    wheel_vel = (1/0.0635)*(np.dot((np.dot(mat_A,mat_B)),mat_C))

    return wheel_vel

def main():  
    global x_pressed
    global base_odom

    rospy.init_node('velocity_converter_mobile_rack', anonymous=True)
    rospy.Subscriber("/joy", Joy, joyCallback)
    rospy.Subscriber("/cmd_vel", Twist, baseVelCallback)
    
    x_pressed = False

    while not rospy.is_shutdown():
        if  base_vel != None:
            [INSERT CODE TO APPLY WHEEL VELOCITIES TO ODRIVE MOTORS]

        rate = rospy.Rate(10)  # 10hz
        rate.sleep()
        # rospy.spin()
    

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
        pass
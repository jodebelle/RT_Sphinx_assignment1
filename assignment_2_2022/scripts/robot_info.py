#! /usr/bin/env python3
"""
.. module:: robot_info
	:platform: Unix
	:synopsis: Development of 3 ROS nodes to interact with a certain environment in which a robot moves in an arena.
.. moduleauthor:: Josephine de Bellefroid

Implementation of a node that prints the distance between the actual position of the robot and the desired position and the average speed of the robot.These parameters are taken from the /Posevelocity topic as a custom message. The frequency of printing is set as a parameter in the launch file.

Subscriber
/PoseVelocity

ROS parameter:
frequency

"""

import rospy
import time
import math

from assignment_2_2022.msg import PoseVelocity

freq = 1.0 #frequency

def sub_callback(msg):

    """
    sub_callback function is called each time a new message is received and calculates the distance and speed. If the specified time period has elapsed since the last print, the robot's information is printed.
    
    """
    global info_printed

    time_t = time.time()*1000 #time in milliseconds
    period=(1/freq)*1000 #period in milliseconds

    if time_t - info_printed > period:
        desired_x = float (rospy.get_param ("des_pos_x"))
        desired_y = float (rospy.get_param ("des_pos_y"))

        position_x = msg.position_x
        position_y = msg.position_y
        
        robot_av_speed_1 = (msg.vel_x * msg.vel_x) + (msg.vel_y * msg.vel_y)
        robot_av_speed = math.sqrt(robot_av_speed_1)
        robot_target_distance = math.dist([desired_x, desired_y], [position_x, position_y])

        print ("The distance between the robot's position and the target'position is: ", robot_target_distance)
        print (" The average speed of the robot is: ", robot_av_speed)

        time_t = time.time()*1000 #time in milliseconds
        info_printed = time_t #update info printed

def pos(msg):
    pos = msg.pose.pose.position
    pos_x = pos_x
    print("position x", pos_x)


def main():
    rospy.init_node('robot_info')
    global freq, info_printed
    info_printed = 0 #last info printed
    freq = rospy.get_param ("frequency")

    # Subscribe to the robot's position and velocity message
    sub_info = rospy.Subscriber("/PoseVelocity", PoseVelocity, sub_callback)
    #node = robot_info()
    #node.run()

    rospy.spin() #wait


if __name__ == '__main__':
    main()

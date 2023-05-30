#! /usr/bin/env python3
"""
.. module:: action_client
	:platform: Unix
	:synopsis: Development of 3 ROS nodes to interact with a certain environment in which a robot moves in an arena.
.. moduleauthor:: Josephine de Bellefroid

Implementation of an action client node that will ask the user to enter two coordinates (x,y). Then, it will publish the position and velocity of the robot as a custom message on the /Posevelocity topic, getting the values from the /odom topic.

Subscriber
    /odom

Publisher
    /PoseVelocity

Action Client:
    /reaching_goal

"""

import rospy
import sys
import select
import actionlib
import actionlib.msg
import assignment_2_2022.msg

from assignment_2_2022.srv import target_srv
from nav_msgs.msg import Odometry
from assignment_2_2022.msg import PoseVelocity
from geometry_msgs.msg import Point, Pose, Twist 

def call_srv():
    """
	This function calls a service node and returns the number of goals cancelled and the number of goals reached.
	
	Args: None.
	
	Returns: A target_srv service object which contains the number of goals cancelled and the number of goals reached.
	
	"""

    rospy.wait_for_service ('srv')
    srv = rospy.ServiceProxy ('srv', target_srv)
    resp = srv()
    print("Number of goals cancelled", resp.cancelled)
    print("Number of goals reached", resp.reached)
  

def pub_data (msg): #function to publish information on position

    """
	Callback function that publishes robot's velocity and current position to the "/PoseVelocity" topic.
	
	Args:
	msg(nav_msgs::Odometry): message published on the '/odom' topic describing robot current position and velocity.
	
	Returns:
	None
	
	"""
    global pub

    PoseVelocity_output = PoseVelocity() #custom msg
    position = msg.pose.pose.position #get position
    velocity = msg.twist.twist.linear #get twist

    #parameters
    PoseVelocity_output_px = position.x
    PoseVelocity_output_py = position.y
    PoseVelocity_output_vx = velocity.x
    PoseVelocity_output_vy = velocity.y

    pub.publish(PoseVelocity_output) #publish message


def action_client():
    """
	This function initializes the action client and asks the user to insert coordinates (x, y) to reach.
	
	Then, it will publish the position and velocity of the robot as a custom message on the /Posevelocity topic, 	 getting the values from the /odom topic.
		
	Once the goal is reached or cancelled, the instructions above are executed again.
		
	The coordinates reach are of type *geometry_msgs::Point* and only the value of x and y are set by the user.
	They are taken from input as two different float and the corresponding field of a *geometry_msgs::Point* variable are set.
	The coordinates to reach are then send to the *assignment_2_2022.msg::Planning* ActionServer as goal.
	
	Args:
	none
	
	Returns:
	none
	 	
	"""

    action_client = actionlib.SimpleActionClient("/reaching_goal", assignment_2_2022.msg.PlanningAction)

    action_client.wait_for_server()

    x=0
    y=0
        
    while not rospy.is_shutdown():
        print (" Enter the coordinates x and y")
        x=float(input("Enter a goal x: "))
        y=float(input("Enter a goal y: "))
        
        #target position
        goal = assignment_2_2022.msg.PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        target_x = x
        target_y = y

        action_client.send_goal(goal)

    
def cancel_target(self):

    """
    
    This function was made to allow the user to cancel a target by clicking on x.
    
    """

    print("press x to cancel the target")
    input=select.select ([sys.stdin],  [], [], 3) [0] #give user 3 seconds to cancel goal

    if input:
        i = sys.stdin.readline().rstrip()
        if(i=="x"):
            self.action_client.cancel_goal()
            print("The goal has been cancelled")
            self.target_x = None
            self.target_y = None

def main():

    """
    
    This function initializes the rospy node, defines a publisher, subscribes to the "/odom" topic to get the robot's current position and velocity, and calls the action_client() function to send a new goal to the robot.
    
    """
    
    rospy.init_node("action_client")
    global pub

    #publisher for custom message
    pub = rospy.Publisher("/PoseVelocity", PoseVelocity, queue_size=10)


    #subscriber for /odom
    odom_sub = rospy.Subscriber("/odom", Odometry, pub_data)
    
    action_client()
    
if __name__ == '__main__':
        main()


    

#! /usr/bin/env python3
"""
.. module:: goal_count
	:platform: Unix
	:synopsis: Development of 3 ROS nodes to interaact with a certain environment in which a robot moves in an arena.
.. moduleauthor:: Josephine de Bellefroid

This python code implements a service node. When the node is called,it prints how many times a desired position was reached or cancelled.


Subscriber
/robot_target/result

Server:
goal_count

"""

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg

from assignment_2_2022.srv import target_srv, target_srvResponse

goal_count = {'reached': 0, 'cancelled': 0}

def result(msg):

    """
	 
    Callback function that updates the goal count when a PlanningActionResult message is received.

    :param msg: a PlanningActionResult message.
    
    """
    
    global goal_reached_callback, goal_cancelled_callback
    
    status=msg.status.status

    if status == 2: #goal preempted
        goal_count['cancelled'] += 1
    elif status == 3: #goal reached
        goal_count['reached'] += 1

def goal_callback(req):
            
    """
    Service callback function that returns the number of goals that have been reached and cancelled.

    :param req: a target_srvRequest message.
    :return: a target_srvResponse message indicating that the goal count has been printed.
     """
            
    print ("Number of goals reached:", goal_count['reached'])
    print ("Number of goals cancelled:", goal_count['cancelled'])
    return target_srvResponse (success = True, message = "Goal count printed")

def goal_reached_callback():

    """
    Callback function that increments the goal reached count when a goal is reached.
    """

    goal_count['reached'] += 1

def goal_cancelled_callback():

    """
    Callback function that increments the goal cancelled count when a goal is cancelled.
    """

    goal_count['cancelled'] += 1

def data(req):

    """
    Service callback function that returns the goal_reached_callback and goal_cancelled_callback functions.

    :param req: a target_srvRequest message.
    :return: a target_srvResponse message containing the goal_reached_callback and goal_cancelled_callback functions.
    """

    global goal_reached_callback, goal_cancelled_callback
    return target_srvResponse(goal_reached_callback, goal_cancelled_callback)

def main():

    """
    The main function that initializes the node, sets up the service and subscribers, and starts the spin loop.
    """
     
    rospy.init_node('goal_count')
    srv= rospy.Service('goal_count', target_srv, goal_callback)
    subscriber_result = rospy.Subscriber('/robot_target/result', assignment_2_2022.msg.PlanningActionResult, result)
    rospy.spin()

if __name__ == "__main__":
    main()


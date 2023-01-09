#! /usr/bin/env python

import rospy
from assignment_2_2022.srv import goal, goalResponse
import actionlib
import actionlib.msg
import assignment_2_2022.msg

# variables to store how many times goals were cancelled or reached
cancelled = 0;
reached = 0;

# callback for result subscriber
def result(msg):
	
    # initialize cancelled and reached as global variables 
    global cancelled, reached
	
    # get the status of the result from the message
    status = msg.status.status
	
    # if status is equal 2, the goal was preempted (cancelled)
    if status == 2:
        cancelled += 1
    # if status is equal 3, the goal was reached
    elif status == 3:
        reached += 1
		
# the service function, use for implement the service 
def get_data(req):
    
    # initialize cancelled and reached as global variables 
    global cancelled, reached
	
    # return the response of the service 
    return goalResponse(reached, cancelled)

def main():
    # initialize the node
    rospy.init_node('service')
	
    # create the service
    srv = rospy.Service('service', goal, get_data)
	
    # initialize the subscriber for the result of the goal
    sub_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, result)
	
    # wait
    rospy.spin()
	
if __name__ == "__main__":
    main()

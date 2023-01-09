#!/usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import assignment_2_2022.msg
from std_srvs.srv import *
import sys
import select
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
from assignment_2_2022.msg import Pos_vel

def publish_values(msg):
	
    # recall the global publisher as pub
    global pub
    
    # get the postion from the message
    pos = msg.pose.pose.position
	
    # get the twist from the message
    velocity = msg.twist.twist.linear
	
    # create custom message (of the type corresponding to the file in the msg folder)
    position_and_velocity = Pos_vel()
	
    # assign the parameters of the custom message
    position_and_velocity.x = pos.x
    position_and_velocity.y = pos.y
    position_and_velocity.velX = velocity.x
    position_and_velocity.velY = velocity.y
	
    # Publish the custom message
    pub.publish(position_and_velocity)

def client():
    # create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2022.msg.PlanningAction)
    # wait for the server to be started
    client.wait_for_server()
    
    # print a message for the user 
    print("\nThis is a program that moves a robot (a toy car) inside a virtual arena." 
            + "\nThe user is asked to enter a point to reach via coordinates and subsequently," 
            + "\nif desired, the point where the robot is to be sent can be changed or the robot" 
            + "\ncan be stopped.")
    
    while not rospy.is_shutdown():
        
        # give input from the utent 
        print("\nENTER THE COORDINATES (x,y) BELOW THAT THE ROBOT MUST REACH")
        positionX = input("Set a target x: ")
        positionY = input("Set a target y: ")
        
        # check if the input is correct 
        if positionX.lstrip('-').replace('.', '', 1).isdigit() and positionY.lstrip('-').replace('.', '', 1).isdigit():
            positionX = float(positionX)
            positionY = float(positionY)
    	
            # create the goal to send to the server
            goal = assignment_2_2022.msg.PlanningGoal()
            goal.target_pose.pose.position.x = positionX
            goal.target_pose.pose.position.y = positionY
    
            # send the goal to the action server
            client.send_goal(goal)
        
            # code for stop the robot if the user want 
            stop_btn = input("If you want to stop the robot press 'S' and ENTER else press only ENTER: ")
           
	    # check if the input is correct 
            if (stop_btn == "S"):
	        # cancel the goal and stop the robot 
                print("The robot has been stopped!")
                client.cancel_goal()
            elif (stop_btn != "S" and stop_btn != ""):
                print("ERROR: input for stop the robot not correct!")
        else:
            print("ERROR: input not valid!")

def main():
    # initialize the node 
    rospy.init_node('input_from_user')
    
    # create a global publisher, call as pub
    global pub
    
    # initialize the publisher to send a msg (using the velocity and the position as parameters)
    pub = rospy.Publisher("/Position_velocity", Pos_vel, queue_size = 1)
    
    # initialize the subscriber to get from Odom the velocity and the position parameters
    sub_from_Odom=rospy.Subscriber("/odom", Odometry, publish_values)
    
    # call the client function
    client()
      
if __name__ == '__main__':
    main()

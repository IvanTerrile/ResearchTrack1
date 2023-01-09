#! /usr/bin/env python

import rospy
import math
import time
from assignment_2_2022.msg import Pos_vel

# initialize the frequancy and the last printed 
freq = 1.0
last = 0

# callback function for the info subscriber
def pos_vel(msg):
	
    global freq, last
	
    # compute the period in ms in milliseconds
    period = (1.0/freq) * 1000
	
    # get the current time in milliseconds 
    current_time = time.time() * 1000
	
	
    if current_time - last > period:
        	
        # get the desired position give by the user
        desidered_x = rospy.get_param("des_pos_x")
        desidered_y = rospy.get_param("des_pos_y")
		
        # get the actual position of the robot 
        actual_x = msg.x
        actual_y = msg.y
		
        # compute the distance between the desidered position and the actual position of the robot 
        dist = math.dist([desidered_x, desidered_y], [actual_x, actual_y])
		
        # compute the average speed of the robot 
        average_speed = math.sqrt(msg.velX**2 + msg.velY**2)
		
        # print information (distance and average speed)
        print( "The distance between the desired position and the actual position is: ", float(round(dist, 5)))
        print( "The average speed is: ", float(round(average_speed, 5)))
        print()
		
        # update last_printed
        last = current_time
	

def main():
	
    # set the frequency as a global variable 
    global frequency
	
    # initialize the node
    rospy.init_node('information')
	
    # get the publishing frequency 
    freq = rospy.get_param("frequency")

    # initialize the subscriber to give the position_velocity as message
    sub_pos = rospy.Subscriber("/Position_velocity", Pos_vel, pos_vel)
	
    # wait
    rospy.spin()
	
if __name__ == "__main__":
    main()	

from __future__ import print_function

import time
from sr.robot import *

a_th = 2.0	# float: Threshold for the control of the linear distance

d_th = 0.4	# float: Threshold for the control of the orientation

R = Robot()	# instance of the class Robot

def drive(speed, seconds):
    """
    Function for setting a linear velocity
    
    Args: 	speed (int): the speed of the wheels
			seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity
    
    Args: 	speed (int): the speed of the wheels
			seconds (int): the time interval
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def find_silver_token():
    """
    Function to find the closest silver token

    Returns:	dist (float): distance of the closest silver token (-1 if no silver token is detected)
				rot_y (float): angle between the robot and the silver token (-1 if no silver token is detected)
				t (int): the code of the token(-1 if no silver token is detected)
    """
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist = token.dist
            rot_y = token.rot_y
            t = token.info.code
    if dist==100:
        return -1, -1, -1
    else:
	    return dist, rot_y, t

def find_golden_token():
    """
    Function to find the closest golden token

    Returns:	dist (float): distance of the closest golden token (-1 if no golden token is detected)
				rot_y (float): angle between the robot and the golden token (-1 if no golden token is detected)
				t (int): the code of the btoken (-1 if no silver token is detected)
    """
    dist=100
    for token in R.see():
        if token.dist < dist and token.info.marker_type is MARKER_TOKEN_GOLD:
            dist=token.dist
            rot_y=token.rot_y
            t = token.info.code
    if dist==100:
	    return -1, -1, -1
    else:
   	    return dist, rot_y, t
   	
def put_near_golden_token(number_of_tokens):
    """
    Function to:
                    1) call the find_golden_token function and find a golden token
                    2) go and release the silver token, already taken, next to the found golden token, if the code of the golden token
                       is not yet in the golden_black_list else turn a bit and find another token starting over from the point above
                    3) decrease the number_of_token of 1 to mark that a pair of golden and silver tokens has been created 
                    4) mark the code of the golden token in the corresponding list (golden_black_list)
                    5) check if the number_of_tokens is equal to 0 and in this case the program is over, otherwise call again the
                       take_silver_token function to find the next silver token to pair with a golden token and pass it the current number_of_tokens
    """
    while 1:
        dist, rot_y, code = find_golden_token()
        if code not in golden_black_list:
            if dist == -1: # if no token is detected, we make the robot turn 
	            print("I don't see any token!!")
	            turn(+10, 1)
            elif dist < d_th + 0.2: # if we are close to the token, we try grab it.
                print("Found it!")
                R.release()
                print("Release")
                drive(-10, 1)
                number_of_tokens -= 1
                golden_black_list.append(code)
                if number_of_tokens == 0:
                    print("You have finish")
                    exit()
                else:
                    take_silver_token(number_of_tokens)
                    break
            elif -a_th <= rot_y <= a_th: # if the robot is well aligned with the token, we go forward
	        print("Ah, that'll do.")
                drive(25, 0.5)
            elif rot_y < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
                print("Left a bit...")
                turn(-2, 0.5)
            elif rot_y > a_th:
                print("Right a bit...")
                turn(+2, 0.5)
        else:
            turn(+10, 1)
                
def take_silver_token(number_of_tokens):
    """
    Function to: 
                    1) call the find_silver_token function and find a silver token 
                    2) go get the silver token, if the code of the silver token is not yet in the silver_black_list else turn a bit and find another token starting over from the point above
                    3) mark the code of the silver token in the corresponding list (silver_vlack_list)
                    4) call the put_near_golden_token function to put the silver token near a golden token and pass it the number_of_tokens
    """
    while 1:
        dist, rot_y, code = find_silver_token()
        if code not in silver_black_list:
            if dist == -1: # if no token is detected, we make the robot turn 
	            print("I don't see any token!!")
	            turn(+10, 1)
            elif dist < d_th: # if we are close to the token, we try grab it.
                print("Found it!")
                R.grab() # if we grab the token, we move the robot forward and on the right, we release the token, and we go back to the initial position
                print("Gotcha!")
                silver_black_list.append(code)
                put_near_golden_token(number_of_tokens)
            elif -a_th <= rot_y <= a_th: # if the robot is well aligned with the token, we go forward
                print("Ah, that'll do.")
                drive(25, 0.5)
            elif rot_y < -a_th: # if the robot is not well aligned with the token, we move it on the left or on the right
                print("Left a bit...")
                turn(-2, 0.5)
            elif rot_y > a_th:
                print("Right a bit...")
                turn(+2, 0.5)
        else:
            turn(+10, 1)

############## MAIN OF THE PROGRAM ##############
"""
Set the number of tokens (half of the total tokens) and call the take_silver_token function
The program works by recursively calling take_silver_token and put_near_golden_token.
The exit and termination condition occurs when number_of_tokens is equal to 0.
"""
silver_black_list = list()	# a list to mark the silver token already taken 
golden_black_list = list()	# a list to mark the golden token already taken 
number_of_tokens = 6		# in this particular execution of the program the number of tokens is 6

take_silver_token(number_of_tokens)

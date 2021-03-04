#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

pi=math.pi
x_dist = 0
y_dist= 0
lis = []
front=1
left=1
right=1
front_nav=1
left_nav=1
right_nav=1
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
orientation=0
x_f=0
y_f=0
x_t=0
y_t=0


def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw_
    global lis
    
    

    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    x_dist = position_.x
    y_dist = position_.y




    
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    #print(x_dist, y_dist)
    lis=[x_dist,y_dist]

def dist(a,b):
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)



def clbk_laser(msg):
    global front
    global left
    global right

    global front_nav
    global left_nav
    global right_nav


    right = min(min(msg.ranges[1:90]), 10)
    #'fright': min(min(msg.ranges[72:143]), 10),
    front = min(min(msg.ranges[177:183]), 10)
        #'fleft':  min(min(msg.ranges[216:287]), 10),
    left = min(min(msg.ranges[270:359]), 10)
    #print (front,left,right)

    right_nav = min(min(msg.ranges[1:10]), 10)
    #'fright': min(min(msg.ranges[72:143]), 10),
    front_nav = min(min(msg.ranges[177:183]), 10)
        #'fleft':  min(min(msg.ranges[216:287]), 10),


 
    left_nav = min(min(msg.ranges[349:359]), 10)
    #print (front,left,right)
   

 
def GoToNextCell(init) :

    global pub
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()
    initial = init

    while not rospy.is_shutdown():
	
        current= lis
        
        
        # if front < 0.3 :
        #     msg1.linear.x = 0
        #     msg1.angular.z = 0
        #     pub.publish(msg1)
        #     print('case1')           
        #     rate.sleep()
        if (dist(initial,current)<0.18):
            if front < 0.075 :
                msg1.linear.x = 0
                msg1.angular.z = 0
                pub.publish(msg1)
                print('wall interrupt')
                break
            
            if right < 0.065 :
                msg1.linear.x = 0.1
                msg1.angular.z = 0.1
                #print('case1')
            elif left < 0.065 :
                msg1.linear.x = 0.1
                msg1.angular.z = -0.1
                #print('case2')
            else:
            
                msg1.linear.x = 0.2
                msg1.angular.z = 0
                # print('case3')
            
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.linear.x =0
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            break

def GoToPreviousCell(init) :

    global pub
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()
    initial = init

    while not rospy.is_shutdown():
	
        current= lis
        
        
        # if front < 0.3 :
        #     msg1.linear.x = 0
        #     msg1.angular.z = 0
        #     pub.publish(msg1)
        #     print('case1')           
        #     rate.sleep()
        if (dist(initial,current)<0.18):
            
            if right < 0.065 :
                msg1.linear.x = -0.05
                msg1.angular.z = -0.05
                print('case1')
            elif left < 0.065 :
                msg1.linear.x = -0.05
                msg1.angular.z = 0.05
                print('case2')
            else:
            
                msg1.linear.x = -0.1
                msg1.angular.z = 0
                print('case3')
            
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.linear.x =0
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            break

def angle_bw(curr,init):
    if (curr<0 and init<0) or (curr>0 and init>0):
        return abs(curr-init)
    elif curr<0 and abs(curr)>pi/2:
        curr=curr+2*pi
        return abs(curr-init)
    elif init<0 and abs(init)>pi/2:
        init=init+2*pi
        return abs(curr-init)
    else:
        return abs(curr-init)      

        

    
def TurnLeft(inp) :
    print('turning left')
    global pub
    initial=inp
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw_      
        
        
        if (angle_bw(current,initial)<pi/2):
            msg1.linear.x = 0
    
            msg1.angular.z = 0.3
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            break

        

def TurnRight(inp) :
    print('turning right')
    print('iniial yaw=',inp)
    global pub
    initial=inp
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw_      
        
        
        if ( angle_bw(current,initial) < pi/2 ):
            msg1.linear.x = 0
    
            msg1.angular.z = -0.3
            pub.publish(msg1)
            rate.sleep()
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            rate.sleep()
            print('final yaw=',yaw_)
            break

        
def is_front_clear() :
    global front_nav
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    # print (front_nav)
    if (front_nav>0.14) :
        print('front clear')
        return True
    else :
        print('front not clear')
        return False    

def is_left_clear() :
    global left_nav
    # print (left_nav)
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if (left_nav>0.14) :
        print('left clear')
        return True
    else :
        print('left not clear')
        return False    
        

def is_right_clear() :
    global right_nav
    # print (right_nav)
    #sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    if (right_nav>0.18) :
        print('right clear')
        return True
    else :
        print('right not clear')
        return False    


# Maze size 
N = 16
  
# A utility function to print solution matrix sol 
def printSolution( sol ): 
      
    for i in sol: 
        for j in i: 
            print(str(j) + " ", "") 
        print("") 
  
# A utility function to check if x, y is valid 
# index for N * N Maze 
def isSafe( maze, x, y ): 
      
    if x >= 0 and x < N and y >= 0 and y < N : #and maze[x][y] == 1: 
        return True
      
    return False
  
""" This function solves the Maze problem using Backtracking.  
    It mainly uses solveMazeUtil() to solve the problem. It  
    returns false if no path is possible, otherwise return  
    true and prints the path in the form of 1s. Please note 
    that there may be more than one solutions, this function 
    prints one of the feasable solutions. """
def solveMaze( maze ): 
    global orientation
    # Creating a 4 * 4 2-D list 
    sol = [ [ 0 for j in range(16) ] for i in range(16) ] 
      
    if solveMazeUtil(maze, 0, 0, sol,orientation) == False: 
        print("Solution doesn't exist"); 
        return False
      
    printSolution(sol) 
    return True
      
# A recursive utility function to solve Maze problem 
def solveMazeUtil(maze, x, y, sol, orientation): 
    print('next loop')

    r=rospy.Rate(1)
    global x_f
    global y_f
    global x_t
    global y_t

    if orientation==0 :
        x_f=1
        y_f=0
        x_t=0
        y_t=1

    if orientation==1 :
        x_f=0
        y_f=1
        x_t=-1
        y_t=0

    if orientation==2 :
        x_f=-1
        y_f=0
        x_t=0
        y_t=-1

    if orientation==3 :
        x_f=0
        y_f=-1
        x_t=1
        y_t=0


    # if (x, y is goal) return True 
    if x == N - 1 and y == N - 1 and maze[x][y]== 1: 
        sol[x][y] = 1
        return True
          
    # Check if maze[x][y] is valid 
    # if isSafe(maze, x, y) == True: 
    #     # mark x, y as part of solution path 
    #     sol[x][y] = 1
          
        # Move forward in x direction 
    if is_left_clear():
        TurnLeft(yaw_)
        GoToNextCell(lis)
        if orientation==3 :
            orientation=0
        else :
            orientation=orientation+1
        truth=solveMazeUtil(maze, x + x_t, y+y_t, sol,orientation) 
        if truth==True:
            return True
        else:
            print('moving right back')
            TurnLeft(yaw_)
            TurnLeft(yaw_)
            GoToNextCell(lis)
            TurnLeft(yaw_)
            if orientation==0 :
                orientation=3
            else :
                orientation=orientation-1
            print('came right back')
        r.sleep()

                
              
        # If moving in x direction doesn't give solution  
        # then Move down in y direction 
    if is_front_clear():
        GoToNextCell(lis)
        truth=solveMazeUtil(maze, x + x_f, y+y_f, sol,orientation) 
        if truth==True:
            return True
        else :
            print('moving back')
            TurnRight(yaw_)
            TurnRight(yaw_)
            GoToNextCell(lis)
            TurnRight(yaw_)
            TurnRight(yaw_)
            print('moved back')
        r.sleep()

    if is_right_clear():
        TurnRight(yaw_)
        GoToNextCell(lis)
        if orientation==0 :
            orientation=3
        else :
            orientation=orientation-1

        truth=solveMazeUtil(maze, x -x_t, y-y_t, sol,orientation) 
        if truth==True:
            return True
        else :
            print('moving left back')
            TurnRight(yaw_)
            TurnRight(yaw_)
            GoToNextCell(lis)
            TurnRight(yaw_)
            if orientation==3 :
                orientation=0
            else :
                orientation=orientation+1
            print('came left back')
        r.sleep()
        # If none of the above movements work then  
        # BACKTRACK: unmark x, y as part of solution path 
    # TurnLeft(yaw_)
    # TurnLeft(yaw_)
	
	#if orientation>=2:
	    #orientation=orientation-2
	#else :
	    #orientation=orientation+2
        #sol[x][y] = 0
    print('previous loop')
    return False


if __name__ == '__main__':
   
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)  
    rospy.init_node('cmd_robot', anonymous=True)

    # Initialising the maze 
    maze = [ [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ] 
               
    solveMaze(maze ) 
  
    
          
    
    
    

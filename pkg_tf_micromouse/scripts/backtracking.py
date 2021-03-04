#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

pose=[]
pi=math.pi
yaw=0
front=1
left=1
right=1
front_nav=1
left_nav=1
right_nav=1
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def clbk_odom(msg):
    global pose
    global yaw

    position_ = msg.pose.pose.position
    x_dist = position_.x
    y_dist = position_.y
    
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    # Saves the odometry data in global variables to be used for further purposes
    yaw= euler[2]
    pose=[x_dist,y_dist]

def clbk_laser(msg):
    global front
    global left
    global right

    global front_nav
    global left_nav
    global right_nav
    

    #Saves the current laser data in global variables
    right = min(msg.ranges[1:90])
    front = min(msg.ranges[176:183])
    left = min(msg.ranges[270:359])

    right_nav = min(msg.ranges[1:10])
    front_nav = min(msg.ranges[176:183])
    left_nav = min(msg.ranges[349:359])

def dist(a,b):
    #using odom
    return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    #using laser
    #return abs(b-a)


def GoToNextCell(initial):
    global pub
    msg1 = Twist()
    while True:

        #Using Odometry Data
        current=pose
        
        #using laser data
        #current=front_nav
        

        while(dist(initial,current)<0.18):
            if front < 0.08 :
                msg1.linear.x = 0
                msg1.angular.z = 0
                pub.publish(msg1)
                print('wall_in_front')
                break
            if right < 0.06 :
                msg1.linear.x = 0.1
                msg1.angular.z = 0.1
                print('wall_in_right')
            elif left < 0.06 :
                msg1.linear.x = 0.1
                msg1.angular.z = -0.1
                print('wall_in_left')
            else:
            
                msg1.linear.x = 0.4
                msg1.angular.z = 0
                print('moving_straight')
            
            pub.publish(msg1)
            #rate.sleep()
        msg1.linear.x =0
        msg1.angular.z =0
        pub.publish(msg1)
        break
    

def GoToPreviousCell(initial):
    global pub
    msg1 = Twist()

    while True:
        #Using Odometry Data
        current=pose
        
        #using laser data
        #current=front_nav
    
        while(dist(initial,current)<0.18):
            if right < 0.06 :
                msg1.linear.x = -0.1
                msg1.angular.z = -0.1
                print('case1')
            elif left < 0.06 :
                msg1.linear.x = -0.1
                msg1.angular.z = 0.1
                print('case2')
            else:
                msg1.linear.x = -0.4
                msg1.angular.z = 0
                print('case3')
            pub.publish(msg1)
        msg1.linear.x =0
        msg1.angular.z =0
        pub.publish(msg1)
        break
   
def TurnLeft(inp) : 
    global pub
    initial=inp
    rate = rospy.Rate(20) # 40hz
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw     
        if (abs(current-initial)<pi/2):
            msg1.linear.x = 0
            msg1.angular.z = 0.5
            pub.publish(msg1)
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            break

def TurnRight(inp) : 
    global pub
    initial=inp
    msg1 = Twist()

    while not rospy.is_shutdown():
        current= yaw     
        if (abs(current-initial)<pi/2):
            msg1.linear.x = 0
            msg1.angular.z = -0.5
            pub.publish(msg1)
        else :
            msg1.angular.z =0
            pub.publish(msg1)
            break

def FrontClear():
    global front_nav
    return front_nav>0.12
def LeftClear():
    global left_nav
    return left_nav>0.12
def RightClear():
    global right_nav
    return right_nav>0.12

def ExploreMaze():
    orientation=0
    sol = [ [ 0 for j in range(16) ] for i in range(16) ]
    SolveMaze(sol,0,0,orientation)

def SolveMaze(sol,x,y,orientation):
    #marking 1 in the solution matrix.
    sol[x][y] = 1
    # here xf and yf represent cahnge in x y coordinates of maze durding forward motion and different orientation.
    #orientation 1 is intitial orientation, next orientations are considered during turn.
    
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
    

    #Check left if it is clear explose left
    if LeftClear():
        print("Moving Left Of ",x,y)
        TurnLeft(yaw)
        GoToNextCell(pose)
        if orientation==3 :
            orientation=0
        else :
            orientation=orientation+1
        truth=solveMazeUtil(sol, x + x_t, y+y_t,orientation)
        if truth==True:
            return True
        else:
            print('returning from left',x,y)
            GoToPreviousCell(pose)
            TurnRight(yaw)
            if orientation==0 :
                orientation=3
            else :
                orientation=orientation-1

    # check right if it is clear explore right
    if RightClear():
        print("Moving Right Of ",x,y)
        TurnRight(yaw)
        GoToNextCell(pose)
        if orientation==0 :
            orientation=3
        else :
            orientation=orientation-1
        truth=solveMazeUtil(sol, x + x_t, y+y_t,orientation)
        if truth==True:
            return True
        else:
            print('returning from Right',x,y)
            GoToPreviousCell(pose)
            TurnLeft(yaw)
            if orientation==3 :
                orientation=0
            else :
                orientation=orientation+1

    # check front if it is clear move straight
    if Frontclear():
        print("Moving Front Of ",x,y)
        GoToNextCell(pose)
        truth=solveMazeUtil(sol, x + x_t, y+y_t, orientation)
        if truth==True:
            return True
        else:
            print('Moving back',x,y)
            GoToPreviousCell(pose)
    
    #if all the three direction are not clear return to initial cell
    sol[x][y] = 0
    return False



if __name__ == "__main__":
    rospy.init_node('cmd_robot', anonymous=True)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)  
    ExploreMaze()

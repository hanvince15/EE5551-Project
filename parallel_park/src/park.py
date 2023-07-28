#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

AtWall = False
ranges = [1000] * 360
yaw = 0
target_yaw = 0

#   Laser callback function:
#   Assigns ranges to the global variable ranges
def LaserCall(msg):
    global ranges
    ranges = msg.ranges
    # print(ranges)

def GetRotation (msg):
    global yaw
    orientation_q = msg.pose.pose.orientation
    position_q = msg.pose.pose.position
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion (orientation_list)
    # print(yaw)


#   Finds the nearest wall to the Burger
def FindWall(scandata):
    min = 100000000
    loc = -1

    for location, distance in enumerate(scandata):
        if distance < min and distance != 0:
            min = distance
            loc = location

    print("Nearest wall is at: " + str(loc))

    #   Returns index of nearest detected object
    return loc

FacedWall = False
#   Rotates until front of the Burger is facing the nearest wall
def FaceWall(scandata):
    global FacedWall, pub
    target = FindWall(scandata)
    # print(scandata)

    print("Direction to the nearest location wall location: " + str(target))
    print("The distance to the wall in that direction: " + str(scandata[target]))
    print("This is the distance that the front sees: " + str(scandata[0]))
    # print("Ranges: " + str(scandata))
    
    if (0 != target):
        if 359 >= target and 180 <= target:
            move.angular.z = -0.1
            pub.publish(move)
        
        elif 1 <= target and 179 >= target:
            move.angular.z = 0.1
            pub.publish(move)

    elif(0 == target):
        FacedWall = True
        move.angular.z = -0.2
        pub.publish(move)
        move.angular.z = 0
        pub.publish(move)


#   Moves the Burger until it is 0.5 meters away from the wall
def GoToWall(scandata, distance):
    global AtWall, pub

    if scandata[0] > distance:
        move.linear.x = 0.1
        move.angular.z = 0.0
        print("Moving towards wall.")
        AtWall = False

    elif scandata[0] <= distance:
        move.linear.x = 0.0
        move.linear.z = 0.0
        print("Arrived at wall.")
        AtWall = True

def CalcTarYaw(yaw, rad):
    return yaw + rad


FirstParallel = True

def BeParallel():
    global target_yaw, yaw, FirstParallel, pub, AtWall, Parallel
    
    if FirstParallel:
        target_yaw = CalcTarYaw(yaw, math.pi/2)
        print("Yaw: " + str(yaw))
        print("Target Yaw: " + str(target_yaw))
        FirstParallel = False
    
    elif not FirstParallel:
        while(round(yaw,2) != round(target_yaw,2)):
            move.angular.z = 0
            if(round(yaw,2) > round(target_yaw,2)):
                move.angular.z = -0.1
                # print("Past target yaw. Yaw: " + str(round(yaw,2)) + " Target Yaw: " + str(round(target_yaw,2)))
                pub.publish(move)
            
            elif(round(yaw,2) < round(target_yaw,2)):
                move.angular.z = 0.1
                # print("Hasn't reached. Yaw: " + str(round(yaw,2)) + " Target Yaw: " + str(round(target_yaw,2)))
                pub.publish(move)     
            
            else:
                print("Error: BeParallel")

        Parallel = True
        move.angular.z = -0.1
        pub.publish(move)
        move.angular.z = 0
        pub.publish(move)

#   Constantly checking if there is a space 
def CheckForSpace(scandata):

    if scandata[270] > 0.45:
        return True
    elif scandata[270] < 0.45:
        return False

FirstFollow = True
target_right = 0.3
wall_check = 0.2

def FollowWall(scandata):

    global FoundSpot, FirstFollow, target_right, pub, wall_check

    FoundSpot = CheckForSpace(scandata)

    print("Scandata[270]: " + str(scandata[270]))

    if FirstFollow and FoundSpot:
        print("Error: Is aligned to wall but distance is too far.")

    elif not FoundSpot:

        if FirstFollow:
            target_right = scandata[270]
            FirstFollow = False
        
        elif not FirstFollow:
            
            if scandata[270] > target_right:
                move.linear.x = 0.05
                move.angular.z = -0.02

            elif scandata[270] < target_right:
                move.linear.x = 0.05
                move.angular.z = 0.01
                wall_check = scandata[270]
            else:
                print("Error: FollowWall.")
    
    elif FoundSpot:
        print("Parking spot has been detected.")
        print("Wall Check:" + str(wall_check))

FrontOfSpace = False
def PrepareManeuver(scandata):
    global FrontOfSpace, wall_check

    print("Target_Right: " + str(wall_check))
    print("Going to the front of the space.")
    print("Scandata[270]: " + str(scandata[270]))

    if scandata[270] > wall_check:
        move.linear.x = 0.1
        move.angular.z = 0.0

    elif scandata[270] <= wall_check:
        move.linear.x = 0
        FrontOfSpace = True

Prepped1 = False
def PrepBackIn():
    global target_yaw, BackedIn, yaw, pub, Prepped1

    print("Performing parallel park.")

    target_yaw = CalcTarYaw(yaw, math.pi/4)
    print(target_yaw)

    print("Target Yaw: " + str(round(target_yaw, 2)))
    print("Yaw: " + str(round(yaw, 2)))

    while(round(yaw, 2) != round(target_yaw, 2)):
        move.angular.z = 0.05
        pub.publish(move)

        move.linear.x = 0
        pub.publish(move)
    
    print("Rotated 45 degrees and ready to back in.")
    move.angular.z = 0.0
    pub.publish(move)
    Prepped1 = True


BackedIn = False

def BackIn(scandata):
    global BackedIn

    if scandata[180] <= 0.15:
        BackedIn = True
        print("Inside the spot.")  

    elif not BackedIn:
        print("Scandata[180]: " + str(scandata[180]))

        move.linear.x = -0.05
        pub.publish(move)

Prepped = False
def PrepAdjustFrontBack():
    global yaw, target_yaw, Prepped

    print("Realigning to wall.")

    target_yaw = CalcTarYaw(yaw, -(math.pi/4))

    print("Target Yaw: " + str(round(target_yaw, 2)))
    print("Yaw: " + str(round(yaw, 2)))

    while(round(yaw, 2) != round(target_yaw, 2)):
        move.angular.z = -0.05
        pub.publish(move)

    move.linear.x = 0
    pub.publish(move)
    Prepped = True


Parked = False
def AdjustFrontBack(scandata):
    global Parked
    print("Adjusting front and back.")

    front = round(scandata[0], 2)
    back = round(scandata[180], 2)

    print("Front: " + str(front))
    print("Back: " + str(back))

    if(front == back):
        Parked = True
        move.linear.x = 0

    elif(front != back):
        if front > back:
            move.linear.x = 0.05

        elif back > front:
            move.linear.x = -0.05




rospy.init_node('parallel_park_node')

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

sub = rospy.Subscriber('/scan', LaserScan, LaserCall)

osub = rospy.Subscriber('/odom', Odometry, GetRotation)

move = Twist()

finding_wall = True
turning = False

Parallel = False
FoundSpot = False

rate = rospy.Rate(10)

while not rospy.is_shutdown():

    while(ranges[0] == 1000):
        pass
        
    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)
    
    if not FacedWall and not AtWall and not Parallel:
        print("Needs to face the wall.")
        FaceWall(ranges)

    elif FacedWall and not AtWall and not Parallel:
        GoToWall(ranges, 0.3)

    elif FacedWall and AtWall and not Parallel:
        print("Aligning right side.")
        BeParallel()

    elif FacedWall and AtWall and Parallel and not FoundSpot:
        print("Following wall and finding a spot.")
        FollowWall(ranges)
    
    elif FoundSpot:

        if not FrontOfSpace:
            PrepareManeuver(ranges)

        elif FrontOfSpace:

            if not Prepped1:
                PrepBackIn()

            elif Prepped1 and not BackedIn and not Parked:
                BackIn(ranges)

            elif BackedIn and not Prepped and not Parked:
                PrepAdjustFrontBack()
            
            elif BackedIn and Prepped and not Parked:
                AdjustFrontBack(ranges)

            elif BackedIn and Parked:
                print("Parking complete.")
                break


    pub.publish(move)

    rate.sleep()


    



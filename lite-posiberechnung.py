#veraltete Datei
# neue Version: lite_team.py

import rospy
import roslaunch
import subprocess
from subprocess import Popen
from math import pi
import math
import time
import numpy as np
import random

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalID

def waitUntil():
    wU = True
    while wU:
        if 'yaw' in globals():
            print("yaw is now declared!")
            wU = False
        time.sleep(3)

def callback(msg):
    global actual_x, actual_y, orientation_z, orientation_w
    actual_x = msg.pose.pose.position.x
    actual_y = msg.pose.pose.position.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

def rotate(target, kP):
    bool = True
    while bool:
        target_rad = target * pi/180
        rot.angular.z = kP * (target_rad-yaw)
        pub_rotate.publish(rot)
        print("Target={} Current:{} Speed:{}".format(target_rad, yaw, rot.angular.z))
        r.sleep()
        if yaw > 1 and target == -180:
            target = 0
        if (target_rad - yaw) > -0.001 and target == 0:
            kP = 0
            rot.angular.z = kP * (target_rad-yaw)
            pub_rotate.publish(rot)
            print("FINISH: Target={} Current:{} Speed:{}".format(target_rad, yaw, rot.angular.z))
            r.sleep()
            bool = False
            break
    
def rotate2(target, kP):
    bool = True
    while bool:
        target_rad = target * pi/180
        rot.angular.z = kP * (target_rad-yaw)
        pub_rotate.publish(rot)
        print("Target={} Current:{} Speed:{}".format(target_rad, yaw, rot.angular.z))
        r.sleep()
        if abs((target_rad - yaw)) < 0.001:
            rot.angular.z = 0.0
            pub_rotate.publish(rot)
            print("FINISH: Target={} Current:{} Speed:{}".format(target_rad, yaw, rot.angular.z))
            r.sleep()
            bool = False
            break

def assign_goal(pose):  
    goal_pose = MoveBaseGoal()        
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def move_to_one_goal(goal_point):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    client.wait_for_server()
    
    sc = SoundClient()
    path_to_sounds = "/opt/ros/melodic/share/sound_play/sounds/"    

    client.send_goal(goal_point)
    success = client.wait_for_result()
    rospy.loginfo("reached destination!")
    sc.playWave(path_to_sounds+"say-beep.wav")

def absolute_coordiantes(distance, angle):
    distance = (distance/100.0)

    # Bot schaut von oben nach unten
    if orientation_w <= 0.35 and orientation_w >= -0.35 and orientation_z >= 0.935:
        angle = -angle - 180

    # Bot schaut von rechts nach links
    if orientation_w > 0.35 and orientation_w <= 0.935 and orientation_z < 0.935 and orientation_z > 0.35:
        angle = -angle + 90

    # Bot schaut von links nach rechts
    if orientation_w > -0.35 and orientation_w <= 0.935 and orientation_z < 0.935 and orientation_z < -0.35:
        angle = angle - 90

    cos = math.cos(math.radians(angle))
    sin = math.sin(math.radians(angle))

    absolute_x = actual_x + distance * cos
    absolute_y = actual_y + distance * sin
    
    return[(absolute_x, absolute_y, 0.0), (0.0, 0.0, orientation_z, orientation_w)]

def stop_movement():
    cancel_pub.publish(cancel_msg)

if __name__ == '__main__':

    # Subscriber, Publisher, Node
    rospy.init_node('init_node', anonymous=True)

    pub_rotate = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub_rotate = rospy.Subscriber("/odom", Odometry, get_rotation)
    r = rospy.Rate(10)
    rot = Twist()

    odom_sub = rospy.Subscriber("/odom", Odometry, callback)

    pub_speed = rospy.Publisher("/cmd_vel", Twist)
    speed = Twist()

    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cancel_msg = GoalID()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/aimotion/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/init_node.launch"])
    launch.start()

    gazebo_node = subprocess.Popen(["roslaunch", "turtlebot3_gazebo", "turtleworld.launch"])
    slam_node = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"])

    rospy.sleep(5)

    # Needed Variables
    counter = 1
    kP = 0.5
    target = -180
    
    actual_obj_names = []
    all_obj_names = []
    actual_obj_coordiantes = []
    all_obj_coordiantes = []

    while not rospy.is_shutdown():
        
        # rotate bot in the beginning
        if counter == 1:
            waitUntil()
            rospy.sleep(1)

            #rotate(target, kP)
            rospy.sleep(1)

            rospy.loginfo("Rotation Done")

            counter = 2
            move_base = subprocess.Popen(["roslaunch", "turtlebot3_navigation", "move_base.launch"])
            
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

        # save map every X seconds
        if counter%10000000 == 0:
            subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "/home/aimotion/tmp"])

        # testweise, um objekt + koordianten nach gewisser zeit zu erhalten
        if counter%20000000 == 0:
            explore_lite.terminate()
            stop_movement()

            rospy.sleep(3)
            counter = 2

            #angle = random.uniform(-180, 180)

            angle = 20

            array = [["hulk", angle, 30.0]]

            print("angel: ", angle)
            
            for object in array:
                actual_obj_names.append(object[0])
                all_obj_names.append(object[0])
                actual_obj_coordiantes.append(absolute_coordiantes(object[2], object[1]))
                all_obj_coordiantes.append(absolute_coordiantes(object[2], object[1]))
        
        counter += 1

        # object received?
        if len(actual_obj_names) > 0:
            rospy.loginfo("received object")
            actual_obj_names.pop()
            move_base = subprocess.Popen(["roslaunch", "turtlebot3_navigation", "move_base.launch"])
            rospy.sleep(3)
            rospy.loginfo("starting coordinates")

            for coordiantes in actual_obj_coordiantes:
                print("coords: ", coordiantes)
                goal = assign_goal(coordiantes)
                move_to_one_goal(goal)
                actual_obj_coordiantes.pop()
                explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
                rospy.sleep(1)


    # shut down everything
    #gazebo_node.terminate()
    explore_lite.terminate()
    move_base.terminate()
    slam_node.terminate()

    launch.shutdown()

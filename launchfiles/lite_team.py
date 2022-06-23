from statistics import median
import statistics
import rospy
import roslaunch
import subprocess
from subprocess import Popen
from math import pi
import math
import time
import numpy as np
import random

from object_detection.msg import ObjectPositionArray, ObjectPosition
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import MarkerArray, Marker
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String, Bool

def waitUntil():
    wU = True
    while wU:
        if 'yaw' in globals():
            print("yaw is now declared!")
            wU = False
        time.sleep(3)

def callback_odom(msg):
    global actual_x, actual_y, orientation_z, orientation_w
    actual_x = msg.pose.pose.position.x
    actual_y = msg.pose.pose.position.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w

def callback_ki(msg):
    global array
    x = msg.objects
    array = []
    for i in x:
        object_id = i.obj_id.lower()
        distance = i.distance
        angle = i.angle
        array.append([object_id, angle, distance])
    #print("array ", array)

def callback_uxd(msg):
    global order
    order = msg.data.lower()

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

def rotate():
    speed = 20
    angle = 370
    clockwise = True

    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    rot.linear.x=0
    rot.linear.y=0
    rot.linear.z=0
    rot.angular.x=0
    rot.angular.y=0

    if clockwise:
        rot.angular.z = -abs(angular_speed)
    else:
        rot.angular.z = abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0 
    while (current_angle < relative_angle):
        pub_rotate.publish(rot)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    rot.angular.z = 0
    pub_rotate.publish(rot)

def assign_goal(pose): 
    print("pose", pose) 
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

def absolute_coordinates(distance, angle):
    distance = (distance/1000.0)

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

def object_detection(array):
    for object in array:
        if object[0] == "captain_america":
            captain_america_coordinates.append(absolute_coordinates(object[2], object[1]))
        if object[0] == "hulk":
            hulk_coordinates.append(absolute_coordinates(object[2], object[1]))
        if object[0] == "iron_man":
            iron_man_coordinates.append(absolute_coordinates(object[2], object[1]))

def anfahrt(finish, explore_lite):
    if uxd_goal[0] == "captain_america":
        if len(captain_america_coordinates) > 0:
            print("Terminiere E_L")
            explore_lite.terminate()
            stop_movement()
            rospy.sleep(3)
            median_coordinates = get_median_of_coordinates(captain_america_coordinates)
            print("median_coords: ", median_coordinates)
            drive_to_coordiantes(median_coordinates)
            print("Starte E_L")
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
            rospy.sleep(1)
            finish += 1

    if uxd_goal[0] == "hulk":
        print("Len Coordi", len(hulk_coordinates))
        if len(hulk_coordinates) > 0:
            print("Terminiere E_L")
            explore_lite.terminate()
            stop_movement()
            rospy.sleep(3)
            median_coordinates = get_median_of_coordinates(hulk_coordinates)
            print("median_coords: ", median_coordinates)
            drive_to_coordiantes(median_coordinates)
            print("Starte E_L")
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
            rospy.sleep(1)
            finish += 1

    if uxd_goal[0] == "iron_man":
        if len(iron_man_coordinates) > 0:
            print("Terminiere E_L")
            explore_lite.terminate()
            stop_movement()
            rospy.sleep(3)
            median_coordinates = get_median_of_coordinates(iron_man_coordinates)
            print("median_coords: ", median_coordinates)
            drive_to_coordiantes(median_coordinates)
            print("Starte E_L")
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
            rospy.sleep(1)
            finish += 1

def drive_to_coordiantes(median_coordinates):
    goal = assign_goal(median_coordinates)
    move_to_one_goal(goal)
    uxd_pub.publish(Bool(True))
    
def get_median_of_coordinates(saved_coordinates):
    median_x = [] 
    median_y = []

    for point in saved_coordinates:
        median_x.append(point[0][0])
        median_y.append(point[0][1])
    
    median_x.sort()
    median_y.sort()

    final_x = statistics.median(median_x)
    final_y = statistics.median(median_y)

    return[(final_x, final_y, 0.0), (0.0, 0.0, orientation_z, orientation_w)]

if __name__ == '__main__':

    # Subscriber, Publisher, Node
    rospy.init_node('init_node', anonymous=True)

    pub_rotate = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    sub_rotate = rospy.Subscriber("/odom", Odometry, get_rotation)
    r = rospy.Rate(10)
    rot = Twist()

    odom_sub = rospy.Subscriber("/odom", Odometry, callback_odom)

    cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
    cancel_msg = GoalID()

    ki_sub = rospy.Subscriber("/detected_objects", ObjectPositionArray, callback_ki)

    uxd_sub = rospy.Subscriber('/current_goal', String, callback_uxd)
    uxd_pub = rospy.Publisher('/goal_success', Bool, queue_size=1)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/aimotion/TBP/init_node.launch"])
    launch.start()

    slam_node = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"])

    rospy.sleep(5)

    counter = 1
    finish = 0
    order = ""
    array = []
    
    # Erhaltenen Reihenfolge der Anfahrt in goals_in_order (UXD)

    hulk_coordinates = []
    captain_america_coordinates = []
    iron_man_coordinates = []

    while not rospy.is_shutdown():
        
        if counter == 1:
            waitUntil()
            rospy.sleep(1)

            #rotate()
            rospy.sleep(1)
            rospy.loginfo("Rotation Done")

            counter = 2

            rospy.sleep(10)

            move_base = subprocess.Popen(["roslaunch", "turtlebot3_navigation", "move_base.launch"])
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

        counter += 1

        if len(array) > 0:
            object_detection(array)
            array = []

        uxd_goal = [order]

        if uxd_goal[0] == "captain_america":
            if len(captain_america_coordinates) > 0:
                print("Terminiere E_L")
                explore_lite.terminate()
                stop_movement()
                rospy.sleep(3)
                median_coordinates = get_median_of_coordinates(captain_america_coordinates)
                print(order)
                print("median_coords: ", median_coordinates)
                print("")
                drive_to_coordiantes(median_coordinates)
                print("Starte E_L")
                explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
                rospy.sleep(1)
                finish += 1

        if uxd_goal[0] == "hulk":
            print("Len Coordi", len(hulk_coordinates))
            if len(hulk_coordinates) > 0:
                print("Terminiere E_L")
                explore_lite.terminate()
                stop_movement()
                rospy.sleep(3)
                median_coordinates = get_median_of_coordinates(hulk_coordinates)
                print(order)
                print("median_coords: ", median_coordinates)
                print("")
                drive_to_coordiantes(median_coordinates)
                print("Starte E_L")
                explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
                rospy.sleep(1)
                finish += 1

        if uxd_goal[0] == "iron_man":
            if len(iron_man_coordinates) > 0:
                print("Terminiere E_L")
                explore_lite.terminate()
                stop_movement()
                rospy.sleep(3)
                median_coordinates = get_median_of_coordinates(iron_man_coordinates)
                print(order)
                print("median_coords: ", median_coordinates)
                print("")
                drive_to_coordiantes(median_coordinates)
                print("Starte E_L")
                explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])
                rospy.sleep(1)
                finish += 1
        
        if counter%5000:
            print("")
            print("")
            print("Hulk: ", hulk_coordinates)
            print("Captain America: ", captain_america_coordinates)
            print("Iron Man: ", iron_man_coordinates)
            print("")
            print("")
        

        if finish == 3:
            explore_lite.terminate()
            move_base.terminate()
            slam_node.terminate()

            launch.shutdown()



    # shut down everything
    explore_lite.terminate()
    move_base.terminate()
    slam_node.terminate()

    launch.shutdown()

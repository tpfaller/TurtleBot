import statistics
import rospy
import roslaunch
import subprocess
import math
import time
import actionlib

from object_detection.msg import ObjectPositionArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String, Bool
from math import pi
from statistics import median



# wait until yaw of robot is declared
def waitUntil():
    wU = True
    while wU:
        if 'yaw' in globals():
            print("yaw is now declared!")
            wU = False
        time.sleep(3)

# return robot coordiantes and orientation
def callback_odom(msg):
    global actual_x, actual_y, orientation_z, orientation_w
    actual_x = msg.pose.pose.position.x
    actual_y = msg.pose.pose.position.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w

# return detected objects
def callback_ki(msg):
    global array
    x = msg.objects
    array = []
    for i in x:
        object_id = i.obj_id.lower()
        distance = i.distance
        angle = i.angle
        array.append([object_id, angle, distance])

# return object approach order
def callback_uxd(msg):
    global order
    order = msg.data.lower()

# return roll, pitch and yaw of robots current position
def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

# rotate 360°
def rotate():
    speed = 20
    angle = 360
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

# collect multiple object coordiantes, delete outlier
def object_detection(array):
    for object in array:
        if object[2] > 2500:
            continue
        if object[0] == "captain_america":
            captain_america_coordinates.append(absolute_coordinates(object[2], object[1]))
        if object[0] == "hulk":
            hulk_coordinates.append(absolute_coordinates(object[2], object[1]))
        if object[0] == "iron_man":
            iron_man_coordinates.append(absolute_coordinates(object[2], object[1]))

# compute absolute coordiantes of detected objects
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

# procedure to drive to an object, if it's the target object
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

# compute median of collected object coordiantes
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

# procedure to drive to goal and return signal when done
def drive_to_coordiantes(median_coordinates):
    goal = assign_goal(median_coordinates)
    move_to_one_goal(goal)
    uxd_pub.publish(Bool(True))

# set goal which the robot drives to
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

# start requirements to drive to coordiante and make a sound when arriving
def move_to_one_goal(goal_point):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    client.wait_for_server()
    
    sc = SoundClient()
    path_to_sounds = "/opt/ros/melodic/share/sound_play/sounds/"    

    client.send_goal(goal_point)
    success = client.wait_for_result()
    rospy.loginfo("reached destination!")
    sc.playWave(path_to_sounds+"say-beep.wav")

# stop movement completely
def stop_movement():
    cancel_pub.publish(cancel_msg)



if __name__ == '__main__':

    # declare subscribers, publisher and an initial node
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

    # start slam and rviz
    slam_node = subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch"])
    rospy.sleep(5)

    # helping attributes
    counter = 1
    finish = 0
    order = ""
    array = []
    hulk_coordinates = []
    captain_america_coordinates = []
    iron_man_coordinates = []

    # loop of robot
    while not rospy.is_shutdown():
        
        # start rotation
        if counter == 1:
            waitUntil()
            rotate()
            rospy.sleep(1)
            rospy.loginfo("Rotation Done")

            counter = 2

            # start move_base and explore_lite to traverse environment
            move_base = subprocess.Popen(["roslaunch", "turtlebot3_navigation", "move_base.launch"])
            explore_lite = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

        counter += 1

        # objects detected -> collect coordiantes
        if len(array) > 0:
            object_detection(array)
            array = []

        uxd_goal = [order]

        # check if we got coordiantes for target object, compute and drive to them
        anfahrt(finish, explore_lite)

        # shutdown if every object has been approached
        if finish == 3:
            explore_lite.terminate()
            move_base.terminate()
            slam_node.terminate()

            launch.shutdown()
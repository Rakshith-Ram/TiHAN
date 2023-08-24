
############################################################  LIDAR NAVIGATION CODE FOR MAINI VEHICLE  ##############################################################



import math 
import numpy as np   
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, UInt16, Float32, Float64, Float32MultiArray


current_x = 0
current_y = 0
steering_angle = 0
heading = 0
wp = 0
current_vel = 0
velocity_value = 0
current_steer = 0
obstacle_distance = 0


def extract_waypoints_from_file(file_path):
    try:
        with open(file_path, 'r') as file:
            file_contents = file.read()
    except FileNotFoundError:
        print("File not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    waypoints = []
    waypoints_data = file_contents.strip().split('\n')
    for line in waypoints_data:
        x, y = map(float, line.strip("[],").split(','))
        waypoints.append([x, y])
    return(waypoints)


def fun_bearing_diff(Required_Bearing):

    global heading, current_bearing
    current_bearing = heading 

    while current_bearing is None:
        current_bearing = heading 

    current_bearing = float(current_bearing)
    bearing_diff = -current_bearing + Required_Bearing

    if bearing_diff < -180:
        bearing_diff = (bearing_diff+360)
    if bearing_diff > 180:
        bearing_diff = bearing_diff-360
         
    return bearing_diff


def callback_ndt_pose(data):
   global current_x
   global current_y
   current_x = data.pose.position.x
   current_y = data.pose.position.y


def callback_cur_steer(data):
   global current_steer
   current_steer=data.data


def callback_min_distance(data):
   global obstacle_distance
   obstacle_distance = data.data


def callback_eular_angle(data):
   global heading
   heading = math.degrees(data.data[2]) 


def listener():
   rospy.init_node("topic_subscriber")
   rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
   rospy.Subscriber("/eular_angle", Float32MultiArray, callback_eular_angle)
   rospy.Subscriber("/currentSteerAngle", Float64, callback_cur_steer)
   rospy.Subscriber("/lidar_min_distance_topic", Float32, callback_min_distance)


def find_nearest_waypoint_index(waypoints):

    distance_threshold = 3
    min_distance = float("inf")
    next_index = float("inf")

    for i in range(len(waypoints) - 1):
        waypoint = waypoints[i]
        waypoint_x, waypoint_y = waypoint
        distance = ((waypoint_x - current_x) ** 2 + (waypoint_y - current_y) ** 2) ** 0.5

        if distance < min_distance and distance < distance_threshold: 
            min_distance = distance
            next_index = i

    return(next_index)


waypoints = extract_waypoints_from_file("/home/rakshith/waypoints/waypoints_testbed")

listener()

rospy.wait_for_message('/ndt_pose', PoseStamped)

wp = find_nearest_waypoint_index(waypoints)

print("Nearest detected waypoint index:", wp)
print("Current co-ordinates : ", current_x, ", ", current_y)


while True:

    distance = math.sqrt((current_x - waypoints[wp][0]) ** 2 + (current_y - waypoints[wp][1]) ** 2)

    
    print("Current_x : ", current_x)
    print("Current_y : ", current_y)
    print("Distance between current pos and waypoint is : ", distance)
    print("Current heading is : ", heading)










    time.sleep(0.1)

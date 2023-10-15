#####################################  COLLECT WAYPOINTS AT UNIFORM DISTANCE  #####################################

import rospy
import math
from geometry_msgs.msg import PoseStamped


def callback(data):
    global waypoint
    
    # Extract x and y coordinates from the PoseStamped message
    x = data.pose.position.x
    y = data.pose.position.y
    
    # Update the current waypoint
    waypoint = [x, y]

def calc_distance(past_wp, waypoint):
    if past_wp is None or waypoint is None:
        return None
    distance = math.sqrt((past_wp[0] - waypoint[0]) ** 2 + (past_wp[1] - waypoint[1]) ** 2)
    return distance

set_distance = 1.0

# Initialize ROS node
rospy.init_node('waypoint_collector', anonymous=True)

waypoint = None

file_path = 'waypoints_unf_dis.txt'

# Subscribe to the topic
rospy.Subscriber('/ndt_pose', PoseStamped, callback)

# Wait for the first message to arrive
rospy.wait_for_message('/ndt_pose', PoseStamped)

past_wp = waypoint

# Start recording waypoints
while not rospy.is_shutdown():
    # Check if the distance has been covered
    d = calc_distance(past_wp, waypoint)
    if d is not None and d >= set_distance:
        past_wp = waypoint
        with open(file_path, 'a') as file:
            file.write(f"[{waypoint[0]},{waypoint[1]}],\n")

    # Clear the waypoint for the next round
    waypoint = None

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#


'''
Created by Rakshith on 16-9-23
'''

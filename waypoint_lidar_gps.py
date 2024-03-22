'''
This code records waypoints at equal distance intervals and prints it in a .txt file
X, Y, Z, q1, q2, q3, q4 parameters from '/ndt_pose' and lat, long from '/an_device/NavSatFix'
'''

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

import rospy
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


def callback2(data):
    global coord
    '''
    rospy.loginfo("Latitude: %f, Longitude: %f", data.latitude, data.longitude)
    '''
    # Extract the latitude and longitude values
    lat = data.latitude
    lon = data.longitude

    # Update current waypoint
    coord = [lat, lon]


def callback(data):
    global waypoint
    '''
    rospy.loginfo("X : %f, Y : %f, Z : %f, q1 : %f, q2 : %f, q3 : %f, q4 : %f", 
                  data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, 
                  data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    '''
    # Extract x and y coordinates from the PoseStamped message
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    q1 = data.pose.orientation.x
    q2 = data.pose.orientation.y
    q3 = data.pose.orientation.z
    q4 = data.pose.orientation.w

    # Update the current waypoint
    waypoint = [round(x,4), round(y,4), round(z,4), round(q1,4), round(q2,4), round(q3,4), round(q4,4)]


def calc_distance(past_wp, waypoint):
    if past_wp is None or waypoint is None:
        return None
    distance = math.sqrt((past_wp[0] - waypoint[0]) ** 2 + (past_wp[1] - waypoint[1]) ** 2)
    return distance


# Distace after which each waypoint has to be collected
set_distance = 1.5

# Initialize ROS node
rospy.init_node('waypoint_collector', anonymous=True)

# Global variables
waypoint = None
coord = None

# Path and name of the file where the waypoints are stored
file_path = 'waypoints_lidar_gps_ud.txt'

# Subscribe to the topics
rospy.Subscriber('/ndt_pose', PoseStamped, callback)
rospy.Subscriber('/an_device/NavSatFix', NavSatFix, callback2)

# Wait for the first message to arrive
rospy.wait_for_message('/ndt_pose', PoseStamped)

past_wp = waypoint

# Start recording waypoints
while not rospy.is_shutdown():
    # Check if the distance has been covered
    d = calc_distance(past_wp, waypoint)
    if d is not None and d >= set_distance:
        past_wp = waypoint
        # Write the waypoint into the file
        with open(file_path, 'a') as file:
            file.write(f"[{waypoint[0]},{waypoint[1]},{waypoint[2]},{waypoint[3]},{waypoint[4]},{waypoint[5]},{waypoint[6]},{coord[0]},{coord[1]}],\n")

    # Clear the waypoint and coord for the next round
    waypoint = None
    coord = None

#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram : [6-2-24]
'''

'''
This code takes in the current GPS coordinates and finds the index of nearest corresponding LiDAR waypoint on the map.
It also publishes the initial position parameters for the NDT localizer algorithm.
'''

#:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

from __future__ import print_function, division, absolute_import
import rospy
from math import sqrt
from math import radians, sin, cos, sqrt, atan2
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped


current_gps = []
#current_gps = [17.60113931352057,78.12665390392331]


def callback(data):
    global current_gps

    # Extract the latitude and longitude values
    lat = data.latitude
    lon = data.longitude

    # Update current GPS coord
    current_gps = [lat, lon]


def extract_data_from_file(file_path):
    
    try:
        with open(file_path, 'r') as file:
            file_contents = file.read()
    except FileNotFoundError:
        print("File not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    points = []
    points_data = file_contents.strip().split('\n')
    for line in points_data:
        x, y, z, q1, q2,q3, q4, lat, lon = map(float, line.strip("[],").split(','))
        points.append([x, y, z, q1, q2,q3, q4, lat, lon])
    return(points)


def haversine(lat1, lon1, lat2, lon2):
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    # Radius of the Earth in meters
    R = 6371000.0

    # Calculate the distance
    distance = R * c
    return distance


def find_nearest_point_index(points_data):
    global current_gps

    distance_threshold = 15
    min_distance = float("inf")
    next_index = float("inf")

    for i in range(len(points_data)):
        point = points_data[i]
        point_lat, point_lon = point[7], point[8]
        distance = haversine(current_gps[0], current_gps[1], point_lat, point_lon)
        #print(distance)

        if distance < min_distance and distance < distance_threshold: 
            min_distance = distance
            next_index = i
    print("The nearest wapoint index is : ", next_index)
    return(next_index)


def publish_initial_pose():
    global found_point

    initial_pose = Pose()
    x = found_point[0] 
    y = found_point[1]
    z = found_point[2]
    qx = found_point[3]
    qy = found_point[4]
    qz = found_point[5]
    qw = found_point[6]
    
    pub_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    # Create PoseWithCovarianceStamped message
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.pose.pose.position.x = x
    initial_pose.pose.pose.position.y = y
    initial_pose.pose.pose.position.z = z
    initial_pose.pose.pose.orientation = Quaternion(qx, qy, qz, qw)
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = 'map'
    rospy.sleep(2)

    # Publish the initial pose
    pub_pose.publish(initial_pose)


# Initialize ROS node
rospy.init_node('finding_initial_pose', anonymous=True)

#Subscribe to the GPS topic
rospy.Subscriber('/an_device/NavSatFix', NavSatFix, callback)
while not current_gps and not rospy.is_shutdown():
        rospy.sleep(0.001)


points_data = extract_data_from_file("")


nearest_point = find_nearest_point_index(points_data)
found_point = points_data[nearest_point]

publish_initial_pose()
print("Published initial pose !")


#:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#


'''
Rakshith Ram  &  Abhishek Thakur  [15-2-24]

Note : Paste waypoints file path in line 125
'''

import math
import matplotlib.pyplot as plt


waypoints = []
offset_waypoints = []


# Function to extract the waypoints from the generated file
def extract_waypoints_from_file(file_path):

    global waypoints

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


# Function to generate another set of waypoints with the given "offset". Also plots the two set of waypoints
def genrate_waypoints(waypoints,offset):
    
    global offset_waypoints

    for i in range(len(waypoints) - 1):

        x1 = waypoints[i][0]
        x2 = waypoints[i+1][0]
        y1 = waypoints[i][1]
        y2 = waypoints[i+1][1]

        theta = math.atan2((y2-y1),(x2-x1))

        offset_x1 = x1 + offset * math.sin(theta)
        offset_y1 = y1 - offset * math.cos(theta)

        offset_point = [offset_x1, offset_y1]
        offset_waypoints.append(offset_point)


# Function to plot original and generated waypoints
def plot_both():

    x_values = [wp[0] for wp in waypoints]
    y_values = [wp[1] for wp in waypoints]

    x_values_2 = [wp[0] for wp in offset_waypoints]
    y_values_2 = [wp[1] for wp in offset_waypoints]

    plt.plot(x_values, y_values, ".b")
    plt.plot(x_values_2, y_values_2, ".r")
    plt.show()


# Write the generated waypoints in a file
def create_offset_waypoints_file():

    file_path = 'offset_waypoints.txt'      #  File of this name will be created

    with open(file_path, 'w') as file:
        for i in range(len(offset_waypoints) - 1):
            file.write(f"[{offset_waypoints[i][0]},{offset_waypoints[i][1]}]\n")



extract_waypoints_from_file("/home/rakshith/waypoints/waypoints_testbed")       # put path of original waypoints here
genrate_waypoints(waypoints, 1)
create_offset_waypoints_file()
plot_both()

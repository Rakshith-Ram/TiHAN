

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

waypoints = extract_waypoints_from_file("/home/rakshith/waypoints/waypoints_testbed")

for i in range(len(waypoints)):
    print(waypoints[i])

    
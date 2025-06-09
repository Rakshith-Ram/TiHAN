########################################  PLOT WAYPOINTS ########################################

import matplotlib.pyplot as plt

# Read waypoints from file
waypoints = []
with open("/home/rakshithram/speed_profiling/testbed.txt", "r") as file:
    for line in file:
        line = line.strip().strip("[],")
        if line:
            x, y = map(float, line.split(','))
            waypoints.append((x, y))

# Separate x and y coordinates
x_coords = [pt[0] for pt in waypoints]
y_coords = [pt[1] for pt in waypoints]

# Plot waypoints
plt.figure(figsize=(8, 6))

plt.plot(x_coords, y_coords, marker='.', linestyle='', color='red')
plt.title("Waypoints Plot")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#
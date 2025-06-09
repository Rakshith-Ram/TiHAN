########################################  INTERPOLATED WAYPOINTS  #########################################

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

def read_waypoints_from_file(filename):
    """
    Read waypoints from a file.
    """
    waypoints = []
    with open(filename, 'r') as file:
        for line in file:
            # Remove leading/trailing brackets and split by comma
            point = line.strip().replace('[', '').replace(']', '').split(',')
            # Convert to float and append
            waypoints.append([float(point[0]), float(point[1])])
    
    return np.array(waypoints)

def save_waypoints_to_file(waypoints, filename):
    """
    Save waypoints to a file.
    """
    with open(filename, 'w') as file:
        for point in waypoints:
            file.write(f"[{point[0]},{point[1]}],\n")

def interpolate_waypoints(waypoints, step_size):
    """
    Interpolate waypoints using cubic splines.
    """
    # Compute cumulative distances along the waypoints
    distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0) ** 2, axis=1)))
    distances = np.insert(distances, 0, 0)  # Insert 0 at the start
    
    # Create cubic splines for x and y
    spline_x = CubicSpline(distances, waypoints[:, 0])
    spline_y = CubicSpline(distances, waypoints[:, 1])
    
    # Generate new target distances based on step_size
    target_distances = np.arange(0, distances[-1], step_size)
    
    # Ensure the last point is included
    if target_distances[-1] < distances[-1]:
        target_distances = np.append(target_distances, distances[-1])
    
    # Evaluate the spline at these distances
    new_x = spline_x(target_distances)
    new_y = spline_y(target_distances)
    
    return waypoints, np.vstack((new_x, new_y)).T

def plot_waypoints(original, interpolated):
    """
    Plot original and interpolated waypoints.
    """
    plt.figure(figsize=(8, 6))
    plt.scatter(original[:, 0], original[:, 1], c='r', marker='o', label='Original Waypoints')
    plt.scatter(interpolated[:, 0], interpolated[:, 1], c='b', marker='.', label='Interpolated Waypoints')
    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Waypoint Interpolation using Cubic Spline')
    plt.grid()
    plt.show()


# Example usage
if __name__ == "__main__":
    input_filename = '/home/rakshithram/speed_profiling/testbed.txt'
    output_filename = 'interpolated.txt'
    
    # Read waypoints from file
    waypoints = read_waypoints_from_file(input_filename)
    
    # Reduced step size for more precise interpolation
    step_size = 0.5
    
    original, interpolated = interpolate_waypoints(waypoints, step_size)
    
    # Save interpolated waypoints to file
    save_waypoints_to_file(interpolated, output_filename)
    
    # Plot waypoints
    plot_waypoints(original, interpolated)


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#


'''
Rakshith Ram [30-05-25]
'''
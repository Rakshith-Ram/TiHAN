###########################################  WAYPOINT CURVATURE  ############################################

import numpy as np
import matplotlib.pyplot as plt

waypoints = []
curvatures = []

input_path = "interpolated_2.txt"
output_path = "waypoints_with_curvature.txt"

with open(input_path, "r") as file:
    for line in file:
        line = line.strip().strip("[],")
        if line:
            x, y = map(float, line.split(','))
            waypoints.append((x, y))

waypoints = np.array(waypoints)


# ------------ Compute Curvature using Triangle-Based Method ------------

def compute_curvature_triangle_based(waypoints, window):
    curvatures = []

    def triangle_curvature(p1, p2, p3):
        # Side lengths
        a = np.linalg.norm(p2 - p3)
        b = np.linalg.norm(p1 - p3)
        c = np.linalg.norm(p1 - p2)

        # Semi-perimeter
        s = (a + b + c) / 2.0

        # Heron's formula for area
        area_term = s * (s - a) * (s - b) * (s - c)
        if area_term <= 0:
            return 0.0

        area = np.sqrt(area_term)

        # Curvature kappa = 4 * Area / (abc)
        if a * b * c == 0:
            return 0.0

        curvature = (4.0 * area) / (a * b * c)
        return curvature

    for i in range(len(waypoints) - 2):
        curv_sum = 0
        count = 0
        for j in range(i, min(i + window, len(waypoints) - 2)):
            p1 = waypoints[j]
            p2 = waypoints[j + 1]
            p3 = waypoints[j + 2]
            curv_sum += triangle_curvature(p1, p2, p3)
            count += 1
        avg_curv = curv_sum / count if count > 0 else 0
        curvatures.append(avg_curv)

    return curvatures

curvatures = compute_curvature_triangle_based(waypoints, window=20)


# ------------ Plot Path and Curvature Vectors ------------

fig1, ax = plt.subplots(figsize=(10, 6))

label_added = False

for i in range(1, len(waypoints) - 1):
    if i >= len(curvatures):
        continue

    p_prev = waypoints[i - 1]
    p_next = waypoints[i + 1]
    p_curr = waypoints[i]

    tangent = p_next - p_prev
    tangent_norm = tangent / np.linalg.norm(tangent) if np.linalg.norm(tangent) != 0 else np.array([1, 0])
    normal = np.array([-tangent_norm[1], tangent_norm[0]])

    scale = 50
    offset = normal * curvatures[i] * scale
    print(f"Curvature at point {i}: {curvatures[i]}")

    start = p_curr
    end = p_curr + offset if curvatures[i] > 0 else p_curr - offset

    if not label_added:
        ax.plot([start[0], end[0]], [start[1], end[1]], color='red', linewidth=1.5, label='Curvature vector')
        label_added = True
    else:
        ax.plot([start[0], end[0]], [start[1], end[1]], color='red', linewidth=1.5)

with open(output_path, "w") as f:
    for i in range(len(curvatures)):
        x, y = waypoints[i + 1]  # Shifted by +1 because curvature starts from index 1
        curvature = curvatures[i]
        f.write(f"[{x:.4f},{y:.4f},{curvature:.6f}],\n")

ax.scatter(waypoints[:, 0], waypoints[:, 1], color='blue', s=7, label='Waypoints')
ax.set_title("Waypoints with Perpendicular Vectors Representing Curvature")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.axis('equal')
ax.grid(True)
ax.legend()
plt.tight_layout()
plt.show(block=False)
plt.pause(0.1)


# ------------ Plot Curvature vs Waypoint Index ------------

fig2 = plt.figure(figsize=(10, 4))
plt.plot(range(len(curvatures)), curvatures, marker='.', linestyle='-', color='green')
plt.title("Waypoint Index vs. Curvature")
plt.xlabel("Waypoint Index")
plt.ylabel("Curvature")
plt.grid(True)
plt.tight_layout()
plt.show(block=False)
plt.pause(0.1)

while plt.get_fignums():
    plt.pause(0.1)


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#

'''
Rakshith Ram [11-06-25]
'''

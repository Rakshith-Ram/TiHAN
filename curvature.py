#########################################  WAYPOINT CURVATURE  ##########################################


import numpy as np
import matplotlib.pyplot as plt

# Read waypoints from file
waypoints = []
curvatures = []

output_path = "/home/rakshithram/speed_profiling/waypoints_with_curvature.txt"

with open("/home/rakshithram/speed_profiling/interpolated_waypoints.txt", "r") as file:
    for line in file:
        line = line.strip().strip("[],")
        if line:
            x, y = map(float, line.split(','))
            waypoints.append((x, y))

waypoints = np.array(waypoints)

# === Step 2: Compute Curvature ===
def compute_curvature_avg_future(waypoints, window):
    curvatures = []

    def single_curvature(p1, p2, p3):
        tangent1 = p2 - p1
        tangent2 = p3 - p2
        cross_product = tangent1[0] * tangent2[1] - tangent1[1] * tangent2[0]
        if cross_product == 0:
            return 0
        return abs(cross_product / (np.linalg.norm(tangent1) * np.linalg.norm(tangent2)))

    for i in range(len(waypoints) - 2):
        curv_sum = 0
        count = 0
        for j in range(i, min(i + window, len(waypoints) - 2)):
            p1 = waypoints[j]
            p2 = waypoints[j + 1]
            p3 = waypoints[j + 2]
            curv_sum += single_curvature(p1, p2, p3)
            count += 1
        avg_curv = curv_sum / count if count > 0 else 0
        curvatures.append(avg_curv)

    return curvatures

curvatures = compute_curvature_avg_future(waypoints, window=20)

# === Step 3: Plot Path and Curvature Vectors ===
fig1, ax = plt.subplots(figsize=(10, 6))

label_added = False

# Draw perpendicular lines based on curvature
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
    offset = normal * np.sign(curvatures[i]) * curvatures[i] * scale
    print(f"Curvature at point {i}: {curvatures[i]}")

    start = p_curr
    if curvatures[i] > 0:
        end = p_curr - offset
    elif curvatures[i] < 0:
        end = p_curr + offset
    else:
        continue

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
plt.show(block=False)   # <-- Don't block here
plt.pause(0.1)          # <-- Let this figure render

# === Step 4: Plot Curvature vs Waypoint Index ===
fig2 = plt.figure(figsize=(10, 4))
plt.plot(range(len(curvatures)), curvatures, marker='.', linestyle='-', color='green')
plt.title("Waypoint Index vs. Curvature")
plt.xlabel("Waypoint Index")
plt.ylabel("Curvature")
plt.grid(True)
plt.tight_layout()
plt.show(block=False)   # <-- Show second window non-blocking
plt.pause(0.1)


while plt.get_fignums():
    plt.pause(0.1)


#::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#


'''
Rakshith Ram [30-05-25]
'''
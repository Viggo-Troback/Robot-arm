from func import *
import pickle
from config.config2 import parts
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

#if there exists a matching points dict, load it. Otherwise run config to generate one (manual)
with open("Data_objects/points_dict.pkl", "rb") as f:
    possible_points = pickle.load(f)

with open("Data_objects/points_tree.pkl", "rb") as f:
    points_tree = pickle.load(f)

with open("Data_objects/neighbors.pkl", "rb") as f:
    neighbors = pickle.load(f)

aim, codes =text_to_points("Chalmers", 20, plot=False)

aim=map_to_polar_field(aim,60,130,plot=False)

closest_path=[]
for points in tqdm(aim, desc="Finding closest neighbours"):
    closest = closest_match(points, possible_points, points_tree)
    closest_path.append(closest)

alt_closest_path=[]
for points in tqdm(aim, desc="Finding closest neighbours (radius)"):
    closest = points_within_radius(points, possible_points, points_tree, 2.5)
    alt_closest_path.append(closest)

true_closest_pairs=[]
for i in tqdm(range(len(alt_closest_path)-1), desc="Finding closest neighbours (True)"):
    pair=find_closest_pair(alt_closest_path[i],alt_closest_path[i+1], possible_points)
    true_closest_pairs.append(pair)

paths=[]
for pair in tqdm(true_closest_pairs, desc="pathfinding"):
    path=pathfinder(pair[0], pair[1], neighbors)
    paths.append(path)

parts, point = forward_kinematics(parts, possible_points[closest_path[0]][0])
plot_transformed_parts(parts, point)

fig = plt.figure(figsize=(18, 10))
gs = gridspec.GridSpec(2, 2, width_ratios=[2, 1], height_ratios=[1, 1])
# 2 rows x 2 columns: left is wider

# --- Subplot 1: All Paths (larger, spans both rows on the left) ---
ax1 = fig.add_subplot(gs[:, 0])  # spans both rows
for path in paths:
    if len(path)<5:
        xs, ys = zip(*path)
        ax1.plot(xs, ys, marker='o', label='Path', zorder=1)
ax1.set_aspect('equal', adjustable='box')
ax1.grid(True)
ax1.set_title("All Paths")
ax1.set_xlabel("X")
ax1.set_ylabel("Y")

# --- Subplot 2: Aim vs Closest Reachable Points (top right) ---
ax2 = fig.add_subplot(gs[0, 1])
aim_x, aim_y = zip(*aim)
ax2.plot(aim_x, aim_y, color='blue', label='Aim Points', alpha=0.8, zorder=2)
path_x, path_y = zip(*closest_path)
ax2.plot(path_x, path_y, color='red', label='Closest Reachable Points', alpha=0.8, zorder=3)
ax2.set_aspect('equal', adjustable='box')
ax2.grid(True)
ax2.set_title("Aim vs Closest Reachable Points")
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.legend()

# --- Subplot 3: All Reachable Tip Positions (bottom right) ---
ax3 = fig.add_subplot(gs[1, 1])

# Plot reachable points
x_vals, y_vals = zip(*possible_points)
ax3.scatter(x_vals, y_vals, color='red', s=10, label='Reachable Points')

# Overlay aim points
aim_x, aim_y = zip(*aim)
ax3.scatter(aim_x, aim_y, color='blue', s=20, label='Aim Points', alpha=0.7)

ax3.set_aspect('equal')
ax3.set_title("All Reachable Tip Positions + Aim Overlay")
ax3.set_xlabel("X")
ax3.set_ylabel("Y")
ax3.legend()

plt.tight_layout()
plt.show()
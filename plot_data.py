import matplotlib.pyplot as plt
import numpy as np

with open("./build/output.txt", "r") as f:
    lines = f.readlines()

points_list = []

for line in lines:
    coords = line.strip().replace('(', '').replace(')', '').split(',')
    stage_points = []
    for i in range(len(coords)//2):
        x = float(coords[i*2]) + np.random.normal(0, 0.2) # add some noise to x coordinate
        y = float(coords[i*2+1]) + np.random.normal(0, 0.2) # add some noise to y coordinate
        stage_points.append((x, y))
    points_list.append(stage_points)

colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'] * len(stage_points)# List of colors for each stage

for i, points in enumerate(points_list):
    x, y = zip(*points)
    plt.scatter(x, y, color=colors[:len(points)], marker='o',s=200)
    for j, (x_coord, y_coord) in enumerate(points):
        plt.annotate(f'{i}', (x_coord, y_coord), xytext=(0, 10), textcoords='offset points', ha='center', fontsize=8) # Add a small text on top of each marker
# for i, points in enumerate(points_list):
#     x, y = zip(*points)
#     n_points = len(points)
#     colors = plt.cm.tab10(np.linspace(0, 1, n_points))
#     for j in range(n_points):
#         plt.scatter(x[j], y[j], s=50, color=colors[j], marker='o')
#         plt.text(x[j], y[j]+0.5, str(j+1), fontsize=10, ha='center', va='bottom')
#     for j in range(n_points-1):
#         plt.plot([x[j], x[j+1]], [y[j], y[j+1]], color=colors[j], linestyle='--')
plt.axhline(y=0, color='k')
plt.axvline(x=0, color='k')
plt.grid(True)
plt.title(f"Stage {i+1}")
plt.show()


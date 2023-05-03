import subprocess
import matplotlib.pyplot as plt
import numpy as np
n=3
# Run the C executable and capture its output
result = subprocess.run(["./main", f"n={n}"], stdout=subprocess.PIPE)
output = result.stdout.decode('utf-8')

# Split the output into lines and remove the trailing separator
lines = output.split('\n')[:-2]

# Split the lines into x, y coordinates
points = []
for line in lines:
    coords = line.strip()[1:-1].split(', ')
    points.append((float(coords[0]), float(coords[1])))

# Separate the points into groups
groups = []
current_group = []
for point in points:
    if len(current_group) == 0 or current_group[0][0] == point[0]:
        current_group.append(point)
    else:
        groups.append(current_group)
        current_group = [point]
groups.append(current_group)

# Plot the points
colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
fig, ax = plt.subplots()
for i, group in enumerate(groups):
    xs, ys = zip(*group)
    ax.scatter(xs, ys, color=colors[i % len(colors)], label=f'Group {i+1}')

ax.legend()
plt.show()

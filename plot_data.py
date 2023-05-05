import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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

fig, ax = plt.subplots()

# Define the update function for the animation
def update(frame):
    ax.clear()
    # Get the points for the current stage
    points = points_list[frame]
    x, y = zip(*points)
    # Plot the points
    ax.scatter(x, y, color=colors[:len(points)], marker='o',s=200)
    # Add axes lines and grid
    ax.axhline(y=0, color='k')
    ax.axvline(x=0, color='k')
    ax.grid(True)
    # Set the title for the current stage
    ax.set_title(f"Stage {frame+1}")

# Create the animation object
ani = FuncAnimation(fig, update, frames=len(points_list), interval=1000, repeat=False)

# Save the animation as a video
ani.save('animation.mp4', fps=1)
plt.show()


# for i, points in enumerate(points_list):
#     x, y = zip(*points)
#     plt.scatter(x, y, color=colors[:len(points)], marker='o',s=200)
    
#     plt.axhline(y=0, color='k')
#     plt.axvline(x=0, color='k')
#     plt.grid(True)
#     plt.title(f"Stage {i+1}")
#     plt.show()





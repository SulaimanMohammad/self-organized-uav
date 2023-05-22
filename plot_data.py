import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

with open("./build/output.txt", "r") as f:
    lines = f.readlines()

points_list = []

# for line in lines:
#     coords = line.strip().replace('(', '').replace(')', '').split(',')
#     stage_points = []
#     for i in range(len(coords)//2):
#         x = float(coords[i*2]) + np.random.normal(0, 0.2) # add some noise to x coordinate
#         y = float(coords[i*2+1]) + np.random.normal(0, 0.2) # add some noise to y coordinate
#         stage_points.append((x, y))
#     points_list.append(stage_points)
for line in lines:
    coords = line.strip().replace('(', '').replace(')', '').split(',')
    stage_points = []
    for i in range(len(coords)//3):
        id = int(coords[i*3])
        x = float(coords[i*3+1]) + np.random.normal(0, 0.2) # add some noise to x coordinate
        y = float(coords[i*3+2]) + np.random.normal(0, 0.2) # add some noise to y coordinate
        stage_points.append((id,x, y))
    points_list.append(stage_points)

colors = colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', 'w', 'aliceblue', 'antiquewhite', 'aqua', 'aquamarine', 'azure', 'beige', 'bisque', 'blanchedalmond', 'blue', 'blueviolet', 'brown', 'burlywood', 'cadetblue', 'chartreuse', 'chocolate', 'coral', 'cornflowerblue', 'cornsilk', 'crimson', 'cyan', 'darkblue', 'darkcyan', 'darkgoldenrod', 'darkgray', 'darkgreen', 'darkkhaki', 'darkmagenta', 'darkolivegreen', 'darkorange', 'darkorchid', 'darkred', 'darksalmon', 'darkseagreen', 'darkslateblue', 'darkslategray', 'darkturquoise', 'darkviolet', 'deeppink', 'deepskyblue', 'dimgray', 'dodgerblue', 'firebrick', 'floralwhite', 'forestgreen', 'fuchsia', 'gainsboro', 'ghostwhite', 'gold', 'goldenrod', 'gray', 'green', 'greenyellow', 'honeydew', 'hotpink', 'indianred', 'indigo', 'ivory', 'khaki', 'lavender', 'lavenderblush', 'lawngreen', 'lemonchiffon', 'lightblue', 'lightcoral', 'lightcyan', 'lightgoldenrodyellow', 'lightgray', 'lightgreen', 'lightpink', 'lightsalmon', 'lightseagreen', 'lightskyblue', 'lightslategray', 'lightsteelblue', 'lightyellow', 'lime', 'limegreen', 'linen', 'magenta', 'maroon', 'mediumaquamarine', 'mediumblue', 'mediumorchid', 'mediumpurple', 'mediumseagreen', 'mediumslateblue', 'mediumspringgreen', 'mediumturquoise', 'mediumvioletred', 'midnightblue', 'mintcream', 'mistyrose', 'moccasin', 'navajowhite', 'navy', 'oldlace', 'olive', 'olivedrab', 'orange', 'orangered', 'orchid', 'palegoldenrod', 'palegreen', 'paleturquoise', 'palevioletred', 'papayawhip', 'peachpuff', 'peru', 'pink', 'plum', 'powderblue', 'purple', 'rebeccapurple', 'red', 'rosybrown', 'royalblue', 'saddlebrown', 'salmon', 'sandybrown', 'seagreen', 'seashell', 'sienna', 'silver', 'skyblue', 'slateblue', 'slategray', 'snow', 'springgreen', 'steelblue', 'tan', 'teal', 'thistle', 'tomato', 'turquoise', 'violet', 'wheat', 'white', 'whitesmoke', 'yellow', 'yellowgreen']* len(stage_points)# List of colors for each stage

fig, ax = plt.subplots()

# Define the update function for the animation
# def update(frame):
#     ax.clear()
#     # Get the points for the current stage
#     points = points_list[frame]
#     x, y = zip(*points)
#     # Plot the points
#     ax.scatter(x, y, color=colors[:len(points)], marker='o',s=200)
#     # Add axes lines and grid
#     ax.axhline(y=0, color='k')
#     ax.axvline(x=0, color='k')
#     ax.grid(True)
#     # Set the title for the current stage
#     ax.set_title(f"Stage {frame+1}")
def update(frame):
    ax.clear()
    # Get the points for the current stage
    points = points_list[frame]
    ids, x, y = zip(*points)
    # Plot the points
    ax.scatter(x, y, color=colors[:len(points)], marker='o', s=200)
    # Add IDs as text on top of each point
    for i in range(len(ids)):
        ax.text(x[i], y[i], str(ids[i]), fontsize=8, ha='center', va='bottom')
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





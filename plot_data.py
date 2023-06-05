import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import os

directory = 'animation_result'

# Create the directory if it doesn't exist
if not os.path.exists(directory):
    os.makedirs(directory)

with open("./build/output.txt", "r") as f:
    lines = f.readlines()

points_list = []

for line in lines:
    coords = line.strip().replace('(', '').replace(')', '').split(',')
    stage_points = []
    for i in range(len(coords)//4):
        id = int(coords[i*4])
        x = float(coords[i*4+1]) #+ np.random.normal(0, 0.2) # add some noise to x coordinate
        y = float(coords[i*4+2]) #+ np.random.normal(0, 0.2) # add some noise to y coordinate
        state= int(coords[i*4+3])
        stage_points.append((id,x, y, state))
    points_list.append(stage_points)

colors =  ['g', 'b', 'c', 'm', 'y', 'aqua', 'aquamarine', 'azure', 'bisque', 'blue', 'blueviolet',
                   'brown', 'burlywood', 'cadetblue', 'chartreuse', 'chocolate', 'coral', 'cornflowerblue',
                   'crimson', 'cyan', 'darkblue', 'darkcyan', 'darkgoldenrod', 'darkgreen',
                   'darkkhaki', 'darkmagenta', 'darkolivegreen', 'darkorange', 'darkorchid', 'darkred',
                   'darksalmon', 'darkseagreen', 'darkslateblue', 'darkslategray', 'darkturquoise', 'darkviolet',
                   'deeppink', 'deepskyblue', 'dimgray', 'dodgerblue', 'firebrick', 'forestgreen', 'fuchsia',
                   'gainsboro', 'gold', 'goldenrod', 'green', 'greenyellow', 'hotpink', 'indianred',
                   'indigo', 'khaki', 'lavender', 'lavenderblush', 'lawngreen', 'lemonchiffon', 'lightblue',
                   'lightcoral', 'lightcyan', 'lightgoldenrodyellow', 'lightgray', 'lightgreen', 'lightpink',
                   'lightsalmon', 'lightseagreen', 'lightskyblue', 'lightslategray', 'lightsteelblue',
                   'lightyellow', 'lime', 'limegreen', 'magenta', 'maroon', 'mediumaquamarine', 'mediumblue',
                   'mediumorchid', 'mediumpurple', 'mediumseagreen', 'mediumslateblue', 'mediumspringgreen',
                   'mediumturquoise', 'mediumvioletred', 'midnightblue', 'navajowhite', 'navy', 'olive',
                   'olivedrab', 'orange', 'orangered', 'orchid', 'palegoldenrod', 'palegreen', 'paleturquoise',
                   'palevioletred', 'papayawhip', 'peachpuff', 'peru', 'pink', 'plum', 'powderblue', 'purple',
                   'rebeccapurple', 'red', 'rosybrown', 'royalblue', 'saddlebrown', 'salmon', 'sandybrown',
                   'seagreen', 'seashell', 'sienna', 'silver', 'skyblue', 'slateblue', 'slategray', 'springgreen',
                   'steelblue', 'tan', 'teal', 'thistle', 'tomato', 'turquoise', 'violet', 'wheat', 'yellow',
                   'yellowgreen'] * len(stage_points)# List of colors for each stage

fig, ax = plt.subplots()
# save tagret
points = points_list[0]
ids, x, y, state = zip(*points)
state_2_points_targets = [(x[i], y[i]) for i in range(len(points)) if state[i] ==10]

def update(frame):
    ax.clear()
    # Get the points for the current stage
    points = points_list[frame]
    ids, x, y, state = zip(*points)
    
    for point in state_2_points_targets:
        ax.scatter(point[0], point[1], facecolor=(1.0, 0.0, 0.0, 0.5), marker='x', s=400)

    state_2_points_irrmovable = [(x[i], y[i]) for i in range(len(points)) if state[i] == 3]
    for point in state_2_points_irrmovable:
        ax.scatter(point[0], point[1],facecolor=(1, 0.0, 0.0, 0.5), marker='o', s=400)

    state_2_points_border = [(x[i], y[i]) for i in range(len(points)) if state[i] == 2]
    for point in state_2_points_border:
        ax.scatter(point[0], point[1], facecolor=(0.5, 0.5, 0.5, 0.7), marker='o', s=600)

    # drone is border and find tagrzt too 
    state_2_points_both = [(x[i], y[i]) for i in range(len(points)) if state[i] == 4]
    for point in state_2_points_both:
        ax.scatter(point[0], point[1], facecolor=(0.5, 0.5, 0.5, 0.5), marker='o', s=500)
    
    free_points = [(x[i], y[i]) for i in range(len(points)) if state[i] ==0 or state[i] ==1]
    for point,i in zip(free_points, range(0,len(colors))) :
        ax.scatter(point[0], point[1], color=colors[i], marker='o', s=300)

    if frame !=0: #  for everything other than the targets
        #ax.scatter(x, y, color=colors[:len(points)], marker='o', s=200)
        # Add IDs as text on top of each point
        for i in range(len(ids)):
            ax.text(x[i], y[i], str(ids[i]), fontsize=8, ha='center', va='bottom')
    
    # Add axes lines and grid
    ax.axhline(y=0, color='k')
    ax.axvline(x=0, color='k')
    ax.grid(True)
    # Set the title for the current stage
    ax.set_title(f"Stage {frame+1}")
    fig.savefig(os.path.join(directory, f'frame_{frame}.jpg'), dpi=300, format='jpg')


# Create the animation object
ani = FuncAnimation(fig, update, frames=len(points_list), interval=1000, repeat=False)

ani.save(os.path.join(directory, 'animation.mp4'), fps=1)
plt.show()







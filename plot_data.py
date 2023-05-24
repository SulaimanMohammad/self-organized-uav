import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

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

colors = colors = ['r', 'g', 'b', 'c', 'm', 'y', 'w', 'aliceblue', 'antiquewhite', 'aqua', 'aquamarine', 'azure', 'beige', 'bisque', 'blanchedalmond', 'blue', 'blueviolet', 'brown', 'burlywood', 'cadetblue', 'chartreuse', 'chocolate', 'coral', 'cornflowerblue', 'cornsilk', 'crimson', 'cyan', 'darkblue', 'darkcyan', 'darkgoldenrod', 'darkgray', 'darkgreen', 'darkkhaki', 'darkmagenta', 'darkolivegreen', 'darkorange', 'darkorchid', 'darkred', 'darksalmon', 'darkseagreen', 'darkslateblue', 'darkslategray', 'darkturquoise', 'darkviolet', 'deeppink', 'deepskyblue', 'dimgray', 'dodgerblue', 'firebrick', 'floralwhite', 'forestgreen', 'fuchsia', 'gainsboro', 'ghostwhite', 'gold', 'goldenrod', 'gray', 'green', 'greenyellow', 'honeydew', 'hotpink', 'indianred', 'indigo', 'ivory', 'khaki', 'lavender', 'lavenderblush', 'lawngreen', 'lemonchiffon', 'lightblue', 'lightcoral', 'lightcyan', 'lightgoldenrodyellow', 'lightgray', 'lightgreen', 'lightpink', 'lightsalmon', 'lightseagreen', 'lightskyblue', 'lightslategray', 'lightsteelblue', 'lightyellow', 'lime', 'limegreen', 'linen', 'magenta', 'maroon', 'mediumaquamarine', 'mediumblue', 'mediumorchid', 'mediumpurple', 'mediumseagreen', 'mediumslateblue', 'mediumspringgreen', 'mediumturquoise', 'mediumvioletred', 'midnightblue', 'mintcream', 'mistyrose', 'moccasin', 'navajowhite', 'navy', 'oldlace', 'olive', 'olivedrab', 'orange', 'orangered', 'orchid', 'palegoldenrod', 'palegreen', 'paleturquoise', 'palevioletred', 'papayawhip', 'peachpuff', 'peru', 'pink', 'plum', 'powderblue', 'purple', 'rebeccapurple', 'red', 'rosybrown', 'royalblue', 'saddlebrown', 'salmon', 'sandybrown', 'seagreen', 'seashell', 'sienna', 'silver', 'skyblue', 'slateblue', 'slategray', 'snow', 'springgreen', 'steelblue', 'tan', 'teal', 'thistle', 'tomato', 'turquoise', 'violet', 'wheat', 'white', 'whitesmoke', 'yellow', 'yellowgreen']* len(stage_points)# List of colors for each stage

fig, ax = plt.subplots()

def update(frame):
    ax.clear()
    # Get the points for the current stage
    points = points_list[frame]
    ids, x, y, state = zip(*points)

    state_2_points_target = [(x[i], y[i]) for i in range(len(points)) if state[i] == 3]
    for point in state_2_points_target:
        ax.scatter(point[0], point[1], facecolor=(1.0, 0.0, 0.0, 0.5), marker='x', s=400)

    state_2_points_border = [(x[i], y[i]) for i in range(len(points)) if state[i] == 2]
    for point in state_2_points_border:
        ax.scatter(point[0], point[1], facecolor=(0.03, 0.03, 0.03, 0.5), marker='o', s=400)

    # drone is border and find tagrzt too
    state_2_points_both = [(x[i], y[i]) for i in range(len(points)) if state[i] == 4]
    for point in state_2_points_both:
        ax.scatter(point[0], point[1], facecolor=(1.0, 0.0, 0.0, 0.5), marker='x', s=600)
        ax.scatter(point[0], point[1], facecolor=(0.03, 0.03, 0.03, 0.5), marker='o', s=400)


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
    fig.savefig(f'frame_{frame}.jpg', dpi=300, format='jpg')



# Create the animation object
ani = FuncAnimation(fig, update, frames=len(points_list), interval=1000, repeat=False)

ani.save('animation.mp4', fps=1)
plt.show()







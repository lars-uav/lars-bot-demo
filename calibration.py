'''
Reflectance Calibration Grid (https://custom-scripts.sentinel-hub.com/custom-scripts/sentinel-2/ndvi/)
NDVI Range          HTML Colour Code

NDVI < -0.5	        #0c0c0c             (0,0,0)
-0.5 < NDVI ≤ 0	    #eaeaea             (234,234,234)
-0.5 < NDVI ≤ 0	    #eaeaea             (204,198,130)
.1 < NDVI ≤ .2	    #91bf51             (145,191,81)
.2 < NDVI ≤ .3	    #70a33f             (112,163,63)
.3 < NDVI ≤ .4	    #4f892d             (79,137,45)
.4 < NDVI ≤ .5	    #306d1c             (48,109,28)
.5 < NDVI ≤ .6	    #0f540a             (15,84,10)
.6 < NDVI ≤ 1.0	    #004400             (0,68,0)
'''

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

color_codes = {
    1: "#000000",  # NDVI < -0.5
    2: "#EAEAEA",  # -0.5 < NDVI ≤ 0
    3: "#ccc682",  # 0 < NDVI ≤ 0.1
    4: "#91bf51",  # 0.1 < NDVI ≤ 0.2
    5: "#70a33f",  # 0.2 < NDVI ≤ 0.3
    6: "#4f892d",  # 0.3 < NDVI ≤ 0.4
    7: "#306d1c",  # 0.4 < NDVI ≤ 0.5
    8: "#0f540a",  # 0.5 < NDVI ≤ 0.6
    9: "#004400"   # 0.6 < NDVI ≤ 1.0
}

def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) / 255.0 for i in (0, 2, 4))

# Map NDVI value to color code
def ndvi_to_color(ndvi):
    if ndvi < -0.5:
        return color_codes[1]
    elif -0.5 <= ndvi <= 0:
        return color_codes[2]
    elif 0 < ndvi <= 0.1:
        return color_codes[3]
    elif 0.1 < ndvi <= 0.2:
        return color_codes[4]
    elif 0.2 < ndvi <= 0.3:
        return color_codes[5]
    elif 0.3 < ndvi <= 0.4:
        return color_codes[6]
    elif 0.4 < ndvi <= 0.5:
        return color_codes[7]
    elif 0.5 < ndvi <= 0.6:
        return color_codes[8]
    else:  # 0.6 < ndvi <= 1.0
        return color_codes[9]

# Make a Random Size and Colour Grid 
ndvi_grid = np.random.rand(3,3)
color_grid = np.array([[hex_to_rgb(ndvi_to_color(ndvi)) for ndvi in row] for row in ndvi_grid])

# Stressed Grid(s) (NDVI between [-0.5 and .1])
stressed_cells = [(i, j) for i in range(ndvi_grid.shape[0]) for j in range(ndvi_grid.shape[1]) if -0.5 <= ndvi_grid[i, j] <= 0.1]
if stressed_cells.__len__() == 0:
    print("No Stressed Grid Cells Found!")
else:
    print("Stressed grid cell locations (x, y):", stressed_cells)

# 1. Plot the Initial (Transparent) Grid
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, 3)
ax.set_ylim(0, 3)

ax.axis('off')

for i in range(4):
    ax.plot([0, 3], [i, i], color='black', linewidth=2)
    ax.plot([i, i], [0, 3], color='black', linewidth=2)
plt.savefig('./transparent_grid.png', bbox_inches='tight', pad_inches=0)
# plt.show()

# 2. Plot the Reflectance Grid and Save the Image
fig, ax = plt.subplots(figsize=(8, 8))
plt.imshow(color_grid)
plt.axis('off')
plt.savefig('./reflectance_grid.png', bbox_inches='tight', pad_inches=0, dpi=700)
# plt.show()


# 3. Plot the Reflectance Grid with Stressed Cells Highlighted
fig, ax = plt.subplots(figsize=(8, 8))
for i in range(ndvi_grid.shape[0]):
    for j in range(ndvi_grid.shape[1]):
        ax.add_patch(Rectangle((j, ndvi_grid.shape[0]-1-i), 1, 1, facecolor=color_grid[i, j]))
        
        if -0.5 <= ndvi_grid[i, j] <= 0.1:
            ax.add_patch(Rectangle((j, ndvi_grid.shape[0]-1-i), 1, 1, fill=False, edgecolor='red', linewidth=5))

ax.set_xlim(0, ndvi_grid.shape[1])
ax.set_ylim(0, ndvi_grid.shape[0])
ax.set_aspect('equal')

# Adjust text placement for the flipped coordinates
for i in range(ndvi_grid.shape[0]):
    for j in range(ndvi_grid.shape[1]):
        ax.text(j + 0.5, (ndvi_grid.shape[0]-1-i) + 0.5, 
                f'NDVI = {ndvi_grid[i, j]:.2f}', 
                ha='center', va='center', 
                color='white' if ndvi_grid[i, j] > 0.3 else 'black')

plt.axis('off')
plt.savefig('./reflectance_grid_stressed.png', bbox_inches='tight', pad_inches=0, dpi=700)
# plt.show()

# 3. Plot the Reflectance Grid with Stressed Cells Highlighted
fig, ax = plt.subplots(figsize=(8, 8), dpi=100)  # Specify DPI for consistent rendering

for i in range(ndvi_grid.shape[0]):
    for j in range(ndvi_grid.shape[1]):
        ax.add_patch(Rectangle((j, ndvi_grid.shape[0]-1-i), 1, 1, facecolor=color_grid[i, j], zorder=1))

for i in range(ndvi_grid.shape[0]):
    for j in range(ndvi_grid.shape[1]):
        if -0.5 <= ndvi_grid[i, j] <= 0.1:
            ax.add_patch(Rectangle((j, ndvi_grid.shape[0]-1-i), 1, 1, fill=False, edgecolor='red', linewidth=3, zorder=2, snap=True))

ax.set_xlim(0, ndvi_grid.shape[1])
ax.set_ylim(0, ndvi_grid.shape[0])
ax.set_aspect('equal')

# Add text labels
for i in range(ndvi_grid.shape[0]):
    for j in range(ndvi_grid.shape[1]):
        ax.text(j + 0.5, (ndvi_grid.shape[0]-1-i) + 0.5, f'NDVI = {ndvi_grid[i, j]:.2f}', ha='center', va='center', color='white' if ndvi_grid[i, j] > 0.3 else 'black', zorder=3)  # Ensure text is on top

plt.axis('off')
# Save with higher DPI for better quality
plt.savefig('reflectance_grid_stressed.png', bbox_inches='tight', pad_inches=0, dpi=700)
# plt.show()
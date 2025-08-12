import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image


# Open the image file
img = Image.open('1000018316.jpg')

# Create figure and axes
fig, ax = plt.subplots()

# Display the image
ax.imshow(img)
# Define the bounding boxes (x, y, width, height) for each object
# The coordinates are estimated based on the image
bounding_boxes = [
    (550, 180, 520, 330),  # Television
    (540, 510, 560, 140),  # TV stand
    (800, 520, 150, 100),  # Laptop on the TV stand
    (600, 650, 400, 40),   # Various items on the TV stand
    (680, 520, 30, 100),   # Water bottle on the table
    (850, 520, 40, 100),   # Remote control on the table
    (100, 620, 380, 150),  # Coffee table
    (30, 580, 250, 300),   # Sofa
    (10, 770, 130, 240),   # Vacuum cleaner
    (590, 770, 60, 40),    # White router or modem on the floor
    (50, 770, 60, 30),     # Pair of slippers
    (170, 620, 60, 90),    # Backpack on the coffee table
    (260, 620, 80, 20),    # Charger on the coffee table
    (10, 770, 50, 30),     # Headphones near the vacuum cleaner
    (100, 100, 40, 250),   # Black floor lamp
    (1050, 770, 160, 130), # Laptop on the floor
    (1100, 900, 100, 10),  # Laptop charger on the floor
    (900, 0, 250, 730),    # Window with a partially open curtain
    (1200, 300, 30, 70)    # Wall-mounted device or switch
]

# Create a rectangle for each bounding box
for bbox in bounding_boxes:
    rect = patches.Rectangle((bbox[0], bbox[1]), bbox[2], bbox[3], linewidth=2, edgecolor='r', facecolor='none')
    ax.add_patch(rect)

# Remove the axis
plt.axis('off')

# Show the image
plt.show()
import cv2
import matplotlib.pyplot as plt
import numpy as np

# Initialize global variables for storing the points
point1 = None
point2 = None
point3 = None
dragging_point = None

# Function to calculate the angle between two lines
def calculate_angle_between_lines(line1, line2):
    # Calculate the angle between two lines
    angle1 = np.arctan2(line1[1][1] - line1[0][1], line1[1][0] - line1[0][0])
    angle2 = np.arctan2(line2[1][1] - line2[0][1], line2[1][0] - line2[0][0])
    angle_subtract = (angle2 - angle1) % (2 * np.pi)
    angle_subtract_norm = angle_subtract % (2 * np.pi)
    # angle_deg = np.degrees(angle_subtract_norm)
    angle_deg = 180 - np.degrees(angle_subtract_norm)
    return angle_deg

# Function to draw points and the angle on the image
def draw_points_and_angle(img, pt1, pt2, pt3):
    img_copy = img.copy()
    if pt1 is not None and pt2 is not None and pt3 is not None:
        # Draw the points and lines between them
        cv2.circle(img_copy, pt1, 5, (0, 255, 0), -1)
        cv2.circle(img_copy, pt2, 5, (0, 255, 0), -1)
        cv2.circle(img_copy, pt3, 5, (0, 255, 0), -1)
        cv2.line(img_copy, pt1, pt2, (255, 0, 0), 2)
        cv2.line(img_copy, pt2, pt3, (255, 0, 0), 2)
        
        # Calculate and display the angle
        angle = calculate_angle_between_lines((pt1, pt2), (pt2, pt3))
        midpoint = ((pt2[0] + pt3[0]) // 2, (pt2[1] + pt3[1]) // 2)
        cv2.putText(img_copy, f"{angle:.2f} deg", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 
                    1, (255, 0, 0), 2, cv2.LINE_AA)
    return img_copy

# Mouse callback function for handling point selection and dragging
def mouse_callback(event, x, y, flags, param):
    global point1, point2, point3, dragging_point

    if event == cv2.EVENT_LBUTTONDOWN:
        if point1 is None:
            point1 = (x, y)
        elif point2 is None:
            point2 = (x, y)
        elif point3 is None:
            point3 = (x, y)
        else:
            # Check if we clicked near one of the points to drag it
            if np.linalg.norm(np.array(point1) - np.array((x, y))) < 10:
                dragging_point = 1
                print('Chose point 1')
            elif np.linalg.norm(np.array(point2) - np.array((x, y))) < 10:
                dragging_point = 2
                print('Chose point 2')
            elif np.linalg.norm(np.array(point3) - np.array((x, y))) < 10:
                dragging_point = 3
                print('Chose point 3')

    elif event == cv2.EVENT_MOUSEMOVE and dragging_point is not None:
        if dragging_point == 1:
            point1 = (x, y)
        elif dragging_point == 2:
            point2 = (x, y)
        elif dragging_point == 3:
            point3 = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        dragging_point = None

# Main function to load image and set up interaction
def main(image_path):
    global point1, point2, point3

    # Load the image
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Unable to load image.")
        return
    
    # Resize the image to half size
    img = cv2.resize(img, (img.shape[1] // 4, img.shape[0] // 4))

    # Create a named window and set the mouse callback
    cv2.namedWindow("Image with Points")
    cv2.setMouseCallback("Image with Points", mouse_callback)

    while True:
        # Draw points and angle on the image
        img_with_points = draw_points_and_angle(img, point1, point2, point3)

        # Show the image
        cv2.imshow("Image with Points", img_with_points)

        # Exit if the user presses the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close all windows when done
    cv2.destroyAllWindows()

# Run the app
if __name__ == "__main__":
    # Example: Provide the path to your image
    image_path = '/home/ben/Pictures/Screenshot from 2024-10-04 17-48-48.png'
    image_path = '/home/ben/Pictures/leahy_test_left.png'
    main(image_path)

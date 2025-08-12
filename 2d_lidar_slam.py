import numpy as np
import matplotlib.pyplot as plt

class LidarSlam:
    def __init__(self):
        self.map_points = []
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.covariance = np.eye(3)

    def predict(self, control):
        # Predict the robot's pose using the control input
        self.robot_pose += control
        self.covariance += np.array([[0.1, 0.0, 0.0],
                                       [0.0, 0.1, 0.0],
                                       [0.0, 0.0, 0.1]])

    def update(self, lidar_data):
        # Update the map points using the ICP algorithm
        map_points_new = []
        for point in lidar_data:
            range, angle = point
            x, y = range * np.cos(angle), range * np.sin(angle)
            map_points_new.append((x, y))
        self.map_points = map_points_new

        # Update the robot's pose using the EKF
        innovation = np.array([0.0, 0.0, 0.0])
        for point in map_points_new:
            innovation += np.array([point[0] - self.robot_pose[0],
                                   point[1] - self.robot_pose[1],
                                   np.arctan2(point[1] - self.robot_pose[1],
                                              point[0] - self.robot_pose[0]) - self.robot_pose[2]])
        innovation /= len(map_points_new)
        self.robot_pose += innovation
        self.covariance -= np.dot(self.covariance, np.dot(np.array([[1.0, 0.0, 0.0],
                                                                  [0.0, 1.0, 0.0],
                                                                  [0.0, 0.0, 1.0]]),
                                        self.covariance))

    def get_map(self):
        return self.map_points

    def get_pose(self):
        return self.robot_pose

# Create a LidarSlam object
slam = LidarSlam()

# Simulate the LiDAR data
for i in range(10):
    lidar_data = [(0.5, 0.0), (1.2, 30.0), (2.1, 60.0), (1.5, 90.0), (0.8, 120.0), (1.0, 150.0), (1.8, 180.0), (2.5, 210.0), (2.0, 240.0), (1.2, 270.0), (0.5, 300.0), (0.2, 330.0)]
    control = np.array([0.1, 0.0, 0.0])  # control input (dx, dy, dtheta)
    slam.predict(control)
    slam.update(lidar_data)

# Plot the map and robot pose
plt.scatter([point[0] for point in slam.get_map()], [point[1] for point in slam.get_map()])
plt.quiver(slam.get_pose()[0], slam.get_pose()[1], np.cos(slam.get_pose()[2]), np.sin(slam.get_pose()[2]))
plt.show()
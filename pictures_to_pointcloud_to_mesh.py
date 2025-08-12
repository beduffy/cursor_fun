import cv2
import numpy as np
from pathlib import Path
import glob
import open3d as o3d


# TODO get images from online and run this
# TODO run on my own images

def extract_features(images):
    """Extract SIFT features from all images."""
    sift = cv2.SIFT_create()
    features = []
    
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = sift.detectAndCompute(gray, None)
        features.append((keypoints, descriptors))
    
    return features


def match_features(features):
    """Match features between image pairs."""
    matcher = cv2.BFMatcher()
    matches_list = []
    
    for i in range(len(features)-1):
        for j in range(i+1, len(features)):
            matches = matcher.knnMatch(features[i][1], features[j][1], k=2)
            # Apply Lowe's ratio test
            good_matches = []
            for m, n in matches:
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
            matches_list.append((i, j, good_matches))
    
    return matches_list


def estimate_pose(matches_list, features, K):
    """Estimate camera poses and triangulate 3D points."""
    points_3d = []
    camera_poses = []
    
    for i, j, matches in matches_list:
        # Get matching points
        pts1 = np.float32([features[i][0][m.queryIdx].pt for m in matches])
        pts2 = np.float32([features[j][0][m.trainIdx].pt for m in matches])
        
        # Essential matrix estimation
        E, mask = cv2.findEssentialMat(pts1, pts2, K)
        
        # Recover pose
        _, R, t, mask = cv2.recoverPose(E, pts1, pts2, K)
        
        # Triangulate points
        P1 = np.dot(K, np.hstack((np.eye(3), np.zeros((3,1)))))
        P2 = np.dot(K, np.hstack((R, t)))
        
        points_4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)
        points_3d.append(points_4d[:3] / points_4d[3])
        
        camera_poses.append((R, t))
    
    return np.hstack(points_3d).T, camera_poses


def create_mesh(points_3d):
    """Create a mesh from 3D points using Poisson reconstruction."""
    # Create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_3d)
    
    # Estimate normals
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    
    # Poisson reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
    
    # Clean up the mesh
    vertices_to_remove = densities < np.quantile(densities, 0.1)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    return mesh


def save_as_stl(mesh, output_path):
    """Save the mesh as STL file."""
    o3d.io.write_triangle_mesh(output_path, mesh, write_ascii=True)


def main():
    # Load images
    image_paths = glob.glob('path/to/your/images/*.jpg')
    images = [cv2.imread(str(path)) for path in image_paths]
    
    # Camera intrinsic matrix (you'll need to calibrate your camera)
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])
    
    # Extract and match features
    features = extract_features(images)
    matches_list = match_features(features)
    
    # Estimate poses and triangulate points
    points_3d, camera_poses = estimate_pose(matches_list, features, K)
    
    # Create mesh
    mesh = create_mesh(points_3d)
    
    # Save as STL
    save_as_stl(mesh, 'output.stl')


if __name__ == '__main__':
    main()



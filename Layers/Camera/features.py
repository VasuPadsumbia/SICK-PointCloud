import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
import cv2

class PointCloudProcessor:
    def __init__(self, wCoordinates, eps=0.5, min_samples=50):
        """
        Initialize the PointCloudProcessor with point cloud data and DBSCAN parameters.

        Args:
            wCoordinates: List of points, each in [X, Y, Z, R, G, B, I] format.
            eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
            min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
        """
        self.wCoordinates = wCoordinates
        self.eps = eps
        self.min_samples = min_samples
        self.points = np.array([[p[0], p[1], p[2]] for p in self.wCoordinates])  # Extract XYZ only
        self.colors = np.array([[p[3], p[4], p[5]] for p in self.wCoordinates]) / 255  # Extract RGB and normalize


    def filter_points_by_depth(self, depth_range):
        """
        Filter points based on their depth.

        Args:
            depth_range: Tuple (min_depth, max_depth) specifying the depth range.

        Returns:
            Filtered points within the specified depth range.
        """
        min_depth, max_depth = depth_range
        mask = (self.points[:, 2] >= min_depth) & (self.points[:, 2] <= max_depth)
        filtered_points = self.points[mask]
        filtered_colors = self.colors[mask]
        print(f"Filtered points count: {filtered_points.shape[0]}")
        return filtered_points, filtered_colors

    def project_to_2d(self, points):
        """
        Project 3D points to a 2D plane (XY plane).

        Args:
            points: NumPy array of shape (N, 3) containing the point cloud.

        Returns:
            Projected 2D points of shape (N, 2).
        """
        projected_points = points[:, :2]
        print(f"Projected points count: {projected_points.shape[0]}")
        return projected_points

    def find_contour(self, points, eps=15, min_samples=10):
        """
        Find the contour of the 2D points using OpenCV.

        Args:
            points_2d: NumPy array of shape (N, 2) containing the 2D points.

        Returns:
            Contour points of the cardboard box.
        """
        if points.shape[0] == 0:
            print("No points to find contour.")
            return np.array([])

        # Project points to 2D plane for contour detection
        points_2d = points[:, :2].astype(np.float32)
        # Apply DBSCAN clustering to group nearby points
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points_2d)
        labels = clustering.labels_

        unique_labels = set(labels)
        largest_cluster_points = np.array([])

        for label in unique_labels:
            if label == -1:  # Ignore noise points
                continue

            cluster_points = points[labels == label]
            if largest_cluster_points.size == 0 or cluster_points.shape[0] > largest_cluster_points.shape[0]:
                largest_cluster_points = cluster_points

        if largest_cluster_points.size == 0:
            print("No valid clusters found.")
            return np.array([]), None
        
        # Compute convex hull for the largest cluster
        hull_indices = cv2.convexHull(largest_cluster_points[:, :2].astype(np.float32), returnPoints=False)
        hull = largest_cluster_points[hull_indices.flatten()]
        #hull_indices = cv2.convexHull(points_2d, returnPoints=False)
        #hull = points[hull_indices[:, 0]]
        #print(f"Contour points count: {hull.shape[0]}")
        # Calculate the centroid of the contour
        centroid = np.mean(hull, axis=0)
        #print(f"Centroid of the contour: {centroid}")
        return hull, centroid

    def extract_objects_and_centers(self):
        """
        Extract objects and their contours from the point cloud.

        Returns:
            List of contours for each identified object.
        """
        
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(self.points)
        labels = clustering.labels_

        # Extract contours for each cluster
        contours = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue  # Skip noise points
            cluster_points = self.points[labels == label]
            # Find contours using OpenCV
            hull = self.find_contour(cluster_points)
            if hull.shape[0] > 0:
                contours.append(hull)
        return contours
    
    def get_color(self, point):
        """
        Get the color of the specified point.

        Args:
            point: NumPy array of shape (3,) containing the point coordinates.

        Returns:
            RGB color of the point.
        """
        # Find the closest point in the point cloud
        distances = np.linalg.norm(self.points - point, axis=1)
        closest_idx = np.argmin(distances)
        color = self.colors[closest_idx]
        return color
    
    def visualize_point_cloud_with_contour(self, contour, centroid):
        """
        Visualize the point cloud with the contour using Open3D.

        Args:
            contour: NumPy array of shape (M, 2) containing the contour points.
        """
        # Create Open3D point cloud object
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(self.points)
        o3d_pc.colors = o3d.utility.Vector3dVector(self.colors)

        if contour.shape[0] == 0:
            print("No contour to visualize.")
            o3d.visualization.draw_geometries([o3d_pc], window_name="Point Cloud")
            return

        # Create Open3D line set for the contour
        lines = [[i, (i + 1) % len(contour)] for i in range(len(contour))]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(contour),
            lines=o3d.utility.Vector2iVector(lines),
        )
        
        # Set the color of the contour to blue
        line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines))])

        # Create a point cloud for the centroid
        centroid_pc = o3d.geometry.PointCloud()
        centroid_pc.points = o3d.utility.Vector3dVector([centroid])
        centroid_pc.colors = o3d.utility.Vector3dVector([[1,0,0]])  # Red color for the centroid

        # Find the closest point in the point cloud to the centroid to get the color
        #centroid_color = self.get_color(centroid)
        #print(f"Centroid color: {centroid_color}")
        
        # Create a line for X=0 and Y=0 (vertical line along Z-axis)
        min_z = np.min(self.points[:, 2])
        max_z = np.max(self.points[:, 2])
        z_line_points = np.array([[0, 0, min_z], [0, 0, max_z]])
        z_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(z_line_points),
            lines=o3d.utility.Vector2iVector([[0, 1]])
        )
        z_line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0]])  # Green color for the Z-axis line

        # Debugging: Print the contour points and lines
        #print("Contour points:")
        #print(contour)
        #print("Lines:")
        #print(lines)
        # Visualize the point cloud and contour
        #o3d.visualization.draw_geometries([o3d_pc, line_set, centroid_pc, z_line_set], window_name="Point Cloud with Contour")

        # Visualize the point cloud and contour with cursor info
        #vis = o3d.visualization.VisualizerWithEditing()
        #vis = o3d.visualization.VisualizerWithKeyCallback()
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Point Cloud with Contour and Cursor Info")
        vis.add_geometry(line_set)
        vis.add_geometry(centroid_pc)
        vis.add_geometry(z_line_set)
        vis.add_geometry(o3d_pc)

        #vis.register_key_callback(ord("P"), self.pick_points)
        vis.run()
        vis.destroy_window()

        # Get picked points
        picked_points = vis.get_picked_points()
        for idx in picked_points:
            print(f"Picked point: {self.points[idx]}")
    def pick_points(self, vis):
        """
        Callback function to print the coordinates of the selected point.
        """
        print("Press 'Shift + Left Mouse Click' to pick points.")
        picked_points = vis.get_picked_points()
        if picked_points:
            for idx in picked_points:
                point = self.points[idx]  # Retrieve full data from the point cloud
                x, y, z = point  # Extract individual values
                # Print the complete point data
                print(f"Picked Point - X: {x}, Y: {y}, Z: {z}")
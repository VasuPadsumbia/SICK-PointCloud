import numpy as np
from Layers.Circles  import fit_circle_3d, generate_circle_points
import open3d as o3d
import cv2
from base.python.Streaming import Data
from Layers.Camera.features import *

def process_and_visualize_point_cloud(wCoordinates, depth_range):
    """
    Process the point cloud to find and visualize the contour of the cardboard box.

    Args:
        wCoordinates: List of points, each in [X, Y, Z, R, G, B, I] format.
        depth_range: Tuple (min_depth, max_depth) specifying the depth range.
    """
    processor = PointCloudProcessor(wCoordinates)
    filtered_points, filtered_colors = processor.filter_points_by_depth(depth_range)
    #points_2d = processor.project_to_2d(filtered_points)
    contour, centroid = processor.find_contour(filtered_points)
    print("Contour: ", contour)
    processor.visualize_point_cloud_with_contour(contour, centroid)

def visualize_point_cloud_with_open3d(point_cloud, type):
    """
    Visualizes a 3D point cloud using Open3D.

    Args:
        point_cloud (np.ndarray): H x W x 3 array containing point cloud data (X, Y, Z).
    """
    # Visualization with picking callback
    def pick_points(vis):
        """
        Callback function to print the coordinates of the selected point.
        """
        print("Press 'Shift + Left Mouse Click' to pick points.")
        picked_points = vis.get_picked_points()
        if picked_points:
            for idx in picked_points:
                point = points[idx]  # Retrieve full data from the point cloud
                x, y, z, r, g, b, intensity = point  # Extract individual values
                # Print the complete point data
                print(f"Picked Point - X: {x}, Y: {y}, Z: {z}, R: {r}, G: {g}, B: {b}, Intensity: {intensity}")
                
    if type == "pcd":
        # Reshape point cloud to N x 3
        points = point_cloud.reshape(-1, 3)

        # Filter out invalid points (NaNs)
        valid_points = points[~np.isnan(points).any(axis=1)]

        if valid_points.size == 0:
            print("No valid points to visualize.")
            return

        # Create Open3D point cloud object
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(valid_points)

        # Assign colors based on depth (Z-axis)
        z_vals = valid_points[:, 2]
        colors = np.zeros_like(valid_points)
        colors[:, 0] = np.interp(z_vals, (z_vals.min(), z_vals.max()), (0, 1))  # Red gradient
        colors[:, 1] = 1 - colors[:, 0]  # Green gradient
        o3d_pc.colors = o3d.utility.Vector3dVector(colors)

        # Visualize the point cloud
        o3d.visualization.draw_geometries([o3d_pc], window_name="3D Point Cloud")
    
    elif type == "ply":
        # Convert to NumPy array
        # Reshape to N x 7
        # Extract XYZ coordinates and RGB colors
        # Debugging: Print the shape and type of point_cloud
        #print("point_cloud type:",(point_cloud.__class__.__name__))
        #print("point_cloud length:", len(point_cloud))
        if len(point_cloud) > 0:
            print("point_cloud[0] type:", point_cloud[0].__class__.__name__)
            print("point_cloud[0] length:", len(point_cloud[0]))

        # Ensure point_cloud is a list of lists or a 2D array
        if isinstance(point_cloud, list):
            point_cloud = [np.array(p) for p in point_cloud]
            point_cloud = np.array(point_cloud)

        # Convert to NumPy array
        try:
            points = np.array(point_cloud).reshape(-1, 7)[:, :3]  # Extract X, Y, Z only
        except ValueError as e:
            print(f"Error reshaping point_cloud: {e}")
            return
        
        colors = np.array(point_cloud).reshape(-1, 7)[:, 3:6] / 255.0  # Normalize RGB to [0, 1]
        # Filter out invalid points (e.g., NaNs)
        valid_mask = ~np.isnan(points).any(axis=1)
        valid_xyz = points[valid_mask]
        valid_rgb = colors[valid_mask]

        # Fit the circle and generate its points
        #circle_center, circle_radius, normal, basis_x, basis_y = fit_circle_3d(valid_xyz)
        #circle_points = generate_circle_points(circle_center, circle_radius, normal, basis_x, basis_y)
        #print("Circle Center:", circle_center)
        #print("Circle Radius:", circle_radius)
        #print("Normal Vector:", normal)
        #print("Circle points shape:", circle_points.shape)
        ## Create Open3D point cloud for the detected circle
        #circle_pc = o3d.geometry.PointCloud()
        #circle_pc.points = o3d.utility.Vector3dVector(circle_points)
        #circle_pc.paint_uniform_color([1, 0, 0])  # Red color for the circle

        # Create Open3D point cloud
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(valid_xyz)
        o3d_pc.colors = o3d.utility.Vector3dVector(valid_rgb)

        #geometry_list = [o3d_pc]
        #
        ## Add contours to the visualization
        #if contours is not None:
        #    for contour in contours:
        #        print("Original contour shape:", np.array(contour).shape)  # Debugging: Print the shape of the contour
        #        if len(contour) % 3 != 0:
        #            print("Skipping contour due to incorrect shape:", np.array(contour).shape)
        #            continue  # Skip contours that cannot be reshaped into (-1, 3)
        #        contour = np.array(contour).reshape(-1, 3)  # Ensure contour is a 2D array with 3 columns
        #        contour_list = contour.tolist()  # Convert NumPy array to list
        #        lines = [[i, (i + 1) % len(contour_list)] for i in range(len(contour_list))]
        #        line_set = o3d.geometry.LineSet(
        #            points=o3d.utility.Vector3dVector(contour_list),
        #            lines=o3d.utility.Vector2iVector(lines),
        #        )
        #        geometry_list.append(line_set)
            
        # Visualize the point cloud
        # Add the picking callback to the visualization window
        # Visualization with increased point size
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="3D Point Cloud with Centers")
        vis.add_geometry(o3d_pc)  # Main point cloud
        #for geometry in geometry_list:
        #    vis.add_geometry(geometry)
        
        #vis.add_geometry(circle_pc)
        vis.run()  # This will allow the user to pick points interactively
        #vis.destroy_window()
        # Get picked points
        picked_points = vis.get_picked_points()
        for idx in picked_points:
            print(f"Picked point: {valid_xyz[idx]}")
        #o3d.visualization.draw_geometries(geometry_list, window_name="3D Point Cloud with Centers")

def plotFrameRealTime(visionary_type: str, data: Data):
    """
    Processes and plots the frame data in real-time from the Visionary camera.
    """
    height = data.cameraParams.height
    width = data.cameraParams.width

    dFrameHeight = 512
    dFrameWidth = 640
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            # Clear the frame and add text for pixel info
            temp_frame = param.copy()
            text = f"X: {x}, Y: {y}, Z: {data.depthmap.distance}, Value: {param[y, x]}"
            cv2.putText(temp_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Z-Map", temp_frame)

    if visionary_type == 'Visionary-S':
        # RGBA to BGRA
        rgba_data = np.uint32(np.reshape(data.depthmap.intensity, (dFrameHeight, dFrameWidth)))
        rgba_data = np.frombuffer(rgba_data, np.uint8).reshape((dFrameHeight, dFrameWidth, 4))
        bgra_data = cv2.cvtColor(rgba_data, cv2.COLOR_RGBA2BGRA)
        #bgra_data = np.flip(np.flip(bgra_data, axis=0), axis=1)  # Flip vertically and horizontally
        #bgra_roi = bgra_data[y_start:y_start + roi_height, x_start:x_start + roi_width]
        bgra_display = bgra_data #bgra_roi

        # Z-Map
        zmap_data = np.array(data.depthmap.distance, dtype=np.uint16).reshape((height, width))
        #zmap_roi = zmap_data[y_start:y_start + roi_height, x_start:x_start + roi_width]
        zmap_display = cv2.normalize(zmap_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        #zmap_display = np.flip(np.flip(zmap_display, axis=0), axis=1)

        # State
        state_data = np.array(data.depthmap.confidence, dtype=np.uint16).reshape((height, width))
        #state_roi = state_data[y_start:y_start + roi_height, x_start:x_start + roi_width]
        state_display = cv2.normalize(state_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        #state_display = np.flip(np.flip(state_display, axis=0), axis=1)

        # Display
        cv2.namedWindow("Z-Map")
        cv2.setMouseCallback("Z-Map", mouse_callback, param=zmap_display)
        cv2.imshow("BGRA Map", bgra_display)
        cv2.imshow("Z-Map", zmap_display)
        cv2.imshow("State Map", state_display)
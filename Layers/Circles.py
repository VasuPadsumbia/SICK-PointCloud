import numpy as np
from scipy.optimize import leastsq

def fit_circle_3d(points):
    """
    Fits a circle to a set of 3D points.
    :param points: List of [X, Y, Z] coordinates
    :return: (circle_center, circle_radius, normal_vector)
    """
    # Convert to NumPy array
    points = np.array(points)[:, :3]  # Extract XYZ only

    # Step 1: Compute the best-fit plane using PCA
    centroid = np.mean(points, axis=0)  # Compute centroid
    cov_matrix = np.cov(points.T)  # Covariance matrix
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)  # Eigen decomposition
    
    normal = eigenvectors[:, 0]  # Smallest eigenvector is normal to the plane

    # Step 2: Project points onto the plane
    projected_points = []
    for p in points:
        vector = p - centroid
        distance_to_plane = np.dot(vector, normal)
        projected_point = p - distance_to_plane * normal  # Projection formula
        projected_points.append(projected_point)
    
    projected_points = np.array(projected_points)
    
    # Step 3: Convert to 2D (Local Plane Coordinates)
    basis_x = eigenvectors[:, 1]  # Second eigenvector (defines X-axis on the plane)
    basis_y = eigenvectors[:, 2]  # Third eigenvector (defines Y-axis)
    
    points_2d = np.array([[np.dot(p - centroid, basis_x), np.dot(p - centroid, basis_y)] for p in projected_points])

    # Step 4: Fit a circle in 2D
    def circle_equation(c, x, y):
        """ Equation of a circle: (x - xc)^2 + (y - yc)^2 = r^2 """
        xc, yc, r = c
        return np.sqrt((x - xc)**2 + (y - yc)**2) - r
    
    x_data, y_data = points_2d[:, 0], points_2d[:, 1]
    center_estimate = np.mean(points_2d, axis=0)  # Initial guess
    radius_estimate = np.mean(np.sqrt((x_data - center_estimate[0])**2 + (y_data - center_estimate[1])**2))
    
    best_fit, _ = leastsq(circle_equation, [center_estimate[0], center_estimate[1], radius_estimate], args=(x_data, y_data))
    xc, yc, radius = best_fit

    # Convert 2D center back to 3D
    circle_center_3d = centroid + xc * basis_x + yc * basis_y

    return circle_center_3d, radius, normal, basis_x, basis_y

def generate_circle_points(center, radius, normal, basis_x, basis_y, num_points=100):
    """
    Generates points forming a circle in 3D space.
    :param center: Center of the circle in 3D
    :param radius: Radius of the circle
    :param normal: Normal vector of the circle's plane
    :param basis_x: X-axis direction in the plane
    :param basis_y: Y-axis direction in the plane
    :param num_points: Number of points to generate
    :return: List of 3D circle points
    """
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_points = [center + radius * (np.cos(theta) * basis_x + np.sin(theta) * basis_y) for theta in angles]
    return np.array(circle_points)
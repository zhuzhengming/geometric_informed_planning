# %%
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from scipy.spatial import cKDTree

# Function to generate a complex point cloud
def generate_complex_point_cloud():
    np.random.seed(42)
    cluster1 = np.random.randn(50, 3) + np.array([5, 5, 5])
    cluster2 = np.random.randn(50, 3) + np.array([10, 10, 10])
    cluster3 = np.random.randn(50, 3) + np.array([15, 5, 10])
    return np.vstack((cluster1, cluster2, cluster3))

# Function to determine visible points from the drone's position
def compute_visible_points(point_cloud, drone_position, angular_resolution=10, max_distance=100):
    tree = cKDTree(point_cloud)
    visible_points = []
    # Convert angular resolution from degrees to radians
    angular_resolution_rad = np.radians(angular_resolution)
    # Create a dictionary to store the closest point in each angular bin
    bins = {}
    for point in point_cloud:
        direction = point - drone_position
        r = np.linalg.norm(direction)
        theta = np.arctan2(direction[1], direction[0])  # azimuth angle
        phi = np.arccos(direction[2] / r)  # elevation angle
        # Compute the bin index
        theta_bin = int(theta // angular_resolution_rad)
        phi_bin = int(phi // angular_resolution_rad)
        # Use the bin index as a key
        bin_key = (theta_bin, phi_bin)
        # If the bin is not occupied, or the current point is closer, update the bin
        if bin_key not in bins or bins[bin_key][0] > r:
            bins[bin_key] = (r, point)
    # Extract the points from the bins
    for bin_key in bins:
        visible_points.append(bins[bin_key][1])
    return np.array(visible_points)

# Generate a complex point cloud
point_cloud = generate_complex_point_cloud()

# Drone's current position
drone_position = np.array([2, 3, 0])

# Safety distance for collision avoidance
# Compute the visible points from the drone's position
point_cloud = compute_visible_points(point_cloud, drone_position)

def extract_2Dplane_PCA(point_cloud: np.array) -> tuple:
    pc_center = np.mean(point_cloud, axis=0)
    point_cloud_centered = point_cloud - pc_center
    cov_matrix = np.cov(point_cloud_centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    normal_vector = eigenvectors[:, 0]
    major_direction = eigenvectors[:, -1]  # Eigenvector with the largest eigenvalue
    point_on_plane = np.mean(point_cloud, axis=0)
    return normal_vector, major_direction, pc_center, point_on_plane

def project_point_onto_plane(point, normal_vector, point_on_plane):
    normal_vector = normal_vector / np.linalg.norm(normal_vector)
    distance = np.dot(point - point_on_plane, normal_vector)
    projected_point = point - distance * normal_vector
    return projected_point

def find_safe_direction(major_direction, pc_center, normal_vector, point_cloud, drone_position, safety_distance=0.5):
    # Project the drone's position onto the plane
    projected_drone_pos = project_point_onto_plane(drone_position, normal_vector, pc_center)
    # Compute a direction parallel to the plane
    if major_direction.dot(pc_center-drone_position) > 0:
        direction_vector = major_direction
    else:
        direction_vector = -major_direction
    # Normalize the direction vector
    direction_vector /= np.linalg.norm(direction_vector)
    # Check for collision along the direction
    for point in point_cloud:
        point_proj = project_point_onto_plane(point, normal_vector, projected_drone_pos)
        if np.linalg.norm(point_proj - projected_drone_pos) < safety_distance:
            direction_vector = -direction_vector
            break
    return direction_vector

# Extract the plane and major direction using PCA
normal_vector, major_direction,pc_center, point_on_plane = extract_2Dplane_PCA(point_cloud)

# Find the safe direction for the drone to move
safe_direction = find_safe_direction(major_direction, pc_center,normal_vector, point_cloud, drone_position, safety_distance=0.5)

# Create meshgrid for plotting the plane
xx, yy = np.meshgrid(np.linspace(min(point_cloud[:, 0]), max(point_cloud[:, 0]), 10),
                     np.linspace(min(point_cloud[:, 1]), max(point_cloud[:, 1]), 10))
zz = (-normal_vector[0] * xx - normal_vector[1] * yy - np.dot(normal_vector, -point_on_plane)) * 1. / normal_vector[2]

# Plotting
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot the original point cloud
point_cloud_handle = ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], c='b', marker='o', label='Point Cloud')

# Plot the fitted plane
plane_handle = ax.plot_surface(xx, yy, zz, alpha=0.5, rstride=1, cstride=1, color='r')

# Plot the drone's position
drone_position_handle = ax.scatter(drone_position[0], drone_position[1], drone_position[2], c='g', marker='^', s=100, label='Drone Position')

# Plot the safe direction vector
ax.quiver(drone_position[0], drone_position[1], drone_position[2],
          safe_direction[0], safe_direction[1], safe_direction[2], color='k', length=1.0, normalize=True, label='Safe Direction')

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Point Cloud and Safe Direction for Drone')

# Create custom legend
handles = [point_cloud_handle, drone_position_handle]
labels = ['Point Cloud', 'Drone Position']
ax.legend(handles=handles, labels=labels)

plt.show()

# %%

import numpy as np
import plotly.graph_objects as go
import plotly.io as pio

# Set the default renderer to ensure it shows up in scripts
pio.renderers.default = 'browser'

# Define parameters
fov_horizontal = 33  # degrees
fov_vertical = 23  # degrees
rotation_z_max = 80  # degrees
rotation_x_max = 29  # degrees
range_distance = 2000  # mm, 2 meters

# Convert to radians
fov_horizontal_rad = np.radians(fov_horizontal / 2)
fov_vertical_rad = np.radians(fov_vertical / 2)
rotation_z_max_rad = np.radians(rotation_z_max)
rotation_x_max_rad = np.radians(rotation_x_max)

# Define a function to calculate FOV points in 3D space
def calculate_fov_points(fov_h_rad, fov_v_rad, range_dist, angle_z, angle_x):
    x = range_dist * np.cos(angle_z) * np.tan(fov_h_rad)
    y = range_dist * np.sin(angle_z)
    z = range_dist * np.sin(angle_x) * np.tan(fov_v_rad)
    return x, y, z

# Generate the FOV surface
theta = np.linspace(-rotation_z_max_rad, rotation_z_max_rad, 50)
phi = np.linspace(-rotation_x_max_rad, rotation_x_max_rad, 50)
theta, phi = np.meshgrid(theta, phi)

# Calculate surface points
x_surface = range_distance * np.cos(theta) * np.tan(fov_horizontal_rad)
y_surface = range_distance * np.sin(theta)
z_surface = range_distance * np.sin(phi) * np.tan(fov_vertical_rad)

# Create the 3D plot
fig = go.Figure()

# Add FOV surface
fig.add_trace(go.Surface(x=x_surface, y=y_surface, z=z_surface, opacity=0.5, colorscale='Viridis'))

# Add a simple representation of the camera
camera_cube = go.Mesh3d(
    x=[-35, 35, 35, -35, -35, 35, 35, -35],
    y=[0, 0, 0, 0, 0, 0, 0, 0],
    z=[-35, -35, 35, 35, -35, -35, 35, 35],
    color='orange',
    opacity=0.5,
)
fig.add_trace(camera_cube)

# Layout adjustments
fig.update_layout(
    scene=dict(
        xaxis=dict(title='X-axis (mm)'),
        yaxis=dict(title='Y-axis (mm)'),
        zaxis=dict(title='Z-axis (mm)'),
        aspectmode='manual',
        aspectratio=dict(x=1, y=2, z=1),
    ),
    title="Camera Field of View Visualization",
    showlegend=False
)

# Display the plot
fig.show()

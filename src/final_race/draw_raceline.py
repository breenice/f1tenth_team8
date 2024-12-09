import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import csv

# Load the map image
map_image_path = 'map/base_map.pgm'
map_image = Image.open(map_image_path)
map_array = np.array(map_image)

# Map metadata from the YAML file
origin_x, origin_y = -6.977912, -3.423147  # Origin of the map
resolution = 0.025000  # Resolution of the map


# Function to convert image coordinates to map coordinates
def image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution):
    x_map = x_img * resolution + origin_x
    y_map = (map_array.shape[0] - y_img) * resolution + origin_y
    return x_map, y_map


# Initialize lists to store clicked points
x_clicked = []
y_clicked = []
line = None
points = None
x_interpolated = []  # Move global variables to top level
y_interpolated = []
point_markers = []  # Store point markers for undo functionality


def catmull_rom_spline(P0, P1, P2, P3, num_points=100):
    """
    Compute Catmull-Rom spline between P1 and P2.
    P0 and P3 are used to calculate tangents.
    """
    # Convert points to numpy arrays for easier computation
    P0, P1, P2, P3 = map(np.array, [P0, P1, P2, P3])
    
    # Create t points
    t = np.linspace(0, 1, num_points)
    
    # Catmull-Rom matrix
    M = np.array([
        [0, 1, 0, 0],
        [-0.5, 0, 0.5, 0],
        [1, -2.5, 2, -0.5],
        [-0.5, 1.5, -1.5, 0.5]
    ])
    
    # Calculate points
    points = []
    for ti in t:
        t_vec = np.array([1, ti, ti**2, ti**3])
        point = 0.5 * ((2*P1) +
                       (-P0 + P2) * ti +
                       (2*P0 - 5*P1 + 4*P2 - P3) * ti**2 +
                       (-P0 + 3*P1 - 3*P2 + P3) * ti**3)
        points.append(point)
    
    return np.array(points)


# Function to generate and display spline
def generate_spline():
    global line, points, x_interpolated, y_interpolated
    if len(x_clicked) >= 4:  # Need at least 4 points for Catmull-Rom
        # Remove old spline and points if they exist
        if line is not None:
            line.remove()
        if points is not None:
            points.remove()
        
        # Sort points clockwise before creating the loop
        x_sorted, y_sorted = x_clicked, y_clicked # sort_points_clockwise(x_clicked, y_clicked)
        
        # Create closed loop by adding points for periodic boundary
        x_loop = x_sorted[-1:] + x_sorted + x_sorted[:2]
        y_loop = y_sorted[-1:] + y_sorted + y_sorted[:2]
        
        # Generate spline segments
        x_spline = []
        y_spline = []
        
        for i in range(len(x_sorted)):
            # Get 4 points for each segment
            x_points = x_loop[i:i+4]
            y_points = y_loop[i:i+4]
            
            # Generate points for this segment
            points = catmull_rom_spline(
                (x_points[0], y_points[0]),
                (x_points[1], y_points[1]),
                (x_points[2], y_points[2]),
                (x_points[3], y_points[3]),
                num_points=50
            )
            
            x_spline.extend(points[:, 0])
            y_spline.extend(points[:, 1])
        
        # Store interpolated points for saving later
        x_interpolated = np.array(x_spline)
        y_interpolated = np.array(y_spline)
        
        # Plot the spline and interpolated points
        line, = ax.plot(x_spline, y_spline, 'b-', linewidth=2)
        points = ax.scatter(x_spline, y_spline, c='g', s=10)
        fig.canvas.draw()


# Function to handle mouse click events
def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x_clicked.append(event.xdata)
        y_clicked.append(event.ydata)
        # Store the point marker for potential undo
        point_marker, = ax.plot(event.xdata, event.ydata, 'ro')  # Mark the clicked point with a red dot
        point_markers.append(point_marker)
        generate_spline()  # Generate spline after each point
        fig.canvas.draw()  # Update the figure to show the new point


# Function to reset the canvas
def reset_canvas():
    global x_clicked, y_clicked, line, points, x_interpolated, y_interpolated, point_markers
    x_clicked = []
    y_clicked = []
    x_interpolated = []
    y_interpolated = []
    if line is not None:
        line.remove()
        line = None
    if points is not None:
        points.remove()
        points = None
    # Remove all point markers
    for marker in point_markers:
        marker.remove()
    point_markers = []
    ax.clear()
    ax.imshow(map_array, cmap='gray')
    ax.set_title('Click to draw points, r to reset, z to undo')
    fig.canvas.draw()


# Function to sort points in clockwise order around their centroid
def sort_points_clockwise(x_points, y_points):
    # Calculate centroid
    centroid_x = sum(x_points) / len(x_points)
    centroid_y = sum(y_points) / len(y_points)
    
    # Calculate angles between points and centroid
    angles = []
    for x, y in zip(x_points, y_points):
        angle = np.arctan2(y - centroid_y, x - centroid_x)
        angles.append(angle)
    
    # Sort points by angle
    sorted_indices = np.argsort(angles)
    x_sorted = [x_points[i] for i in sorted_indices]
    y_sorted = [y_points[i] for i in sorted_indices]
    
    return x_sorted, y_sorted


# Function to handle key press events
def onkey(event):
    global line, points, x_interpolated, y_interpolated
    if event.key == 'r':  # Reset canvas when 'r' is pressed
        reset_canvas()
    elif event.key == 'z' and x_clicked:  # Undo last point when 'z' is pressed
        x_clicked.pop()
        y_clicked.pop()
        point_markers[-1].remove()  # Remove the last point marker
        point_markers.pop()
        # Remove spline if it exists
        if line is not None:
            line.remove()
            line = None
        if points is not None:
            points.remove()
            points = None
        generate_spline()  # Regenerate spline after undo
        fig.canvas.draw()


# Function to save the clicked points to a CSV file, adjusted for map coordinates
def save_race_line(filename='race_line.csv'):
    if len(x_interpolated) == 0:
        print("No points to save - please create a spline first")
        return
        
    with open(filename, mode='w') as file:
        writer = csv.writer(file)
        # Save the interpolated points instead of just clicked points
        for x_img, y_img in zip(x_interpolated, y_interpolated):
            x_map, y_map = image_to_map_coordinates(x_img, y_img, origin_x, origin_y, resolution)
            writer.writerow([x_map, y_map, 0.0, 1.0])  # Assigning a constant speed of 1.0 for demonstration
    print("Race line saved to", filename)


# Set up the plot
fig, ax = plt.subplots()
ax.imshow(map_array, cmap='gray')
ax.set_title('Click to draw points, r to reset, z to undo')
fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', onkey)

plt.show()

# Save the race line after the plot window is closed
save_race_line('demoline.csv')

import csv
import numpy as np
import pygame
import sys

from sailboat_physics import SailboatPhysics
from sailboat_controller import SailboatController

# Define some waypoints for the sailboat to follow
waypoints = [
    (45.501389, -73.567222),
    (45.501944, -73.566944),
    (45.502778, -73.565833),
    (45.503333, -73.565556),
    (45.504444, -73.564167),
    (45.504722, -73.563889),
    (45.505833, -73.562778),
    (45.506667, -73.561944),
    (45.507222, -73.561667),
    (45.508333, -73.560556)
]

# Convert waypoints to latitude and longitude
latitudes = []
longitudes = []
for waypoint in waypoints:
    lat, lon = waypoint
    latitudes.append(lat)
    longitudes.append(lon)

# Create a SailboatPhysics object to simulate the physical behavior of the sailboat
sailboat_physics = SailboatPhysics()

# Create a SailboatController object to control the sails and rudder
controller = SailboatController(latitudes, longitudes)

# Initialize lists to store positions, velocities, and angles
latitudes = []
longitudes = []
speeds = []
headings = []
rolls = []
yaws = []

# Initialize pygame for visualization
pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Sailboat Simulation")

# Main control loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Update sailboat physics
    lat, lon, v_wind, theta_wind, phi, psi, v, omega = sailboat_physics.update()

    # Update controller with current state
    should_continue = controller.update(lat, lon, v_wind, theta_wind, phi, psi)
    if not should_continue:
        break

    # Update sailboat physics based on actuator inputs
    sailboat_physics.set_sails(controller.sail_control)
    sailboat_physics.set_rudder(controller.rudder_control)

    # Append current positions, velocities, and angles to lists
    latitudes.append(lat)
    longitudes.append(lon)
    speeds.append(v)
    headings.append(psi)
    rolls.append(phi)
    yaws.append(omega)

    # Draw sailboat on screen
    screen.fill((255, 255, 255))
    pygame.draw.circle(screen, (0, 0, 0), (int(lon * 100), int(lat * 100)), 2)

    # Draw arrow representing heading
    x1 = int(lon * 100)
    y1 = int(lat * 100)
    x2 = int(x1 + np.cos(psi) * 10)
    y2 = int(y1 + np.sin(psi) * 10)
    pygame.draw.line(screen, (0, 0, 0), (x1, y1), (x2, y2))

    # Draw legend with velocity and position
    font = pygame.font.Font(None, 36)
    text = "v = {:.2f} m/s, lat = {:.2f}, lon = {:.2f}".format(v, lat, lon)
    text_surface = font.render(text, True, (0, 0, 0))
    screen.blit(text_surface, (10, 10))

    pygame.display.flip()

# Save positions, velocities, and angles to file
data = np.column_stack((latitudes, longitudes, speeds, headings, rolls, yaws))
with open("sailboat_data.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["latitude", "longitude", "speed", "heading", "roll", "yaw"])
    writer.writerows(data)

import pygame
import sys
import numpy as np


def transform_waypoints(waypoints, screen_width, screen_height):
    lats = [x[0] for x in waypoints]
    lons = [x[1] for x in waypoints]
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons), max(lons)
    lat_range, lon_range = max_lat - min_lat, max_lon - min_lon
    lat_factor = screen_height / lat_range
    lon_factor = screen_width / lon_range

    def transform(lat, lon):
        x = int((lon - min_lon) * lon_factor)
        y = int((lat - min_lat) * lat_factor)
        return x, y

    return [transform(lat, lon) for (lat, lon) in waypoints]

# Example usage
#transformed_waypoints = transform_waypoints(waypoints, 640, 480)


# Initialize pygame window
screen_width, screen_height = 640, 480
pygame.init()
screen = pygame.display.set_mode((screen_width, screen_height))
screen.fill((255,0,0))


# Define waypoints
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

# Convert waypoints to screen coordinates
scale_factor = 100000  # Used to scale down waypoints for display
#waypoints_scaled = [(int(x * scale_factor), int(y * scale_factor)) for (x, y) in waypoints]
waypoints_scaled = transform_waypoints(waypoints, 640, 480)

# Wait for user to close window
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
    
        
    # Draw waypoints
    for (x, y) in waypoints_scaled:
        #print('Drawing', x, y)
        pygame.draw.circle(screen, (255, 255, 255), (x, y), 10)

    # Update display
    pygame.display.update()

import math

# Set the physical constants
MASS = 1000  # in kg
GRAVITY = 9.81  # in m/s^2
WATER_DENSITY = 1000  # in kg/m^3
SAIL_DRAG_COEFFICIENT = 0.5
SAIL_LIFT_COEFFICIENT = 1.0
SAIL_AREA = 20  # in m^2
HULL_DRAG_COEFFICIENT = 1.0
HULL_FRONTAL_AREA = 5  # in m^2

# Set the initial state of the sailboat
position = (0.0, 0.0)  # in m
velocity = (0.0, 0.0)  # in m/s
heading = 0.0  # in degrees
sail_1_angle = 0.0  # in degrees
sail_2_angle = 0.0  # in degrees

def update_sailboat(wind_direction, wind_speed, sail_1_angle, sail_2_angle):
    # Calculate the wind forces acting on the sails
    sail_1_force = wind_speed**2 * SAIL_AREA * SAIL_DRAG_COEFFICIENT * math.cos(math.radians(wind_direction - sail_1_angle))
    sail_2_force = wind_speed**2 * SAIL_AREA * SAIL_DRAG_COEFFICIENT * math.cos(math.radians(wind_direction - sail_2_angle))
    
    # Calculate the lift forces acting on the sails
    sail_1_force += wind_speed**2 * SAIL_AREA * SAIL_LIFT_COEFFICIENT * math.sin(math.radians(wind_direction - sail_1_angle))
    sail_2_force += wind_speed**2 * SAIL_AREA * SAIL_LIFT_COEFFICIENT * math.sin(math.radians(wind_direction - sail_2_angle))
    
    # Calculate the drag force acting on the hull
    hull_force = 0.5 * WATER_DENSITY * HULL_DRAG_COEFFICIENT * HULL_FRONTAL_AREA * velocity[0]**2

    # Calculate the acceleration of the sailboat
    acceleration = (sail_1_force + sail_2_force + hull_force) / MASS
    
    # Update the velocity of the sailboat
    velocity = (velocity[0] + acceleration * dt, velocity[1] + acceleration * dt)
    
    # Update the position of the sailboat
    position = (position[0] + velocity[0] * dt, position[1] + velocity[1] * dt)
    
    # Update the heading of the sailboat
    heading += velocity[0] / (2 * math.pi * RADIUS)
    
    # Return the updated state of the sailboat
    return position, velocity, heading


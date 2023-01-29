import math

class PIDController:
    def __init__(self, kp, ki, kd):
        # Set the PID constants
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Initialize the error variables
        self.error = 0
        self.integral = 0
        self.derivative = 0
        
        # Initialize the setpoint
        self.setpoint = 0
        
        # Initialize the output
        self.output = 0
    
    def set_setpoint(self, setpoint):
        # Set the setpoint
        self.setpoint = setpoint
        
        # Reset the error variables
        self.error = 0
        self.integral = 0
        self.derivative = 0
        
        # Reset the output
        self.output = 0
    
    def update(self, measurement):
        # Calculate the error
        self.error = self.setpoint - measurement
        
        # Update the integral
        self.integral += self.error
        
        # Calculate the derivative
        self.derivative = self.error - self.previous_error
        self.previous_error = self.error
        
        # Calculate the output
        self.output = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative


def parse_state_string(state_string):
    # Split the state string into a list of values
    values = state_string.split(",")
    
    # Convert the values to floats
    latitude = float(values[0])
    longitude = float(values[1])
    yaw = float(values[2])
    roll = float(values[3])
    velocity = float(values[4])
    acceleration = float(values[5])
    wind_velocity = float(values[6])
    wind_direction = float(values[7])
    
    # Return the state variables as a tuple
    return latitude, longitude, yaw, roll, velocity, acceleration, wind_velocity, wind_direction

def get_distance_and_bearing(position1, position2):
    # Convert the positions to radians
    lat1 = math.radians(position1[0])
    long1 = math.radians(position1[1])
    lat2 = math.radians(position2[0])
    long2 = math.radians(position2[1])
    
    # Calculate the distance and bearing - distance haversine formula with radius of earth
    distance = 6371 * math.acos(math.sin(lat1)*math.sin(lat2) + math.cos(lat1)*math.cos(lat2)*math.cos(long2-long1))
    bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
    
    # Convert the bearing to degrees
    bearing = math.degrees(bearing)
    
    # Return the distance and bearing
    return distance, bearing

def get_position_and_heading(latitude, longitude, yaw):
    # Convert the latitude and longitude to degrees
    position = (latitude, longitude)
    
    # Convert the yaw angle to degrees
    heading = yaw
    
    # Return the position and heading
    return position, heading

def control_sailboat(state_string, waypoints):
    # Parse the state string to get the state variables
    latitude, longitude, yaw, roll, velocity, acceleration, wind_velocity, wind_direction = parse_state_string(state_string)
    
    # Get the current position and heading of the sailboat
    position, heading = get_position_and_heading(latitude, longitude, yaw)
    
    # Calculate the distance and bearing to the next waypoint
    distance, bearing = get_distance_and_bearing(position, waypoints[0])
    
    # Set the target angle for the rudder
    rudder_angle = 0
    
    # Adjust the rudder angle based on the bearing
    if bearing > 10:
        rudder_angle = 15
    elif bearing < -10:
        rudder_angle = -15
    
    # Set the rudder angle
    #set_rudder_angle(rudder_angle)
    
    # Set the target angles for the sails
    sail_1_angle = 0
    sail_2_angle = 0
    
    # Adjust the sail angles based on the wind direction and speed
    if wind_direction > 180:
        sail_1_angle = 45
        sail_2_angle = -45
    elif wind_direction > 90:
        sail_1_angle = 30
        sail_2_angle = 0
    elif wind_direction > 0:
        sail_1_angle = 0
        sail_2_angle = 30
    else:
        sail_1_angle = -45
        sail_2_angle = 45
    
    # Set the sail angles
    #set_sail_angle(1, sail_1_angle)
    #set_sail_angle(2, sail_2_angle)
    
    # Check if the sailboat has reached the waypoint
    if distance < 5:
        # Remove the waypoint from the list
        waypoints.pop(0)

def control_sailboat_pid(state_string, waypoints):
    # Parse the state string to get the state variables
    latitude, longitude, yaw, roll, velocity, acceleration, wind_velocity, wind_direction = parse_state_string(state_string)

    # Get the current position and heading of the sailboat
    position, heading = get_position_and_heading(latitude, longitude, yaw)

    # Calculate the distance and bearing to the next waypoint
    distance, bearing = get_distance_and_bearing(position, waypoints[0])

    # Set the target angle for the rudder
    rudder_angle = 0

    # Set the PID constants for the rudder controller
    kp = 0.5
    ki = 0.1
    kd = 0.1

    # Initialize the rudder PID controller
    rudder_pid = PIDController(kp, ki, kd)

    # Set the setpoint for the rudder PID controller
    rudder_pid.set_setpoint(0)

    # Update the rudder PID controller with the current error
    rudder_pid.update(bearing)

    # Set the rudder angle based on the output of the PID controller
    rudder_angle = rudder_pid.output

    # Set the rudder angle
    set_rudder_angle(rudder_angle)

    # Set the target angles for the sails
    sail_1_angle = 0
    sail_2_angle = 0

    # Set the PID constants for the sail controllers
    kp = 0.5
    ki = 0.1
    kd = 0.1

    # Initialize the sail PID controllers
    sail_1_pid = PIDController(kp, ki, kd)
    sail_2_pid = PIDController(kp, ki, kd)

    # Set the setpoint for the sail PID controllers
    sail_1_pid.set_setpoint(0)
    sail_2_pid.set_setpoint(0)

    # Update the sail PID controllers with the current error
    sail_1_pid.update(wind_direction)
    sail_2_pid.update(wind_direction)

    # Set the sail angles based on the output of the PID controllers
    sail_1_angle = sail_1_pid.output
    sail_2_angle = sail_2_pid.output

    # Set the sail angles
    set_sail_angle(1, sail_1_angle)
    set_sail_angle(2, sail_2_angle)

    # Check if the sailboat has reached the waypoint
    if distance < 5:
        # Remove the waypoint from the list
        waypoints.pop(0)

import math
import numpy as np


class SailboatPhysics:
    def __init__(self):
        self.sail1 = 0 # angle of sail 1, in degrees
        self.sail2 = 0 # angle of sail 2, in degrees
        self.rudder = 0 # angle of rudder, in degrees
        self.latitude = 0 # current latitude of the boat
        self.longitude = 0 # current longitude of the boat
        self.yaw = 0 # yaw angle of the boat, in degrees
        self.roll = 0 # roll angle of the boat, in degrees
        self.velocity = 0 # velocity of the boat, in m/s
        self.acceleration = 0 # acceleration of the boat, in m/s^2
        self.wind_velocity = 0 # velocity of the wind, in m/s
        self.wind_direction = 0 # direction of the wind, in degrees
        
    def update_parameters(self, wind_velocity, wind_direction):
        # Update sail and rudder angles based on wind velocity and direction, boat's velocity, yaw, roll, acceleration and location
        
        self.wind_velocity = wind_velocity
        self.wind_direction = wind_direction
        
        # set the sail angles using a control algorithm
        # the algorithm can take into account the wind velocity and direction, 
        # boat's velocity, yaw, roll, acceleration, latitude and longitude
        self.sail1, self.sail2 = self.control_algorithm()
        
        # set the rudder angle using a control algorithm
        self.rudder = self.rudder_algorithm()

    def control_algorithm(self, waypoints):
        # Calculate the direction to the next waypoint
        waypoint_latitude, waypoint_longitude = waypoints[0]
        waypoint_direction = calculate_direction(self.latitude, self.longitude, waypoint_latitude, waypoint_longitude)
        
        # desired sail angle
        sail1_desired = waypoint_direction - 45
        sail2_desired = waypoint_direction + 45

        # current sail angle
        sail1_current = self.sail1
        sail2_current = self.sail2

        # PID controller parameters
        Kp = 2
        Ki = 0.1
        Kd = 0.05
        
        # PID controller for sail1
        error = sail1_desired - sail1_current
        self.sail1 += Kp * error + Ki * self.wind_velocity + Kd * self.velocity
        
        # PID controller for sail2
        error = sail2_desired - sail2_current
        self.sail2 += Kp * error + Ki * self.wind_velocity + Kd * self.velocity
        
        return self.sail1, self.sail2

        
    def rudder_algorithm(self, waypoints):
        # Calculate the direction to the next waypoint
        waypoint_latitude, waypoint_longitude = waypoints[0]
        waypoint_direction = calculate_direction(self.latitude, self.longitude, waypoint_latitude, waypoint_longitude)
        
        # desired yaw angle
        yaw_desired = waypoint_direction - 90

        # current yaw angle
        yaw_current = self.yaw

        # PID controller parameters
        Kp = 0.5
        Ki = 0.01
        Kd = 0.01
        
        # PID controller for yaw
        error = yaw_desired - yaw_current
        rudder_angle = Kp * error + Ki * self.wind_velocity + Kd * self.velocity
        
        # check if the angle is within the acceptable range 
        max_rudder_angle = 30 # degrees
        if abs(rudder_angle) > max_rudder_angle:
            rudder_angle = max_rudder_angle * np.sign(rudder_angle)
            
        self.rudder = rudder_angle
        return self.rudder

    def calculate_moment_of_inertia(self):
        # calculate moment of inertia of the hull
        I_hull = (1/12) * self.hull_mass * (self.hull_length**2 + self.hull_width**2)
        # calculate moment of inertia of the keel
        I_keel = self.keel_mass * (self.keel_length**2 + self.keel_width**2) / 12
        # calculate moment of inertia of the mast
        I_mast = self.mast_mass * (self.mast_length**2 + self.mast_width**2) / 12
        # calculate moment of inertia of the sails
        I_sails = self.sails_mass * (self.sails_length**2 + self.sails_width**2) / 12
        # calculate moment of inertia of the rigging
        I_rigging = self.rigging_mass * (self.rigging_length2 + self.rigging_width2) / 12
        # calculate the total moment of inertia
        I_total = I_hull + I_keel + I_mast + I_sails + I_rigging
        # Add the moment of inertia for each of the other components, such as the keel, mast, sails, rigging, etc.
        # add the distance from the center of mass of each component to the center of mass of the entire sailboat.
        I_total += (self.hull_mass + self.keel_mass + self.mast_mass + self.sails_mass + self.rigging_mass) * (self.distance_hull_to_cm2 + self.distance_keel_to_cm2 + self.distance_mast_to_cm2 + self.distance_sails_to_cm2 + self.distance_rigging_to_cm**2)
        self.moment_of_inertia = I_total


    def update_position(self, dt, waypoints):
        # Update position, yaw, roll, velocity, and acceleration based on current sail and rudder angles, wind velocity and direction, and time elapsed since last update
        # dt is the elapsed time since the last update, in seconds
        # waypoints is a list of (latitude, longitude) tuples representing the sailboat's destination waypoints

        # calculate the moment of inertia
        #self.moment_of_inertia = self.mass * (self.length**2 + self.width**2) / 12
        self.calculate_moment_of_inertia()

        # calculate the lift force and drag force of the first sail
        lift_coefficient1 = 0.1 #example
        drag_coefficient1 = 0.2 #example
        lift1 = lift_coefficient1 * 0.5 * self.density * self.wind_velocity**2 * self.sail1_area
        drag1 = drag_coefficient1 * 0.5 * self.density * self.wind_velocity**2 * self.sail1_area

        # calculate the lift force and drag force of the second sail
        lift_coefficient2 = 0.3 #example
        drag_coefficient2 = 0.4 #example
        lift2 = lift_coefficient2 * 0.5 * self.density * self.wind_velocity**2 * self.sail2_area
        drag2 = drag_coefficient2 * 0.5 * self.density * self.wind_velocity**2 * self.sail2_area

        # calculate the sail force based on the lift and drag of the sails
        sail_force = lift1 + lift2 - drag1 - drag2

        # calculate the rudder force based on current rudder angle, velocity and water drag
        rudder_force = self.rudder_area * self.velocity * math.sin(math.radians(self.rudder))

        # update acceleration
        self.acceleration = (sail_force + rudder_force) / self.mass

        # update velocity
        self.velocity += self.acceleration * dt

        # calculate the change in position
        delta_distance = self.velocity * dt

        # check if there are any waypoints
        if len(waypoints) > 0:
            # get the next waypoint
            target_lat, target_long = waypoints[0]
            # calculate the angle to the waypoint
            waypoint_direction = math.degrees(math.atan2(target_long - self.longitude, target_lat - self.latitude))
            # calculate the difference in direction between the current heading and the waypoint
            direction_difference = (waypoint_direction - self.yaw) % 360
            # adjust the rudder to steer towards the waypoint
            self.rudder = direction_difference / 2
            # calculate the distance to the waypoint
            distance_to_waypoint = math.sqrt((target_lat - self.latitude)**2 + (target_long - self.longitude)**2) * self.earth_radius
            # check if the sailboat has reached the waypoint
            if distance_to_waypoint < delta_distance:
                # remove the waypoint from the list
                waypoints.pop(0)

        delta_latitude = delta_distance * math.cos(math.radians(self.yaw)) / (self.earth_radius * math.cos(math.radians(self.latitude)))
        delta_longitude = delta_distance * math.sin(math.radians(self.yaw)) / self.earth_radius

        # update position
        self.latitude += delta_latitude
        self.longitude += delta_longitude

        # update yaw and roll
        self.yaw += rudder_force * dt / self.moment_of_inertia
        self.roll += sail_force * dt / self.moment_of_inertia




def calculate_direction(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    bearing = math.atan2(math.sin(lon2-lon1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1))
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

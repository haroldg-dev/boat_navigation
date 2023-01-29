import numpy as np
import skfuzzy as fuzz

class SailboatFuzzyController:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.current_waypoint = 0

        # Define input variables
        self.wind_speed = np.zeros(1)
        self.wind_direction = np.zeros(1)
        self.heading_error = np.zeros(1)
        self.speed_error = np.zeros(1)

        # Define output variables
        self.sail_control = np.zeros(1)
        self.rudder_control = np.zeros(1)

        # Define membership functions
        self.wind_speed_low = fuzz.trimf(self.wind_speed, [0, 0, 5])
        self.wind_speed_medium = fuzz.trimf(self.wind_speed, [0, 5, 10])
        self.wind_speed_high = fuzz.trimf(self.wind_speed, [5, 10, 10])

        self.wind_direction_upwind = fuzz.trimf(self.wind_direction, [0, 0, 90])
        self.wind_direction_downwind = fuzz.trimf(self.wind_direction, [90, 180, 180])

        self.heading_error_small = fuzz.trimf(self.heading_error, [-180, -180, -90])
        self.heading_error_large = fuzz.trimf(self.heading_error, [90, 180, 180])

        self.speed_error_negative = fuzz.trimf(self.speed_error, [-10, -10, 0])
        self.speed_error_positive = fuzz.trimf(self.speed_error, [0, 10, 10])

        self.sail_control_low = fuzz.trimf(self.sail_control, [-45, -45, 0])
        self.sail_control_high = fuzz.trimf(self.sail_control, [0, 45, 45])

        self.rudder_control_left = fuzz.trimf(self.rudder_control, [-45, -45, 0])
        self.rudder_control_right = fuzz.trimf(self.rudder_control, [0, 45, 45])

    def update(self, lat, lon, v_wind, theta_wind, phi, psi):
        # Calculate errors
        waypoint_lat, waypoint_lon = self.waypoints[self.current_waypoint]
        heading_error = np.arctan2(np.sin(psi - waypoint_lon), np.cos(psi - waypoint_lon))
        speed_error = v_wind - np.sqrt((lat - waypoint_lat)**2 + (lon - waypoint_lon)**2)

        # Update input variables
        self.wind_speed.itemset(0, v_wind)
        self.wind_direction.itemset(0, theta_wind)
        self.heading_error.itemset(0, heading_error)
        self.speed_error.itemset(0, speed_error)

        # Fuzzify input variables
        wind_speed_low = fuzz.interp_membership(self.wind_speed, self.wind_speed_low, v_wind)
        wind_speed_medium = fuzz.interp_membership(self.wind_speed, self.wind_speed_medium, v_wind)
        wind_speed_high = fuzz.interp_membership(self.wind_speed, self.wind_speed_high, v_wind)

        wind_direction_upwind = fuzz.interp_membership(self.wind_direction, self.wind_direction_upwind, theta_wind)
        wind_direction_downwind = fuzz.interp_membership(self.wind_direction, self.wind_direction_downwind, theta_wind)

        heading_error_small = fuzz.interp_membership(self.heading_error, self.heading_error_small, heading_error)
        heading_error_large = fuzz.interp_membership(self.heading_error, self.heading_error_large, heading_error)

        speed_error_negative = fuzz.interp_membership(self.speed_error, self.speed_error_negative, speed_error)
        speed_error_positive = fuzz.interp_membership(self.speed_error, self.speed_error_positive, speed_error)

        # Define rules
        rule1 = np.fmax(wind_direction_upwind, heading_error_large)
        sail_control_low = np.fmin(rule1, wind_speed_low)

        rule2 = np.fmax(wind_direction_downwind, heading_error_small)
        sail_control_high = np.fmin(rule2, wind_speed_medium)

        rule3 = np.fmax(speed_error_negative, heading_error_large)
        rudder_control_left = np.fmin(rule3, wind_speed_medium)

        rule4 = np.fmax(speed_error_positive, heading_error_small)
        rudder_control_right = np.fmin(rule4, wind_speed_medium)

        # Aggregate output variables
        sail_control = np.fmax(sail_control_low, sail_control_high)
        rudder_control = np.fmax(rudder_control_left, rudder_control_right)

        # Defuzzify output variables
        self.sail_control.itemset(0, fuzz.defuzz(self.sail_control, sail_control, 'mom'))
        self.rudder_control.itemset(0, fuzz.defuzz(self.rudder_control, rudder_control, 'mom'))

        # Check if sailboat has reached waypoint
        if np.abs(lat - waypoint_lat) < 0.01 and np.abs(lon - waypoint_lon) < 0.01:
            self.current_waypoint += 1
            if self.current_waypoint >= len(self.waypoints):
                return False
        return True


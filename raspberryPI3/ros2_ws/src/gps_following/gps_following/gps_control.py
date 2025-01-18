#!/usr/bin/env python3
import csv
from math import radians, degrees, cos, sin, sqrt, atan2
import rclpy
from rclpy.node import Node
from interfaces.msg import Gnss
from interfaces.msg import GnssStatus


class GnssListener(Node):
    def __init__(self):
        super().__init__('gnss_listener')

        # Create a subscription to listen to GNSS data from the topic '/gnss_data'
        self.subscription = self.create_subscription(
            Gnss,
            '/gnss_data',
            self.listener_callback,
            10  # Buffer size
        )

        # Create a publisher to send GNSS status messages
        self.publisher = self.create_publisher(GnssStatus, '/gnss_status', 10)

        # Load the GPS points from the itinerary CSV file
        self.itinerary = self.load_itinerary_from_csv('gnss_data_long.csv')

        if not self.itinerary:
            self.get_logger().error("No valid GPS points loaded from the itinerary!")
            rclpy.shutdown()

        self.current_target_index = 0  # Start at the first target in the itinerary
        self.previous_lat = None
        self.previous_lon = None

        # Define fixed offsets for latitude and longitude corrections (adjust as needed)
        self.lat_offset = 0.000023239  # Example offset in latitude
        self.lon_offset = 0.000022377  # Example offset in longitude

    def load_itinerary_from_csv(self, file_name):
        # Load GPS points from a CSV file
        itinerary = []
        try:
            with open(file_name, mode='r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:  # Skip empty rows
                        try:
                            # Extract latitude and longitude from the row
                            lat, lon = float(row[0]), float(row[1])
                            itinerary.append((lat, lon))
                        except ValueError as e:
                            self.get_logger().error(f"Invalid data in row {row}: {e}")
        except FileNotFoundError:
            self.get_logger().error(f"File {file_name} not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading {file_name}: {e}")
        return itinerary

    def listener_callback(self, msg):
        # Retrieve the current vehicle coordinates and apply the offset

        if msg.quality == 1 or msg.quality == 2:
            current_lat = msg.latitude
            current_lon = msg.longitude
        else:
            current_lat = msg.latitude + self.lat_offset  # Apply latitude correction
            current_lon = msg.longitude + self.lon_offset  # Apply longitude correction

        # Get the coordinates of the current target and the final destination
        target_lat, target_lon = self.itinerary[self.current_target_index]
        final_lat, final_lon = self.itinerary[-1]  # Last point in the itinerary

        if self.previous_lat is not None and self.previous_lon is not None:
            # Calculate the distance to the target using the Haversine formula
            distance = self.haversine(current_lat, current_lon, target_lat, target_lon)

            # Calculate the direction (bearing) towards the current position
            current_direction = self.calculate_bearing(self.previous_lat, self.previous_lon, current_lat, current_lon)

            # Calculate the direction (bearing) towards the new point
            target_direction = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)  # Example target point

            # Calculate the angle difference between current and target directions
            angle_difference = self.calculate_angle_difference(current_direction, target_direction)

            # Create a new status message
            status_msg = GnssStatus()
            status_msg.current_latitude = current_lat
            status_msg.current_longitude = current_lon
            status_msg.target_latitude = target_lat
            status_msg.target_longitude = target_lon
            status_msg.distance_to_target = distance
            status_msg.previous_latitude = self.previous_lat
            status_msg.previous_longitude = self.previous_lon
            status_msg.current_direction = float(current_direction)
            status_msg.target_direction = target_direction
            status_msg.turn_angle = angle_difference

            # Set the direction message based on the angle difference
            if angle_difference > 10:
                status_msg.direction_message = f"Turn Right by {angle_difference:.2f} degrees"
            elif angle_difference < -10:
                status_msg.direction_message = f"Turn Left by {abs(angle_difference):.2f} degrees"
            else:
                status_msg.direction_message = "Go Straight"

            # Check if the vehicle is close enough to the target (within 10 cm)
            if distance < 30:  # 0.0001 km = 10 cm
                # Move to the next target in the itinerary
                self.current_target_index += 1
                if self.current_target_index >= len(self.itinerary):
                    status_msg.status_message = "End of itinerary reached!"
                    self.current_target_index = 0  # Restart from the beginning of the itinerary
                else:
                    status_msg.status_message = f"Arrived at target {self.current_target_index}. Moving to next target."
            else:
                status_msg.status_message = f"Navigating to target {self.current_target_index}."

            # Publish the status message
            self.publisher.publish(status_msg)

        self.previous_lat = current_lat
        self.previous_lon = current_lon

    def haversine(self, lat1, lon1, lat2, lon2):
        # Convert coordinates to radians
        lat1 = radians(lat1)
        lon1 = radians(lon1)
        lat2 = radians(lat2)
        lon2 = radians(lon2)

        # Calculate the differences in latitudes and longitudes
        dlat = lat2 - lat1
        dlon = lon2 - lon1

        # Apply the Haversine formula
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        R = 637100000  # Earth radius in kilometers
        return R * c  # Return distance in kilometers

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate the bearing (angle) between two GPS points."""
        lat1, lat2 = map(radians, [lat1, lat2])
        delta_lon = radians(lon2 - lon1)
        x = sin(delta_lon) * cos(lat2)
        y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(delta_lon)
        return (degrees(atan2(x, y)) + 360) % 360

    def calculate_angle_difference(self, current_direction, target_direction):
        """Calculate the difference in angle between the current and target directions."""
        angle_difference = (target_direction - current_direction + 360) % 360
        if angle_difference > 180:
            angle_difference -= 360
        return angle_difference


def main(args=None):
    rclpy.init(args=args)
    gnss_listener = GnssListener()
    rclpy.spin(gnss_listener)
    gnss_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

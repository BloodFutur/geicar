import rclpy
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import csv

# Path to your ROS 2 bag file
bag_file_path = "../rosbag_data/odometry_GPS_IMU.db3"

# Path to save the CSV file
csv_file_path = "../rosbag_data/filtered_odometry_positions_IMU.csv"

# Initialize the ROS 2 context
rclpy.init()

# Create the bag reader and open the bag file
bag_reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr", output_serialization_format="cdr"
)
bag_reader.open(storage_options, converter_options)

# Define the topic to extract
odometry_topic = "/odometry/filtered"  # Adjust to match your topic name

# Open the CSV file for writing
with open(csv_file_path, mode="w", newline="") as csvfile:
    csv_writer = csv.writer(csvfile)
    # Write header
    csv_writer.writerow(["timestamp", "position_x", "position_y", "position_z",
                         "orientation_x", "orientation_y", "orientation_z", "orientation_w"])

    # Loop through the messages in the bag
    while bag_reader.has_next():
        (topic, data, timestamp) = bag_reader.read_next()

        if topic == odometry_topic:
            odometry_msg = deserialize_message(data, Odometry)

            # Extract position and orientation
            position = odometry_msg.pose.pose.position
            orientation = odometry_msg.pose.pose.orientation

            # Write the extracted data to the CSV file
            csv_writer.writerow([
                timestamp,
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            
            # Print the position and orientation
            print(f"Timestamp: {timestamp}")
            print(f"  Position -> x: {position.x}, y: {position.y}, z: {position.z}")
            print(f"  Orientation (quaternion) -> x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}")


print(f"Odometry data extracted and saved to {csv_file_path}")

# Shutdown ROS 2 context
rclpy.shutdown()


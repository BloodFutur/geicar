import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

# Attention : build package "interfaces" before running thí script
reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(
    uri='../rosbag_data/db3/firstbag.db3',  
    storage_id='sqlite3'
)
converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr'
)
reader.open(storage_options, converter_options)
topics = reader.get_all_topics_and_types()
for topic in topics:
    print(f"Topic: {topic.name}, Type: {topic.type}")

time_feedback = []
left_feedback = []
right_feedback = []
time_command = []
left_command = []
right_command = []
while reader.has_next():
    (topic_name, serialized_msg, t) = reader.read_next()
    message_type = next((item.type for item in topics if item.name == topic_name), None)
    if message_type:
        try:
            msg_class = get_message(message_type)
            msg = deserialize_message(serialized_msg, msg_class)
            #print(f"Topic: {topic_name}, Time: {t}")

            if topic_name == '/motors_feedback': 
                fields = dir(msg)
                for field_name in fields:
                    field_value = getattr(msg, field_name)
                    if(field_name == 'left_rear_speed'):
                        left_feedback.append(field_value)
                        time_feedback.append(t)
                    if(field_name == 'right_rear_speed'):
                        right_feedback.append(field_value)
            if topic_name == '/motors_order': 
                fields = dir(msg)
                for field_name in fields:
                    field_value = getattr(msg, field_name)
                    if(field_name == 'left_rear_pwm'):
                        field_value = field_value - 50
                        left_command.append(field_value)
                        time_command.append(t)
                    if(field_name == 'right_rear_pwm'):
                        field_value = field_value - 50
                        right_command.append(field_value)

        except Exception as e:
            print(f"Error processing message from topic '{topic_name}': {e}")

base_time = min(time_command)
normalized_time_command = [time - base_time for time in time_command]
normalized_time_feedback = [time - base_time for time in time_feedback]


import pandas as pd

left_feedback_df = pd.DataFrame({'time': normalized_time_feedback, 'left_feedback': left_feedback})
right_feedback_df = pd.DataFrame({'time': normalized_time_feedback, 'right_feedback': right_feedback})

# left_feedback_file = '../rosbag_data/csv/left_feedback.csv'
# right_feedback_file = '../rosbag_data/csv/right_feedback.csv'

# left_feedback_file = '../rosbag_data/csv/left_feedback_2.csv'
# right_feedback_file = '../rosbag_data/csv/right_feedback_2.csv'

left_feedback_file = '../rosbag_data/csv/left_feedback_3.csv'
right_feedback_file = '../rosbag_data/csv/right_feedback_3.csv'

left_feedback_df.to_csv(left_feedback_file, index=False)
right_feedback_df.to_csv(right_feedback_file, index=False)

left_feedback_df, right_feedback_df

left_command_df = pd.DataFrame({'time': normalized_time_command, 'left_command': left_command})
right_command_df = pd.DataFrame({'time': normalized_time_command, 'right_command': right_command})

# left_command_file = '../rosbag_data/csv/left_command.csv'
# right_command_file = '../rosbag_data/csv/right_command.csv'

# left_command_file = '../rosbag_data/csv/left_command_2.csv'
# right_command_file = '../rosbag_data/csv/right_command_2.csv'

left_command_file = '../rosbag_data/csv/left_command_3.csv'
right_command_file = '../rosbag_data/csv/right_command_3.csv'

left_command_df.to_csv(left_command_file, index=False)
right_command_df.to_csv(right_command_file, index=False)

left_command_file, right_command_file

import pandas as pd
import matplotlib.pyplot as plt


left_feedback_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/left_feedback.csv" 
right_feedback_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/right_feedback.csv"  
left_command_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/left_command.csv" 
right_command_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/right_command.csv" 

left_feedback_df = pd.read_csv(left_feedback_file)
right_feedback_df = pd.read_csv(right_feedback_file)
left_command_df = pd.read_csv(left_command_file)
right_command_df = pd.read_csv(right_command_file)

fig, axs = plt.subplots(2, 1, figsize=(10, 8)) 
fig.suptitle('Feedback and Command Comparison From Last Bag File')

axs[0].plot(left_feedback_df['time'], left_feedback_df['left_feedback'], marker='o', color='blue', label='Feedback')
axs[0].plot(left_command_df['time'], left_command_df['left_command'], marker='o', color='green', label='Command')
axs[0].set_title('Left Motor Feedback over Time')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Left Motor Feedback')
axs[0].legend(loc='upper left')
axs[0].grid(True)

axs[1].plot(right_feedback_df['time'], right_feedback_df['right_feedback'], marker='o', color='red', label='Feedback')
axs[1].plot(right_command_df['time'], right_command_df['right_command'], marker='o', color='green', label='Command')
axs[1].set_title('Right Motor Feedback over Time')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Right Motor Feedback')
axs[1].legend(loc='upper left')
axs[1].grid(True)

left_feedback_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/left_feedback_3.csv" 
right_feedback_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/right_feedback_3.csv"  
left_command_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/left_command_3.csv" 
right_command_file = "geicar/raspberryPI3/ros2_ws/src/rosbag_data/csv/right_command_3.csv" 


fig, axs = plt.subplots(2, 1, figsize=(10, 8)) 
fig.suptitle('Feedback and Command Comparison From First Bag File')

axs[0].plot(left_feedback_df['time'], left_feedback_df['left_feedback'], marker='o', color='blue', label='Feedback')
axs[0].plot(left_command_df['time'], left_command_df['left_command'], marker='o', color='green', label='Command')
axs[0].set_title('Left Motor Feedback over Time')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Left Motor Feedback')
axs[0].legend(loc='upper left')
axs[0].grid(True)

axs[1].plot(right_feedback_df['time'], right_feedback_df['right_feedback'], marker='o', color='red', label='Feedback')
axs[1].plot(right_command_df['time'], right_command_df['right_command'], marker='o', color='green', label='Command')
axs[1].set_title('Right Motor Feedback over Time')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Right Motor Feedback')
axs[1].legend(loc='upper left')
axs[1].grid(True)

plt.legend()
plt.tight_layout()
plt.show()

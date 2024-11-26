import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Path to the csv files
left_feedback_file = "../rosbag_data/csv/left_feedback.csv" 
right_feedback_file = "../rosbag_data/csv/right_feedback.csv"  
left_command_file = "../rosbag_data/csv/left_command.csv" 
right_command_file = "../rosbag_data/csv/right_command.csv" 

# Load the data from the csv files
left_feedback_df = pd.read_csv(left_feedback_file)
right_feedback_df = pd.read_csv(right_feedback_file)
left_command_df = pd.read_csv(left_command_file)
right_command_df = pd.read_csv(right_command_file)

left_feedback_list = left_feedback_df['left_feedback'].to_list()
right_feedback_list = right_feedback_df['right_feedback'].to_list()
left_command_list = left_command_df['left_command'].to_list()
right_command_list = right_command_df['right_command'].to_list()

#Remove the values below 10 to calculate the static gain
left_command_list = [x for x in left_command_list if x > 10]
left_feedback_list = [x for x in left_feedback_list if x > 10]
right_command_list = [x for x in right_command_list if x > 10]
right_feedback_list = [x for x in right_feedback_list if x > 10]


# Get the static gain with the average of 200 last values of response and input
# The static gain is the ratio of the average output to the average input

# LEFT MOTOR
average_output_left = np.mean(left_feedback_list[-200:])
average_input_left = np.mean(left_command_list[-200:])  
K_left = average_output_left / average_input_left
print(f"Static Gain Left: {K_left:.3f}")

# RIGHT MOTOR
average_output_right = np.mean(right_feedback_list[-200:])
average_input_right = np.mean(right_command_list[-200:])
K_right = average_output_right / average_input_right
print(f"Static Gain Right: {K_right:.3f}")


#Get the time constant = time when the system reaches 63.2% of final value - time when the system begins to respond to the input
#Time in ros2 is in nanoseconds, so we need to divide by 1000000000 to get the time in seconds

# LEFT MOTOR
#Generate a spline representation of the data to get 10000 points on the curve
tck, u = splprep([left_feedback_df['time'], left_feedback_df['left_feedback']], s=0)
u_new = np.linspace(0, 1, 10000)
left_time, left_feedback = splev(u_new, tck)

commanded_moment_left_index = -1
for i in range(len(left_feedback)):
    #Get the time when the system begins to respond to the input
    if left_feedback[i] > 0.02 and commanded_moment_left_index < 0:
        commanded_moment_left_index = i
    #Get the time when the system reaches 63.2% of final value
    if left_feedback[i] > average_output_left*0.632:
        constant_time_left_index = i
        tau_left = (left_time[i] - left_time[commanded_moment_left_index])/1000000000
        break
print(f"Time Constant Left: {tau_left:.3f}")

# RIGHT MOTOR
#Generate a spline representation of the data to get 10000 points on the curve
tck, u = splprep([right_feedback_df['time'], right_feedback_df['right_feedback']], s=0)
u_new = np.linspace(0, 1, 10000)
right_time, right_feedback = splev(u_new, tck)

commanded_moment_right_index = -1
for i in range(len(right_feedback)):
    #Get the time when the system begins to respond to the input
    if right_feedback[i] > 0.03 and commanded_moment_right_index < 0:
        commanded_moment_right_index = i
    #Get the time when the system reaches 63.2% of final value
    if right_feedback[i] > average_output_right*0.632:
        constant_time_right_index = i
        tau_right = (right_time[i] - right_time[commanded_moment_right_index])/1000000000
        break
print(f"Time Constant Right: {tau_right:.3f}")

# Plot the feedback over time
fig, axs = plt.subplots(2, 1, figsize=(10, 10)) 

axs[0].scatter(left_time[constant_time_left_index]/1000000000, left_feedback[constant_time_left_index], color='red', label='63% of Steady State')
axs[0].scatter(left_time[commanded_moment_left_index]/1000000000, left_feedback[commanded_moment_left_index], color='green', label='Commanded Moment')
axs[0].plot(left_time/1000000000, left_feedback, marker='o', color='blue', label='Feedback', markersize=1)
axs[0].plot(left_command_df['time']/1000000000, left_command_df['left_command'], color='green', label='Command')
axs[0].set_title('Left Motor Feedback over Time')
axs[0].text(0, 5, f"Static Gain: {K_left:.3f}\nTime Constant: {tau_left:.3f}")
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Feedback(rpm)')
axs[0].legend(loc='upper left')
axs[0].grid()

axs[1].scatter(right_time[constant_time_right_index]/1000000000, right_feedback[constant_time_right_index], color='red', label='63% of Steady State')
axs[1].scatter(right_time[commanded_moment_right_index]/1000000000, right_feedback[commanded_moment_right_index], color='green', label='Commanded Moment')
axs[1].plot(right_time/1000000000, right_feedback, marker='o', color='blue', label='Feedback', markersize=1)
axs[1].plot(right_command_df['time']/1000000000, right_command_df['right_command'], color='green', label='Command')
axs[1].set_title('Right Motor Feedback over Time')
axs[1].text(0, 5, f"Static Gain: {K_right:.3f}\nTime Constant: {tau_right:.3f}")
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Feedback(rpm)')
axs[1].legend(loc='upper left')
axs[1].grid()

plt.show()
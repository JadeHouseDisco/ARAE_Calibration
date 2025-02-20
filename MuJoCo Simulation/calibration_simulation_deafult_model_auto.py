import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import csv
import os
import time
import json
from utils.maths import *

# Simulation settings (Only Modify This!) ------------------------------------------------------------------------------------------ #
tolerance = 0.02  # 2% error tolerance for error thresholding
variation_threshold = 0.01 # Variation tolerance for variation theresholding
time_window = 5.0  # Time window in seconds for error thresholding and variation thresholding
max_simulation_time = 180 # Maximum simulation time in seconds
# ---------------------------------------------------------------------------------------------------------------------------------- #

# Load and access simulation data
with open('simulation_data.json', 'r') as file:
    json_data = json.load(file)

error_run = []
log_all = []

for position in range(6):

    print("Running simulation for position " + str(position))

    q1, q21, q31 = json_data["positions"][position]

    # Designated folder for saving outputs
    output_folder = "results/position" + str(position)
    os.makedirs(output_folder, exist_ok=True)

    # Load the simulation model
    model = mujoco.MjModel.from_xml_path("main_SN475_ARAE.xml")
    data = mujoco.MjData(model)

    # PID parameters for each motor
    Kp = [50.0, 50.0, 50.0]  # Proportional gains
    Ki = [40.0, 40.0, 40.0]     # Integral gains
    Kd = [2.0, 2.0, 2.0]     # Derivative gains

    # PID error tracking
    e_prev = [0.0, 0.0, 0.0]  # Previous error
    e_integral = [0.0, 0.0, 0.0]  # Integral of error

    dt = model.opt.timestep  # Simulation time step

    # Variables to log data during simulation
    time_log, error_window, position_log, torque_log, humerus_relative_log  = [], [], [], [], []
    window_steps = int(time_window / dt)  # Number of steps in the window

    mujoco.mj_step(model, data)

    # Access body IDs
    pelvis_id = model.body('pelvis').id
    torso_id = model.body('torso').id
    femur_r_id = model.body('femur_r').id
    humerus_r_id = model.body('humerus_r').id
    ulna_r_id = model.body('ulna_r').id
    radius_r_id = model.body('radius_r').id
    hand_r_id = model.body('hand_r').id

    # Extract original positions from the model
    pelvis_pos_original = model.body_pos[pelvis_id]
    torso_pos_original = model.body_pos[torso_id]
    femur_r_pos_original = model.body_pos[femur_r_id]
    humerus_r_pos_original = model.body_pos[humerus_r_id]
    ulna_r_pos_original = model.body_pos[ulna_r_id]
    radius_r_pos_original = model.body_pos[radius_r_id]
    hand_r_pos_original = model.body_pos[hand_r_id]
    femur_to_humerus_r = -femur_r_pos_original + torso_pos_original + humerus_r_pos_original

    upper_arm_length = extract_length(ulna_r_pos_original)
    forearm_length = extract_length(hand_r_pos_original)
    femur_to_humeris = extract_length(femur_to_humerus_r)
    pelvis_to_femur = extract_length(femur_r_pos_original)

    # Simulation initilization
    simulation_start_time = time.time()

    while True:
        current_time = time.time()
        elapsed_time = current_time - simulation_start_time  # Elapsed time in seconds

        if elapsed_time >= max_simulation_time:
            reason = "Maximum simulation time elapsed"
            break

        mujoco.mj_step(model, data)

        # Get actual joint positions
        q1_actual = data.qpos[model.joint('Joint_q1_Blue').id]
        q21_actual = data.qpos[model.joint('Joint_q2_Green').id]
        q31_actual = data.qpos[model.joint('Joint_q3_Orange').id]
        q4 = data.sensordata[model.sensor('Link9_Encoder').id]
        q5 = data.sensordata[model.sensor('Link10_Encoder').id]

        # Compute errors and PID control
        e = [q1 - q1_actual, q21 - q21_actual, q31 - q31_actual]
        de = [(e[i] - e_prev[i]) / dt for i in range(3)]
        e_integral = [e_integral[i] + e[i] * dt for i in range(3)]

        tau_pid = [Kp[i]*e[i] + Ki[i]*e_integral[i] + Kd[i]*de[i] for i in range(3)]
        torques = [tau_pid[i] for i in range(3)]

        # Apply torques
        motor_1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'Motor 1')
        motor_2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'Motor 2')
        motor_3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'Motor 3')
        data.ctrl[motor_1_id] = torques[0]
        data.ctrl[motor_2_id] = torques[1]
        data.ctrl[motor_3_id] = torques[2]

        # Error check for thresholding
        errors_within_tolerance = [
            abs(e[0] / q1) <= tolerance,
            abs(e[1] / q21) <= tolerance,
            abs(e[2] / q31) <= tolerance,
        ]
        error_window.append(all(errors_within_tolerance))
        position_log.append([q1_actual, q21_actual, q31_actual, q4, q5])
        time_log.append(data.time)
        torque_log.append(torques)

        humerus_r_xpos = data.xpos[humerus_r_id]
        pelvis_xpos = data.xpos[pelvis_id]
        humerus_relative = humerus_r_xpos - pelvis_xpos
        humerus_relative_log.append(humerus_relative)

        # Check if conditions are met or max time is reached
        if len(error_window) == window_steps and all(error_window):
            reason = "Error thresholding satisfied"
            break

        # Keep the window size consistent
        if len(error_window) > window_steps:
            error_window.pop(0)

        # Compute standard deviation for each variable in the window
        if len(position_log) >= (window_steps):
            q1_variation = np.std([entry[0] for entry in position_log[-window_steps:]])
            q21_variation = np.std([entry[1] for entry in position_log[-window_steps:]])
            q31_variation = np.std([entry[2] for entry in position_log[-window_steps:]])
            q4_variation = np.std([entry[3] for entry in position_log[-window_steps:]])
            q5_variation = np.std([entry[4] for entry in position_log[-window_steps:]])

            # Check if all variations are below the threshold
            if (q1_variation < variation_threshold and
                q21_variation < variation_threshold and
                q31_variation < variation_threshold and
                q4_variation < variation_threshold and
                q5_variation < variation_threshold):
                reason = "Variation thresholding satisfied"
                break

        e_prev = e

    if reason == "Maximum simulation time elapsed":
        error_run.append({"position": position})

    else:
        # Compute averages
        avg_positions = [sum(pos[i] for pos in position_log[-window_steps:]) / min(window_steps, len(position_log)) for i in range(5)]
        avg_torques = [sum(torque[i] for torque in torque_log[-window_steps:]) / min(window_steps, len(torque_log)) for i in range(3)]
        avg_humerus_relative = [sum(entry[i] for entry in humerus_relative_log[-window_steps:]) / min(window_steps, len(humerus_relative_log)) for i in range(3)]

        # Compute position error in percentage
        position_error = [
            abs((json_data["positions"][position][i] - avg_positions[i]) / json_data["positions"][position][i]) 
            for i in range(3)
        ]

        # Save results to a file
        avg_results_path = os.path.join(output_folder, "results.csv")
        with open(avg_results_path, "w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Reason for Termination", reason])
            writer.writerow(["Metric", "q1", "q21", "q31", "q4", "q5", "Humerus-Pelvis x", "Humerus-Pelvis y", "Humerus-Pelvis z"])
            writer.writerow(["Desired Position"] + json_data["positions"][position])
            writer.writerow(["Actual Position"] + avg_positions + avg_humerus_relative)
            writer.writerow(["Position Error"] + position_error)
            writer.writerow(["Actual Torque"] + avg_torques)

        # Extract columns for plotting
        q1_values = [entry[0] for entry in position_log]
        q21_values = [entry[1] for entry in position_log]
        q31_values = [entry[2] for entry in position_log]
        q4_values = [entry[3] for entry in position_log]
        q5_values = [entry[4] for entry in position_log]

        # Plot joint positions
        plt.figure()
        plt.plot(time_log[:len(q1_values)], q1_values, label='Motor 1 Position (q1)')
        plt.plot(time_log[:len(q21_values)], q21_values, label='Motor 2 Position (q21)')
        plt.plot(time_log[:len(q31_values)], q31_values, label='Motor 3 Position (q31)')
        plt.plot(time_log[:len(q4_values)], q4_values, label='Encoder 1 Position (q4)')
        plt.plot(time_log[:len(q5_values)], q5_values, label='Encoder 2 Position (q5)')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Position')
        plt.title('Positions Over Time')
        positions_plot_path = os.path.join(output_folder, "motor_positions_plot.png")
        plt.savefig(positions_plot_path)
        plt.close()

        # Extract columns for plotting torques
        torque_1_values = [entry[0] for entry in torque_log]
        torque_2_values = [entry[1] for entry in torque_log]
        torque_3_values = [entry[2] for entry in torque_log]

        # Plot motor torques
        plt.figure()
        plt.plot(time_log[:len(torque_1_values)], torque_1_values, label='Motor 1 Torque')
        plt.plot(time_log[:len(torque_2_values)], torque_2_values, label='Motor 2 Torque')
        plt.plot(time_log[:len(torque_3_values)], torque_3_values, label='Motor 3 Torque')
        plt.legend()
        plt.xlabel('Time (s)')
        plt.ylabel('Torque (Nm)')
        plt.title('Torque Applied by Motors Over Time')
        plt.ylim(-100, 100)
        torques_plot_path = os.path.join(output_folder, "motor_torques_plot.png")
        plt.savefig(torques_plot_path)
        plt.close()

        log_all.append([position, avg_positions[0], avg_positions[1], avg_positions[2], avg_positions[3], avg_positions[4], avg_torques[0], avg_torques[1], avg_torques[2], avg_humerus_relative[0], avg_humerus_relative[1], avg_humerus_relative[2]])


if len(error_run) > 0:
    print("Errors occurred in the following runs:")
    for error in error_run:
        print(f"Subject {error['subject']}, Position {error['position']}")
else:
    print("All runs completed cleanly")

results_path = os.path.join("results", "results.csv")
with open(results_path, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Position", "q1", "q21", "q31", "q4", "q5", "t1", "t2", "t3", "Humerus-Pelvis x", "Humerus-Pelvis y", "Humerus-Pelvis z"])
    for i in range(len(log_all)):
        writer.writerow(log_all[i])
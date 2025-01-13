import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
import csv
import os
import time
import json
from utils.maths import *

def idc_calculate(q1, q21, q31, q4, q5, U_L, F_L, T_L, T_W):
    g = 9.81
    U_M = 4.05
    F_M = 3
    Ugf = 0.436
    Lgf = 0.43

    l1 = 0.068
    l21 = 0.430
    l22 = 0.446
    l31 = 0.100
    l32 = 0.430
    x1 = 0.05432
    x21 = 0.21431
    y21 = 0.01885
    lg21 = np.sqrt(x21**2 + y21**2)
    r21 = np.arctan2(y21, x21)
    x22 = 0.33271
    y22 = 0.0
    lg22 = np.sqrt(x22**2 + y22**2)
    r22 = np.arctan2(y22, x22)
    x31 = 0.04632
    x32 = 0.215

    m1 = 1.51806
    m21 = 0.25668
    m22 = 0.55976
    m31 = 0.09410
    m32 = 0.14479

    # Transform Points to pelvis frame
    # Declare Fixed Values
    q2_1 = np.pi / 2 - q21
    q22 = q31 - q2_1 + np.pi
    q32 = 2 * np.pi - (q31 - q2_1)

    alpha_1 = 90
    alpha_21 = 0
    alpha_22 = 0
    alpha_3 = 0
    alpha_31 = 0
    alpha_32 = 0
    alpha_4 = -90
    alpha_5 = 0
    alpha_6 = 0
    alpha_7 = 0

    l4 = 0.071 + 0.04
    l5 = 0.0895
    l6 = 0.14
    l7 = F_L

    # Convert to radians
    alpha_1 = np.radians(alpha_1)
    alpha_21 = np.radians(alpha_21)
    alpha_22 = np.radians(alpha_22)
    alpha_3 = np.radians(alpha_3)
    alpha_31 = np.radians(alpha_31)
    alpha_32 = np.radians(alpha_32)
    alpha_4 = np.radians(alpha_4)
    alpha_5 = np.radians(alpha_5)
    alpha_6 = np.radians(alpha_6)
    alpha_7 = np.radians(alpha_7)

    # DH parameters
    a1 = 0
    d1 = l1
    a21 = l21
    d21 = 0
    a3 = l22 - l31
    d3 = 0
    a4 = 0
    d4 = -l4
    a5 = l5
    d5 = 0.04
    a6 = l6
    d6 = 0
    a7 = l7
    d7 = 0
    a22 = l22
    d22 = 0
    a31 = l31
    d31 = 0
    a32 = l32
    d32 = 0

    # DH parameters matrix "table"
    PT = np.array([
        [q1, alpha_1, a1, d1],
        [q2_1, alpha_21, a21, d21],
        [q22, alpha_22, a3, d3],
        [q4, alpha_4, a4, d4],
        [q5, alpha_5, a5, d5],
        [np.pi/2, alpha_6, a6, d6],
        [np.pi, alpha_7, a7, d7]
    ])

    # Compute individual transformation matrices
    H0_1 = transformation_matrix(0, PT)
    H1_21 = transformation_matrix(1, PT)
    H21_22 = transformation_matrix(2, PT)
    H22_4 = transformation_matrix(3, PT)
    H4_5 = transformation_matrix(4, PT)
    H5_6 = transformation_matrix(5, PT)
    H6_7 = transformation_matrix(6, PT)

    # Transfer matrix from robot base to points
    H0_4 = H0_1 @ H1_21 @ H21_22 @ H22_4
    H0_5 = H0_4 @ H4_5
    H0_6 = H0_5 @ H5_6
    H0_7 = H0_6 @ H6_7

    # Position vector from base to end-effector
    P0_0 = np.array([0, 0, 0, 1])
    P0_4 = H0_4 @ P0_0
    P0_5 = H0_5 @ P0_0
    x0 = P0_5[0]
    y0 = P0_5[1]
    z0 = P0_5[2]

    # Transformation matrix from robot frame to pelvis frame
    Tr_p = Trans(-0.4808, -0.1, 0.0704) @ Rotz(-np.pi / 2)

    # Transforming robot base points to pelvis base points
    Pp_5 = Tr_p @ P0_5

    P0_e = H0_6 @ P0_0
    Pp_e = Tr_p @ P0_e

    P0_w = H0_7 @ P0_0
    Pp_w = Tr_p @ P0_w

    # Transform to shoulder frame
    # Finding shoulder point in pelvis frame
    Pp_e_x = Pp_e[0]
    Pp_e_y = Pp_e[1]
    Pp_e_z = Pp_e[2]

    Eproj_r = np.sqrt(U_L**2 - (Pp_e_x - T_W)**2)  # Projected radius of elbow position in sagittal plane
    l_H_Eproj = np.sqrt(Pp_e_y**2 + Pp_e_z**2)    # Distance between projected elbow joint and hip joint in sagittal plane
    cal_HS = Pp_e_z + Eproj_r

    # Handling intersection calculations
    if cal_HS >= T_L:
        amend_Eproj_r = Eproj_r
        intersection1, intersection2 = circcirc(0, 0, T_L, Pp_e_y, Pp_e_z, amend_Eproj_r)
        Pp_s = np.array([T_W, intersection2[0], intersection2[1]])
    else:
        amend_Eproj_r = T_L - Pp_e_z  # Forcing the projected radius + Pp_e_z to equal T_L
        intersection1, intersection2 = circcirc(0, 0, T_L, Pp_e_y, Pp_e_z, amend_Eproj_r)
        Pp_s = np.array([T_W, intersection2[0], intersection2[1]])

    # Circle parameters
    y1, z1 = 0, 0
    r1 = T_L
    y2 = Pp_e_y
    z2 = Pp_e_z
    r2 = amend_Eproj_r

    # Distance between circle centers
    d = np.sqrt((y2 - y1)**2 + (z2 - z1)**2)

    # Calculate intersection points
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(r1**2 - a**2)

    y2m = y1 + a * (y2 - y1) / d
    z2m = z1 + a * (z2 - z1) / d

    yout = y2m + h * (z2 - z1) / d
    zout = z2m - h * (y2 - y1) / d

    Pp_s = np.array([T_W, yout, zout])

    # Transform pelvis base points to shoulder base points
    Ps_e = Trans(-Pp_s[0], -Pp_s[1], -Pp_s[2]) @ Pp_e
    Ps_w = Trans(-Pp_s[0], -Pp_s[1], -Pp_s[2]) @ Pp_w

    # Find human joint angles
    P_w = np.array([Ps_w[0], Ps_w[1], Ps_w[2]])
    P_e = np.array([Ps_e[0], Ps_e[1], Ps_e[2]])
    P_s = np.array([0, 0, 0])

    U_cal = np.sqrt(Ps_e[0]**2 + Ps_e[1]**2 + Ps_e[2]**2)
    h4 = np.pi / 2 - np.arccos((F_L**2 + U_cal**2 - np.linalg.norm(P_w - P_s)**2) / (2 * F_L * U_cal))  # Elbow flexion
    h2 = np.arcsin(-P_e[2] / U_cal)  # Shoulder flexion (X-axis)
    h1 = np.arctan2(P_e[0] / np.cos(h2), -P_e[1] / np.cos(h2))  # Shoulder abduction (Z-axis)

    vf = Ps_w[0] * np.cos(h1) + Ps_w[1] * np.sin(h1)
    h3 = np.arctan2(
        -(Ps_w[0] * np.sin(h1) * np.sin(h2) - Ps_w[1] * np.cos(h1) * np.sin(h2) + Ps_w[2] * np.cos(h2)) / (F_L * np.cos(h4)),
        vf / (F_L * np.cos(h4))
    )  # Shoulder rotation

    # Applying human arm dynamic model to find support force
    Lr_f = F_L / 2

    # Clamp `h4` to the range [-80° to 80°] in radians
    h4 = np.clip(h4, -80 * np.pi / 180, 80 * np.pi / 180)

    # Gravity torque components
    t1g = 0
    t2g = (F_M * (F_L * Lgf * np.cos(h2) * np.sin(h4) - U_L * np.cos(h2) +
                F_L * Lgf * np.cos(h4) * np.sin(h2) * np.sin(h3)) -
        U_L * U_M * Ugf * np.cos(h2)) * g
    t3g = (-F_L * Lgf * F_M * np.cos(h2) * np.cos(h3) * np.cos(h4)) * g
    t4g = (F_M * (F_L * Lgf * np.cos(h4) * np.sin(h2) +
                F_L * Lgf * np.cos(h2) * np.sin(h3) * np.sin(h4))) * g

    # Transformation matrix
    T_s_r = np.array([
        [0, -1,  0],
        [1,  0,  0],
        [0,  0,  1]
    ])

    # Jacobian matrix
    J_h_2 = np.array([
        [
            U_L * np.cos(h1) * np.cos(h2) -
            Lr_f * np.cos(h4) * (np.cos(h3) * np.sin(h1) + np.cos(h1) * np.sin(h2) * np.sin(h3)) -
            Lr_f * np.cos(h1) * np.cos(h2) * np.sin(h4),
            Lr_f * np.sin(h1) * np.sin(h2) * np.sin(h4) - U_L * np.sin(h1) * np.sin(h2) -
            Lr_f * np.cos(h2) * np.cos(h4) * np.sin(h1) * np.sin(h3),
            -Lr_f * np.cos(h4) * (np.cos(h1) * np.sin(h3) + np.cos(h3) * np.sin(h1) * np.sin(h2)),
            -Lr_f * np.sin(h4) * (np.cos(h1) * np.cos(h3) - np.sin(h1) * np.sin(h2) * np.sin(h3)) -
            Lr_f * np.cos(h2) * np.cos(h4) * np.sin(h1)
        ],
        [
            U_L * np.cos(h2) * np.sin(h1) +
            Lr_f * np.cos(h4) * (np.cos(h1) * np.cos(h3) - np.sin(h1) * np.sin(h2) * np.sin(h3)) -
            Lr_f * np.cos(h2) * np.sin(h1) * np.sin(h4),
            U_L * np.cos(h1) * np.sin(h2) - Lr_f * np.cos(h1) * np.sin(h2) * np.sin(h4) +
            Lr_f * np.cos(h1) * np.cos(h2) * np.cos(h4) * np.sin(h3),
            -Lr_f * np.cos(h4) * (np.sin(h1) * np.sin(h3) - np.cos(h1) * np.cos(h3) * np.sin(h2)),
            Lr_f * np.cos(h1) * np.cos(h2) * np.cos(h4) -
            Lr_f * np.sin(h4) * (np.cos(h3) * np.sin(h1) + np.cos(h1) * np.sin(h2) * np.sin(h3))
        ],
        [
            0,
            Lr_f * np.cos(h2) * np.sin(h4) - U_L * np.cos(h2) +
            Lr_f * np.cos(h4) * np.sin(h2) * np.sin(h3),
            -Lr_f * np.cos(h2) * np.cos(h3) * np.cos(h4),
            Lr_f * np.cos(h4) * np.sin(h2) + Lr_f * np.cos(h2) * np.sin(h3) * np.sin(h4)
        ]
    ])

    # Torque vector
    Tg_h_2 = np.array([t1g, t2g, t3g, t4g])

    # Pseudo-inverse of the transpose of J_h_2
    pv_J_h = np.linalg.pinv(J_h_2.T)

    # Support force in the shoulder frame
    F_r2_s = pv_J_h @ Tg_h_2

    # Transform support force back to the robot frame
    F_r2 = T_s_r @ F_r2_s

    # Find torque required by each motor
    wl4 = 0.6774 * g
    F = np.array([
        F_r2[0],
        F_r2[1],
        wl4 + F_r2[2]
    ])

    # Jacobian matrix
    J = np.array([
        [
            -l21 * np.cos(q21 - np.pi / 2) * np.sin(q1) -
            np.cos(q21 + q31 + np.pi / 2) * np.cos(q21 - np.pi / 2) * np.sin(q1) * (l22 - l31) -
            np.sin(q21 + q31 + np.pi / 2) * np.sin(q1) * np.sin(q21 - np.pi / 2) * (l22 - l31),
            -l21 * np.cos(q1) * np.sin(q21 - np.pi / 2),
            np.cos(q21 + q31 + np.pi / 2) * np.cos(q1) * np.sin(q21 - np.pi / 2) * (l22 - l31) -
            np.sin(q21 + q31 + np.pi / 2) * np.cos(q1) * np.cos(q21 - np.pi / 2) * (l22 - l31)
        ],
        [
            l21 * np.cos(q1) * np.cos(q21 - np.pi / 2) +
            np.cos(q21 + q31 + np.pi / 2) * np.cos(q1) * np.cos(q21 - np.pi / 2) * (l22 - l31) +
            np.sin(q21 + q31 + np.pi / 2) * np.cos(q1) * np.sin(q21 - np.pi / 2) * (l22 - l31),
            -l21 * np.sin(q1) * np.sin(q21 - np.pi / 2),
            np.cos(q21 + q31 + np.pi / 2) * np.sin(q1) * np.sin(q21 - np.pi / 2) * (l22 - l31) -
            np.sin(q21 + q31 + np.pi / 2) * np.cos(q21 - np.pi / 2) * np.sin(q1) * (l22 - l31)
        ],
        [
            0,
            -l21 * np.cos(q21 - np.pi / 2),
            np.cos(q21 + q31 + np.pi / 2) * np.cos(q21 - np.pi / 2) * (l22 - l31) +
            np.sin(q21 + q31 + np.pi / 2) * np.sin(q21 - np.pi / 2) * (l22 - l31)
        ]
    ])

    # Calculate joint forces
    JF = J.T @ F

    # Torque values
    t1g = JF[0]
    t21g = (
        JF[1]
        + (-m22 * (l32 * np.sin(q21 + q31 - (5 * np.pi) / 2) * np.sin(q31) +
                l32 * np.cos(q21 + q31 - (5 * np.pi) / 2) * np.cos(q31))
        - m32 * (x32 * np.cos(q21 + q31 - (5 * np.pi) / 2) * np.cos(q31) +
                    x32 * np.sin(q21 + q31 - (5 * np.pi) / 2) * np.sin(q31))
        - lg21 * m21 * np.cos(r21 - q21 + np.pi / 2))
        * g
    )
    t31g = (
        JF[2]
        + (m22 * (l31 * np.cos(q31) +
                lg22 * np.cos(q21 + q31 + r22 + np.pi / 2) *
                (np.cos(q21 + q31 - (5 * np.pi) / 2) * np.cos(q31) +
                np.sin(q21 + q31 - (5 * np.pi) / 2) * np.sin(q31))
                - lg22 * np.sin(q21 + q31 + r22 + np.pi / 2) *
                (np.cos(q21 + q31 - (5 * np.pi) / 2) * np.sin(q31) -
                np.sin(q21 + q31 - (5 * np.pi) / 2) * np.cos(q31)))
        + l31 * m32 * np.cos(q31) + m31 * x31 * np.cos(q31))
        * g
    )

    return [t1g, t21g, t31g]

# Simulation settings (Only Modify This!) ------------------------------------------------------------------------------------------ #
# subject = 1 # Human antrhopometric data to use for simulation
position = 4 # Robot position for calibration

tolerance = 0.02  # 2% error tolerance for error thresholding
variation_threshold = 0.01 # Variation tolerance for variation theresholding
time_window = 5.0  # Time window in seconds for error thresholding and variation thresholding

# ---------------------------------------------------------------------------------------------------------------------------------- #

# Load and access simulation data
with open('simulation_data.json', 'r') as file:
    json_data = json.load(file)

q1, q21, q31 = json_data["positions"][position]
# anthropometric_data =  json_data["anthropometric_data"][subject]

# Designated folder for saving outputs
# output_folder = "results/subject" + str(subject) + "/position" + str(position)
output_folder = "results/position" + str(position)
os.makedirs(output_folder, exist_ok=True)

# Load the simulation model
model = mujoco.MjModel.from_xml_path("main_SN475_ARAE.xml")
data = mujoco.MjData(model)

# PID parameters for each motor
Kp = [110.0, 110.0, 110.0]  # Proportional gains
Ki = [50.0, 50.0, 50.0]     # Integral gains
Kd = [3.0, 3.0, 3.0]     # Derivative gains

# PID error tracking
e_prev = [0.0, 0.0, 0.0]  # Previous error
e_integral = [0.0, 0.0, 0.0]  # Integral of error

dt = model.opt.timestep  # Simulation time step

# Variables to log data during simulation
time_log, error_window, position_log, torque_log  = [], [], [], []
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

# Extract unit directions from the original XML
# pelvis_to_femur_unit = extract_direction(femur_r_pos_original)
# torso_to_humerus_r_unit = extract_direction(humerus_r_pos_original)
# femur_to_humerus_r_unit = extract_direction(femur_to_humerus_r)
# humerus_to_upper_arm_unit = extract_direction(ulna_r_pos_original)
# upper_arm_to_forearm_unit = extract_direction(hand_r_pos_original)

# Extract anthropometric data from JSON
# pelvis_to_femur_new = anthropometric_data["Pelvis_to_Femur_cm"] / 100  # Convert to meters
# femur_to_humeris_new = anthropometric_data["Femur_to_Shoulder_cm"] / 100  # Convert to meters

# upper_arm_new = anthropometric_data["Upper_Arm_Length_cm"] / 100  # Convert to meters
# upper_arm_mass_kg = anthropometric_data["Upper_Arm_Mass_kg"]
# upper_arm_com_m = upper_arm_new * (anthropometric_data["Upper_Arm_CoM_percent"] / 100)

# forearm_new = anthropometric_data["Forearm_Length_cm"] / 100  # Convert to meters
# forearm_mass_kg = anthropometric_data["Forearm_Mass_kg"]
# forearm_com_m = forearm_new * (anthropometric_data["Forearm_CoM_percent"] / 100)

# Update positions using unit directions and new lengths
# model.body_pos[femur_r_id] = pelvis_to_femur_new * pelvis_to_femur_unit

# torso_to_humerus_r_new = model.body_pos[femur_r_id] + (femur_to_humeris_new * femur_to_humerus_r_unit) - torso_pos_original
# model.body_pos[humerus_r_id] = torso_to_humerus_r_new

# model.body_pos[ulna_r_id] = upper_arm_new * humerus_to_upper_arm_unit
# model.body_pos[hand_r_id] = forearm_new * upper_arm_to_forearm_unit

# # Update mass and center of mass
# model.body_mass[humerus_r_id] = upper_arm_mass_kg
# model.body_ipos[humerus_r_id] = [0,-upper_arm_com_m,0]

# model.body_mass[ulna_r_id] = 0
# model.body_mass[radius_r_id] = forearm_mass_kg
# model.body_ipos[radius_r_id] = [0,-forearm_com_m,0]

# Simulation initilization
simulation_start_time = time.time()
viewer = mujoco.viewer.launch_passive(model, data)

# Configure the camera
viewer.cam.lookat[:] = [0.0, 0.0, 0.8]  # Camera focus point (x, y, z)
viewer.cam.distance = 1.5               # Distance from the focus point
viewer.cam.azimuth = 135                 # Azimuth angle (horizontal rotation in degrees)
viewer.cam.elevation = -10             # Elevation angle (vertical rotation in degrees)

while viewer.is_running():
    current_time = time.time()
    elapsed_time = current_time - simulation_start_time  # Elapsed time in seconds
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
    tau_idc = idc_calculate(q1, q21, q31, q4, q5, upper_arm_length, forearm_length, femur_to_humeris, pelvis_to_femur)
    torques = [tau_idc[i] + tau_pid[i] for i in range(3)]

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
    
    viewer.sync()

# Compute averages
avg_positions = [sum(pos[i] for pos in position_log[-window_steps:]) / min(window_steps, len(position_log)) for i in range(5)]
avg_torques = [sum(torque[i] for torque in torque_log[-window_steps:]) / min(window_steps, len(torque_log)) for i in range(3)]

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
    writer.writerow(["Metric", "q1", "q21", "q31", "q4" , "q5"])
    writer.writerow(["Desired Position"] + json_data["positions"][position])
    writer.writerow(["Actual Position"] + avg_positions)
    writer.writerow(["Position Error"] + position_error)
    writer.writerow(["Actual Torque"] + avg_torques)

print(f"Simulation ended: {reason}")
print(f"Desired Joint Positions: q1={q1}, q21={q21}, q31={q31}")
print(f"Actual Joint Positions: q1={avg_positions[0]:.3f}, q21={avg_positions[1]:.3f}, q31={avg_positions[2]:.3f}")
print(f"Actual Torques: tau1={avg_torques[0]:.3f}, tau2={avg_torques[1]:.3f}, tau3={avg_torques[2]:.3f}")

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
plt.show()

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
plt.show()
import numpy as np

# Humerus data
m_humerus = 1.0542687374296
p_local_humerus = np.array([0, -0.142672, 0])

# Ulna data
m_ulna = 0.433345
p_local_ulna = np.array([0, -0.116243, 0])
p_body_ulna = np.array([0.0113997, -0.248283, -0.00832171])

# Radius data
m_radius = 0.505395
p_body_radius = np.array([-0.00648801, -0.0125449, 0.0251563])
p_local_radius = np.array([0, -0.122358, 0])

# Hand data
p_body_hand = np.array([-0.00893079, -0.239428, 0.013817])

# Upper arm COM calculation
upper_arm_length = np.linalg.norm(p_body_ulna)
percentage_length = (np.linalg.norm(p_local_humerus) / upper_arm_length) * 100
print("Upper Arm Mass:", m_humerus)
print("Upper Arm Length:", upper_arm_length)
print("Upper Arm COM as Percentage of Length:", percentage_length)

# Distal end of the forearm (position of the radius body)
p_hand = p_body_radius + p_body_hand

# Compute the length of the forearm
forearm_length = np.linalg.norm(p_hand)

# Compute global COMs
p_global_ulna = p_local_ulna
p_global_radius = p_body_radius + p_local_radius

# Compute overall COM of the forearm
total_mass = m_ulna + m_radius
p_forearm_com = (m_ulna * p_global_ulna + m_radius * p_global_radius) / total_mass

# Compute the percentage of the forearm length where the COM is located
com_distance_from_elbow = np.linalg.norm(p_forearm_com)
percentage_length = (com_distance_from_elbow / forearm_length) * 100

print("Forearm Mass:", total_mass)
print("Forearm Length:", forearm_length)
print("Forearm COM as Percentage of Length:", percentage_length)

# Femur to humeris calculation
femur_to_pelvis = np.array([0, 0, -0.1429514])
pelvis_to_torso = np.array([-0.108118, 0.0875038, 0])
torso_to_humeris = np.array([0.00301476, 0.354987, 0.162444])
femur_to_humeris = femur_to_pelvis + pelvis_to_torso + torso_to_humeris
print("Length from femur to humeris (T_L):", np.linalg.norm(femur_to_humeris))

# Pelvis to robot base
pelvis = np.array([0, 0, 0.555])
robot_base = np.array([-0.275, -0.385, 0.6885])
pelvis_to_robot_base = robot_base - pelvis
print("Vector from pelvis to robot base:", pelvis_to_robot_base)

rotz_90 = np.array([[0, -1,  0],
                    [1,  0,  0],
                    [0,  0,  1]])
pelvis_to_robot_base_rotated = pelvis_to_robot_base @ rotz_90
print("Vector from pelvis to robot base (rotated):", pelvis_to_robot_base_rotated)
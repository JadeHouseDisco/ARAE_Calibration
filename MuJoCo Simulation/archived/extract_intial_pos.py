import mujoco
import numpy as np

# Load your model and data
model = mujoco.MjModel.from_xml_path("main_SN475_ARAE.xml")
data = mujoco.MjData(model)

# Initialize simulation
mujoco.mj_step(model, data)

# # Get joint world positions and joint positions
# joint_world_pos = data.xpos

# # Print body names, world positions, and joint positions
# for i in range(model.nbody):
#     # Get the body name using mj_id2name
#     body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
#     if body_name and "Link" in body_name:  # Only print if the body has a name
#         print(f"Body: {body_name}, World Position: {joint_world_pos[i]}")

# q1 
# q2 Link3 -> Link8
# q3 
# q4
# q5 

link7_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link7")
link3_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link3")
link1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Link1")

R7 = data.xmat[link7_id].reshape(3,3)
R3 = data.xmat[link3_id].reshape(3,3)
R1 = data.xmat[link1_id].reshape(3,3)

R_7_3 = R7.T @ R3
R_7_1 = R7.T @ R1

X7 = R7[:, 0]
Y7 = R7[:, 1]
Z7 = R7[:, 2]
X3 = R3[:, 0]
X1 = R1[:, 0]

X_7_3 = R_7_3[:, 0]
X_7_1 = R_7_1[:, 0]

q21 = np.arcsin(np.dot(X_7_3, Y7) / (np.linalg.norm(X_7_3) * np.linalg.norm(Y7)))
q1_1 = np.arcsin(np.dot(X_7_3, Z7) / (np.linalg.norm(X_7_3) * np.linalg.norm(Z7)))

q31 = np.arcsin(np.dot(X_7_1, Y7) / (np.linalg.norm(X_7_1) * np.linalg.norm(Y7)))
q1_2 = np.arcsin(np.dot(X_7_1, Z7) / (np.linalg.norm(X_7_1) * np.linalg.norm(Z7)))

# q21 = np.arccos(np.dot(X7, X3) / (np.linalg.norm(X7) * np.linalg.norm(X3)))
# q31 = np.arccos(np.dot(X7, X1) / (np.linalg.norm(X7) * np.linalg.norm(X1)))

print(q1_1)
print(q1_2)
print(q21)
print(q31)
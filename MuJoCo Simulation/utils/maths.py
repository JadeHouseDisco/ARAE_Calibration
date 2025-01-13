import numpy as np
import matplotlib.pyplot as plt

# Helper function for DH transformation
def dh_matrix(param):
    theta, alpha, a, d = param
    H = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return H

# Translation matrix
def Trans(x, y, z):
    T = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])
    return T

# Rotation matrix about z-axis
def Rotz(angle):
    Rz = np.array([
        [np.cos(angle), -np.sin(angle), 0, 0],
        [np.sin(angle), np.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return Rz

def circcirc(x1, y1, r1, x2, y2, r2):
    # Calculate the distance between the centers
    d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    # Check if circles intersect
    if d > (r1 + r2) or d < abs(r1 - r2) or d == 0:
        # No solution or infinite solutions
        return None, None
    
    # Calculate the distance from the first circle's center to the line joining the points of intersection
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = np.sqrt(r1**2 - a**2)
    
    # Calculate the midpoint between the intersection points
    x0 = x1 + a * (x2 - x1) / d
    y0 = y1 + a * (y2 - y1) / d
    
    # Calculate the intersection points
    x_inter1 = x0 + h * (y2 - y1) / d
    y_inter1 = y0 - h * (x2 - x1) / d
    
    x_inter2 = x0 - h * (y2 - y1) / d
    y_inter2 = y0 + h * (x2 - x1) / d
    
    return (x_inter1, y_inter1), (x_inter2, y_inter2)

# Transformation matrices
def transformation_matrix(i, PT):
    return np.array([
        [np.cos(PT[i, 0]), -np.sin(PT[i, 0]) * np.cos(PT[i, 1]), np.sin(PT[i, 0]) * np.sin(PT[i, 1]), PT[i, 2] * np.cos(PT[i, 0])],
        [np.sin(PT[i, 0]), np.cos(PT[i, 0]) * np.cos(PT[i, 1]), -np.cos(PT[i, 0]) * np.sin(PT[i, 1]), PT[i, 2] * np.sin(PT[i, 0])],
        [0, np.sin(PT[i, 1]), np.cos(PT[i, 1]), PT[i, 3]],
        [0, 0, 0, 1]
    ])

def extract_direction(pos):
    vector = np.array(pos) # Direction vector
    length = np.linalg.norm(vector)  # Length
    unit_direction = vector / length if length > 0 else np.zeros(3)  # Unit directiom
    return unit_direction


def extract_length(pos):
    vector = np.array(pos) # Direction vector
    length = np.linalg.norm(vector)  # Length
    return length
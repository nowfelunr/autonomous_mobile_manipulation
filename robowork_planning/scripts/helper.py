import numpy as np
import math
import json
import os

def calculate_new_point(x, y, z, qx, qy, qz, qw, distance_from_goal=1.0):


    p = np.array([x, y, z])
    # Compute rotation matrix from quaternion
    R = quaternion_to_rotation_matrix([qw, qx, qy, qz])
    
    # Extract Z-axis rotation angle (yaw) from the rotation matrix

    yaw = np.arctan2(R[1, 0], R[0, 0])

    # Create a new quaternion from the Z-axis rotation
    qw_new = np.cos(yaw / 2)
    qz_new = np.sin(yaw / 2)

    # New quaternion representing rotation around Z-axis only
    new_quaternion = (0, 0, qz_new, qw_new)

    # Direction vector in the XY plane
    direction_vector = np.array([np.cos(yaw), np.sin(yaw), 0])

    # Distance to move along the direction vector
    distance = distance_from_goal

    # Calculate the new point
    new_point = p + distance * direction_vector

    #print("Quaternion of the projection on the XY plane:", new_quaternion)
    #print("New x, y, z coordinate after moving:", new_point)

    return new_point, new_quaternion

# Convert quaternion to rotation matrix
def quaternion_to_rotation_matrix(q):
    qw, qx, qy, qz = q
    xx, yy, zz = qx**2, qy**2, qz**2
    xy, xz, yz, wx, wy, wz = qx*qy, qx*qz, qy*qz, qw*qx, qw*qy, qw*qz

    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
    ])



def distance_between_points(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)


class PointCustom:
    def __init__(self, x, y, z, qx, qy, qz, qw):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw



def log_status(log_entry):
    log_file = os.path.join(os.path.dirname(__file__), 'log.json')

    with open(log_file, 'w') as file:  
        json.dump(log_entry, file)
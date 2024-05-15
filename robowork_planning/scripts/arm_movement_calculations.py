
import math
import numpy as np
def distance_between_points(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
def quaternion_to_axis_angle(qx, qy, qz, qw):
    # Calculate the rotation angle
    # Ensure qw is within the range of [-1, 1] before calculating arc cosine
    angle = 2 * math.acos(qw)
    
    # Calculate the norm of the vector (qx, qy, qz)
    norm = math.sqrt(qx**2 + qy**2 + qz**2)
    
    # Handle the case when the norm is very small
    if norm < 1e-8:  
        return angle, (0, 0, 0)
    
    # Normalize the axis vector
    axis_x = qx / norm
    axis_y = qy / norm
    axis_z = qz / norm
    
    return angle, (axis_x, axis_y, axis_z)

def adjust_angle_and_create_quaternion(axis, angle):
    # Calculate the new angle
    new_angle = angle + math.pi
    
    # Unpack the axis components
    u_x, u_y, u_z = axis
    
    # Calculate the new quaternion components
    new_qw = math.cos(new_angle / 2)
    sin_half_new_angle = math.sin(new_angle / 2)
    new_qx = sin_half_new_angle * u_x
    new_qy = sin_half_new_angle * u_y
    new_qz = sin_half_new_angle * u_z
    
    # Return the new quaternion
    return (new_qx, new_qy, new_qz, new_qw)


def get_arm_placement_quaternion(qx, qy, qz, qw):
    # Calculate the rotation angle and axis
    angle, axis = quaternion_to_axis_angle(qx, qy, qz, qw)
    
    # Adjust the angle and create the new quaternion
    new_quaternion = adjust_angle_and_create_quaternion(axis, angle)
    
    return new_quaternion

def quaternion_to_rotation_matrix(qw, qx, qy, qz):
    # Normalize the quaternion to ensure the rotation matrix is orthogonal
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    # Compute the rotation matrix elements
    r11 = 1 - 2*(qy**2 + qz**2)
    r12 = 2*(qx*qy - qz*qw)
    r13 = 2*(qx*qz + qy*qw)

    r21 = 2*(qx*qy + qz*qw)
    r22 = 1 - 2*(qx**2 + qz**2)
    r23 = 2*(qy*qz - qx*qw)

    r31 = 2*(qx*qz - qy*qw)
    r32 = 2*(qy*qz + qx*qw)
    r33 = 1 - 2*(qx**2 + qy**2)

    # Form the rotation matrix
    rotation_matrix = np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ])

    return rotation_matrix



def rotate_vector(rotation_matrix, vector):
    # Ensure the input vector is a numpy array
    vector = np.array(vector)
    
    # Ensure the input vector is a column vector (3x1)
    if len(vector.shape) == 1:
        vector = vector.reshape((3, 1))

    # Perform matrix multiplication to rotate the vector
    rotated_vector = np.dot(rotation_matrix, vector)

    # Make sure the rotated vector is a 1D array
    rotated_vector = rotated_vector.flatten()
    # Return the rotated vector
    return rotated_vector

def move_point_by_vector(point, vector, distance):
    # Ensure the input point and vector are numpy arrays
    point = np.array(point)
    vector = np.array(vector)
    
    # Calculate the new point
    new_point = point + distance * vector
    
    return new_point

def get_arm_placement_point(x,y,z,qx,qy,qz,qw,distance_from_goal=0.1):
    rotation_matrix = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    new_vector = rotate_vector(rotation_matrix, [1, 0, 0])
    new_point = move_point_by_vector([x, y, z], new_vector, distance_from_goal)
    
    return new_point



def rostopic_arm_command(x, y, z, qx, qy, qz, qw):
    """
    Print a ROS topic command to publish the pose of the end effector.

    Parameters:
    x (float): X coordinate of the position
    y (float): Y coordinate of the position
    z (float): Z coordinate of the position
    qx (float): X component of the quaternion
    qy (float): Y component of the quaternion
    qz (float): Z component of the quaternion
    qw (float): W component of the quaternion
    """
    command = ("rostopic pub --once -s /endeffector_goal_pose geometry_msgs/PoseStamped '"
               "{\"header\": {\"seq\": 0, \"stamp\": \"now\", \"frame_id\": \"map\"}, "
               "\"pose\": {\"position\": {\"x\": " + str(x) + ", \"y\": " + str(y) + 
               ", \"z\": " + str(z) + "}, \"orientation\": {\"x\": " + str(qx) +
               ", \"y\": " + str(qy) + ", \"z\": " + str(qz) + ", \"w\": " + str(qw) +
               "}}}'")
    print(command)
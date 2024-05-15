import numpy as np
import math
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


def compute_euclidian_distance(point1, point2):
    """Compute the Euclidean distance between two points."""
    return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)

def calculate_new_point_for_base(x, y, z, qx, qy, qz, qw, distance_from_goal=1.0):


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




def rostopic_base_movement_command(x, y, z, qx, qy, qz, qw):
    # rostopic pub --once -s /rviz_2d_nav_goal geometry_msgs/PoseStamped "{header: {seq: 0, stamp: now, frame_id: 'map'}, pose: {position: {x: 1.31780139, y: -0.9729503, z: 0.38229873}, orientation: {x: 0, y: 0, z: 0.3831361547189317, w: 0.923691878792485}}}"
    command = ("rostopic pub --once -s /rviz_2d_nav_goal geometry_msgs/PoseStamped '"
                "{\"header\": {\"seq\": 0, \"stamp\": \"now\", \"frame_id\": \"map\"}, "
                "\"pose\": {\"position\": {\"x\": " + str(x) + ", \"y\": " + str(y) + 
                ", \"z\": " + str(z) + "}, \"orientation\": {\"x\": " + str(qx) +
                ", \"y\": " + str(qy) + ", \"z\": " + str(qw) + ", \"w\": " + str(-qz) +
                "}}}'")
    print(command)



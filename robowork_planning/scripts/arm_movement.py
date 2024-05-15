import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys
import math

# moveit_commander.roscpp_initialize(sys.argv)

# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
# group_name = "main_arm_SIM"
# move_group = moveit_commander.MoveGroupCommander(group_name)
def move_arm(x, y, z, qx, qy, qz, qw):
    """
    Publish a ROS topic command to move the robotic arm to a specified pose using rospy.

    Parameters:
    x (float): X coordinate of the position
    y (float): Y coordinate of the position
    z (float): Z coordinate of the position
    qx (float): X component of the quaternion
    qy (float): Y component of the quaternion
    qz (float): Z component of the quaternion
    qw (float): W component of the quaternion
    """
    # Initialize the ROS node (if not already initialized)
   
    # Create a publisher object
    pub = rospy.Publisher('/endeffector_goal_pose', PoseStamped, queue_size=10)
    
    # Wait for publisher to get fully set up (important in networks with delays)
    rospy.sleep(1)

    # Create a PoseStamped message
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    # Publish the message
    pub.publish(pose)

    # Sleep to give the subscriber time to process the message
    rospy.sleep(1)


def move_arm_to_home(move_group):
     # Initialize moveit_commander and a rospy node
    # moveit_commander.roscpp_initialize(sys.argv)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "main_arm_SIM"
    # move_group = moveit_commander.MoveGroupCommander(group_name)

    move_group.set_named_target("home")
    plan = move_group.plan()  
    success = move_group.go(wait=True)  # Execute the plan

    move_group.stop()
    move_group.clear_pose_targets()

def quaternion_angular_difference(q1, q2):
    """Calculate the angular difference between two quaternions."""
    dot_product = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    # Clamp the dot product to avoid math domain errors in acos and ensure shortest rotation path
    dot_product = max(min(abs(dot_product), 1.0), -1.0)
    return 2 * math.acos(dot_product)

def are_poses_similar(pose1, pose2, pos_tolerance, ori_tolerance):
    """
    Check if two poses are similar within given position and orientation tolerances.
    
    Parameters:
    pose1 (Pose): First pose to compare.
    pose2 (Pose): Second pose to compare.
    pos_tolerance (float): Tolerance for position difference (in meters).
    ori_tolerance (float): Tolerance for orientation difference (in radians).

    Returns:
    bool: True if poses are similar within the given tolerances, False otherwise.
    """
    # Calculate position difference
    pos_diff = math.sqrt((pose1.position.x - pose2.position.x) ** 2 +
                         (pose1.position.y - pose2.position.y) ** 2 +
                         (pose1.position.z - pose2.position.z) ** 2)

    # Calculate orientation difference
    ori_diff = quaternion_angular_difference(pose1.orientation, pose2.orientation)

    # Check if both position and orientation differences are within tolerances
    return pos_diff <= pos_tolerance and ori_diff <= ori_tolerance
   
    




def monitor_arm(move_group, initial_pose):
    total_elapsed_time = 0
    # moveit_commander.roscpp_initialize(sys.argv)

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "main_arm_SIM"
    # move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get the current joint values for later use
    # current_joints = move_group.get_current_joint_values()

    # Check if the current position is the same as the goal position
    # initial_pose = move_group.get_current_pose().pose
    # new_pose = move_group.get_current_pose().pose
    # while total_elapsed_time < 10:
        
    #     new_pose = move_group.get_current_pose().pose
    #     print("Is similar pose: ", are_poses_similar(initial_pose, new_pose, 0.05, 0.1))
    #     if not are_poses_similar(initial_pose, new_pose, 0.05, 0.1)  and total_elapsed_time > 8:
    #         rospy.loginfo("The arm has been moved successfully.")
    #         return True
        
    #     total_elapsed_time += 1
    #         # if total_elapsed_time > 10:
    #         #     return False
    #     rospy.sleep(1)
    rospy.sleep(10)
    new_pose = move_group.get_current_pose().pose

    # Extra add
    move_group.stop()
    move_group.clear_pose_targets()

    if not are_poses_similar(initial_pose, new_pose, 0.05, 0.1):
        rospy.loginfo("The arm has been moved successfully.")
        return True
    rospy.loginfo("Could not move the arm.")
    return False
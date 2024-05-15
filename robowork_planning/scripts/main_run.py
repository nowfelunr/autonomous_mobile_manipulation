#!/usr/bin/env python
import sys
import rospy

from geometry_msgs.msg import PoseStamped, Point
import numpy as np
from nav_msgs.msg import Odometry
import csv
from base_movement_calculations import compute_euclidian_distance
from base_movement import move_backward
from helper import distance_between_points, PointCustom, log_status
from base_movement_calculations import calculate_new_point_for_base, rostopic_base_movement_command
from arm_movement_calculations import get_arm_placement_point, get_arm_placement_quaternion, distance_between_points, rostopic_arm_command
from arm_movement import move_arm, monitor_arm, move_arm_to_home
import moveit_commander
import os
from tsp import nearest_neighbor
import time
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "main_arm_SIM"
move_group = moveit_commander.MoveGroupCommander(group_name)



arm_forward_distance = 0.5
# Global variable to store the robot's current position
current_position = Point()
reached_goal = False

# Callback function for the robot's position
def position_callback(data):
    global current_position
    current_position = data.pose.pose.position


def monitor_distance(goal_pose, check_rate):
    global reached_goal
    reached_goal = False
    """Monitors distance to the goal at a set interval and prints it."""
    rospy.Subscriber('/bvr_SIM/odom', Odometry, position_callback)
    rate = rospy.Rate(check_rate)  # Set the rate of checking as per 'check_rate' Hz

    last_three_distances = []

    while not reached_goal and not rospy.is_shutdown():
        distance_to_goal = compute_euclidian_distance(current_position, goal_pose.pose.position)
        last_three_distances.append(round(distance_to_goal, 2))
        if len(last_three_distances) == 3:
            rospy.loginfo("Last five distances: {}".format(last_three_distances))
            if len(set(last_three_distances)) == 1:
                reached_goal = True
                last_three_distances = []
                rospy.loginfo("Reached the goal!")
               
                # if arm_movement:
                #     move_arm()
            else:
                last_three_distances = last_three_distances[1:]
        
        rospy.loginfo("Distance to goal: {:.2f} meters".format(distance_to_goal))
        rate.sleep()

def move_robot_to_point(x, y, z, qx, qy, qz, qw, arm_movement = True):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    goal_pose.pose.orientation.x = qx
    goal_pose.pose.orientation.y = qy
    goal_pose.pose.orientation.z =  qw
    goal_pose.pose.orientation.w = -qz
    
    # Publish the goal position
    goal_pub = rospy.Publisher('/rviz_2d_nav_goal', PoseStamped, queue_size=1)
    rospy.sleep(1)  # Sleep to ensure the publisher is established
    goal_pub.publish(goal_pose)

    # Start monitoring the distance to the goal, checking every 1 second
    monitor_distance(goal_pose, check_rate=1)


def is_valid_point(x, y):
    if x > 2:
        if y < 1 and y > - 1:
            return False
    
    return True
        
full_log = []


if __name__ == '__main__':
    try:
        rospy.init_node('main_run')
        csv_path = os.path.join(os.path.dirname(__file__), 'boat.csv')
        csv_file = open(csv_path, 'r')
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader)  # Skip the header row

        all_points = {}
        visited_points = []

        # Load the points from the CSV file to the all_points dictionary. The key is the index of the point
        # and the value is the Point object
        for i, row in enumerate(csv_reader):
            x, y, z, qx, qy, qz, qw = row
            x, y, z, qx, qy, qz, qw = float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)
            all_points[i] = PointCustom(x, y, z, qx, qy, qz, qw)

        while all_points.keys():
            # Seed the random number generator
            np.random.seed(int(time.time()))
            # Select a random point from the all_points dictionary
            random_point_index = np.random.choice(list(all_points.keys()))
            random_point = all_points[random_point_index]

            # print(f"Random point: {random_point}")
            rospy.loginfo("Random point: {}".format(random_point))
            current_point = dict()
            current_point['cluser_points'] = []
            current_point['x'] = random_point.x
            current_point['y'] = random_point.y
            current_point['z'] = random_point.z
            current_point['qx'] = random_point.qx
            current_point['qy'] = random_point.qy
            current_point['qz'] = random_point.qz
            current_point['qw'] = random_point.qw
            current_point['index'] = random_point_index 
            

            base_moving_destination = calculate_new_point_for_base(random_point.x, random_point.y, random_point.z, random_point.qx, random_point.qy, random_point.qz, random_point.qw, distance_from_goal=1.0)
            base_moving_point, base_moving_quaternion = base_moving_destination
            rospy.loginfo("Base moving point:".format(base_moving_point))
            x_1, y_1, z_1 = base_moving_point
            qx_1, qy_1, qz_1, qw_1 = base_moving_quaternion
            if not is_valid_point(x_1, y_1):
                rospy.loginfo("Invalid point. {}. Skipping...".format(base_moving_point))
                current_point['status'] = False
                continue
            current_point['status'] = True
            #nearby_points = {key: value for key, value in all_points.items() if distance_between_points(PointCustom(base_moving_point[0], base_moving_point[1], base_moving_point[2], 0, 0, 0, 0), value) <= 1.}
            nearby_points = {key: value for key, value in all_points.items() if distance_between_points(PointCustom(base_moving_point[0], base_moving_point[1], base_moving_point[2], 0, 0, 0, 0), value) <= 1.}
            
            # pre append base_moving_point to nearby_points
            nearby_points[-1] = PointCustom(base_moving_point[0], base_moving_point[1], base_moving_point[2], 0, 0, 0, 0)

            nearby_points = nearest_neighbor(nearby_points)

            # Delete the point from nearby points where index is -1
            del nearby_points[-1]

            # Sort the nearby points by distance from the random_point
            # sorted_nearby_points = sorted(nearby_points.items(), key=lambda x: distance_between_points(random_point, x[1]))
            
            rospy.loginfo("Total nearby points: {}".format(len(nearby_points)))
            current_point['total_cluster_points'] = len(nearby_points)
            full_log.append(current_point)
            log_status(full_log)
            move_robot_to_point(x_1, y_1, z_1, qx_1, qy_1, qz_1, qw_1, arm_movement=False)
            rospy.loginfo("Y = {}".format(y_1))
            
            
            print(type(nearby_points))
            for point_index, point in nearby_points:
                current_cluster_point = dict()
                current_cluster_point['x'] = point.x
                current_cluster_point['y'] = point.y
                current_cluster_point['z'] = point.z
                current_cluster_point['qx'] = point.qx
                current_cluster_point['qy'] = point.qy
                current_cluster_point['qz'] = point.qz
                current_cluster_point['qw'] = point.qw
                current_cluster_point['index'] = point_index
                current_cluster_point['status'] = 'Attempting...'
                current_point['cluser_points'].append(current_cluster_point)

                # update in the full log
                full_log[-1] = current_point
                log_status(full_log)

                robot_arm_quaternion = get_arm_placement_quaternion(point.qx, point.qy, point.qz, point.qw)
                robot_arm_point = get_arm_placement_point(point.x, point.y, point.z, point.qx, point.qy, point.qz, point.qw, distance_from_goal=.3)
                rospy.loginfo("Arm placement point: {}".format(robot_arm_point))
                rospy.loginfo("Arm placement quaternion: {}".format(robot_arm_quaternion))
                
                initial_arm_pose = move_group.get_current_pose().pose
                move_arm(robot_arm_point[0], robot_arm_point[1], robot_arm_point[2], robot_arm_quaternion[0], robot_arm_quaternion[1], robot_arm_quaternion[2], robot_arm_quaternion[3])
                # rostopic_arm_command(robot_arm_point[0], robot_arm_point[1], robot_arm_point[2], robot_arm_quaternion[0], robot_arm_quaternion[1], robot_arm_quaternion[2], robot_arm_quaternion[3])
                # monitor_arm(robot_arm_point[0], robot_arm_point[1], robot_arm_point[2], robot_arm_quaternion[0], robot_arm_quaternion[1], robot_arm_quaternion[2], robot_arm_quaternion[3]) 
                
                log_status(full_log)
                
                arm_successful = monitor_arm(initial_pose=initial_arm_pose, move_group=move_group)
                if arm_successful:
                    current_cluster_point['status'] = 'Reached'
                    all_points.pop(point_index)
                else:
                    current_cluster_point['status'] = 'Unreachable'
                current_point['cluser_points'][-1] = current_cluster_point
                full_log[-1] = current_point

                log_status(full_log)
                rospy.sleep(1)




           

            
            # print(f"Base moving point: {base_moving_point}")
           
            # print("\n")

            move_arm_to_home(move_group=move_group)
            move_backward()
            

            rospy.sleep(1)
 
        
    except rospy.ROSInterruptException:
        pass

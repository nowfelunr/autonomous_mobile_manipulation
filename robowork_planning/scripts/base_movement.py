import rospy
from geometry_msgs.msg import Twist

def move_backward():
   
    # Create a publisher object
    pub = rospy.Publisher('/bvr_SIM/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish the message
    rate = rospy.Rate(10)  # 10 Hz

    # Create a Twist message instance
    move_cmd = Twist()

    # Set linear velocity (m/s)
    move_cmd.linear.x = -0.5  # Move forward at 0.5 m/s
    move_cmd.angular.z = 0.0  # No angular velocity

    # Give ROS time to setup the publisher
    rospy.sleep(1)

    # Publish the message for a few seconds
    for i in range(30):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop the robot
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
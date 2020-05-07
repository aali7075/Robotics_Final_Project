import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import argparse



turtle_pose = None
rel_x_pos = None
interim_positions = [[30.5,17.5,1],[19.5,29,1],[32,29,1],[34,-7,1],[19,2.5,1],
[28.5,-7,1],[15,-1,1],[-5.5,-8.5,1],[10.5,-6,1],[10.5,-1.5,1],[15,2.5,1],[15,6.5,1],
[30.5,12.5,1],[30.5,6.5,1],[18.5,9,1],[-8.5,-8.5,1],[1.5,6.5,1],[-7.5,22.5,1],[-3.5,22.5,1],[6.5,17.5,1],[10.5,18,1]]
interim_positions=interim_positions[::-1]

def callback_update_odometry(data):
    # Receives nav_msgs/Odometry.msg
    global turtle_pose
    turtle_pose = data

def callback_get_ball_rel_pos(data):
    global rel_x_pos
    blob_msg = data
    rel_x_pos =  blob_msg.x

def main():
    global turtle_pose
    test = Point()

    rospy.init_node("goal_node")
    goal_pub = rospy.Publisher("/tb3_0/move_base_simple/goal", PoseStamped, queue_size=10)
    velo_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
    odom_sub= rospy.Subscriber('/tb3_0/odom', Odometry, callback_update_odometry)
    obj_detect = rospy.Subscriber("/tb3_0/rel_ball_loc/", Point,  callback_get_ball_rel_pos)
    rospy.sleep(1)

    print("working")
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()

    current_goal = interim_positions.pop()
    #make sure there are still goal positions to go to
    while(current_goal):
        p.pose.position.x = float(current_goal[0])
        p.pose.position.y = float(current_goal[1])
        p.pose.orientation.w = float(current_goal[2]) # 2D so it only has x,y,theta. w is theta

        goal_pub.publish(p)
        print(turtle_pose.pose.pose.position.x)

        while(turtle_pose.pose.pose.position.x!=current_goal[0] and turtle_pose.pose.pose.position.y!=current_goal[1] and turtle_pose.pose.pose.orientation.w!=current_goal):
            if rel_x_pos:
                #stop turtlebot
                p = PoseStamped()
                p.header.frame_id = "/map"
                p.header.stamp = rospy.Time.now()
                p.pose.position.x = turtle_pose.pose.pose.position.x
                p.pose.position.y = turtle_pose.pose.pose.position.y
                p.pose.orientation.w = turtle_pose.pose.pose.orientation.w
                goal_pub.publish(p)

                #wait for robot to get into position and stop
                rospy.sleep(10)

                vel_msg = Twist()
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                while(rel_x_pos < 0.1 and rel_x_pos > -0.1):
                    if rel_x_pos > 0:
                        vel_msg.angular.z = -0.1
                    if rel_x_pos < 0:
                        vel_msg.angular.z = 0.1
                    velo_pub.publish(vel_msg)

                vel_msg.angular.z = 0
                velo_pub.publish(vel_msg)
                break

        current_goal = interim_positions.pop()
if __name__ == "__main__":
    main()

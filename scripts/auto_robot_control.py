import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from actionlib_msgs.msg import GoalID, GoalStatusArray
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import argparse

ball_pos = [8.8,-1.7]

turtle_pose = None
rel_x_pos = None
goal_status = None
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

def callback_get_goal_status(data):
    global goal_status
    goal_status = data

def main():
    global turtle_pose
    global rel_x_pos
    global goal_status

    print("working")
    # p = PoseStamped()
    # p.header.frame_id = "map"
    # p.header.stamp = rospy.Time.now()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"

    current_goal = interim_positions.pop()
    #make sure there are still goal positions to go to

    while(current_goal):
        # p.pose.position.x = float(current_goal[0])
        # p.pose.position.y = float(current_goal[1])
        # p.pose.orientation.w = float(current_goal[2]) # 2D so it only has x,y,theta. w is
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(current_goal[0])
        goal.target_pose.pose.position.y = float(current_goal[1])
        goal.target_pose.pose.orientation.w = float(current_goal[2])

        # goal_pub.publish(p)
        client.send_goal(goal)
        print("First goal published")

        while(turtle_pose.pose.pose.position.x!=current_goal[0] and turtle_pose.pose.pose.position.y!=current_goal[1] and turtle_pose.pose.pose.orientation.w!=current_goal):
            if rel_x_pos:
                print("ball spotted")
                #stop turtlebot
                #doesnt fucking work
                # p.header.stamp = rospy.Time.now()
                # p.pose.position.x = turtle_pose.pose.pose.position.x * 1.2
                # p.pose.position.y = turtle_pose.pose.pose.position.y * 1.2
                # p.pose.orientation.w = turtle_pose.pose.pose.orientation.w  * 1.2
                # goal_pub.publish(p)
                # rospy.sleep(30)

                c = GoalID()
                cancel_pub.publish(c)
                print("TB OG navgoal stopped")

                vel_msg = Twist()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                while(rel_x_pos):
                    thresh = .75
                    turtle_x = turtle_pose.pose.pose.position.x
                    turtle_y = turtle_pose.pose.pose.position.y
                    print(turtle_x,turtle_y)
                    if (turtle_x > ball_pos[0] - thresh
                            and turtle_x < ball_pos[0] + thresh
                            and turtle_y > ball_pos[1] - thresh
                            and turtle_y < ball_pos[1] + thresh):
                        print("Ball reached")
                        vel_msg = Twist()
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.linear.x = 0
                        vel_msg.angular.x = 0
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = 0
                        velo_pub.publish(vel_msg)
                        return
                    print(rel_x_pos)
                    if(rel_x_pos > 0.1 or rel_x_pos < -0.1):
                        while(rel_x_pos > 0.1) :
                            print("Rotating clockwise")
                            print(turtle_x,turtle_y)
                            vel_msg.angular.z = -0.1
                            vel_msg.linear.x = 0
                            velo_pub.publish(vel_msg)
                        while(rel_x_pos < -0.1):
                            print("Rotating counter clockwise")
                            print(turtle_x,turtle_y)
                            vel_msg.angular.z = 0.1
                            vel_msg.linear.x = 0
                            velo_pub.publish(vel_msg)
                    else:
                        print("Going Straight")
                        vel_msg.angular.z = 0
                        vel_msg.linear.x = .5
                        velo_pub.publish(vel_msg)

                vel_msg = Twist()
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.linear.x = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0
                velo_pub.publish(vel_msg)
                return

        current_goal = interim_positions.pop()
if __name__ == "__main__":
    rospy.init_node("auto_robo_node")
    goal_pub = rospy.Publisher("/tb3_0/move_base_simple/goal", PoseStamped, queue_size=10)
    velo_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
    cancel_pub = rospy.Publisher("/tb3_0/move_base/cancel", GoalID, queue_size=10)
    odom_sub= rospy.Subscriber('/tb3_0/odom', Odometry, callback_update_odometry)
    obj_detect_sub = rospy.Subscriber("/tb3_0/rel_ball_loc/", Point,  callback_get_ball_rel_pos)
    client = actionlib.SimpleActionClient('/tb3_0/move_base',MoveBaseAction)
    client.wait_for_server()
    goal_status_sub = rospy.Subscriber("/tb3_0/move_base/status", GoalStatusArray, callback_get_goal_status)
    rospy.sleep(1)
    main()

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
import signal
import subprocess
import argparse
import os

comp_odom = None
human_odom = None
#x and y coords of ball
ball_coords = [8.8,-1.7]

def callback_update_odometry_0(data):
    # Receives nav_msgs/Odometry.msg
    global comp_odom
    comp_odom = data

def callback_update_odometry_1(data):
    # Receives nav_msgs/Odometry.msg
    global human_odom
    human_odom = data

def monitor():
    global comp_odom
    global human_odom

    while(True):
        if(comp_odom):
            if(comp_odom.pose.pose.position.x > 8.3 and comp_odom.pose.pose.position.x < 9.3):
                if(comp_odom.pose.pose.position.y > -2.2 and comp_odom.pose.pose.position.y < -1.2):
                    end = rospy.Time()
                    total = begin-end
                    total = total.to_sec()

                    img = cv2.imread("you_lose.png")
                    height = img.shape[0]
                    width = img.shape[1]
                    y = int(height*.15)
                    x = int(width*.5)
                    txt = "The robot took " + str(total) + " to find the ball"
                    cv2.putText(img, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,), 2)
                    cv2.imshow("You Lose", img)

                    return

        if(human_odom):
            if(human_odom.pose.pose.position.x > 8.3 and human_odom.pose.pose.position.x < 9.3):
                if(human_odom.pose.pose.position.y > -2.2 and human_odom.pose.pose.position.y < -1.2):
                    end = rospy.Time()
                    total = begin-end
                    total = total.to_sec()

                    img = cv2.imread("you_win.jpg")
                    height = img.shape[0]
                    width = img.shape[1]
                    y = height*.15
                    x = width*.5
                    txt = "You took " + str(total) + " to find the ball"
                    cv2.putText(img, txt, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)
                    cv2.imshow("You Win", img)

                    return


if __name__ == '__main__':
    dir_path = os.path.dirname(os.path.realpath(__file__))
    ball_path = dir_path + "/ball_detect.py"
    control_path = dir_path + "/auto_robot_control.py"

    pro1 = subprocess.Popen(["python",ball_path])
    pro2 = subprocess.Popen(["python",control_path])

    begin= rospy.Time()
    rospy.init_node("winner_monitor")
    computer_odom_sub= rospy.Subscriber('/tb3_0/odom', Odometry, callback_update_odometry_0)
    human_odom_sub= rospy.Subscriber('/tb3_1/odom', Odometry, callback_update_odometry_1)
    monitor()
    os.killpg(os.getpgid(pro1.pid), signal.SIGTERM)
    os.killpg(os.getpgid(pro2.pid), signal.SIGTERM)
    cv2.waitKey(0)

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import argparse



turtle_pose = None

def callback_update_odometry(data):
    # Receives nav_msgs/Odometry.msg
    global turtle_pose
    turtle_pose = data


def main(args):
    global turtle_pose
    rospy.init_node("goal_node")
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    odom_sub= rospy.Subscriber('/odom', Odometry, callback_update_odometry)
    rospy.sleep(1)

    print("working")
    p = PoseStamped()
    p.header.frame_id = "/map"
    p.header.stamp = rospy.Time.now()

    p.pose.position.x = float(args.x[0])
    p.pose.position.y = float(args.y[0])

    p.pose.orientation.w = float(args.theta[0]) # 2D so it only has x,y,theta. w is theta


    print(turtle_pose.pose.pose.position.y)

    if(p.pose.position.y==-100000):
        print("working")
        p.pose.position.y = turtle_pose.pose.pose.position.y
    if(p.pose.position.x==-100000):
        p.pose.position.x = turtle_pose.pose.pose.position.x
    if(p.pose.orientation.w==-100000):
        p.pose.orientation.w = turtle_pose.pose.pose.orientation.w

    print(p.pose.position.x, p.pose.position.y, p.pose.orientation.w)


    goal_pub.publish(p)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Turtle Simulation Environment")
    #TAKE THE LAST POSITION OF THE ROBOT FIND THE ODOM OF THE TURTLE
    parser.add_argument('-x','--x', nargs=1, default=[-100000], help='Starting x of Turtle in world coordinates')
    parser.add_argument('-y','--y', nargs=1, default=[-100000], help='Starting y of Turtle in world coordinates')
    parser.add_argument('-theta','--theta', nargs=1, default=[-100000], help='Starting theta of Turtle in world coordinates')

    args = parser.parse_args()

    main(args)

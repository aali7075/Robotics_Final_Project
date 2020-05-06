from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import rospy
import numpy as np

l_b_1 = np.array([0, 200, 94])
u_b_1 = np.array([10, 255, 140])
l_b_2 = np.array([170, 200, 94])
u_b_2 = np.array([255, 255, 140])

def do_blob_detection(mask):
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 20000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia =True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)
    blobs = detector.detect(mask)

    return blobs

def get_relative_pos(image, keypoints):
    points2f =	cv2.KeyPoint_convert(keypoints)
    print(points2f)

    rows = float(image.shape[0])
    cols = float(image.shape[1])
    print(rows, cols)

    center_x    = 0.5*cols
    center_y    = 0.5*rows

    x = (points2f[0,0]- center_x)/(center_x)
    y = (points2f[0,1] - center_y)/(center_y)
    return(x,y)

def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as error:
        print(error)

    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #red overlaps between 170 and 10 in the negative direction so need two masks
    mask1 = cv2.inRange(hsv_image, l_b_1, u_b_1)
    mask2 = cv2.inRange(hsv_image, l_b_2, u_b_2)
    full_mask = mask1 + mask2
    rev_mask = 255-full_mask

    #do blob detection and draw around ball in image
    keypoints =  do_blob_detection(rev_mask)
    im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # cv2.imshow("blob", im_with_keypoints)
    # cv2.waitKey(0)

    #publish ball point relative to center
    x,y = get_relative_pos(cv_image,keypoints)
    blob_msg = Point()
    blob_msg.x = x
    blob_msg.y = y
    image_pub.publish(blob_msg)

if __name__ == '__main__':
    rospy.init_node('ball_detection', anonymous=True)
    image_pub = rospy.Publisher("/turtlebot3_waffle_pi/ball_pos",Point,queue_size=1)
    image_sub = rospy.Subscriber("/turtlebot3_waffle_pi/camera1/image_raw", Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()

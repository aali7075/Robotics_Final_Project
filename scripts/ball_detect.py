from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import rospy
import numpy as np

l_b_1 = np.array([0, 200, 80])
u_b_1 = np.array([10, 255, 160])
l_b_2 = np.array([170, 200, 80])
u_b_2 = np.array([255, 255, 160])

def do_blob_detection(mask):
    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 200000000

    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by Convexity
    params.filterByConvexity = False
    params.minConvexity = 0.5

    # Filter by Inertia
    params.filterByInertia =False
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)
    blobs = detector.detect(mask)

    return blobs

def get_relative_pos(image, keypoints):
    points2f =	cv2.KeyPoint_convert(keypoints)

    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(cols, rows)

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
    if keypoints:

        #publish ball point relative to center negative for left, positive for right - scaled to one
        x,y = get_relative_pos(cv_image,keypoints)
        print("publishing: ",x,y)
        blob_msg = Point()
        blob_msg.x = x
        blob_msg.y = y
        image_pub.publish(blob_msg)

    #mask and image with keypoints
    im_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("mask", rev_mask)
    cv2.imshow("blob", im_with_keypoints)
    cv2.waitKey(2)


if __name__ == '__main__':
    rospy.init_node('ball_detection', anonymous=True)
    image_pub = rospy.Publisher("/tb3_0/rel_ball_loc/",Point,queue_size=1)
    image_sub = rospy.Subscriber("/tb3_0/camera/rgb/image_raw", Image, callback)
    rospy.spin()
    cv2.destroyAllWindows()

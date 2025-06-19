#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class BallDetector:
    def __init__(self):
        rospy.init_node('ball_detector')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/target_detected', Bool, queue_size=1)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.show = rospy.get_param('~show_image', True)

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"cv_bridge error: {e}")
            return

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
            param1=50, param2=20, minRadius=10, maxRadius=80)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
            rospy.loginfo("[Detector] Ball detected!")
            self.pub.publish(True)
        else:
            rospy.loginfo("[Detector] No circle detected")
            self.pub.publish(False)

        if self.show:
            cv2.imshow("Ball Detection", img)
            cv2.waitKey(1)

if __name__ == '__main__':
    try:
        BallDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

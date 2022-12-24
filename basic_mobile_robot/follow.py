#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CAMERA_TOPIC = "/camera/image_raw"


class ImageViewer(Node):
    def __init__(self):
        super().__init__("viewer")
        self.image_sub = self.create_subscription(Image, CAMERA_TOPIC, self.callback, 10)
        self.bridge = CvBridge()

    def find_line(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 10,  10,  10])
        upper_yellow = np.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # BEGIN CROP
        h, w, d = frame.shape
        search_top = int(3*h/4)
        search_bot = int(search_top + 60)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # END CROP
        # BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        # END FINDER
        # BEGIN CIRCLE
        return cx, cy
        
        # END CIRCLE

    def callback(self,data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cx, cy = self.find_line(frame)
            cv2.circle(frame, (cx, cy), 20, (0,0,255), -1)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("image", frame)
        cv2.waitKey(3)

def main(args=None):
    try:
        rclpy.init()
        image_converter = ImageViewer()
        rclpy.spin(image_converter)
        image_converter.destroy_node
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
if __name__ == '__main__':   
    main()

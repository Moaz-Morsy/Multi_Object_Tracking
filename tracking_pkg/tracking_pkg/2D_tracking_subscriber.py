#!/usr/bin/env python3

import sys

CENTERTRACK_PATH = "/home/moaz/Desktop/test_tracking/CenterTrack/src/lib"
sys.path.append(CENTERTRACK_PATH)

ENV_PATH = "/home/moaz/anaconda3/envs/CenterTrack/lib"
sys.path.append(ENV_PATH)

import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import time

from detector import Detector
from opts import opts

# Instantiate CvBridge
bridge = CvBridge()

class my_node (Node):
    def __init__(self):
        super().__init__("track_2D_node")
        self.get_logger().info("2D tracking subscriber is started")

        MODEL_PATH = "/home/moaz/Desktop/test_tracking/CenterTrack/models/coco_tracking.pth"
        TASK = "tracking"
        DATASET = "coco"
        THRESHOLD = 0.1
        self.opt = opts().init("{} --load_model {} --dataset {} --track_thresh {} --debug 2".format(TASK, MODEL_PATH,DATASET,THRESHOLD).split(' '))
        self.detector = Detector(self.opt)
        self.detector.pause = False

        self.new_frame_time = 0
        self.fps_list = []

        self.create_subscription(Image,"/zed2/zed_node/rgb/image_rect_color",self.img_cb, rclpy.qos.qos_profile_sensor_data)

    # def nothing(self,x):
    #     pass

    def img_cb(self,message):
        self.new_frame_time = time.time()

        frame = bridge.imgmsg_to_cv2(message, "bgr8")
        cv2.imshow("input", frame)

        ret = self.detector.run(frame)['results']

        fps = int(1/(time.time()-self.new_frame_time))
        self.fps_list.append(fps)
        print(f"FPS is {int(sum(self.fps_list)/len(self.fps_list))}")

        k = cv2.waitKey(1)

          
def main (args=None):
    rclpy.init(args=args)
    node=my_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()



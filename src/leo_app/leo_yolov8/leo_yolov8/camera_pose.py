#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

import numpy as np

from time import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO

class YoloDetection(Node):
# 初始化函数，包括加载模型和创建订阅者
    def __init__(self):
        # 加载模型
        pt_path = sys.path[0]
        self.model = YOLO(pt_path + '/config/yolov8n-pose.pt')
        # model_path = os.path.join('/home/leo/leo_agv/src/leo_app/leo_yolov8/yolov8n-pose.pt') # 构造模型文件的绝对路径
        # self.model = YOLO(model_path)
        # self.model = YOLO("yolov8n-pose.pt")
        
        # 创建订阅者
        super().__init__('Yolo')
        # 创建日志记录器
        self.logger = get_logger("yolov8")
        rgb_topic = self.declare_parameter("rgb_topic_name").get_parameter_value().string_value
        depth_topic = self.declare_parameter("depth_topic_name").get_parameter_value().string_value
        self.image_sub = self.create_subscription(Image, rgb_topic, self.detect, 10)
        
        self.bridge = CvBridge()
        self.device = 'cpu'
        self.conf = 0.5  # 设置模型置信度



    def detect(self, msg):
        # 处理接收到的图像信息
        self.logger.info("==== get image ====")
        # self.bridge = CvBridge()
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.model(color_image)
    
        # 在帧上显示检测结果
        annotated_frame = results[0].plot()

        # 设置显示窗口的名称和大小
        cv2.namedWindow('My Window', cv2.WINDOW_NORMAL)  # 使用cv2.WINDOW_NORMAL标志可以调整窗口大小
        cv2.resizeWindow('My Window', 800, 600)  # 设置窗口大小为800x600像素

        # 显示带有注释的帧
        cv2.imshow("My Window", annotated_frame)

        # 如果按下'q'键则跳出循环
        if cv2.waitKey(60) & 0xFF == ord("q"):  # 等待60毫秒，如果按下'q'键则跳出循环
            return


def main():
    rclpy.init()
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

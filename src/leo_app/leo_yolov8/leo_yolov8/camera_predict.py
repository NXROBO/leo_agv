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
        self.model = YOLO(pt_path + '/config/yolov8n.pt')
        
        # 创建订阅者
        super().__init__('Yolo')
        # 创建日志记录器
        self.logger = get_logger("yolov8")
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.detect, 10)
        
        self.bridge = CvBridge()
        self.device = 'cpu'
        self.conf = 0.5  # 设置模型置信度

    def predict(self, frame):
        # 使用模型进行预测

        results = self.model(frame, conf=self.conf)

        for result in results:
            self.boxes = result.boxes.data # 用于边界框输出的Boxes对象

        return results

    def plot_boxes(self, results, frame):
        # 计算帧率和绘制边界框

        fps = 1000.0/ results[0].speed['inference']
        cv2.putText(frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        frame = results[0].plot()

        return frame

    def detect(self, msg):
        # 处理接收到的图像信息
        self.logger.info("==== get image ====")
        # self.bridge = CvBridge()
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.predict(color_image)
        size = int(self.boxes.size()[0])
        i = 0

        for i in range(size):
            m_x = (int(self.boxes[i][2])-int(self.boxes[i][0]))/2
            m_y = (int(self.boxes[i][3])-int(self.boxes[i][1]))/2
            m_w = int(self.boxes[i][2])-int(self.boxes[i][0])
            m_h = int(self.boxes[i][3])-int(self.boxes[i][1])
            ip = int(self.boxes[i][5])

            print("m_x =",m_x)
            print("m_y =",m_y)
            print("m_w =",m_w)
            print("m_h =",m_h)
            print("ip =",ip)
            print("================")

        
        # self.logger.info("keypoint=" + str(keypoints))
        # print("boxes=",self.boxes)
        # print("size=",size[0])
        frame = self.plot_boxes(results, color_image)
        self.boxes = None
        cv2.imshow('Object Detection', frame)
        cv2.waitKey(3)

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

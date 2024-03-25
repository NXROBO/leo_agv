#!/usr/bin/env python3
#
# Copyright (C) 2024 深圳创想未来机器人有限公司. All Rights Reserved 
#
# @Time    : 2024-03-25
# @Author  : litian.zhuang
# @Email   : litian.zhuang@nxrobo.com
#
import sys
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from leo_common_interfaces.msg import InferenceResult
from leo_common_interfaces.msg import VisionInference

bridge = CvBridge()

class YoloV8_Camera_Subscriber(Node):

    def __init__(self):
        super().__init__('yolov8_camera_subscriber')
        pt_path = sys.path[0]
        print(pt_path)
        print(pt_path + '/yolov8n.pt')
        self.model = YOLO(pt_path + '/yolov8n.pt')  #('~/leo_agv/src/leo_app/leo_object_detector/yolov8_object_detector/scripts/yolov8n.pt')
        self.yolov8_inference = VisionInference()
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10)
        self.camera_subscription 
        self.yolov8_pub = self.create_publisher(VisionInference, "/object_detector", 1)
        self.img_pub = self.create_publisher(Image, "/object_detector_image", 1)

    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)
        self.yolov8_inference.header.frame_id = "object_detector"
        self.yolov8_inference.header.stamp = yolov8_camera_subscriber.get_clock().now().to_msg()
        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                score = float(box.conf)     #get the score
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[0])
                self.inference_result.left = int(b[1])
                self.inference_result.bottom = int(b[2])
                self.inference_result.right = int(b[3])
                self.inference_result.score = score
                self.yolov8_inference.vision_inference.append(self.inference_result)
            #yolov8_camera_subscriber.get_logger().info(f"{self.yolov8_inference}")
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.vision_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    yolov8_camera_subscriber = YoloV8_Camera_Subscriber()
    rclpy.spin(yolov8_camera_subscriber)
    rclpy.shutdown()

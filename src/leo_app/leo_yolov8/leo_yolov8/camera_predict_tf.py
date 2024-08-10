#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import time

import numpy as np
import tf2_ros
import tf_transformations
# from time import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

import cv2
from ultralytics import YOLO

class YoloDetection(Node):
# 初始化函数，包括加载模型和创建订阅者
    def __init__(self):
        # 加载模型
        model_path = os.path.join('/home/leo/leo_agv/src/leo_app/leo_yolov8/yolov8n.pt')  # 构造模型文件的绝对路径
        self.model = YOLO(model_path)
        
        # 创建订阅者
        super().__init__('Yolo')
        # 创建日志记录器
        self.logger = get_logger("yolov8")
        # rgb图像话题
        self.image_sub = self.create_subscription(Image, '/camera1/camera1/color/image_raw', self.detect, 10)
        time.sleep(5)
        self.camera_reference_frame = "camera_link" # 摄像头坐标系

        # 最终图像输出话题
        self.debug_pub = self.create_publisher(
            Image, "/debug_image", 1
        )

        # 深度图像话题
        self.depth_sub = self.create_subscription(
            Image, "/camera1/camera1/aligned_depth_to_color/image_raw", self.camera_depth_cb, 10
        )
        
        self.bridge = CvBridge()
        self.device = 'cpu'
        self.conf = 0.5  # 设置模型置信度
        
        self.fx = 613.4248046875
        self.fy = 612.1553955078125
        self.cx = 330.230224609375
        self.cy = 240.59852600097656
        
        self.obj = []

        # 物体坐标变换广播
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        # 物体id发布
        self.obj_pub = self.create_publisher(
            String, "/object_id", 10
        )

    def predict(self, frame):
        # 使用模型进行预测

        results = self.model(frame, conf=self.conf,show_labels=False,show_conf=False,show_boxes=False)

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
        # self.logger.info("==== get image ====")
        # self.bridge = CvBridge()
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.predict(self.color_image)
        # size = self.boxes.size()
        # self.logger.info("keypoint=" + str(keypoints))
        # print("boxes=",self.boxes)
        # print("size=",size)
        frame = self.plot_boxes(results, self.color_image)
        faces_message = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        self.debug_pub.publish(faces_message)
        # self.boxes = None
        
        #cv2.imshow('Object Detection', frame)
        #cv2.waitKey(3)

    # 深度图像回调函数
    def camera_depth_cb(self, msg):
        # print("get depth image")
        # 将ROS消息转换为OpenCV格式的深度图像
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        # 将深度图像数据归一化到0-1范围
        depth_array = np.array(self.depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        # 将归一化后的深度图像转换为8位无符号整数
        depth_8 = (depth_array * 255).round().astype(np.uint8)
        cv_depth = np.zeros_like(self.color_image)
 
        # 将深度图像赋值给RGB图像的深度通道
        try:
            cv_depth[:, :, 0] = depth_8
            cv_depth[:, :, 1] = depth_8
            cv_depth[:, :, 2] = depth_8
        except:
            self.obj.clear()
            return   
         
        
        # 遍历boxes的值
        if len(self.boxes) == 0:
            # print("没有找到物体")
            self.obj_pub.publish(String())
            self.obj.clear()
            return
        size = int(self.boxes.size()[0])
        # print("=======", self.boxes)
        
        for i in range(size):
            ip = int(self.boxes[i][5])
            if ip in [46, 47, 49]:
                self.obj.append(ip)
        # print("----------", self.obj)

        # 46 香蕉 47 苹果 49 橙子
        if 46 in self.obj:
            obj_id = 46
        if 46 not in self.obj and 47 in self.obj:
            obj_id = 47
        if 46 not in self.obj and 47 not in self.obj:
            obj_id = 49

        for i in self.boxes:
            if i[-1] == obj_id:
                m_x = int(i[0])
                m_y = int(i[1])
                m_w = int(i[2])-int(i[0])
                m_h = int(i[3])-int(i[1])
                ip = int(i[5])
                # print("=-=-=-=-=", m_x, m_y)
                # print("m_x =",m_x)
                # print("m_y =",m_y)
                # print("m_w =",m_w)
                # print("m_h =",m_h)
                # print("ip =",ip)
                # print("================")
                
                # 绘制矩形框
                # cv2.rectangle(self.color_image, (int(i[0]), int(i[1])),(int(i[2]), int(i[3])), color=(0, 255, 0), thickness=2)
                    
                # 获取ROI区域的深度图像
                roi_depth = self.depth_image[m_y: m_y+m_h, m_x: m_x+m_w]
                
                # 计算ROI区域内有效像素点数量和深度值总和
                n = 0
                sum = 0
                for i in range(0, roi_depth.shape[0]):
                    for j in range(0, roi_depth.shape[1]):
                        value = roi_depth.item(i, j)
                        if value > 0:
                            n += 1
                            sum += value
                if n == 0 or sum == 0:
                    return 

            
                # 计算ROI区域内平均深度值
                mean_z = sum / n
                depth = mean_z * 0.001
                # print("depth=",depth)
            
                # 计算物体在相机坐标系下的三维坐标
                x = (m_x - self.cx) / self.fx
                y = (m_y - self.cy) / self.fy
                point_x = depth * x
                point_y = depth * y
                point_z = depth


            
                # 创建TF变换消息并发布
                cube_tf = TransformStamped()
                cube_tf.header.stamp = self.get_clock().now().to_msg()
                cube_tf.header.frame_id = self.camera_reference_frame
                cube_tf.child_frame_id = "object_"+str(ip)
                cube_tf.transform.translation.x = -float(point_y)  # 将Y轴坐标赋值给X轴
                cube_tf.transform.translation.y = -float(point_x)  # 将X轴坐标赋值给Y轴
                cube_tf.transform.translation.z = -float(point_z)  # 将Z轴坐标赋值给Z轴
                # ori = tf_transformations.quaternion_from_euler(3.14, 0, -theta)  # 计算姿态的四元数
                cube_tf.transform.rotation.x = 1.0
                cube_tf.transform.rotation.y = 0.0
                cube_tf.transform.rotation.z = 0.0
                cube_tf.transform.rotation.w = 0.0
                msg = String()
                msg.data = str(ip)
                self.obj_pub.publish(msg)
                self.tf_pub.sendTransform(cube_tf)  # 发布TF变换消息

                # 合并RGB图像和深度图像，并发布调试信息
                rgbd = np.concatenate((self.color_image, cv_depth), axis=1)
                try:
                    # faces_message = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
                    # self.debug_pub.publish(faces_message)
                    pass
                except CvBridgeError as e:
                    print(e)
        self.obj.clear()
            
        
     

def main():
    rclpy.init()
    node = YoloDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    node.destroy_node()
    rclpy.shutdown()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

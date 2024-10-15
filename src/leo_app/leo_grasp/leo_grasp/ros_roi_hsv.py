#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger


from time import time

from std_msgs.msg import Header, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import json


import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory

# 定义ImgShow类，继承自Node类
class ImgShow(Node):
    # 初始化函数
    def __init__(self):
        # 调用父类的初始化函数，设置节点名称为'ImgShow'
        super().__init__('ImgShow')
        # 创建日志记录器
        self.logger = get_logger("ImgShow")
        # 创建订阅者，订阅'/camera/color/image_raw'话题，并将回调函数设置为self.image_callback
        self.image_sub = self.create_subscription(Image, '/camera1/camera1/color/image_raw', self.image_callback, 10)

        # 创建CvBridge
        self.bridge = CvBridge()
        self.hsv_dict = {}
        hsv_values = {}
        # 操作说明
        print("********===========================================================================********")
        print("================                  操作说明                                 ================ ")
        print("********========     1,在显示的图片上,选取想要识别的颜色;                  ========******** ")
        print("********========     2,选取后暂时以字典保存,待全部颜色识别完后,按s进行保存;========********")
        print("********========     3,r代表红色,g代表绿色,b代表蓝色,y代表黄色;            ========********")
        print("********========     4,q代表刷新,s代表保存.                                ========********")
        print("********===========================================================================********")

        # leo_vision_dir = get_package_share_directory('leo_vision')
        # self.hsv_file_path = os.path.join(leo_vision_dir,'configs', 'hsv_values.txt')  # hsv文件地址

        leo_grasp_dir = get_package_share_directory('leo_grasp')
        self.hsv_file_path = os.path.join(leo_grasp_dir,'param', 'hsv_values.txt')  # hsv文件地址

        # with open(self.hsv_file_path,'r') as file:
        #     hsv_values = json.load(file)
        #     print(hsv_values['green'])
         

    # 定义一个函数来处理接收到的图像数据
    def image_callback(self, img):
        # 将图像数据转换为OpenCV格式
        self.cv2_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

        # 显示用户使用操作提示
        text1 = f"Please drag and select the color\n"
        text2 = "q:refresh, s:save, r:red, g:green, b:blue, y:yellow"

        # 打印文本信息
        cv2.putText(self.cv2_img, text1, (30,40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)
        cv2.putText(self.cv2_img, text2, (30,100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 1)

        # 显示BGR图像
        cv2.imshow('color_image', self.cv2_img)


        # 将BGR图像转换为HSV图像
        self.hsv_image = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2HSV)
        # 显示HSV图像
        # cv2.imshow('HSV Image', self.hsv_image)
        # 设置鼠标回调函数，用于在BGR图像上获取鼠标位置的颜色值
        cv2.setMouseCallback('color_image', self.mouse_callback)
        # 等待键盘输入，获取按键对应的ASCII码
        c = cv2.waitKey(1)

        if c == 114 or c == 103 or c == 98 or c == 121:
            # self.detect_save(c)
            color_name = self.map_color_id_to_name(c)
            self.hsv_dict[color_name] = (self.roi_h, self.roi_s, self.roi_v)
            print("================     当前hsv_dict字典内容为",self.hsv_dict)

        elif c == 115:
            print('保存全部字典')
            self.save_hsv_values_to_file()
            
        elif c == 113:
            cv2.destroyWindow('mask')
            cv2.destroyWindow('roi_image')


    # 定义一个函数来处理鼠标移动事件，并打印当前鼠标位置的HSV值
    def mouse_callback(self, event, x, y, flags, param):

        # 获取鼠标左键按下事件
        if event == cv2.EVENT_LBUTTONDOWN:

            # 打印鼠标左键按下的值
            # print("检测到鼠标按下的位置为", y,x)

            # 记录鼠标左键按下的位置
            self.start_y,self.start_x = y,x
        
        # 获取鼠标左键松开事件
        elif event == cv2.EVENT_LBUTTONUP:

            # 初始化参数
            roi_x,roi_y = 0,0
            sum_h,sum_s,sum_v = 0,0,0
            roi_num = 0 
            thresh = 20
            
            # 打印鼠标拖拽的停止值
            # print("检测到鼠标松开的位置为", y,x)
            # 记录鼠标松开的位置
            self.end_y,self.end_x = y,x

            # 复制图像
            self.roi_img = self.hsv_image.copy()

            # 遍历循环ROI区域的HSV值
            for roi_y in range(min(self.start_y, self.end_y), max(self.start_y, self.end_y)):
                for roi_x in range(min(self.start_x, self.end_x), max(self.start_x, self.end_x)):
                    h, s, v = self.roi_img[roi_y, roi_x]
                    sum_h += h
                    sum_s += s
                    sum_v += v
                    roi_num += 1
                    

            # 计算ROI区域HSV值的平均值
            self.roi_h,self.roi_s,self.roi_v = int(sum_h/roi_num),int(sum_s/roi_num),int(sum_v/roi_num)
            # print("sum_nun",roi_num)
            print("================     检测到的[h,s,v]为",self.roi_h,self.roi_s,self.roi_v,)
            print("================     观察mask图片,如果满意，在图片窗口输入颜色对应英文的首字母，如'r','g','b','y'")
            print("================*****************************************************************================")

            # 判断是否拖拽成功
            if roi_num > 0:

                # 设定HSV阈值
                minHSV = np.array([self.roi_h, self.roi_s-thresh,self.roi_v-thresh])
                maxHSV = np.array([self.roi_h+thresh, self.roi_s+thresh,self.roi_v+thresh])

                # 掩膜处理
                mask = cv2.inRange(self.roi_img, minHSV , maxHSV)
                cv2.namedWindow('mask')
                cv2.moveWindow('mask', 200, 700)  # 将窗口移动到指定位置
                cv2.imshow('mask', mask)

                # 画框处理
                cv2.rectangle(self.roi_img, (self.start_x,self.start_y),(self.end_x,self.end_y),(255,0,0),2)
                cv2.imshow('roi_image', self.roi_img)
                cv2.moveWindow('roi_image', 900, 700)  # 将窗口移动到指定位置
                cv2.imshow('roi_image', self.roi_img)
            else:
                print("请在BGR图像上拖拽选取颜色")

            return
                

    def map_color_id_to_name(self, color_id):

        color_map = {
            103: 'green',
            114: 'red',
            98: 'blue',
            121: 'yellow'
            # Add more mappings as needed
        }
        return color_map.get(color_id, 'unknown')
    
    def save_hsv_values_to_file(self):
        with open(self.hsv_file_path, 'w') as file:
            json.dump(self.hsv_dict, file)
        print("HSV values saved to leo_agv/install/leo_grasp/share/leo_grasp/param/hsv_values.txt")
                

# 定义主函数
def main(): 
    # 初始化ROS环境
    rclpy.init()
    # 创建ImgShow节点实例
    node = ImgShow()
    try:
        # 开始ROS事件循环，程序将在这里持续运行直到收到键盘中断信号
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 打印“Shutting down”信息并退出程序
        print("Shutting down")
    # 清理资源，关闭窗口并退出程序
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
# 如果脚本是直接运行的（而不是作为模块导入的），则执行main函数
if __name__ == '__main__': main()

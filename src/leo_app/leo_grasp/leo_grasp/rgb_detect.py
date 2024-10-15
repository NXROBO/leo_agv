# TF2_Color_Detector

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
# import tf.transformations as transfromations
import tf2_ros
import tf_transformations
import json

import cv2
# from color import ColorCubeGet2DwithAngle # 颜色识别
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory

class Detector(Node):
    def __init__(self):
        super().__init__("color_detect")
        self.bgr = [151, 105, 0] # blue 
        self.yellow = [47, 161, 203] # yellow
        self.red = [31, 102, 156] # red
        self.blue = [204, 204, 51] # low blue -------

        # self.bgr = [225, 105, 65]  # low blue1
        # self.bgr = [201, 161, 51] # low blue2
        self.buff = True

        self.hsv_thresh = 30
        self.bridge = CvBridge()
        self.camera_reference_frame = "camera_color_optical_frame" # 摄像头坐标系
        self.color = (50, 255, 50)
        
        self.cv_rgb = None

        self.fx = 613.4248046875
        self.fy = 612.1553955078125
        self.cx = 330.230224609375
        self.cy = 240.59852600097656

        self.trans_x = 0

        # 相机内参
        '''
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_cb, 10
        )
        '''

        # RGB图像话题
        self.image_sub = self.create_subscription(
            Image, "/camera1/camera1/color/image_raw", self.camera_rgb_cb, 1
        )
        # 深度图像话题
        self.depth_sub = self.create_subscription(
            Image, "/camera1/camera1/aligned_depth_to_color/image_raw", self.camera_depth_cb, 1
        )

        # 最终图像输出话题
        self.debug_pub = self.create_publisher(
            Image, "/debug_image", 1
        )

        # 物体坐标变换广播
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        leo_grasp_dir = get_package_share_directory('leo_grasp')
        self.hsv_file_path = os.path.join(leo_grasp_dir,'param', 'hsv_values.txt')  # 标记地图yaml文件地址

        with open(self.hsv_file_path,'r') as file:
            self.hsv_values = json.load(file)
            print(self.hsv_values ['red'])
        

    # 相机内参回调
    '''
    def camera_info_cb(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
    '''
    # RGB图像回调
    def camera_rgb_cb(self, msg):
        self.cv_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        

    # 深度图像回调
    def camera_depth_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        depth_array = np.array(self.depth_image, dtype=np.float32)
        cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

        depth_8 = (depth_array * 255).round().astype(np.uint8)
        cv_depth = np.zeros_like(self.cv_rgb)
        # print(cv_depth)
        try:
            cv_depth[:, :, 0] = depth_8
            cv_depth[:, :, 1] = depth_8
            cv_depth[:, :, 2] = depth_8
        except:
            return

        min_rect = self.ColorCubeGet2DwithAngle(self.cv_rgb, self.bgr, self.hsv_thresh)
            
        if min_rect is None:
            # print("None")
            mask3 = np.zeros_like(self.rgb)
            debug = np.concatenate((self.rgb, mask3), axis=1)
            cv2.imshow("debug_image", debug)
            cv2.waitKey(1)
            return
     
        min_rect, contours = min_rect

        # 创建一个黑色图像，大小与原始图像相同
        mask2 = np.zeros_like(self.rgb)
        for cnt in contours:
            mask = np.zeros_like(self.rgb)
            cv2.drawContours(mask, [cnt], -1, (255, 255, 255), thickness=cv2.FILLED)
            roi = cv2.bitwise_and(self.rgb, mask)
            mask2 = cv2.bitwise_or(mask2, roi)
        
        debug = np.concatenate((self.cv_rgb, mask2), axis=1)
        cv2.imshow("debug_image", debug)
        cv2.waitKey(1)


        pixel_x, pixel_y = min_rect[0]
        width, height = min_rect[1]
        theta = min_rect[2]
        theta = math.radians(theta)
        
        # box = cv2.boxPoints(min_rect)
        # box = np.int64(box)
        
        # cv2.polylines(img=self.cv_rgb, pts=[box], isClosed=True, color=(0, 255, 0), thickness=2)
        m_x, m_y, m_w, m_h = int(pixel_x), int(pixel_y), int(width), int(height)
        roi_depth = self.depth_image[m_y: m_y+m_h, m_x: m_x+m_w]

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
        
        mean_z = sum / n
        depth = mean_z * 0.001
        # print(self.fx, self.fy, self.cx, self.cy)
        x = (pixel_x - self.cx) / self.fx
        y = (pixel_y - self.cy) / self.fy
        point_x = depth * x
        point_y = depth * y
        point_z = depth
        

        cv2.circle(cv_depth, (int(pixel_x), int(pixel_y)), 3, (0, 255, 0), 5)
        cv2.circle(self.cv_rgb, (int(pixel_x), int(pixel_y)), 50, (255, 0, 255), 3)

        rgbd = np.concatenate((self.cv_rgb, cv_depth), axis=1)

        # cv2.imshow("debug_image",rgbd)
        # cv2.waitKey(1)

        cube_tf = TransformStamped()
        cube_tf.header.stamp = self.get_clock().now().to_msg()
        cube_tf.header.frame_id = self.camera_reference_frame
        cube_tf.child_frame_id = "object"
        cube_tf.transform.translation.x = float(point_x) 
        cube_tf.transform.translation.y = float(point_y) 
        cube_tf.transform.translation.z = float(point_z)   
        # cube_tf.transform.translation.x = 0.0
        # cube_tf.transform.translation.y = 0.0
        # cube_tf.transform.translation.z = 0.0
        ori = tf_transformations.quaternion_from_euler(0, 0,3.14+theta)  
        cube_tf.transform.rotation.x = ori[0]
        cube_tf.transform.rotation.y = ori[1]
        cube_tf.transform.rotation.z = ori[2]
        cube_tf.transform.rotation.w = ori[3]
        self.tf_pub.sendTransform(cube_tf)


        try:
            # faces_message = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
            # self.debug_pub.publish(faces_message)
            pass
        except CvBridgeError as e:
            print(e)

    def ColorCubeGet2DwithAngle(self, cv_image, bgr, thresh):
        brightHSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        
        red_hsv = self.hsv_values['red']
        blue_hsv = self.hsv_values['blue']
        yellow_hsv = self.hsv_values['yellow']


        # red = [31, 102, 156] # red
        # blue = [204, 204, 51] # low blue ------
        # yellow = [47, 161, 203] # yellow

        # red_hsv = cv2.cvtColor(np.uint8([[red]]), cv2.COLOR_BGR2HSV)[0][0]
        # blue_hsv = cv2.cvtColor(np.uint8([[blue]]), cv2.COLOR_BGR2HSV)[0][0]
        # yellow_hsv = cv2.cvtColor(np.uint8([[yellow]]), cv2.COLOR_BGR2HSV)[0][0]
            
        red_minHSV = np.array([red_hsv[0], red_hsv[1] - thresh, red_hsv[2] - thresh])
        red_maxHSV = np.array([red_hsv[0] + thresh, red_hsv[1] + thresh, red_hsv[2] + thresh])

        blue_minHSV = np.array([blue_hsv[0]-thresh, blue_hsv[1]-thresh, blue_hsv[2]-thresh])
        blue_maxHSV = np.array([blue_hsv[0]+thresh, blue_hsv[1]+thresh, blue_hsv[2]+ thresh])

        yellow_minHSV = np.array([yellow_hsv[0] - thresh, yellow_hsv[1] - thresh, yellow_hsv[2] - thresh])
        yellow_maxHSV = np.array([yellow_hsv[0] + thresh, yellow_hsv[1] + thresh, yellow_hsv[2] + thresh])

        mask_red = cv2.inRange(brightHSV, red_minHSV, red_maxHSV)
        mask_blue = cv2.inRange(brightHSV, blue_minHSV, blue_maxHSV)
        mask_yellow = cv2.inRange(brightHSV, yellow_minHSV, yellow_maxHSV)


        # 合并所有颜色的掩码
        mask_combined = cv2.bitwise_or(mask_red, mask_blue)
        resultHSV = cv2.bitwise_or(mask_combined, mask_yellow)

        #3.使用卷积进行图像过滤
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5)) #5*5矩形卷积核
        resultHSV = cv2.erode(resultHSV, kernel) #腐蚀
        resultHSV = cv2.dilate(resultHSV, kernel) #膨胀
        #4.转换为灰度图
        # resultGray = cv2.cvtColor(resultHSV, 6) #cv2.COLOR_BGR2GRAY
        npImg = np.asarray(resultHSV)
        # #5.应用二进制阈值
        th = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]
        # cv2.imshow("faces_message",th)
        #6.检测图像中的轮廓
        (contours, hierarchy) = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        #7.获取水平矩形/最小矩形
        for contour in contours:
            area = cv2.contourArea(contour)
            # print("area=",area)
            if area < 4500:
                continue
            # (x, y, w, h) = cv2.boundingRect(contour)  #水平矩形
            min_rect = cv2.minAreaRect(contour)  #最小矩形
            return min_rect, contours
            # max_rect = cv2.maxAreaRect(contour)  #最小矩形
            # return max_rect
        


def main(args=None):
    rclpy.init(args=args)
    
    detect = Detector()
    rclpy.spin(detect)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
        


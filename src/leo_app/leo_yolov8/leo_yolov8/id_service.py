#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String

class MyService(Node):
    def __init__(self):
        super().__init__('id_service')
        self.latest_msg = String()
        self.srv = self.create_service(SetBool, 'id_service', self.set_bool_callback)
        self.sub = self.create_subscription(String, 'object_id', self.my_topic_callback, 10)

    def set_bool_callback(self, request, response):
        response.success = request.data
        response.message = self.latest_msg.data
        # self.get_logger().info('Incoming request\n%s' % request.data)
        return response

    def my_topic_callback(self, msg):
        self.latest_msg = msg
        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    my_service = MyService()
    rclpy.spin(my_service)
    my_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

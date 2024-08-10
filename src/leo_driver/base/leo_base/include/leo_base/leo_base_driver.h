#ifndef LEO_BASE_DRIVER_H
#define LEO_BASE_DRIVER_H
#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "leo_base/leo_base_serial.h"
#include "leo_base/leo_base_constants.h"
#include "leo_base/leo_base_interface.h"
#include "leo_base/msg/leo_base_sensor.hpp"
#include "leo_base/msg/leo_base_dock.hpp"
#include "leo_base/msg/gyro_message.hpp"
#include "leo_base/msg/leo_base_odom.hpp"
#include "leo_base/srv/leo_base_control.hpp"

#include "leo_base/kfilter.hpp"
#include "leo_base/mylock.hpp"

#define NODE_VERSION 0.01
#define LEOBASETIMEOUT (1000 * 1e3) // ����1s
#define COUNT_TIMES 20
using namespace std::chrono_literals;

namespace NxLeoBase
{

  class LeoBaseDriver : public rclcpp::Node
  {
  public:
    LeoBaseDriver(std::string node_name);
    ~LeoBaseDriver();

  private:
    void hex_printf(unsigned char *buf, int len);
    void getLeobaseComData(unsigned char *buf, int len);
    unsigned char checkSum(unsigned char *buf);
    int pubGyroMessage(unsigned char *buf, int len);
    void dealMessageSwitch(unsigned char *recvbuf);
    void startComParamInit();
    void resetOdomCb(const leo_base::msg::LeoBaseOdom::SharedPtr odom);
    void cmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    int pubWheelJointStates(double linear_speed, double angular_speed);
    void handleGoDock(const std_msgs::msg::String::SharedPtr msg);
    void handleSearchDock(const std_msgs::msg::String::SharedPtr msg);
    void process_base_receive_thread();
    void checkSerialGoon();
    void publish_odom();
    void handle_base_control_cmd(const std::shared_ptr<leo_base::srv::LeoBaseControl::Request> request,
                                                std::shared_ptr<leo_base::srv::LeoBaseControl::Response> response);

  private:
    std::string base_frame_id;
    std::string odom_frame_id;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // tf::TransformBroadcaster tf_broadcaster;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_sub;
    rclcpp::Subscription<leo_base::msg::LeoBaseOdom>::SharedPtr odom_reset_sub;
    std::shared_ptr<LeoBaseInterface> interface_port_;

    std::string serial_port;
    // rclcpp::Timer stimer;
    rclcpp::TimerBase::SharedPtr stimer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr fback_cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub;
    rclcpp::Publisher<leo_base::msg::LeoBaseSensor>::SharedPtr rb_sensor_pub;
    rclcpp::Publisher<leo_base::msg::LeoBaseDock>::SharedPtr rb_dock_pub;
    rclcpp::Publisher<leo_base::msg::GyroMessage>::SharedPtr gyro_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Service<leo_base::srv::LeoBaseControl>::SharedPtr service_lbc;
    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;
    unsigned char ret_short_cuts;
    int idx;
    unsigned int countSerial, lastCountSerial;
    double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
        odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
    double robot_yaw;
    NxLeoBase::KFilter odom_x_kfilter, odom_y_kfilter;
    double left_wheel_position, right_wheel_position;
    std::shared_ptr<std::thread> base_receive_thread_;
    char driver_board_version[100];
  };

}

#endif // LEO_BASE_DRIVER_H
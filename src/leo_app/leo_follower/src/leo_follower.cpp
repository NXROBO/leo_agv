/*
 * Copyright (c) 2016, SHENZHEN NXROBO Co.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Xiankai Chen (xiankai.chen@nxrobo.com) and Jian Song
 *		(jian.song@nxrobo.com)
 */

#include <chrono>
#include <memory>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <sstream>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <boost/make_shared.hpp>


#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
//#include <pcl_ros/transforms.h>
using namespace std::chrono_literals;
#define MAX_LINEAR_X 0.3


class NxFollowerNode : public rclcpp::Node
{

private:
  // cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub;

  // point clound subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

  double min_y_;   /**< The minimum y position of the points in the box. */
  double max_y_;   /**< The maximum y position of the points in the box. */
  double min_x_;   /**< The minimum x position of the points in the box. */
  double max_x_;   /**< The maximum x position of the points in the box. */
  double max_z_;   /**< The maximum z position of the points in the box. */
  double goal_z_;  /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  double z_thre;
  double x_thre;
  double max_vx;                 /*max velocity x*/
  double max_vz;                 /*max velocity z*/
  double max_depth_, min_depth_; /**< The maximum z position of the points in the box. */
  double goal_depth_;            /**< The distance away from the robot to hold the centroid */
  double depth_thre;
  double y_thre;

public:
  NxFollowerNode() : min_y_(0.1), max_y_(0.5), min_x_(-0.2), max_x_(0.2), max_z_(0.8), goal_z_(0.6), z_scale_(1.0), x_scale_(5.0), Node("leo_follower")
  {

    min_x_ = -0.2;
    max_x_ = 0.2;
    min_y_ = -0.1;
    max_y_ = 0.3;
    max_z_ = 1.5;
    goal_z_ = 0.7;
    z_scale_ = 0.8;
    x_scale_ = 2;
    z_thre = 0.05;
    x_thre = 0.05;
    y_thre = 0.087222222;

    max_vx = 0.4;
    max_vz = 0.8;

    max_depth_ = 2;
    min_depth_ = 0.4;
    goal_depth_ = 0.9;
    depth_thre = 0.1;
    y_thre = 0.087222222;

    cmdvel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 1, std::bind(&NxFollowerNode::pointCloudCb, this, std::placeholders::_1));
  }
  virtual ~NxFollowerNode()
  {
  }

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    // PCL still uses boost::shared_ptr internally
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // This will convert the message into a pcl::PointCloud
    pcl::fromROSMsg(*msg, *cloud);

    // Number of points observed
    unsigned int n = 0;
    pcl::PointXYZRGB pt;
    for (int kk = 0; kk < cloud->points.size(); kk++)
    {
      pt = cloud->points[kk];

      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          // Add the point to the totals
          x += pt.x;
          z += pt.z;

          n++;
        }
      }
    }

    if (n > 2000)
    {

      x /= n;
      z /= n;
      if (z > max_z_)
      {
        publish_cmdvel(0.0, 0.0, 0.0);
        return;
      }

      pubCmd(-x, z);
    }
    else
    {
      publish_cmdvel(0.0, 0.0, 0.0);
    }
  }

  void pubCmd(const float &y, const float &depth)
  {
    double curr_dist = sqrt(y * y + depth * depth);
    if (curr_dist == 0)
    {
      publish_cmdvel(0.0, 0.0, 0.0);
      return;
    }

    float x_linear = 0;
    float z_angular = 0;
    float z_scale = 1.2;
    float x_scale = 5.0; // 2.0
    x_linear = (depth - goal_depth_) * z_scale;
    z_angular = asin(y / curr_dist) * x_scale;

    if (depth_thre > fabs(depth - goal_depth_))
      x_linear = 0;
    if (y_thre > y && y > -y_thre)
      z_angular = 0;
    if(x_linear > MAX_LINEAR_X)
      x_linear = MAX_LINEAR_X;
    else if(x_linear < -MAX_LINEAR_X)
      x_linear = -MAX_LINEAR_X;        

    publish_cmdvel(x_linear, 0.0, z_angular);
  }
  void publish_cmdvel(float x, float y, float z)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = x;
    cmd.linear.y = y;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = z;
    cmdvel_pub->publish(cmd);
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NxFollowerNode>());
  rclcpp::shutdown();
  return 0;
}

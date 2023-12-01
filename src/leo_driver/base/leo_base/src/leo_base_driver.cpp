/*
 *  Copyright (c) 2023, NXROBO Ltd.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *  
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */


#include "leo_base/leo_base_interface.h"
#include "leo_base/leo_base_driver.h"
int flag = 0;

namespace NxLeoBase
{

  LeoBaseDriver::LeoBaseDriver(std::string new_serial_port) : rclcpp::Node("leo_base_node")
  {
    robot_yaw = 0;
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("base_frame_id", "base_footprint");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("battery/status", 0);
    this->get_parameter_or<std::string>("serial_port", serial_port, new_serial_port);
    this->get_parameter_or<std::string>("base_frame_id", base_frame_id, "base_footprint");
    this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id, "odom");
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LeoBaseDriver::cmdVelReceived, this, std::placeholders::_1));
    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 1);
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    fback_cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("leo_base/command/velocity", 1);
    odom_reset_sub = this->create_subscription<leo_base::msg::LeoBaseOdom>(
        "leo_base/odom/reset", 1, std::bind(&LeoBaseDriver::resetOdomCb, this, std::placeholders::_1));
    gyro_pub = this->create_publisher<leo_base::msg::GyroMessage>("leo_base/gyro", 5);
    rb_sensor_pub = this->create_publisher<leo_base::msg::LeoBaseSensor>("leo_base/sensor", 5);
    rb_dock_pub = this->create_publisher<leo_base::msg::LeoBaseDock>("leo_base/dock", 5);
    dock_sub = this->create_subscription<std_msgs::msg::String>(
        "leo_base/handle_go2dock", 10, std::bind(&LeoBaseDriver::handleGoDock, this, std::placeholders::_1));
    search_sub = this->create_subscription<std_msgs::msg::String>(
        "leo_base/handle_search_dock", 10, std::bind(&LeoBaseDriver::handleSearchDock, this, std::placeholders::_1));
    wheel_joint_pub = this->create_publisher<sensor_msgs::msg::JointState>("wheel_states", 5);
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    left_wheel_position = 0;
    right_wheel_position = 0;
    // publish wheel joint state with 0,0
    pubWheelJointStates(0, 0);
    startComParamInit();

    stimer = this->create_wall_timer(1s, std::bind(&LeoBaseDriver::checkSerialGoon, this));

    idx = 0;
    for (int x = 0; x < COUNT_TIMES; x++)
    {
      fb_time[x] = 0;
      odom_x[x] = 0;
      odom_y[x] = 0;
      odom_yaw[x] = 0;
      vel_x_list[x] = 0;
    }

    RCLCPP_INFO(this->get_logger(), "inint ComDealDataNode");
  }
  LeoBaseDriver::~LeoBaseDriver()
  {
    exit(0);
  }

  void LeoBaseDriver::startComParamInit()
  {
    interface_port_ = std::shared_ptr<LeoBaseInterface>(new LeoBaseInterface(serial_port));
    interface_port_->resetOdometry();
    if (interface_port_->openSerialPort() < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Could not connect to %s.",serial_port.c_str());
      return;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "connect to leobase sucessfully.");
    }
    base_receive_thread_ = std::shared_ptr<std::thread>(new std::thread(std::bind(&LeoBaseDriver::process_base_receive_thread, this)));
  }

  void LeoBaseDriver::resetOdomCb(const leo_base::msg::LeoBaseOdom::SharedPtr odom)
  {
    interface_port_->setOdometry(odom->x, odom->y, odom->yaw);
    printf("setting odom!\n");
  }

  void LeoBaseDriver::cmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
  {
    interface_port_->drive(1, cmd_vel->linear.x, cmd_vel->angular.z);
    countSerial++;
  }

  int LeoBaseDriver::pubWheelJointStates(double linear_speed, double angular_speed)
  {
    double now_time = this->get_clock()->now().seconds();

    int left_speed_mm_s =
        (int)((-linear_speed - LEOBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Left wheel velocity in mm/s
    int right_speed_mm_s =
        (int)((-linear_speed + LEOBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Right wheel velocity in mm/s
    left_wheel_position += left_speed_mm_s * 0.001 / 50 / (LEOBASE_AXLE_LENGTH / 2);
    right_wheel_position += right_speed_mm_s * 0.001 / 50 / (LEOBASE_AXLE_LENGTH / 2);

    while (left_wheel_position > M_PI || left_wheel_position < -M_PI || right_wheel_position > M_PI ||
           right_wheel_position <= -M_PI)
    {
      if (left_wheel_position > M_PI)
      {
        left_wheel_position -= 2 * M_PI;
      }
      else if (left_wheel_position < -M_PI)
      {
        left_wheel_position += 2 * M_PI;
      }

      if (right_wheel_position > M_PI)
      {
        right_wheel_position -= 2 * M_PI;
      }
      else if (right_wheel_position < -M_PI)
      {
        right_wheel_position += 2 * M_PI;
      }
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = rclcpp::Time(now_time);
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "left_wheel_joint";
    joint_state.position[0] = left_wheel_position;
    joint_state.name[1] = "right_wheel_joint";
    joint_state.position[1] = right_wheel_position;

    wheel_joint_pub->publish(joint_state);

    return 0;
  }

  union Char2Float
  {
    float value;
    unsigned char buffer[4];
  };
short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}

  int LeoBaseDriver::pubGyroMessage(unsigned char *buf, int len)
  {
    leo_base::msg::GyroMessage gyro;
    double now_time = this->get_clock()->now().seconds();
    float acvx, acvy, acvz, anvx, anvy, anvz, roll, pitch, yaw;
    sensor_msgs::msg::Imu car_imu;
    tf2::Quaternion q;
    if (len < 18)
      return -1;

    gyro.acvx = (float)CharToShort(&buf[0])/32768*16;
    gyro.acvy = (float)CharToShort(&buf[2])/32768*16;
    gyro.acvz = (float)CharToShort(&buf[4])/32768*16;
    gyro.anvx = (float)CharToShort(&buf[6])/32768*2000;
    gyro.anvy = (float)CharToShort(&buf[8])/32768*2000;
    gyro.anvz = (float)CharToShort(&buf[10])/32768*2000;
    gyro.roll = (float)CharToShort(&buf[12])/32768*180;
    gyro.pitch = (float)CharToShort(&buf[14])/32768*180;
    gyro.yaw = (float)CharToShort(&buf[16])/32768*180;


    robot_yaw = gyro.yaw / 180 * 3.1415926535898;

    acvx = (gyro.acvx * 9.8); // m/s^2
    acvy = (gyro.acvy * 9.8);
    acvz = (gyro.acvz * 9.8);

    anvx = gyro.anvx;
    anvy = gyro.anvy;
    anvz = gyro.anvz;

    roll = gyro.roll / 180 * M_PI;
    pitch = gyro.pitch / 180 * M_PI;
    yaw = gyro.yaw / 180 * M_PI;

    q.setRPY(roll, pitch, yaw); // tf2::createQuaternionFromRPY(roll, pitch, yaw);
    car_imu.orientation.x = q.x();
    car_imu.orientation.y = q.y();
    car_imu.orientation.z = q.z();
    car_imu.orientation.w = q.w();
    car_imu.orientation_covariance[0] = pow(0.0017, 2); //
    car_imu.orientation_covariance[4] = pow(0.0017, 2);
    car_imu.orientation_covariance[8] = pow(0.0017, 2);

    car_imu.angular_velocity.x = anvx * M_PI / 180.0; // rad/s
    car_imu.angular_velocity.y = anvy * M_PI / 180.0;
    car_imu.angular_velocity.z = anvz * M_PI / 180.0;
    car_imu.angular_velocity_covariance[0] = pow(0.1, 2);
    car_imu.angular_velocity_covariance[4] = pow(0.1, 2);
    car_imu.angular_velocity_covariance[8] = pow(0.1, 2);

    car_imu.linear_acceleration.x = acvx; // m/s^2
    car_imu.linear_acceleration.y = acvy;
    car_imu.linear_acceleration.z = acvz;
    car_imu.linear_acceleration_covariance[0] = pow(0.1, 2);
    car_imu.linear_acceleration_covariance[4] = pow(0.1, 2);
    car_imu.linear_acceleration_covariance[8] = pow(0.1, 2);
    car_imu.header.stamp = rclcpp::Time(now_time);
    car_imu.header.frame_id = "IMU_link";
    pub_imu->publish(car_imu);

#if 0
	cout<<"acvx="<<gyro.acvx<<",acvy="<<gyro.acvy<<",acvz="<<gyro.acvz<<",anvx=" <<gyro.anvx<<",anvy=" <<gyro.anvy<<",anvz="<<gyro.anvz<<endl;
	cout<<"roll="<<gyro.roll<<",pitch="<<gyro.pitch<<",yaw="<<gyro.yaw<<endl;
	cout<<"bx="<<gyro.bx<<",by="<<gyro.by<<",bz="<<gyro.bz<<endl;
#endif

    gyro_pub->publish(gyro);
    return 0;
  }

  void LeoBaseDriver::publish_odom()
  {
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    rclcpp::Time time_now = this->get_clock()->now();

    odom_msg->header.frame_id = odom_frame_id;
    odom_msg->child_frame_id = base_frame_id;
    odom_msg->header.stamp = time_now;

    odom_msg->pose.pose.position.x = interface_port_->odometry_x_;
    odom_msg->pose.pose.position.y = interface_port_->odometry_y_;
    odom_msg->pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, interface_port_->odometry_yaw_);

    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    odom_msg->twist.twist.linear.x = interface_port_->robot_vel_[0];
    odom_msg->twist.twist.angular.z = interface_port_->robot_vel_[2];

    // TODO(Will Son): Find more accurate covariance.
    // odom_msg->pose.covariance[0] = 0.05;
    // odom_msg->pose.covariance[7] = 0.05;
    // odom_msg->pose.covariance[14] = 1.0e-9;
    // odom_msg->pose.covariance[21] = 1.0e-9;
    // odom_msg->pose.covariance[28] = 1.0e-9;
    // odom_msg->pose.covariance[35] = 0.0872665;

    // odom_msg->twist.covariance[0] = 0.001;
    // odom_msg->twist.covariance[7] = 1.0e-9;
    // odom_msg->twist.covariance[14] = 1.0e-9;
    // odom_msg->twist.covariance[21] = 1.0e-9;
    // odom_msg->twist.covariance[28] = 1.0e-9;
    // odom_msg->twist.covariance[35] = 0.001;

    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

    odom_tf.header.frame_id = odom_frame_id;
    odom_tf.child_frame_id = base_frame_id;
    odom_tf.header.stamp = time_now;

    odom_pub_->publish(std::move(odom_msg));

    if (true) {
      tf_broadcaster_->sendTransform(odom_tf);
    }
  }
  void LeoBaseDriver::dealMessageSwitch(unsigned char *recvbuf)
  {
    static bool last_touch = false;
    static bool last_plug = false;
    int curr_idx = (idx + COUNT_TIMES - 1) % COUNT_TIMES;
    //double now_time = this->get_clock()->now().seconds();

    fb_time[curr_idx] = interface_port_->current_time; // set leo base time which is different from ros time
    double odometry_x_ = interface_port_->odometry_x_;
    double odometry_y_ = interface_port_->odometry_y_;
    interface_port_->parseComInterfaceData(recvbuf, 0);
    pubGyroMessage(recvbuf+20, 18);

    interface_port_->calculateOdometry_new();
/*    if (true)
    { // use gyro's yaw
      interface_port_->odometry_yaw_ = robot_yaw;
    }

    // first, we'll publish the transforms over tf

    geometry_msgs::msg::TransformStamped odom_trans;
    rclcpp::Time time_now = this->get_clock()->now();
    odom_trans.header.stamp = time_now;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = interface_port_->odometry_x_;
    odom_trans.transform.translation.y = interface_port_->odometry_y_;
    //    ROS_DEBUG("x=%f,y=%f",interface_port_->odometry_x_,interface_port_->odometry_y_);
    odom_trans.transform.translation.z = 0.0;
    tf2::Quaternion base_yaw;
    base_yaw.setRPY(0, 0, interface_port_->odometry_yaw_);
    geometry_msgs::msg::Quaternion odom_quaternion = tf2::toMsg(base_yaw);
    odom_trans.transform.rotation = odom_quaternion;

    tf_broadcaster->sendTransform(odom_trans);

    // next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = time_now;
    odom.header.frame_id = odom_frame_id;

    // printf("%f,%f\n",interface_port_->odometry_x_,interface_port_->odometry_y_);
    // set the position
    odom.pose.pose.position.x = interface_port_->odometry_x_;
    odom.pose.pose.position.y = interface_port_->odometry_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quaternion;
    double est_x = odom_x_kfilter.predict(interface_port_->wheel_dist);

    odom_x[curr_idx] = est_x;
    odom_yaw[curr_idx] = interface_port_->odometry_yaw_;

    dt = (fb_time[curr_idx] - fb_time[idx]) * 0.0001;
    vel_x_list[curr_idx] = (odom_x[curr_idx] - odom_x[idx]) / dt;
    vel_x = 0;
    for (int i = 0; i < COUNT_TIMES; i++)
    {
      vel_x += vel_x_list[i];
    }
    vel_x = vel_x / COUNT_TIMES;

    vel_y = 0; //(odom_y[curr_idx] - odom_y[idx])/dt;

    double delodom = (odom_yaw[curr_idx] - odom_yaw[idx]);
    if (delodom > 3.14159265359)
    {
      delodom = delodom - 2 * 3.14159265359;
    }
    if (delodom < -3.14159265359)
    {
      delodom = delodom + 2 * 3.14159265359;
    }
    vel_yaw = delodom / dt;

    double tmp_dist = 0;
    fb_dist[curr_idx] = interface_port_->wheel_dist;
    for (int i = 0; i < COUNT_TIMES; i++)
    {
      tmp_dist += fb_dist[i];
    }

    double fb_x = tmp_dist / dt;

    idx = (idx + 1) % COUNT_TIMES;

    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_yaw;
    // publish the odom's message

    // add covariance
    odom.pose.covariance[0] = pow(0.01, 2);
    odom.pose.covariance[1] = pow(0.05, 2);
    odom.pose.covariance[5] = pow(0.1, 2);
    odom_pub->publish(odom);

    // printf("odom x: %f\n",odom.pose.pose.position.x);
    // publish the feedback's twist message from the leo base
    fback_cmd_vel_pub->publish(odom.twist.twist);*/

    // publish wheel joint state
    pubWheelJointStates(vel_x, vel_yaw);
    // publish irbumper

    publish_odom();

    leo_base::msg::LeoBaseSensor sensor_msg;

    sensor_msg.ir_bumper_left = interface_port_->ir_bumper_[LEFT];
    sensor_msg.ir_bumper_front_left = interface_port_->ir_bumper_[FRONT_LEFT];
    sensor_msg.ir_bumper_front = interface_port_->ir_bumper_[FRONT];
    sensor_msg.ir_bumper_front_right = interface_port_->ir_bumper_[FRONT_RIGHT];
    sensor_msg.ir_bumper_right = interface_port_->ir_bumper_[RIGHT];
    sensor_msg.ir_bumper_back_left = interface_port_->ir_bumper_[BACK_LEFT];
    sensor_msg.ir_bumper_back_right = interface_port_->ir_bumper_[BACK_RIGHT];

    sensor_msg.cliff_left = interface_port_->cliff_[LEFT];
    sensor_msg.cliff_front_left = interface_port_->cliff_[FRONT_LEFT];
    sensor_msg.cliff_front_right = interface_port_->cliff_[FRONT_RIGHT];
    sensor_msg.cliff_right = interface_port_->cliff_[RIGHT];
    sensor_msg.cliff_back_left = interface_port_->cliff_[BACK_LEFT];
    sensor_msg.cliff_back_right = interface_port_->cliff_[BACK_RIGHT];

    sensor_msg.wheel_drop_left = interface_port_->wheel_drop_[LEFT];
    sensor_msg.wheel_drop_right = interface_port_->wheel_drop_[RIGHT];
    sensor_msg.wheel_over_current_left = interface_port_->wheel_over_current_[LEFT];
    sensor_msg.wheel_over_current_right = interface_port_->wheel_over_current_[RIGHT];

    rb_sensor_pub->publish(sensor_msg);

    leo_base::msg::LeoBaseDock dock_msg;
    dock_msg.search_dock = interface_port_->search_dock_;
    dock_msg.touch_charge = interface_port_->touch_charge_;
    dock_msg.plug_charge = interface_port_->plug_charge_;
    if ((last_touch != interface_port_->touch_charge_) || (last_plug != interface_port_->plug_charge_))
    {
      if (interface_port_->touch_charge_)
        this->set_parameter(rclcpp::Parameter("battery/status", 1));
      else if (interface_port_->plug_charge_)
        this->set_parameter(rclcpp::Parameter("battery/status", 2));
      else
        this->set_parameter(rclcpp::Parameter("battery/status", 0));
      last_touch = interface_port_->touch_charge_;
      last_plug = interface_port_->plug_charge_;
    }
    dock_msg.dock_dir_left = interface_port_->dock_direction_[LEFT];
    dock_msg.dock_dir_right = interface_port_->dock_direction_[RIGHT];
    dock_msg.dock_dir_front = interface_port_->dock_direction_[FRONT];
    dock_msg.dock_dir_back = interface_port_->dock_direction_[BACK];

    rb_dock_pub->publish(dock_msg);

  }

  void LeoBaseDriver::handleGoDock(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "dock get string is :" << msg->data << std::endl;
    if (msg->data == "open")
      interface_port_->goDock(1);
    else
      interface_port_->goDock(0);
  }

  void LeoBaseDriver::handleSearchDock(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "handleSearchDock %s", msg->data.c_str());
    if (msg->data == "open")
      interface_port_->searchDock(1);
    else
      interface_port_->searchDock(0);
  }
  void LeoBaseDriver::hex_printf(unsigned char *buf, int len)
  {
#if 1
    int i;
    for (i = 0; i < len; i++)
    {
      printf("%02x ", buf[i]);
    }
    printf("\n");
#endif
  }

  void LeoBaseDriver::process_base_receive_thread()
  {
    unsigned char buf[1024];
    int len;
    while (rclcpp::ok())
    {
      if (interface_port_->GetDataGram(buf, &len) == 0)
      {
        // hex_printf(buf, len);
        getLeobaseComData(buf, len);
      }
    }
  }

  unsigned char LeoBaseDriver::checkSum(unsigned char *buf)
  {
    unsigned char sum = 0;
    int i;
    for (i = 0; i < buf[2]; i++)
    {
      sum += buf[i+3];
    }
    return sum;
  }

  void LeoBaseDriver::getLeobaseComData(unsigned char *buf, int len)
  {
    int i;
    static unsigned int count = 0;
    long long timediff;
    unsigned char tmpbuf[2550];
    static unsigned char recvbuf[2550];
    struct timeval currenttime;

    static struct timeval headertime;
    static int firsttime = 1;
    if (firsttime)
    {
      gettimeofday(&headertime, NULL);
      firsttime = 0;
    }
    gettimeofday(&currenttime, NULL);

    if (count == 0)
    {
      headertime = currenttime;
    }
    timediff = (currenttime.tv_sec - headertime.tv_sec) * 1000000 + (currenttime.tv_usec - headertime.tv_usec);

    if (timediff > LEOBASETIMEOUT)
    {
      count = 0;
       RCLCPP_ERROR(this->get_logger(), "nx-base time out-%lld\n", timediff);
      headertime = currenttime;
    }
    if ((len + count) > 2048)
    {
      count = 0;
      // RCLCPP_ERROR(this->get_logger(), "nx-base receive data too long!");
      return;
    }
    memcpy(recvbuf + count, buf, len);
    count += len;
  BACKCHECK:
    if (count > 2)
    {
      //hex_printf(buf, len);
      int checkcount = count - 1;
      for (i = 0; i < checkcount; i++)
      {
        if ((recvbuf[i] == 0x55) && (recvbuf[i + 1] == 0xAA))
        {
          if (i > 0)
          {
            count = count - i;
            memcpy(tmpbuf, recvbuf + i, count);
            memcpy(recvbuf, tmpbuf, count);
          }
          break;
        }
      }
#if 0
        if(i!=0)
        {
            for(j=0;j<count; j++)
                printf(L_GREEN "%02X " NONE ,(unsigned char)recvbuf[j]);    //
            printf("\n");
        }
#endif
      if (i == checkcount)
      {
        if (recvbuf[checkcount] == 0x55)
        {
          count = 1;
          recvbuf[0] = 0x55;
        }
        else
        {
          count = 0;
        }
      }
      if (count > 3)
      {
        unsigned int framelen = recvbuf[2]+5;
        if (recvbuf[2] < 2)
        {
          count = 0;
        }
        else
        {
          if (count >= framelen)
          {
#if 0
                    for(j=0;j<framelen; j++)
                        printf("%02X ",(unsigned char)recvbuf[j]);
                    printf("\n");
#endif

            if ((recvbuf[0] == 0x55) && (recvbuf[1] == 0xAA) && (recvbuf[framelen - 1] == 0x7D)) // check the header and end
            {
              if (checkSum(recvbuf) == recvbuf[framelen - 2])
              {
                if ((recvbuf[3] == 0x02)) // respone
                {
                  if(recvbuf[4] == 0x02)  //wheel message cmd
                  {
                    dealMessageSwitch(recvbuf+5);
                  }
                }
              }
              else
              {
                //             RCLCPP_ERROR(this->get_logger(), "Leobase-check sum error");
#if 0
                            for(j=0;j<framelen; j++)
                                printf(L_RED "%02X " NONE ,(unsigned char)recvbuf[j]);
                            printf("\n");
#endif
              }
            }
            else
            {
//            RCLCPP_ERROR(this->get_logger(), "Leobase-header or ender error error");
#if 0
                        for(j=0; j<framelen; j++)
                            printf(L_RED "%02X " NONE ,(unsigned char)recvbuf[j]);
                        printf("\n");
#endif
            }
            if (count > framelen)
            {
              memcpy(tmpbuf, recvbuf + framelen, count - framelen);
              memcpy(recvbuf, tmpbuf, count - framelen);
              count = count - framelen;
              headertime = currenttime;
              goto BACKCHECK;
            }
            count = 0;
          }
        }
      }
    }
  }

  void LeoBaseDriver::checkSerialGoon()
  {
    if (countSerial == lastCountSerial)
    {
      interface_port_->drive(0, 0.0, 0.0);
      // RCLCPP_ERROR(this->get_logger(), "checkSerialGoon");
    }
    else
    {
      lastCountSerial = countSerial;
    }
  }
}

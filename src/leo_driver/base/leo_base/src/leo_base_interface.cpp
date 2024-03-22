/*
 *  Copyright (c) 2022, NXROBO Ltd.
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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <cstdlib>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

// *****************************************************************************
// Constructor
NxLeoBase::LeoBaseInterface::LeoBaseInterface(std::string dev_name)
{
  this->resetOdometry();
  this->wheel_dist = 0;

  encoder_counts_[LEFT] = 0;
  encoder_counts_[RIGHT] = 0;

  last_encoder_counts_[LEFT] = 0;
  last_encoder_counts_[RIGHT] = 0;

  is_first_time_left = true;
  is_first_time_right = true;
  serial_port_ = std::shared_ptr<LeoSerial>(new LeoSerial(dev_name, 1000000));
}

// *****************************************************************************
// Destructor
NxLeoBase::LeoBaseInterface::~LeoBaseInterface()
{
  // Clean up!
  // delete serial_port_;
}

// *****************************************************************************
// Open the serial port
int NxLeoBase::LeoBaseInterface::openSerialPort()
{
  if (serial_port_->OpenSerial() < 0)
  {
    return -1;
  }

  return (0);
}

int NxLeoBase::LeoBaseInterface::GetDataGram(unsigned char *r_buffer, int *length)
{
  return serial_port_->GetDataGram(r_buffer, length);
}
// *****************************************************************************
// check sum
unsigned char NxLeoBase::LeoBaseInterface::checkSum(unsigned char *buf)
{
  unsigned char sum = 0;
  int i;
  for (i = 0; i < buf[2]; i++)
  {
    sum += buf[i + 3];
  }
  return sum;
}

// *****************************************************************************
// Close the serial port
int NxLeoBase::LeoBaseInterface::closeSerialPort()
{
  this->drive(0, 0.0, 0.0);
  usleep(1000 * 100);
  serial_port_->CloseSerial();

  return (0);
}

// *****************************************************************************
// Set the speeds
int NxLeoBase::LeoBaseInterface::drive(unsigned char enable, double linear_speed, double angular_speed)
{
  // int left_speed_mm_s =
  // (int)((linear_speed-LEOBASE_AXLE_LENGTH*angular_speed/2)*1e3);		// Left
  // wheel velocity in mm/s
  // int right_speed_mm_s =
  // (int)((linear_speed+LEOBASE_AXLE_LENGTH*angular_speed/2)*1e3);	// Right
  // wheel velocity in mm/s
  // 调换了左右轮子的速度,使得半弧向后
  int left_speed_mm_s =
      (int)((linear_speed - LEOBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Left wheel velocity in mm/s
  int right_speed_mm_s =
      (int)((linear_speed + LEOBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Right wheel velocity in mm/s

  // printf("%d,%d\n",left_speed_mm_s,right_speed_mm_s);
  return this->driveDirect(enable, left_speed_mm_s, right_speed_mm_s);
}

// *****************************************************************************
// Set the motor speeds
int NxLeoBase::LeoBaseInterface::driveDirect(unsigned char enable, int left_speed, int right_speed)
{
  // Limit velocity
  int16_t left_speed_mm_s = MAX(left_speed, -LEOBASE_MAX_LIN_VEL_MM_S);
  left_speed_mm_s = MIN(left_speed, LEOBASE_MAX_LIN_VEL_MM_S);
  int16_t right_speed_mm_s = MAX(right_speed, -LEOBASE_MAX_LIN_VEL_MM_S);
  right_speed_mm_s = MIN(right_speed, LEOBASE_MAX_LIN_VEL_MM_S);

  int16_t left_speed_rpm = left_speed_mm_s * LEOBASE_MM_TO_RPM;
  int16_t right_speed_rpm = right_speed_mm_s * LEOBASE_MM_TO_RPM;

  // printf("left_wheel_speed,right_wheel_speed,%d,%d\n",left_speed_pwm_s,right_speed_pwm_s);
  return this->driveRPM(enable, -left_speed_rpm, right_speed_rpm);
  return (0);
}
// *****************************************************************************
// Set the motor PWMs
int NxLeoBase::LeoBaseInterface::driveRPM(unsigned char enable, int left_speed_rpm_s, int right_speed_rpm_s)
{
  // Compose comand

  unsigned char buffer[12];
  buffer[0] = 0x55;                               // headcode1
  buffer[1] = 0xAA;                               // headcode2
  buffer[2] = 0x07;                               // len
  buffer[3] = 0x01;                               // request
  buffer[4] = 0x01;                               // cmd
  buffer[5] = enable;                             // motor_enable
  buffer[6] = (char)left_speed_rpm_s;             // data1
  buffer[7] = (char)(left_speed_rpm_s >> 8);      // data2
  buffer[8] = (char)right_speed_rpm_s;            // data3
  buffer[9] = (char)(right_speed_rpm_s >> 8);     // data4
  buffer[10] = checkSum((unsigned char *)buffer); // checksum
  buffer[11] = 0x7D;                              // endcode
  // printf("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-\n",
  //       buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],
  //       buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]);

  serial_port_->WriteBuffer(buffer, buffer[2] + 5);

  return (0);
}

// *****************************************************************************

int NxLeoBase::LeoBaseInterface::parseSenseState(unsigned char *buffer, int index)
{
  // cliff, Bumps, wheeldrops
  this->cliff_[FRONT_RIGHT] = (((buffer[index] >> 1) & 0x01) == 0) ? 1 : 0;
  this->cliff_[FRONT_LEFT] = (((buffer[index] >> 0) & 0x01) == 0) ? 1 : 0;
  this->cliff_[BACK_RIGHT] = (((buffer[index] >> 3) & 0x01) == 0) ? 1 : 0;
  this->cliff_[BACK_LEFT] = (((buffer[index] >> 2) & 0x01) == 0) ? 1 : 0;

  this->ultrasonic_[LEFT] = (buffer[index] >> 4) & 0x01;
  this->ultrasonic_[FRONT] = (buffer[index] >> 5) & 0x01;
  this->ultrasonic_[RIGHT] = (buffer[index] >> 6) & 0x01;

  this->ir_bumper_[RIGHT] = (buffer[index + 1] >> 3) & 0x01;
  this->ir_bumper_[BACK] = (buffer[index + 1] >> 4) & 0x01;
  this->ir_bumper_[FRONT_RIGHT] = (buffer[index + 1] >> 5) & 0x01;
  this->ir_bumper_[FRONT_LEFT] = (buffer[index + 1] >> 7) & 0x01;
  this->ir_bumper_[LEFT] = (buffer[index + 1] >> 6) & 0x01;

  this->search_dock_ = (buffer[index + 2] >> 5) & 0x01;
  this->touch_charge_ = (buffer[index + 2] >> 6) & 0x01;
  this->plug_charge_ = (buffer[index + 2] >> 7) & 0x01;

  this->dock_direction_[LEFT] = (buffer[index + 3] >> 4) & 0x01;
  this->dock_direction_[FRONT] = (buffer[index + 3] >> 5) & 0x01;
  this->dock_direction_[RIGHT] = (buffer[index + 3] >> 6) & 0x01;
  this->dock_direction_[BACK] = (buffer[index + 3] >> 7) & 0x01;

  this->dock_ = ((buffer[index] >> 2) & 0x01);
  return (0);
}

int NxLeoBase::LeoBaseInterface::parseRightEncoderCounts(unsigned char *buffer, int index)
{
  // Right encoder counts
  unsigned int right_encoder_counts = buffer2unsigned_int(buffer, index);
  right_encoder_counts = -right_encoder_counts;
  //  printf("Right Encoder: %d,%d,%d\n",
  //  right_encoder_counts,last_encoder_counts_[RIGHT],right_encoder_counts-last_encoder_counts_[RIGHT]);

  if (is_first_time_right ||
      right_encoder_counts == last_encoder_counts_[RIGHT]) // First time, we need 2 to make it work!
  {
    encoder_counts_[RIGHT] = 0;
    is_first_time_right = false;
  }
  else
  {
    encoder_counts_[RIGHT] = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);

    if (encoder_counts_[RIGHT] > LEOBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_[RIGHT] = encoder_counts_[RIGHT] - LEOBASE_MAX_ENCODER_COUNTS;
    if (encoder_counts_[RIGHT] < -LEOBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_[RIGHT] = LEOBASE_MAX_ENCODER_COUNTS + encoder_counts_[RIGHT];
  }
  /*static double rc_count = 0;
      rc_count +=   encoder_counts_[RIGHT];
    std::cout<<"sum rec:"<<rc_count<<rc_count*WHEEL_PULSES_TO_M<<std::endl;*/
  last_encoder_counts_[RIGHT] = right_encoder_counts;
  //    printf("Right Encoder: %d\n", encoder_counts_[RIGHT]);
  return 0;
}

int NxLeoBase::LeoBaseInterface::parseLeftEncoderCounts(unsigned char *buffer, int index)
{
  // Left encoder counts
  unsigned int left_encoder_counts = buffer2unsigned_int(buffer, index);
  left_encoder_counts = left_encoder_counts;
  //  printf("Left Encoder: %d,%d,%d\n", left_encoder_counts,
  //  last_encoder_counts_[LEFT],left_encoder_counts-last_encoder_counts_[LEFT]);

  if (is_first_time_left ||
      left_encoder_counts == last_encoder_counts_[LEFT]) // First time, we need 2 to make it work!
  {
    encoder_counts_[LEFT] = 0;
    is_first_time_left = false;
  }
  else
  {
    encoder_counts_[LEFT] = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);

    if (encoder_counts_[LEFT] > LEOBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_[LEFT] = encoder_counts_[LEFT] - LEOBASE_MAX_ENCODER_COUNTS;
    if (encoder_counts_[LEFT] < -LEOBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_[LEFT] = LEOBASE_MAX_ENCODER_COUNTS + encoder_counts_[LEFT];
  }

  last_encoder_counts_[LEFT] = left_encoder_counts;
  return 0;
}
int NxLeoBase::LeoBaseInterface::parseWheelStatus(unsigned char *buffer, int index)
{
  this->wheel_current_[LEFT] = ((short)(buffer[index] | (buffer[index + 1] << 8))) / 10;
  this->wheel_current_[RIGHT] = ((short)(buffer[index + 2] | (buffer[index + 3] << 8))) / 10;
  this->wheel_velocity_[LEFT] = ((short)(buffer[index + 4] | (buffer[index + 5] << 8))) * M_PI * WHEEL_DIAMETER_M / 60;
  this->wheel_velocity_[RIGHT] = ((short)(buffer[index + 6] | (buffer[index + 7] << 8))) * M_PI * WHEEL_DIAMETER_M / 60;
  this->wheel_status_[LEFT] = (buffer[index + 8] | (buffer[index + 9] << 8));
  this->wheel_status_[RIGHT] = (buffer[index + 10] | (buffer[index + 11] << 8));

  return 0;
}
union Char2Float_
{
  float value;
  unsigned char buffer[4];
};
int NxLeoBase::LeoBaseInterface::parseImuData(unsigned char *buffer, int index)
{
  short temp = ((short)buffer[index + 1] << 8) | buffer[index];
  float gyro_yaw = (float(temp)) / 32768 * 180;
  imu_angle = gyro_yaw / 180 * 3.1415926535898;

  return 0;
}

void NxLeoBase::LeoBaseInterface::parseComInterfaceData(unsigned char *buf, int len)
{
  // int i ;
  // for(i=0;i<20;i++)
  // {
  //   printf("%02X,",buf[i]);
  // }
  // printf("\n");
  // printf("%02X,%02X,%02X,%02X\n",buf[0],buf[1],buf[2],buf[3]);
  parseLeftEncoderCounts(buf, 0);  // 左轮
  parseRightEncoderCounts(buf, 4); // 右轮
  parseWheelStatus(buf, 8);
  parseImuData(buf, 36); // imu_yaw

  parseSenseState(buf, 38);
}
int NxLeoBase::LeoBaseInterface::buffer2signed_int(unsigned char *buffer, int index)
{
  unsigned int unsigned_int;
  unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
  return (int)unsigned_int;
}

unsigned int NxLeoBase::LeoBaseInterface::buffer2unsigned_int(unsigned char *buffer, int index)
{
  unsigned int unsigned_int;
  unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
  return unsigned_int;
}

// Calculate LEOBASE odometry
void NxLeoBase::LeoBaseInterface::calculateOdometry_new()
{
  // double dist = (encoder_counts_[RIGHT]*WHEEL_PULSES_TO_M +
  // encoder_counts_[LEFT]*WHEEL_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*WHEEL_PULSES_TO_M -
  // encoder_counts_[LEFT]*WHEEL_PULSES_TO_M) / -LEOBASE_AXLE_LENGTH;
  // 该版本特性：半弧为向前的方向
  // double dist = (encoder_counts_[RIGHT]*WHEEL_PULSES_TO_M +
  // encoder_counts_[LEFT]*WHEEL_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*WHEEL_PULSES_TO_M -
  // encoder_counts_[LEFT]*WHEEL_PULSES_TO_M) / LEOBASE_AXLE_LENGTH;
  // 该版本特性：半弧为向后的方向
  double step_time;
  double c_t;
  double v = 0.0;
  double w = 0.0;
  static unsigned int last_base_time = 0;
  double theta = 0.0;
  double delta_theta = 0.0;

  static double last_theta = 0.0;
  double dist =
      ((double)encoder_counts_[RIGHT] * WHEEL_PULSES_TO_M + (double)encoder_counts_[LEFT] * WHEEL_PULSES_TO_M) /
      -2.0;

  if (true)
  {
    theta = imu_angle;
    delta_theta = theta - last_theta;
  }
  else
  {
    theta = ((double)encoder_counts_[RIGHT] * WHEEL_PULSES_TO_M - (double)encoder_counts_[LEFT] * WHEEL_PULSES_TO_M) /
            LEOBASE_AXLE_LENGTH;
    delta_theta = theta;
  }
  // Update odometry
  this->odometry_x_ += dist * cos(this->odometry_yaw_ + (delta_theta / 2.0));
  this->odometry_y_ += dist * sin(this->odometry_yaw_ + (delta_theta / 2.0));
  this->odometry_yaw_ += delta_theta;
  // printf(" %lf, odometry_yaw_: %lf\n", delta_theta, this->odometry_yaw_);

  this->wheel_dist = this->wheel_dist + dist;
  // compute odometric instantaneouse velocity
  c_t = current_time - last_base_time;
  last_base_time = current_time;
  step_time = c_t * 0.0001;
  v = dist / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
}

// *****************************************************************************
// Reset LEOBASE odometry
void NxLeoBase::LeoBaseInterface::resetOdometry()
{
  this->setOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set LEOBASE odometry
void NxLeoBase::LeoBaseInterface::setOdometry(double new_x, double new_y, double new_yaw)
{
  this->odometry_x_ = new_x;
  this->odometry_y_ = new_y;
  this->odometry_yaw_ = new_yaw;
}

// *****************************************************************************
// Go to the dock
int NxLeoBase::LeoBaseInterface::goDock(int dock)
{
  int opcode;
  if (dock)
    opcode = 0x0010;
  else
    opcode = 0x0005;
  return 1; //waiting ...
}

// *****************************************************************************
// search to the dock
int NxLeoBase::LeoBaseInterface::searchDock(int flag)
{
  int opcode = 0x0017; // search cmd
  unsigned char value;
  if (flag)
    value = 0x00;
  else
    value = 0x01;
  return 1; //waiting ...
}
// *****************************************************************************

// EOF

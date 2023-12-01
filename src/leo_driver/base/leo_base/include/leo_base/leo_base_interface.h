#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <boost/asio.hpp>
#include <algorithm>
#include <iterator>
#include "leo_base/leo_base_serial.h"


#ifndef LEO_BASE_INTERFACE_H_
#define LEO_BASE_INTERFACE_H_

namespace NxLeoBase
{


// Positions
#define LEFT 0
#define RIGHT 1
#define FRONT_LEFT 2
#define FRONT_RIGHT 3
#define CENTER_LEFT 4
#define CENTER_RIGHT 5
#define FRONT 6
#define BACK 7
#define CENTER 8
#define BACK_LEFT 9
#define BACK_RIGHT 10
#define ENDPOS 11


// leobase Dimensions
#define WHEEL_DIAMETER_M 0.170
#define WHEEL_DIAMETER_MM 170
#define LEOBASE_AXLE_LENGTH 0.38886

#define LEOBASE_MAX_LIN_VEL_MM_S 1500
#define M_PI 3.1415926535897932384626433832795

//! leobase max encoder counts
#define LEOBASE_MAX_ENCODER_COUNTS 0xFFFFFFF
#define LEOBASE_M_TO_RPM (60/(M_PI*WHEEL_DIAMETER_M))
#define LEOBASE_MM_TO_RPM (60/(M_PI*WHEEL_DIAMETER_MM))
#define WHEEL_PULSES_TO_M  ((M_PI*WHEEL_DIAMETER_M)/4096)
#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

class LeoBaseInterface
{
public:
  LeoBaseInterface(std::string dev_name);
  ~LeoBaseInterface();
  std::shared_ptr<LeoSerial> serial_port_;
  int openSerialPort();
  int closeSerialPort();
  int GetDataGram(unsigned char* r_buffer, int *length);
  int getSensorPackets();
  void calculateOdometry();
  void calculateOdometry_new();

  int drive(unsigned char enable, double linear_speed, double angular_speed);
  int driveDirect(unsigned char enable, int left_speed, int right_speed);
  int driveRPM(unsigned char enable, int left_speed_rpm_s, int right_speed_rpm_s);
  int max();
  int goDock(int dock);
  int searchDock(int flag);
  unsigned char checkSum(unsigned char *buf);
  void parseComInterfaceData(unsigned char *buf, int index);
  void resetOdometry();
  void setOdometry(double new_x, double new_y, double new_yaw);
  double wheel_dist;
  double odometry_x_;
  double odometry_y_;
  double odometry_yaw_;
  double robot_vel_[3];
   float imu_angle;  
  bool cliff_[ENDPOS];  //! Cliff sensors. Indexes: LEFT FRONT_LEFT FRONT_RIGHT
  bool bumper_[ENDPOS];     //! Bumper sensors. Indexes: LEFT RIGHT
  bool ir_bumper_[ENDPOS];  //! IR bumper sensors. Indexes: LEFT FRONT_LEFT
  bool wheel_over_current_[2];  //! Wheel current over: Indexes: LEFT RIGHT
  bool wheel_drop_[2];          //! Wheel drop sensors: Indexes: LEFT RIGHT
  bool dock_;                     //! Whether the leobase is docked or not.
  bool control_;                  //! The leo base is control or not
  bool error_;                    //! The leo base is error or not
  float voltage_;     //! Battery voltage in volts.
  float current_;     //! Battery current in amps.
  char temperature_;  //! Battery temperature in C degrees.
  float charge_;      //! Battery charge in Ah.
  float capacity_;    //! Battery capacity in Ah
  bool search_dock_;             //! search dock
  bool touch_charge_;            //! touch charge
  bool plug_charge_;             //! plug charge
  bool dock_direction_[ENDPOS];  //! dock direction
  unsigned int current_time;  //! diff time.
private:
  int parseImuData(unsigned char *buffer, int index);

  int parseSenseState(unsigned char *buffer, int index);
  int parseLeftEncoderCounts(unsigned char *buffer, int index);
  int parseRightEncoderCounts(unsigned char *buffer, int index);
  int buffer2signed_int(unsigned char *buffer, int index);
  unsigned int buffer2unsigned_int(unsigned char *buffer, int index);
  int startOI(void);
  int sendOpcode9(int code, unsigned char value);	
  int sendOpcode(int code);
  int encoder_counts_[2];
  unsigned int last_encoder_counts_[2];
  int parseWheelDiffTime(unsigned char *buffer, int index);
  bool is_first_time_left, is_first_time_right;
};
}

#endif  // LEO_BASE_INTERFACE_H_

// EOF

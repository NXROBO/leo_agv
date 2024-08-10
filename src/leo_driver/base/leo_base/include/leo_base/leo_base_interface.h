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
#define LEOBASE_MAX_ENCODER_COUNTS 0xFFFFFFFF
#define LEOBASE_M_TO_RPM (60 / (M_PI * WHEEL_DIAMETER_M))
#define LEOBASE_MM_TO_RPM (60 / (M_PI * WHEEL_DIAMETER_MM))
#define WHEEL_PULSES_TO_M ((M_PI * WHEEL_DIAMETER_M) / 4096)
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
    int GetDataGram(unsigned char *r_buffer, int *length);
    void calculateOdometry_new();
    int drive(unsigned char enable, double linear_speed, double angular_speed);
    int driveDirect(unsigned char enable, int left_speed, int right_speed);
    int driveRPM(unsigned char enable, int left_speed_rpm_s, int right_speed_rpm_s);
    int shortCuts(unsigned char cmd, unsigned char value);
    int getEmbedSoftwareVersion();
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
    double imu_yaw_;

    float imu_angle;
    bool cliff_[ENDPOS];              // 地下探空检测
    bool ir_bumper_[ENDPOS];          // 红外检测
    bool mechainical_bumper_[ENDPOS]; // 机械碰撞，防撞条碰撞
    bool ultrasonic_[ENDPOS];         // 超声波检测

    unsigned char wheel_type;   // 轮子类型
    bool wheel_lock;            // 轮子锁住／释放状态
    bool enable_bumper;         // 使能防撞条碰撞保护功能
    bool bumper_protect_status; // 是否进入碰撞保护状态

    bool dock_;                      // ���״̬.
    float voltage_;                  //! Battery voltage in volts.
    char temperature_;               //! Battery temperature in C degrees.
    float charge_;                   //! Battery charge in Ah.
    float capacity_;                 //! Battery capacity in Ah
    bool search_dock_;               //! search dock
    bool touch_charge_;              //! touch charge
    bool plug_charge_;               //! plug charge
    bool dock_direction_[ENDPOS];    //! dock direction
    float wheel_current_[2];         // ���ӵĵ���
    float wheel_velocity_[2];        // ���ӵ��ٶ�
    unsigned short wheel_status_[2]; // ���ӵ�״̬
    int encoder_counts_[2];
    unsigned int last_encoder_counts_[2];
    unsigned int current_time; //! diff time.
    bool is_first_time_left, is_first_time_right;

  private:
    int parseImuData(unsigned char *buffer, int index);
    int parseSenseState(unsigned char *buffer, int index);
    int parseLeftEncoderCounts(unsigned char *buffer, int index);
    int parseRightEncoderCounts(unsigned char *buffer, int index);
    int buffer2signed_int(unsigned char *buffer, int index);
    unsigned int buffer2unsigned_int(unsigned char *buffer, int index);
    int parseWheelStatus(unsigned char *buffer, int index);
  };
}

#endif // LEO_BASE_INTERFACE_H_

// EOF
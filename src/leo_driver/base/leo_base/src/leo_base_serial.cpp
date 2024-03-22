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

#include "leo_base/leo_base_serial.h"
#include <fcntl.h>

namespace NxLeoBase
{

    LeoSerial::LeoSerial(std::string dev_name, uint32_t baudrate = 1000000) : serial_name_(dev_name),
                                                                              baudrate_(baudrate)
    {
    }

    LeoSerial::~LeoSerial()
    {
        if (fd_ != -1)
        {
            CloseSerial();
        }
    }

    int LeoSerial::OpenSerial()
    {
        int i;
        int speed_arr[] = {B1500000, B1000000, B460800, B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
        unsigned int name_arr[] = {1500000, 1000000, 460800, 230400, 115200, 19200, 9600, 4800, 2400, 1200, 300};
        fd_ = -1;
        fd_ = open(serial_name_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1)
        {
            printf("open serial failed :%s fail\n", serial_name_.c_str());
            return -1;
        }

        struct termios options;
        bzero(&options, sizeof(options));
        cfmakeraw(&options); // ÉèÖÃÎªRawÄ£Ê½
        // 设置串口输入波特率和输出波特率
        for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
        {
            if (baudrate_ == name_arr[i])
            {
                cfsetispeed(&options, speed_arr[i]);
                cfsetospeed(&options, speed_arr[i]);
                printf("使用波特率：%d\n", baudrate_);
                break;
            }
        }
        if (i == sizeof(speed_arr) / sizeof(int))
        {
            printf("do not support the baudrate：%d\n", baudrate_);
            close(fd_);
            return -1;
        }
        options.c_cflag |= CLOCAL; // 保证程序不会占用串口
        //  options.c_cflag &= ~CLOCAL;
        options.c_cflag |= CREAD;    // 能够从串口读取输入数据
        options.c_cflag &= ~CRTSCTS; // 不使用流控制
        options.c_cflag &= ~CSIZE;   // 先把数据位清零
        options.c_cflag |= CS8;      // 8位数据位
        options.c_cflag &= ~PARENB;  // 无奇偶校验
        options.c_cflag &= ~CSTOPB;  // 1个停止位
        options.c_oflag &= ~OPOST;   // 置输出标志：不执行输出处理
        options.c_cc[VMIN] = 0;      // 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。
        options.c_cc[VTIME] = 0;     // 如果有数据可用，则read最多返回所要求的字节数，如果无数据可用，则read立即返回0。

        tcflush(fd_, TCIFLUSH); // 清除正收到的数据，且不会读取出来。

        // 设置属性

        if (tcsetattr(fd_, TCSANOW, &options) != 0)
        {
            printf("set serial device error\n");
            return -1;
        }
        printf("open serial: %s successful!\n", serial_name_.c_str());
        return 0;
    }
    int LeoSerial::CloseSerial()
    {
        if (close(fd_) < 0)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }

    void LeoSerial::hex_printf(unsigned char *buf, int len)
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

    int LeoSerial::WriteBuffer(uint8_t *buf, uint16_t length)
    {
        return write(fd_, buf, length);
    }

    int LeoSerial::GetDataGram(unsigned char *r_buffer, int *length)
    {
        int fs_sel;
        fd_set fs_read;

        static int first_bit = 1;
        struct timeval time;
        if (fd_ == -1)
        {
            if (first_bit)
            {
                first_bit = 0;
                printf("GetDataGram: Serial is not open");
            }
            return -1;
        }

        if (1) //(!stream_stopped_)
        {
            FD_ZERO(&fs_read);
            FD_SET(fd_, &fs_read);
            time.tv_sec = 1; // mTimeLimit;
            time.tv_usec = 0;
            fs_sel = select(fd_ + 1, &fs_read, NULL, NULL, &time);
            if (fs_sel > 0)
            {
                if (FD_ISSET(fd_, &fs_read))
                {
                    *length = read(fd_, r_buffer, RECV_BUFFER_SIZE);
                    // hex_printf(r_buffer, *length);
                }
            }
            else
            {
                printf("GetDataGram: No full reply available for read after 1s");
                return -1;
            }
        }
        return 0;
    }
}

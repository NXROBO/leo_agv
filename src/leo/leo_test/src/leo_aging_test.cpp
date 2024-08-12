
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>


class MotionTest : public rclcpp::Node
{
private:
  geometry_msgs::msg::Twist cmdvel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  boost::thread t;

public:
  MotionTest(): Node("leo_test_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    t = boost::thread(boost::bind(&MotionTest::testMotion, this));
  }

  ~MotionTest()
  {
    // t.interrupt();
    t.join();
  }

  bool testMotion()
  {
    rclcpp::Time current_time = this->now();
    double current_time_sec = current_time.seconds();
    double prev_sec = current_time_sec;
    int sec = 10;
    rclcpp::Rate loop_rate(0.2);
    while (1)
    {
      if (this->now().seconds() - prev_sec < sec){
              testTurnBody(0, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      }
      else if (this->now().seconds() - prev_sec > sec && this->now().seconds() - prev_sec < 20)
      {
          testTurnBody(0, -1);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
      }
      else
      {
        prev_sec = this->now().seconds();
      }

      }
                  
    
    
    // while (1)
    // {
    //   //逆时针旋转
    //   if (this->now().seconds() - prev_sec > sec)
    //     break;
    //   printf("ni shi zhen");
    //   testTurnBody(0, 1);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    // prev_sec = this->now().seconds();
    // while (1)
    // {
    //   //顺时针旋转
    //   if (this->now().seconds() - prev_sec > sec)
    //     break;
    //   printf("shen shi zhen");
    //   testTurnBody(0, -1);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
    // prev_sec = this->now().seconds();
    // while (1)
    // {
    //   //逆时针同心圆旋转
    //   if (this->now().seconds() - prev_sec > sec)
    //     break;
    //   printf("nishizhen tongxinyuan");
    //   testTurnBody(0, 1);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }

    // prev_sec = this->now().seconds();
    // while (1)
    // {
    //   //顺时针同心圆旋转
    //   if (this->now().seconds() - prev_sec > sec)
    //     break;
    //   printf("shunshizhen tongxinyuan");
    //   testTurnBody(0, -1);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // }
  }

  void testTurnBody(float linearx, float angularz)
  {
    geometry_msgs::msg::Twist cmdvel_;
    cmdvel_.linear.x = linearx;
    cmdvel_.angular.z = angularz;
    pub_->publish(cmdvel_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto test_node = std::make_shared<MotionTest>();
  rclcpp::spin(test_node);
  rclcpp::shutdown();
  return 0;
}

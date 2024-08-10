#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "aubo_arm");


  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.508;  // 0.768
    msg.position.y = 0.102; // -0.028
    msg.position.z = 0.342;  // 0.202
    msg.orientation.x = 0.711; 
    msg.orientation.y = 0.703; 
    msg.orientation.z = 0.011; 
    msg.orientation.w = 0.016;
    return msg;
  }();

    // Create a plan to that target pose
  move_group_interface.setPoseTarget(target_pose, "wrist3_Link");
  
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }(); 

  // Execute the plan
  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown(); // <--- This will cause the spin function in the thread to
                      // return
  // spinner.join();     // <--- Join the thread before exiting
  return 0;
}

/*

看物品: Translation: [0.655, -0.003, 0.677]
             Quaternion [0.711, 0.703, 0.011, 0.016]

运货-前: Translation: [0.341, 0.109, 0.592]
              Quaternion [0.704, 0.691, 0.118, 0.115]

运货-后: Translation: [-0.086, 0.029, 0.561]
              Quaternion [-0.450, 0.878, 0.145, -0.078
              
放置物品: Translation: [0.731, -0.025, 0.354]
                Quaternion [0.719, 0.673, 0.131, 0.115]

*/

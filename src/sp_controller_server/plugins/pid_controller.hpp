#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp> // 位姿消息
#include <geometry_msgs/msg/twist.hpp> // 速度消息
#include <geometry_msgs/msg/twist_stamped.hpp> // 速度消息（带时间戳）
#include <nav_msgs/msg/path.hpp> // 路径消息
#include <rclcpp/rclcpp.hpp> // ROS2 核心
#include <tf2_ros/buffer.h> // TF 缓存

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;
  // override ：显式声明“我在重写基类的虚函数” 。如果签名写错（比如参数类型不对），
  // 编译器立刻报错而不是静默生成一个新函数。强烈建议以后写重写函数时都带上它

  void configure(
    const rclcpp::Node::SharedPtr & node, 
    const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  std::string plugin_name_;  // 插件实例名configure存下来，读参数时拼前缀用
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // TF缓存指针，configure存下来备用
  rclcpp::Node::SharedPtr node_;  // 宿主节点指针，读 yaml 参数用
  nav_msgs::msg::Path global_plan_;  //当前要跟踪的全局路径——类的核心状态
  std::string base_frame_id_;  // 机器人本体坐标系名（"base_link"）

  double kp_v_ = 0.0;
  double v_max_ = 0.0;
  double lookahead_dist_ = 0.0;
  double arrive_tol_ = 0.0;

  double kd_v_ = 0.0;
  double decel_dist_ = 0.0;
  double a_max_ = 0.0;

  //state
  double last_ey_ = 0.0;
  double last_speed_ = 0.0;
};

}

#endif
#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

//匿名命名空间的工具函数
namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

//必填参数读取器
template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

}

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");
  // TODO: 在此读取参数
  kp_v_ = require_param<double>(node_, plugin_name_ + ".kp_v");
  v_max_ = require_param<double>(node_, plugin_name_ + ".v_max");
  lookahead_dist_ = require_param<double>(node_, plugin_name_ + ".lookahead_distance");
  arrive_tol_ = require_param<double>(node_, plugin_name_ + ".arrive_tol");

  kd_v_ = require_param<double>(node_, plugin_name_ + ".kd_v");
  decel_dist_ = require_param<double>(node_, plugin_name_ + ".decel_dist");
  a_max_ = require_param<double>(node_, plugin_name_ + ".a_max");
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  // TODO：在此实现路径跟踪
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  if(global_plan_.poses.empty()) return cmd_vel;


  // Find the point in the path which is closest to the current position
  double best_d2 = std::numeric_limits<double>::max();
  size_t nearest = 0;
  for(size_t i = 0; i < global_plan_.poses.size(); i++){
    const auto & p = global_plan_.poses[i].pose.position;
    double dx = p.x - pose.pose.position.x;
    double dy = p.y - pose.pose.position.y;
    double d2 = dx*dx + dy*dy;
    if(d2 < best_d2){
      best_d2 = d2;
      nearest = i;}
  }


  // Find the target in the path depond on the lookahead_dist_
  size_t target = nearest;
  double acc = 0.0;
  for(size_t i = nearest; i < global_plan_.poses.size() - 1; i++){
    const auto & a = global_plan_.poses[i].pose.position;
    const auto & b = global_plan_.poses[i+1].pose.position;
    acc += std::hypot(b.x - a.x, b.y - a.y);
    target += 1;
    if(acc >= lookahead_dist_ + 0.5 * last_speed_) break;
  }


  // project target position into the car coordinate system
  const auto & q = pose.pose.orientation;
  double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const auto & tp = global_plan_.poses[target].pose.position;
  double mx = tp.x - pose.pose.position.x;
  double my = tp.y - pose.pose.position.y;

  double ex = std::cos(yaw) * mx + std::sin(yaw) * my;
  double ey = -std::sin(yaw) * mx + std::cos(yaw) * my;


  // control and calculate v with lateral damping
  const auto & gp = global_plan_.poses.back().pose.position;
  double dist_goal = std::hypot(gp.x - pose.pose.position.x, gp.y - pose.pose.position.y);

  // stop
  if(dist_goal < arrive_tol_) {
    last_ey_ = 0.0;
    last_speed_ = 0.0;
    return cmd_vel;
  }

  double d_ey = ey - last_ey_;
  last_ey_ = ey;

  double vx = ex * kp_v_;
  double vy = ey * kp_v_ + d_ey * kd_v_;
  double speed = std::hypot(vx, vy);

  // decelerate near the destination
  double v_allow = v_max_;
  if (dist_goal < decel_dist_){
    v_allow = v_max_ * dist_goal / decel_dist_;
  }
  v_allow = std::max(v_allow, 0.1);
  if (speed > v_allow){
    vx *= v_allow / speed;
    vy *= v_allow / speed;
    speed = v_allow;
  }

  // acceleration limiting
  double dt = 0.02;
  double dv_max = a_max_ * dt;
  speed = std::clamp(speed, last_speed_ - dv_max, last_speed_ + dv_max);
  last_speed_ = speed;
  if(speed > 1e-6){
    double scale = speed / std::hypot(vx,vy);
    vx *= scale;
    vy *= scale;
  }
  

  cmd_vel.twist.linear.x = vx;
  cmd_vel.twist.linear.y = vy;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

}

// 插件导出
PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
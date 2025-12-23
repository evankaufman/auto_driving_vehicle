#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

using namespace std::chrono_literals;

struct VehicleState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct FrenetState
{
  double s{0.0};
  double s_dot{0.0};
  double d{0.0};
  double d_dot{0.0};
  double d_ddot{0.0};
};

struct DriveCommand
{
  double torque_fl{0.0};
  double torque_fr{0.0};
  double torque_rl{0.0};
  double torque_rr{0.0};
  double steering_angle{0.0};
  double steering_rate{0.0};
};

class MpcCtrlNode : public rclcpp::Node
{
public:
  MpcCtrlNode()
  : Node("mpc_ctrl")
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/model/suv_vehicle/odometry");
    drive_cmd_topic_ = declare_parameter<std::string>(
      "drive_cmd_topic", "/mpc_ctrl/drive_cmd");
    ref_path_topic_ = declare_parameter<std::string>(
      "ref_path_topic", "/mpc_ctrl/reference_path");
    mpc_path_topic_ = declare_parameter<std::string>(
      "mpc_path_topic", "/mpc_ctrl/mpc_path");

    use_dummy_cmd_ = declare_parameter<bool>("use_dummy_cmd", true);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MpcCtrlNode::on_odom, this, std::placeholders::_1));

    drive_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      drive_cmd_topic_, 10);
    ref_path_pub_ = create_publisher<nav_msgs::msg::Path>(ref_path_topic_, 10);
    mpc_path_pub_ = create_publisher<nav_msgs::msg::Path>(mpc_path_topic_, 10);

    timer_ = create_wall_timer(50ms, std::bind(&MpcCtrlNode::on_timer, this));
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Horizontal position comes straight from gz msg
    latest_state_.x = msg->pose.pose.position.x;
    latest_state_.y = msg->pose.pose.position.y;

    // Get yaw from quaternion
    const double qx = msg->pose.pose.orientation.x;
    const double qy = msg->pose.pose.orientation.y;
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    latest_state_.yaw = std::atan2(siny_cosp, cosy_cosp);

    // Twist is expressed wrt body frame, so bring that to inertial frame
    const double cos_yaw = std::cos(latest_state_.yaw);
    const double sin_yaw = std::sin(latest_state_.yaw);
    latest_state_.vx = cos_yaw * msg->twist.twist.linear.x - sin_yaw * msg->twist.twist.linear.y;
    latest_state_.vy = sin_yaw * msg->twist.twist.linear.x + cos_yaw * msg->twist.twist.linear.y;

    // Assuming z is always up, angular velocity can be taken directly from gz msg
    latest_state_.wz = msg->twist.twist.angular.z;

    last_stamp_ = msg->header.stamp;
    have_state_ = true;
  }

  /// @brief  Converts Cartesian coordinates to Frenet Coordinates
  /// @param state 
  /// @return Freenet coordinates
  FrenetState cartesian_to_frenet(const VehicleState &cart_state)
  {
    FrenetState frenet_state;

    // Let's start simple... follow x-axis at 1 m/sec
    frenet_state.s = 0.0;
    frenet_state.s_dot = cart_state.vx;

    frenet_state.d = cart_state.y;
    frenet_state.d_dot = cart_state.vy;
    frenet_state.d_ddot = 0.0; // TODO: maybe pull this from estimation or last cmd

    return frenet_state;
  }

  std::vector<FrenetState> reference_trajectory(const FrenetState &current)
  {
    // TODO: Replace with desired reference trajectory in Frenet frame.
    std::vector<FrenetState> ref;
    ref.reserve(20);
    double s = current.s;
    for (int i = 0; i < 20; ++i) {
      FrenetState f;
      f.s = s + i * 0.5;
      f.s_dot = current.s_dot;
      ref.push_back(f);
    }
    return ref;
  }

  DriveCommand dummy_cmd(double t_sec)
  {
    DriveCommand cmd;
    const double max_torque = 100.0;
    if (t_sec < 3.0) {
      cmd.torque_fl = 0.0;
      cmd.torque_fr = 0.0;
      cmd.torque_rl = 0.0;
      cmd.torque_rr = 0.0;
    } else if (t_sec < 13.0) {
      const double ramp = (t_sec - 3.0) / 10.0;
      const double torque = max_torque * ramp;
      cmd.torque_fl = torque;
      cmd.torque_fr = torque;
      cmd.torque_rl = torque;
      cmd.torque_rr = torque;
    } else {
      cmd.torque_fl = max_torque;
      cmd.torque_fr = max_torque;
      cmd.torque_rl = max_torque;
      cmd.torque_rr = max_torque;
    }

    cmd.steering_angle = 10.0 * M_PI / 180.0;
    cmd.steering_rate = 0.0;

    return cmd;
  }

  DriveCommand mpc_controller(
    const FrenetState &current,
    const std::vector<FrenetState> &reference)
  {
    // TODO: Implement MPC. This stub returns zeros.
    (void)current;
    (void)reference;
    return DriveCommand{};
  }

  void publish_paths(
    const std::vector<FrenetState> &ref,
    const std::vector<FrenetState> &mpc)
  {
    // TODO: Convert Frenet trajectories to Cartesian and publish as nav_msgs/Path.
    nav_msgs::msg::Path ref_path;
    nav_msgs::msg::Path mpc_path;
    ref_path.header.stamp = now();
    mpc_path.header.stamp = ref_path.header.stamp;
    ref_path.header.frame_id = "map";
    mpc_path.header.frame_id = "map";

    for (const auto &f : ref) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = ref_path.header;
      pose.pose.position.x = f.s;
      pose.pose.position.y = f.d;
      pose.pose.orientation.w = 1.0;
      ref_path.poses.push_back(pose);
    }

    for (const auto &f : mpc) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = mpc_path.header;
      pose.pose.position.x = f.s;
      pose.pose.position.y = f.d;
      pose.pose.orientation.w = 1.0;
      mpc_path.poses.push_back(pose);
    }

    ref_path_pub_->publish(ref_path);
    mpc_path_pub_->publish(mpc_path);
  }

  void on_timer()
  {
    if (!have_state_) {
      return;
    }

    FrenetState frenet = cartesian_to_frenet(latest_state_);
    std::cout << "Frenet state: (s: " << frenet.s
              << ", s_dot: " << frenet.s_dot
              << ", d: " << frenet.d
              << ", d_dot: " << frenet.d_dot
              << ", d_ddot: " << frenet.d_ddot
              << ")"
              << std::endl;
    // FrenetState frenet = cartesian_to_frenet(smooth_along_x_1mps, 1.0, latest_state_);
    auto reference = reference_trajectory(frenet);

    DriveCommand cmd;
    if (use_dummy_cmd_) {
      if (!have_start_time_) {
        start_time_ = now();
        have_start_time_ = true;
      }
      const double t_sec = (now() - start_time_).seconds();
      cmd = dummy_cmd(t_sec);
      std::cout << "cmd.steering_angle = " << cmd.steering_angle << std::endl;
    } else {
      cmd = mpc_controller(frenet, reference);
    }

    std_msgs::msg::Float64MultiArray out;
    out.data = {
      cmd.torque_fl,
      cmd.torque_fr,
      cmd.torque_rl,
      cmd.torque_rr,
      cmd.steering_angle,
      cmd.steering_rate
    };
    drive_pub_->publish(out);

    // Placeholder MPC trajectory: echo reference for now.
    publish_paths(reference, reference);
  }

  VehicleState latest_state_;
  rclcpp::Time last_stamp_;
  bool have_state_{false};

  bool use_dummy_cmd_{true};
  bool have_start_time_{false};
  rclcpp::Time start_time_;

  std::string odom_topic_;
  std::string drive_cmd_topic_;
  std::string ref_path_topic_;
  std::string mpc_path_topic_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcCtrlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

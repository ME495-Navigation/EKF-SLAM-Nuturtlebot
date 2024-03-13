/// \file odometry.cpp
/// \brief This is a ROS2 node that publishes odometry messages and the odometry transform.
/// PARAMETERS:
///   body_id (string): The name of the robot's body frame.
///   odom_id (string): The name of the robot's odometry frame.
///   wheel_left (string): The name of the left wheel joint.
///   wheel_right (string): The name of the right wheel joint.
///   wheel_radius (double): The radius of the wheels.
///   track_width (double): The distance between the wheels.
/// PUBLISHES:
///   odom (nav_msgs::msg::Odometry): The odometry message.
///   tf (tf2_msgs::msg::TFMessage): The transform message.
/// SUBSCRIBES:
///   joint_states (sensor_msgs::msg::JointState): The joint state message.
/// SERVICES:
///   initial_pose (nuturtle_control::srv::InitialPose): The service to set the initial pose of the robot.

// publishes odometry messages and the odometry transform
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
private:
  // params
  double wheel_radius = 999.0;
  double track_width = 999.0;
  std::string body_id;
  std::string odom_id;
  std::string wheel_left;
  std::string wheel_right;

  // joint state callback
  void jointstate_callback(const sensor_msgs::msg::JointState & js)
  {
    // calculate next wheel pos
    turtlelib::WheelAng w;
    // w.right_ang = js.position.at(0) + wheel_pos_last.right_ang;
    w.right_ang = js.position.at(0) + js.velocity.at(0) / 200.0;
    // w.left_ang = js.position.at(1) - wheel_pos_last.left_ang;
    w.left_ang = js.position.at(1) + js.velocity.at(1) / 200.0;

    // update last wheel pos to current wheel pos
    wheel_pos_last.right_ang = js.position.at(0);
    wheel_pos_last.left_ang = js.position.at(1);

    // use fk to update q
    turtlelib::Twist2D tw = diffdrive.f_kin(w.right_ang, w.left_ang);
    // take curr q
    turtlelib::Transform2D q = diffdrive.get_q();
    // get time now
    odom_f.header.stamp = get_clock()->now();
    // set q.x and q.y
    odom_f.pose.pose.position.x = q.translation().x;

    odom_f.pose.pose.position.y = q.translation().y;
    // set q.theta
    tf2::Quaternion Q;
    // RCLCPP_ERROR_STREAM(get_logger(), "q: " << q);
    Q.setRPY(0, 0, q.rotation());
    odom_f.pose.pose.orientation.x = Q.x();
    odom_f.pose.pose.orientation.y = Q.y();
    odom_f.pose.pose.orientation.z = Q.z();
    odom_f.pose.pose.orientation.w = Q.w();
    // now twist
    odom_f.twist.twist.linear.x = tw.x;
    odom_f.twist.twist.linear.y = tw.y;
    odom_f.twist.twist.angular.z = tw.omega;

    // update transform stamped
    geometry_msgs::msg::TransformStamped tf;
    // take time now with transform
    tf.header.stamp = get_clock()->now();
    // set frame id
    tf.header.frame_id = odom_id;
    // set child frame id
    tf.child_frame_id = body_id;
    // set translation
    tf.transform.translation.x = q.translation().x;
    tf.transform.translation.y = q.translation().y;
    tf.transform.translation.z = 0.0;
    // set rotation
    tf.transform.rotation.x = Q.x();
    tf.transform.rotation.y = Q.y();
    tf.transform.rotation.z = Q.z();
    tf.transform.rotation.w = Q.w();

    // create pose stamped for path
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = odom_id;
    pose.pose.position.x = q.translation().x;
    pose.pose.position.y = q.translation().y;
    
    pose.pose.orientation.x = Q.x();
    pose.pose.orientation.y = Q.y();
    pose.pose.orientation.z = Q.z();
    pose.pose.orientation.w = Q.w();

    // create path - and publish every 100 timesteps to avoid lagging
    if (timestep_%100 == 1){
      path.header.stamp = get_clock()->now();
      path.header.frame_id = odom_id;
      path.poses.push_back(pose);
      // publish path
      path_pub_->publish(path);
    }

    // publish odom and tf
    odom_pub->publish(odom_f);
    tf_broadcaster->sendTransform(tf);
    timestep_++;
  }

  // initial pose service
  void initial_pose(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    // reset simulation
    turtlelib::Transform2D reset_rob {{request->x, request->y}, request->w};
    diffdrive.q_new(reset_rob);
  }

  // other to init
  size_t timestep_;
  turtlelib::DiffDrive diffdrive{wheel_radius, track_width};
  turtlelib::WheelAng wheel_pos_last;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_sub;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;
  nav_msgs::msg::Odometry odom_f;
  nav_msgs::msg::Path path;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

public:
  Odometry()
  : Node("odometry"), timestep_(0)
  {
    // declare parameters + log error message if does not exist
    declare_parameter("rate", 200);

    declare_parameter("body_id", body_id);
    body_id = get_parameter("body_id").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "body_id: " << body_id);
    if (body_id.empty()) {
      throw std::logic_error("body_id is empty!");
    }

    declare_parameter("odom_id", odom_id);
    odom_id = get_parameter("odom_id").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "odom_id: " << odom_id);
    // set default to odom if left empty
    if (odom_id.empty()) {
      odom_id = "odom";
    }

    declare_parameter("wheel_left", wheel_left);
    wheel_left = get_parameter("wheel_left").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "wheel_left: " << wheel_left);
    if (wheel_left.empty()) {
      throw std::logic_error("wheel_left is empty!");
    }

    declare_parameter("wheel_right", wheel_right);
    wheel_right = get_parameter("wheel_right").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "wheel_right: " << wheel_right);
    if (wheel_right.empty()) {
      throw std::logic_error("wheel_right is empty!");
    }

    declare_parameter("wheel_radius", wheel_radius);
    wheel_radius = get_parameter("wheel_radius").as_double();

    declare_parameter("track_width", track_width);
    track_width = get_parameter("track_width").as_double();

    // publisher
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // path publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path_blue", 10);

    // subscriber
    jointstate_sub = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &Odometry::jointstate_callback, this, std::placeholders::_1));

    // frame stuff for odom object and tf object
    odom_f.header.frame_id = odom_id;
    odom_f.child_frame_id = body_id;

    // broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // initial pose service
    // initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
    //     "initial_pose",
    //     std::bind(&Odometry::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
    initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
      "initial_pose", std::bind(
        &Odometry::initial_pose, this, std::placeholders::_1, std::placeholders::_2));

    // additional initializations
    diffdrive = {wheel_radius, track_width};
    wheel_pos_last.left_ang = 0.0;
    wheel_pos_last.right_ang = 0.0;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}

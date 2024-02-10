// main simulation node
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/qos.hpp>
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    // set parameters
    declare_parameter("rate", 200);
    rate_hz = get_parameter("rate").as_int();
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

    declare_parameter("x0", 0.0);
    x_ = get_parameter("x0").as_double();

    declare_parameter("y0", 0.0);
    y_ = get_parameter("y0").as_double();

    declare_parameter("theta0", 0.0);
    theta_ = get_parameter("theta0").as_double();

    declare_parameter("arena_x_length", 4.0);
    arena_x_length_ = get_parameter("arena_x_length").as_double();

    declare_parameter("arena_y_length", 4.0);
    arena_y_length_ = get_parameter("arena_y_length").as_double();

    declare_parameter("obstacles/x", obstacle_x_);
    obstacle_x_ = get_parameter("obstacles/x").as_double_array();

    declare_parameter("obstacles/y", obstacle_y_);
    obstacle_y_ = get_parameter("obstacles/y").as_double_array();

    declare_parameter("obstacles/r", 0.038);
    obstacle_r_ = get_parameter("obstacles/r").as_double();

    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();

    declare_parameter("track_width", 0.16);
    track_width = get_parameter("track_width").as_double();

    declare_parameter("wheel_radius", 0.033);
    wheel_radius = get_parameter("wheel_radius").as_double();

    declare_parameter("encoder_ticks_per_rad", 651.8986);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();

    wheel_vel = {0.0, 0.0};
    diffdrive = {wheel_radius, track_width};

    // timestep publisher
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    // timer
    timer_ = create_wall_timer(rate, std::bind(&Nusim::timer_callback, this));
    // reset service
    reset_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));
    // transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // teleport service
    teleport_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));
    // wall publisher
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      qos_policy);

    // obstacle publisher
    obs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_policy);

    // sensor data publisher
    sensor_data_pub_ = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);

    // wheel command subscriber
    wheelcom_sub_ = this->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd",
      10,
      std::bind(&Nusim::wheelcom_callback, this, std::placeholders::_1));

    // set up simulation
    move_robot(x_, y_, theta_);
    create_arena();
    show_obstacles();
  }

private:
  // main timer
  void timer_callback()
  {
    std_msgs::msg::UInt64 msg;
    msg.data = timestep_;
    timestep_pub_->publish(msg);

    // update robot state
    double right_ang_new = wheel_vel.right_ang / 200.0 + diffdrive.get_phi().right_ang;
    double left_ang_new = wheel_vel.left_ang / 200.0 + diffdrive.get_phi().left_ang;
    // double right_ang_new = wheel_vel.right_ang / 200.0;
    // double left_ang_new = wheel_vel.left_ang / 200.0;

    diffdrive.f_kin(right_ang_new, left_ang_new);
    // update transform
    tf.transform.translation.x = diffdrive.get_q().translation().x;
    tf.transform.translation.y = diffdrive.get_q().translation().y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, diffdrive.get_q().rotation());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    // publish transform
    tf.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(tf);

    // update + publish sensor data
    sensor_data.right_encoder = static_cast<int>(right_ang_new * encoder_ticks_per_rad);
    sensor_data.left_encoder = static_cast<int>(left_ang_new * encoder_ticks_per_rad);
    sensor_data.stamp = get_clock()->now();
    sensor_data_pub_->publish(sensor_data);

    timestep_++;
  }

  // reset service - restores state of simulation
  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    move_robot(0.0, 0.0, 0.0);
    RCLCPP_INFO(get_logger(), "Resetting System!");
  }

  // broadcast transform between nusim/world and red/base_footprint
  void move_robot(double x, double y, double theta)
  {
    // define transform
    // geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "nusim/world";
    tf.child_frame_id = "red/base_footprint";

    x_ = x;
    y_ = y;
    theta_ = theta;
    // calculate transform
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.0;

    // initialize quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    // send transform
    // tf_broadcaster_->sendTransform(tf);
  }

  // teleport turtle to given location
  void teleport(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
    RCLCPP_INFO(get_logger(), "Moving robot!");
    move_robot(x_, y_, theta_);
  }

  // create a wall marker
  visualization_msgs::msg::Marker make_wall(int id, double scale[], const double pose[])
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "nusim/world";
    m.header.stamp = get_clock()->now();
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = scale[0];
    m.scale.y = scale[1];
    m.scale.z = scale[2];

    m.pose.position.x = pose[0];
    m.pose.position.y = pose[1];
    m.pose.position.z = pose[2];
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.id = id;
    m.frame_locked = true;
    return m;
  }

  // show arena
  void create_arena()
  {
    // visualization_msgs needed
    auto arena = visualization_msgs::msg::MarkerArray();
    auto wall = visualization_msgs::msg::Marker();

    // location
    double pose[3] = {0.0, 0.0, 0.0};
    pose[2] = wall_height;

    double scale[3] = {0.0, 0.0, 0.0};
    scale[2] = wall_height / 2;

    for (int i = 0; i < 4; ++i) {
      if (i == 0 || i == 2) {
        //left
        scale[0] = arena_x_length_;
        scale[1] = wall_thick;
        pose[0] = 0;
        pose[1] = arena_y_length_ / 2;
      }
      if (i == 2) {
        pose[1] *= -1;
      }
      if (i == 1 || i == 3) {
        scale[0] = wall_thick;
        scale[1] = arena_y_length_;
        pose[0] = arena_x_length_ / 2;
        pose[1] = 0;
      }
      if (i == 3) {
        pose[0] *= -1;
      }
      wall = make_wall(i, scale, pose);
      arena.markers.push_back(wall);
    }
    wall_pub_->publish(arena);
  }

  // display obstacles in arena
  void show_obstacles()
  {
    // visualization_msgs needed
    auto obs = visualization_msgs::msg::MarkerArray();
    auto ob = visualization_msgs::msg::Marker();

    // check if x and y are same if not log error
    if (obstacle_x_.size() != obstacle_y_.size()) {
      RCLCPP_ERROR_STREAM(get_logger(), "x and y vectors for the obstacles are not the same size!");
      return;
    }

    for (int i = 0; i < int(obstacle_x_.size()); ++i) {
      ob.header.frame_id = "nusim/world";
      ob.header.stamp = get_clock()->now();
      ob.type = visualization_msgs::msg::Marker::CYLINDER;
      ob.action = visualization_msgs::msg::Marker::ADD;
      ob.color.r = 1.0;
      ob.color.a = 1.0;
      ob.id = i + 4;
      ob.frame_locked = true;
      ob.scale.x = obstacle_r_ * 2;
      ob.scale.y = obstacle_r_ * 2;
      ob.scale.z = wall_height;
      ob.pose.position.x = obstacle_x_[i];
      ob.pose.position.y = obstacle_y_[i];
      ob.pose.position.z = wall_height / 2;

      obs.markers.push_back(ob);
    }
    obs_pub_->publish(obs);
  }

  void wheelcom_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    wheel_vel =
    {msg->left_velocity * motor_cmd_per_rad_sec, msg->right_velocity * motor_cmd_per_rad_sec};
  }

  // initialize variables
  size_t timestep_;
  double rate_hz = 0.0;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double wall_height = 0.25;
  double wall_thick = 0.20;
  double arena_x_length_ = 4.0;
  double arena_y_length_ = 4.0;
  std::vector<double> obstacle_x_ = {0.25, 0.75};
  std::vector<double> obstacle_y_ = {0.25, -0.25};
  float obstacle_r_ = 0.05;
  double motor_cmd_per_rad_sec;
  double track_width;
  double wheel_radius;
  double encoder_ticks_per_rad;
  turtlelib::DiffDrive diffdrive{wheel_radius, track_width};
  turtlelib::WheelAng wheel_vel = {0.0, 0.0};
  nuturtlebot_msgs::msg::SensorData sensor_data;


  std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000.0 / 200.0));
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcom_sub_;
  geometry_msgs::msg::TransformStamped tf = geometry_msgs::msg::TransformStamped();

};

int main(int argc, char * argv[])
{
  // spin the node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}

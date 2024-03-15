// include necessary headers/libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "turtlelib/kalman.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class Slam : public rclcpp::Node
{
private:
    // params
    double wheel_radius = 999.0;
    double track_width = 999.0;
    std::string slam_body_id;
    std::string slam_odom_id;
    std::string wheel_left;
    std::string wheel_right;

    // joint state callback
    void jointstate_callback(const sensor_msgs::msg::JointState & js)
    {
        // calculate next wheel pos
        turtlelib::WheelAng w;
        w.right_ang = js.position.at(0) - js_msg_old.position.at(0);
        w.left_ang = js.position.at(1) - js_msg_old.position.at(1);
        
        // use fk to update q
        turtlelib::Transform2D q = diff_drive.get_q();
        
        // publish new robot config as odom msg
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = get_clock()->now();
        odom_msg.header.frame_id = slam_odom_id;
        odom_msg.child_frame_id = slam_body_id;
        odom_msg.pose.pose.position.x = q.translation().x;
        odom_msg.pose.pose.position.y = q.translation().y;
        odom_msg.pose.pose.position.z = 0.0;
        
        tf2::Quaternion Q;
        tf2::Quaternion Q_map;
        Q.setRPY(0, 0, q.rotation());
        odom_msg.pose.pose.orientation.x = Q.x();
        odom_msg.pose.pose.orientation.y = Q.y();
        odom_msg.pose.pose.orientation.z = Q.z();
        odom_msg.pose.pose.orientation.w = Q.w();
        odom_pub->publish(odom_msg);

        // publish new robot config as tf
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = get_clock()->now();
        odom_tf.header.frame_id = slam_odom_id;
        odom_tf.child_frame_id = slam_body_id;
        odom_tf.transform.translation.x = q.translation().x;
        odom_tf.transform.translation.y = q.translation().y;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = Q.x();
        odom_tf.transform.rotation.y = Q.y();
        odom_tf.transform.rotation.z = Q.z();
        odom_tf.transform.rotation.w = Q.w();
        tf_broadcaster->sendTransform(odom_tf);
        js_msg_old = js;

        odom_to_rob = turtlelib::Transform2D{turtlelib::Vector2D{q.translation().x, q.translation().y}, q.rotation()};
        map_to_odom = map_to_rob * odom_to_rob.inv();

        geometry_msgs::msg::TransformStamped map_odom_tf;
        map_odom_tf.header.stamp = get_clock()->now();
        map_odom_tf.header.frame_id = "map";
        map_odom_tf.child_frame_id = slam_odom_id;
        map_odom_tf.transform.translation.x = map_to_odom.translation().x;
        map_odom_tf.transform.translation.y = map_to_odom.translation().y;
        map_odom_tf.transform.translation.z = 0.0;
        Q_map.setRPY(0, 0, map_to_odom.rotation());

        map_odom_tf.transform.rotation.x = Q_map.x();
        map_odom_tf.transform.rotation.y = Q_map.y();
        map_odom_tf.transform.rotation.z = Q_map.z();
        map_odom_tf.transform.rotation.w = Q_map.w();
        tf_map_broadcaster->sendTransform(map_odom_tf);
    }

    // check params and throw error if not set
    void check_params()
    {
        if(slam_body_id.empty() || wheel_right.empty() || wheel_left.empty() || wheel_radius == 0.0 || track_width == 0.0)
        {
            RCLCPP_ERROR(get_logger(), "SLAM params not set!");
            throw std::runtime_error("SLAM params not set!");
        }
    }

    // initial pose service callback
    void initial_pose(
        const std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
        std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
    {
        // set initial pose
        turtlelib::Transform2D s {{request->x, request->y}, request->w};
        diff_drive.q_new(s);
    }

    // fake sensor callback
    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr lidar_msg)
    {
        // get robot config
        auto q = diff_drive.get_q();

        // use EKF
        // ekf prediction - take in current robot twist
        
        turtlelib::Twist2D tw = {q.translation().x, q.translation().y, q.rotation()};
        
        ekf.predict(turtlelib::Twist2D{tw.x, tw.y, tw.omega});
        // print output of predict
        // RCLCPP_ERROR(get_logger(), "Predicted state: %f %f %f %f %f", ekf.get_state_estimate()(0), ekf.get_state_estimate()(1), ekf.get_state_estimate()(2), ekf.get_state_estimate()(3), ekf.get_state_estimate()(4));
        // update ekf
        // RCLCPP_ERROR(get_logger(), "FINISHED PREDICTION");
        for(size_t i = 0; i < lidar_msg->markers.size(); i++)
        {
            // RCLCPP_ERROR(get_logger(), "ENTERED UPDATE LOOP");
            // get marker
            visualization_msgs::msg::Marker marker = lidar_msg->markers[i];
            // get marker position
            if(marker.action == visualization_msgs::msg::Marker::ADD)
            {
                // RCLCPP_ERROR(get_logger(), "ENTERED UPDATE LOOP 22222222222222222");
                ekf.update(marker.pose.position.x, marker.pose.position.y, i);
                // print output of update
                // RCLCPP_ERROR(get_logger(), "Updated state: %f %f %f %f %f", ekf.get_state_estimate()(0), ekf.get_state_estimate()(1), ekf.get_state_estimate()(2), ekf.get_state_estimate()(3), ekf.get_state_estimate()(4));
            }
        }

        // get state from ekf
        auto xk = ekf.get_state_estimate();
        auto rob_pos = turtlelib::Vector2D{xk(1), xk(2)};
        // add to slam path and publish
        tf2::Quaternion Q_st;
        Q_st.setRPY(0, 0, xk(0));
        path.header.stamp = get_clock()->now();
        path.header.frame_id = "nusim/world";
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = rob_pos.x;
        pose.pose.position.y = rob_pos.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = Q_st.x();
        pose.pose.orientation.y = Q_st.y();
        pose.pose.orientation.z = Q_st.z();
        pose.pose.orientation.w = Q_st.w();
        if (timestep_%100 == 1){
            path.poses.push_back(pose);
            slam_path_pub_->publish(path);
        }
        
        auto rob_theta = xk(0);
        map_to_rob = turtlelib::Transform2D{rob_pos, rob_theta};

        // publish slam obstacles as marker array
        visualization_msgs::msg::MarkerArray obs;
        for(size_t i =0; i < lidar_msg->markers.size(); i++)
        {
            visualization_msgs::msg::Marker ob;
            ob.header.stamp = get_clock()->now();
            ob.header.frame_id = "nusim/world";
            ob.type = visualization_msgs::msg::Marker::CYLINDER;
            ob.id = lidar_msg->markers.at(i).id + 1000;
            ob.pose.position.x = xk(3+2*i);
            ob.pose.position.y = xk(4+2*i);
            ob.pose.position.z = lidar_msg->markers.at(i).pose.position.z;
            ob.scale.x = lidar_msg->markers.at(i).scale.x;
            ob.scale.y = lidar_msg->markers.at(i).scale.y;
            ob.scale.z = lidar_msg->markers.at(i).scale.z;
            ob.color.g = 1.0;
            ob.color.a = 1.0;
            ob.action = visualization_msgs::msg::Marker::ADD;
            obs.markers.push_back(ob);
        }
        obs_pub_->publish(obs);
    }   
    
    

    turtlelib::DiffDrive diff_drive{wheel_radius, track_width};
    
    // other to init
    size_t timestep_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_sub;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_srv_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr slam_path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_map_broadcaster;
    sensor_msgs::msg::JointState js_msg_old;
    turtlelib::KalmanFilter ekf;
    turtlelib::Transform2D odom_to_rob;
    turtlelib::Transform2D map_to_rob;
    turtlelib::Transform2D map_to_odom;
    nav_msgs::msg::Path path;

    

public:
    Slam()
    : Node("slam"), timestep_(0)
    {
        // declare parameters
        declare_parameter("wheel_radius", wheel_radius);
        wheel_radius = get_parameter("wheel_radius").as_double();

        declare_parameter("track_width", track_width);
        track_width = get_parameter("track_width").as_double();

        declare_parameter("slam_body_id", slam_body_id);
        slam_body_id = get_parameter("slam_body_id").as_string();

        declare_parameter("slam_odom_id", slam_odom_id);
        slam_odom_id = get_parameter("slam_odom_id").as_string();

        declare_parameter("wheel_left", wheel_left);
        wheel_left = get_parameter("wheel_left").as_string();

        declare_parameter("wheel_right", wheel_right);
        wheel_right = get_parameter("wheel_right").as_string();

        // check params
        check_params();

        // init diff drive
        diff_drive = {wheel_radius, track_width};
        
        // init publishers and subscribers
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        jointstate_sub = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(
                &Slam::jointstate_callback, this, std::placeholders::_1));

        initial_pose_srv_ = create_service<nuturtle_control::srv::InitialPose>(
            "initial_pose", std::bind(
                &Slam::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
        
        fake_sensor_sub = create_subscription<visualization_msgs::msg::MarkerArray>(
            "fake_sensor", 10, std::bind(
                &Slam::fake_sensor_callback, this, std::placeholders::_1));
        
        obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obs_map", 10);

        slam_path_pub_ = create_publisher<nav_msgs::msg::Path>("path_green", 10);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_map_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        js_msg_old.position = {0.0, 0.0};
       
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
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
        
        // update last wheel pos to current wheel pos
        wheel_pos_last.right_ang = js.position.at(0);
        wheel_pos_last.left_ang = js.position.at(1);

        // use fk to update q
        turtlelib::Twist2D tw = diff_drive.f_kin(w.right_ang, w.left_ang);
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
        tf2::QUaternion Q_map;
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

        geometry_msgs::TransformStamped map_odom_tf;
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
        turtlelib::Transform2D s = turtlelib::Transform2D{turtlelib::Vector2D{request->x, request->y}, request->theta};
        diff_drive.q_new(s);
        response->msg_feedback = "Initial pose set!";
    }

    // fake sensor callback
    void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr lidar_msg)
    {
        // get robot config
        turtlelib::Transform2D q = diff_drive.get_q();

        // use EKF
        // ekf prediction - take in current robot twist
        turtlelib::Twist2D tw = {q.translation().x, q.translation().y, q.rotation()};
        ekf.predict(tw);
        // update ekf
        for(size_t i = 0; i < lidar_msg->markers.size(); i++)
        {
            // get marker
            visualization_msgs::msg::Marker marker = lidar_msg->markers[i]
            // get marker position
            if(marker.action == visualization_msgs::msg::Marker::ADD)
            {
                ekf.update(marker.pose.position.x, marker.pose.position.y, i);
            }
        }

        // get state from ekf
        
    }
    

    turtlelib::DiffDrive diff_drive;
    // other to init
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
    : Node("slam")
    {
       
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
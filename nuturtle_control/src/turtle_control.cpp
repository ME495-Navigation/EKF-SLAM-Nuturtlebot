// enables control of the turtlebot via geometry_msgs/msg/Twist
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




using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
private:
    // callback for cmd_vel
    void cmdvel_callback(const geometry_msgs::msg::Twist & t)
    {
        // convert twist to twist2d --> t.linear.y is 0.0
        auto tw = turtlelib::Twist2D{t.linear.x, 0.0, t.angular.z};
        // compute wheel commands
        auto wheel_cmd = diff_drive.i_kin(tw);
        // calculate wheel_vel
        auto wheel_vel = turtlelib::WheelAng{wheel_cmd.right_ang/motor_cmd_per_rad_sec, wheel_cmd.left_ang/motor_cmd_per_rad_sec};
        // check if wheel_vel is within max limit
        if (wheel_vel.right_ang > motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Right wheel velocity exceeds max limit! Setting to motor_cmd_max.");
            wheel_vel.right_ang = motor_cmd_max;
        }
        if (wheel_vel.left_ang > motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Left wheel velocity exceeds limit! Setting to motor_cmd_max.");
            wheel_vel.left_ang = motor_cmd_max;
        }
        // check if wheel_vel is within min limit
        if (wheel_vel.right_ang < -motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Right wheel velocity exceeds min limit! Setting to -motor_cmd_max.");
            wheel_vel.right_ang = -motor_cmd_max;
        }
        if (wheel_vel.left_ang < -motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Left wheel velocity exceeds min limit! Setting to -motor_cmd_max.");
            wheel_vel.left_ang = -motor_cmd_max;
        }
        // publish wheel commands
        wheelcom_pub->publish(wheel_vel);
    }

    // callback for sensor_data
    void sensordata_callback(const sensor_msgs::msg::JointState & s)
    {
        // calculate joint positions
        auto right_wheel_jointpos = s.right_encoder/encoder_ticks_per_rad;
        auto left_wheel_jointpos = s.left_encoder/encoder_ticks_per_rad;

        // need dt to calculate joint velocities -- need header stamp to calculate current and future timestamp
        if(first)
        {
            joints.header.stamp = s.stamp;
            joints.position = {right_wheel_jointpos, left_wheel_jointpos};
            joints.velocity = {0.0, 0.0}; // initial velocity is 0
            first = false;
        }
        else
        {
            // calculate dt
            auto dt = s.stamp.sec + s.stamp.nanosec*1e-9 - joints.header.stamp.sec - joints.header.stamp.nanosec*1e-9;
            joints.header.stamp = s.stamp;
            // calculate joint velocities
            auto right_wheel_jointvel = (right_wheel_jointpos - joints.position.at(0))/dt;
            auto left_wheel_jointvel = (left_wheel_jointpos - joints.position.at(1))/dt;
            // update joint positions
            joints.position = {right_wheel_jointpos, left_wheel_jointpos};
            // update joint velocities
            joints.velocity = {right_wheel_jointvel, left_wheel_jointvel};
            // publish joint states
            jointstate_pub->publish(joints);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensordata_sub;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcom_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;

    turtlelib::DiffDrive diff_drive;
    sensor_msgs::msg::JointState joints;
    bool first = true;


public:
    TurtleControl():Node("turtle_control")
    {   
        declare_parameter("rate", 200);

        // take params defined in nuturtle_description diff_params.yaml
        declare_parameter("wheel_radius", 0.033);
        const auto wheel_radius = get_parameter("wheel_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius);

        declare_parameter("track_width", 0.16);
        const auto track_width = get_parameter("track_width").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "track width: " << track_width);

        declare_parameter("motor_cmd_max", 265);
        const auto motor_cmd_max = get_parameter("motor_cmd_max").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd max: " << motor_cmd_max);

        declare_parameter("motor_cmd_per_rad_sec", 0.024);
        const auto motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd per rad sec: " << motor_cmd_per_rad_sec);

        declare_parameter("encoder_ticks_per_rad", 651.8986);
        const auto encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "encoder ticks per rad: " << encoder_ticks_per_rad);

        declare_parameter("collision_radius", 0.11);
        const auto collision_radius = get_parameter("collision_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "collision radius: " << collision_radius);
        
        // publishers - wheel commands and joint states
        wheelcom_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
        jointstate_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // subscribers - cmd_vel and sensor_data
        cmdvel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TurtleControl::cmdvel_callback, this, std::placeholders::_1));
        sensordata_sub = create_subscription<sensor_msgs::msg::JointState>("sensor_data", 10, std::bind(&TurtleControl::sensordata_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    // spin the node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControl>());
    rclcpp::shutdown();
    return 0;
}

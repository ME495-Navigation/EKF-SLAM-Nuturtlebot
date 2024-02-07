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
    // params
    double wheel_radius;
    double track_width;
    double motor_cmd_max;
    double motor_cmd_per_rad_sec;
    double encoder_ticks_per_rad;
    double collision_radius;

    // callback for cmd_vel
    void cmdvel_callback(const geometry_msgs::msg::Twist & t)
    {
        // convert twist to twist2d --> t.linear.y is 0.0
        const auto w_body = t.angular.z;
        const auto x_body = t.linear.x;
        const auto y_body = 0.0;
        turtlelib::Twist2D tw{w_body, x_body, y_body};
        // compute wheel state
        turtlelib::WheelAng wheel_st = diff_drive.i_kin(tw);
        // set up wheel commands
        nuturtlebot_msgs::msg::WheelCommands wheelcom;
        // set up wheel velocities
        wheelcom.right_velocity = wheel_st.right_ang/motor_cmd_per_rad_sec;
        wheelcom.left_velocity = wheel_st.left_ang/motor_cmd_per_rad_sec;
        // check if wheel_vel is within max limit
        if (wheelcom.right_velocity > motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Right wheel velocity exceeds max limit! Setting to motor_cmd_max.");
            wheelcom.right_velocity = motor_cmd_max;
        }
        if (wheelcom.left_velocity > motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Left wheel velocity exceeds limit! Setting to motor_cmd_max.");
            wheelcom.left_velocity = motor_cmd_max;
        }
        // check if wheel_vel is within min limit
        if (wheelcom.right_velocity < -motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Right wheel velocity exceeds min limit! Setting to -motor_cmd_max.");
            wheelcom.right_velocity = -motor_cmd_max;
        }
        if (wheelcom.left_velocity < -motor_cmd_max)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Left wheel velocity exceeds min limit! Setting to -motor_cmd_max.");
            wheelcom.left_velocity = -motor_cmd_max;
        }
        // publish wheel commands
        wheelcom_pub->publish(wheelcom);
    }

    // callback for sensor_data
    void sensordata_callback(const nuturtlebot_msgs::msg::SensorData & s)
    {
        // calculate joint positions
        auto right_wheel_jointpos = static_cast<double>(s.right_encoder)/encoder_ticks_per_rad;
        auto left_wheel_jointpos = static_cast<double>(s.left_encoder)/encoder_ticks_per_rad;

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
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensordata_sub;
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheelcom_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_pub;

    turtlelib::DiffDrive diff_drive{0.0,0.0};
    sensor_msgs::msg::JointState joints;
    bool first = true;


public:
    TurtleControl():Node("turtle_control")
    {   
        declare_parameter("rate", 200);

        // take params defined in nuturtle_description diff_params.yaml
        declare_parameter("wheel_radius", 0.033);
        wheel_radius = get_parameter("wheel_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius);
        // if(wheel_radius.empty())
        // {
        //     throw std::logic_error("wheel_radius is empty!");
        // }

        declare_parameter("track_width", 0.16);
        track_width = get_parameter("track_width").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "track width: " << track_width);
        // if(track_width.empty())
        // {
        //     throw std::logic_error("track_width is empty!");
        // }

        declare_parameter("motor_cmd_max", 265);
        motor_cmd_max = get_parameter("motor_cmd_max").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd max: " << motor_cmd_max);
        // if(motor_cmd_max.empty())
        // {
        //     throw std::logic_error("motor_cmd_max is empty!");
        // }

        declare_parameter("motor_cmd_per_rad_sec", 0.024);
        motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd per rad sec: " << motor_cmd_per_rad_sec);
        // if(motor_cmd_per_rad_sec.empty())
        // {
        //     throw std::logic_error("motor_cmd_per_rad_sec is empty!");
        // }

        declare_parameter("encoder_ticks_per_rad", 651.8986);
        encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "encoder ticks per rad: " << encoder_ticks_per_rad);
        // if(encoder_ticks_per_rad.empty())
        // {
        //     throw std::logic_error("encoder_ticks_per_rad is empty!");
        // }

        declare_parameter("collision_radius", 0.11);
        collision_radius = get_parameter("collision_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "collision radius: " << collision_radius);
        // if(collision_radius.empty())
        // {
        //     throw std::logic_error("collision_radius is empty!");
        // }
        
        // publishers - wheel commands and joint states
        wheelcom_pub = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
        jointstate_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // subscribers - cmd_vel and sensor_data
        cmdvel_sub = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&TurtleControl::cmdvel_callback, this, std::placeholders::_1));
        sensordata_sub = create_subscription<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10, std::bind(&TurtleControl::sensordata_callback, this, std::placeholders::_1));
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

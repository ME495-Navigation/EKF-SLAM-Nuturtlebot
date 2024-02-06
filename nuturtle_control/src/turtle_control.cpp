// enables control of the turtlebot via geometry_msgs/msg/Twist
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/qos.hpp>


using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
private:
    /* data */
public:
    TurtleControl():Node("turtle_control")
    {   
        declare_parameter("rate", 200);

        // take params defined in nuturtle_description diff_params.yaml
        declare_parameter("wheel_radius", 0.033);
        double wheel_radius = get_parameter("wheel_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "wheel radius: " << wheel_radius);

        declare_parameter("track_width", 0.16);
        double track_width = get_parameter("track_width").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "track width: " << track_width);

        declare_parameter("motor_cmd_max", 265);
        double motor_cmd_max = get_parameter("motor_cmd_max").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd max: " << motor_cmd_max);

        declare_parameter("motor_cmd_per_rad_sec", 0.024);
        double motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "motor cmd per rad sec: " << motor_cmd_per_rad_sec);

        declare_parameter("encoder_ticks_per_rad", 651.8986);
        double encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "encoder ticks per rad: " << encoder_ticks_per_rad);

        declare_parameter("collision_radius", 0.11);
        double collision_radius = get_parameter("collision_radius").as_double();
        RCLCPP_INFO_STREAM(get_logger(), "collision radius: " << collision_radius);
        
        // 
    }

}

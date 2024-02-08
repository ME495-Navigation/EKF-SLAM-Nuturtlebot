#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nuturtle_control/srv/control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class Circle : public rclcpp::Node
{
    public:
        Circle() : Node("circle")
        {
            // params
            declare_parameter("frequency", 100);
            frequency = get_parameter("frequency").as_int();
            
            // publisher
            cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            // services
            control_srv_ = this->create_service<nuturtle_control::srv::Control>("control", std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
            reverse_srv_ = this->create_service<std_srvs::srv::Empty>("reverse", std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
            stop_srv_ = this->create_service<std_srvs::srv::Empty>("stop", std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));

            // timer
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency), std::bind(&Circle::timer_callback, this));

            stop_s = 0;
        }

    private:
        void timer_callback()
        {
            if (stop_s == 0)
            {
                cmd_vel_pub->publish(vels);
            }
        }

        void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request, std::shared_ptr<nuturtle_control::srv::Control::Response>)
        {
            stop_s = 0;
            vels.angular.z = request->velocity;
            vels.linear.x = request->radius * request->velocity;
        }

        // don't need the request OR the response
        void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            vels.angular.z = -vels.angular.z;
            vels.linear.x = -vels.linear.x;
        }

        // don't need the request OR the response
        void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>, std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            geometry_msgs::msg::Twist tw;
            cmd_vel_pub->publish(tw);
            stop_s = 1;
        }

        // params
        int frequency;
        int stop_s;
        geometry_msgs::msg::Twist vels;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
        rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Circle>());
    rclcpp::shutdown();
    return 0;
}

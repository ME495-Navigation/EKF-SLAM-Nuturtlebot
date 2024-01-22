// main simulation node
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
    public:
        Nusim()
        : Node("nusim"), timestep_(0)
        {
            // set parameters
            declare_parameter("rate", 200.0);

            // make timestep zero by default
            // timestep_ = 0;
            timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

            timer_ = create_wall_timer(rate,std::bind(&Nusim::timer_callback, this));

            reset_ = create_service<std_srvs::srv::Empty>("~/reset",std::bind(&Nusim::reset,this,std::placeholders::_1,std::placeholders::_2));
        }
    private:
        // main timer
        void timer_callback()
        {   
            std_msgs::msg::UInt64 msg;
            msg.data = timestep_;
            timestep_pub_->publish(msg);
            timestep_++;
        }

        // reset service - restores state of simulation
        void reset(std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            timestep_ = 0;
        }

        // initialize variables
        size_t timestep_;
        std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000.0 / 200.0));
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
};

int main(int argc, char * argv[])
{
    // spin the node
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}
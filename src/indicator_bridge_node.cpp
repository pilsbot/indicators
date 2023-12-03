#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"

#include "../rp2040_firmware/include/protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <arpa/inet.h>  //ntoh

using namespace std::chrono_literals;

// ros::Subscriber<std_msgs::Bool> il ("lighting/indicator/left",
//     [](const std_msgs::Bool& msg){lights.setIndicatorLeft(msg.data);});
// ros::Subscriber<std_msgs::Bool> ir ("lighting/indicator/right",
//     [](const std_msgs::Bool& msg){lights.setIndicatorRight(msg.data);});
// ros::Subscriber<std_msgs::Bool> br ("lighting/brake",
//     [](const std_msgs::Bool& msg){lights.setBrake(msg.data);});
// ros::Subscriber<std_msgs::Bool> hl ("lighting/headlight",
//     [](const std_msgs::Bool& msg){lights.setHeadlight(msg.data);});
// ros::Subscriber<std_msgs::Bool> pa ("lighting/party",

enum class Topic
{
    indicatorLeft,
    indicatorRight,
    brake,
    headlight,
    party,
};

constexpr const char*
getTopicName(const Topic& topic)
{
    switch (topic)
    {
        case Topic::indicatorLeft:
            return "lighting/indicator/left";
        case Topic::indicatorRight:
            return "lighting/indicator/right";
        case Topic::brake:
            return "lighting/indicator/left";
        case Topic::headlight:
            return "lighting/indicator/left";
        case Topic::party:
            return "lighting/indicator/right";
        default:
            return "lighting/invalid_shit_happend_AUTCH";
    }
}

class IndicatorBridge : public rclcpp::Node
{
  public:
    IndicatorBridge()
    : Node("pilsbot_indicator_bridge")
    {
        // https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
        std::function<void(const std_msgs::msg::Byte::SharedPtr msg)> fcn =
            std::bind(&IndicatorBridge::topic_callback, this, Topic::indicatorLeft, std::placeholders::_1);
        subscription_ = this->create_subscription<std_msgs::msg::Byte>(getTopicName(Topic::indicatorLeft), 10, fcn);
    }

  private:
    void topic_callback(const Topic type, const std_msgs::msg::Byte::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: %s, '%02X'", getTopicName(type), msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndicatorBridge>());
  rclcpp::shutdown();
  return 0;
}
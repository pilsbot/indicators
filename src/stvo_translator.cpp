#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <pilsbot_indicators/topics.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class AckermannToLightingNode : public rclcpp::Node
{
    struct Parameter
    {
        std::string ackermann_topic_from;
        std::string prefix_lighting;
        double limit_publish_rate_hz;

        double turning_threshold_rad;
    } param_;

    using Time = std::chrono::time_point<std::chrono::system_clock>;

    static constexpr unsigned qos = 10; // TODO: What 10? Chickens?

  public:
    AckermannToLightingNode()
    : Node("pilsbot_stvo_converter"), last_publish_(Time::min())
    {
        this->declare_parameter<std::string>("ackermann_topic_from", "/pilsbot_velocity_controller/cmd_vel");
        this->declare_parameter<std::string>("prefix_lighting", "");
        this->declare_parameter<double>("limit_publish_rate_hz", 5);
        this->declare_parameter<double>("turning_threshold_rad", .31415);

        param_.ackermann_topic_from = this->get_parameter("ackermann_topic_from").as_string();
        param_.prefix_lighting = this->get_parameter("prefix_lighting").as_string();
        param_.limit_publish_rate_hz = this->get_parameter("limit_publish_rate_hz").as_double();
        param_.turning_threshold_rad = this->get_parameter("turning_threshold_rad").as_double();

        lightingPublishers_.reserve(indicators::topics.size());
        for (const auto& topic : indicators::topics)
        {
            RCLCPP_INFO(this->get_logger(),
                "setting up publisher on " + param_.prefix_lighting + indicators::getTopicName(topic));
            lightingPublishers_.emplace_back(
                this->create_publisher<std_msgs::msg::Byte>(
                    param_.prefix_lighting + indicators::getTopicName(topic), qos));
        }

        RCLCPP_INFO(this->get_logger(),
                "setting up listener on %s", param_.ackermann_topic_from.c_str());
        this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                    param_.ackermann_topic_from, qos, std::bind(
                      &AckermannToLightingNode::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        // TODO
        std::cout << msg->drive.speed << " " << msg->drive.steering_angle << std::endl;

        if (msg->drive.speed < last_msg_.drive.speed || msg->drive.speed == 0)
        {
            std::cout << "Would light brakes" << std::endl;
        }

        if (msg->drive.steering_angle > param_.turning_threshold_rad)
        {
            std::cout << "Would blink left" << std::endl;
        }
        else if (msg->drive.steering_angle < -param_.turning_threshold_rad)
        {
            std::cout << "would blink right" << std::endl;
        }

        last_msg_ = *msg;
    }

    std::vector<rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr> lightingPublishers_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;


    ackermann_msgs::msg::AckermannDriveStamped last_msg_;
    Time last_publish_; // todo: Use, and also with timeout to send ignored msg
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannToLightingNode>());
  rclcpp::shutdown();
  return 0;
}
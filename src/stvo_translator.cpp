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

        double turning_threshold_rad;
        bool hazard_when_backing_up;
        double backwards_hazard_m_s;

        uint8_t brake_intensity;
        uint8_t indicator_intensity;


    } param_;

    using Time = std::chrono::time_point<std::chrono::system_clock>;

  public:
    AckermannToLightingNode()
    : Node("pilsbot_stvo_converter"), last_state_({0}), last_publish_(Time::min())
    {
        this->declare_parameter<std::string>("ackermann_topic_from", "pilsbot_velocity_controller/cmd_vel");
        this->declare_parameter<std::string>("prefix_lighting", "");

        this->declare_parameter<double>("turning_threshold_rad", .31415);
        this->declare_parameter<double>("backwards_hazard_m_s", .5);

        param_.ackermann_topic_from = this->get_parameter("ackermann_topic_from").as_string();
        param_.prefix_lighting = this->get_parameter("prefix_lighting").as_string();
        param_.limit_publish_rate_hz = this->get_parameter("limit_publish_rate_hz").as_double();
        param_.turning_threshold_rad = this->get_parameter("turning_threshold_rad").as_double();

        lightingPublishers_.reserve(indicators::topics.size());
        for (const auto& topic : indicators::topics)
        {
            const auto& offset = getOffsetFromTopic(topic);
            RCLCPP_INFO(this->get_logger(),
                "setting up publisher on " + param_.prefix_lighting + indicators::getTopicName(topic));
            lightingPublishers_[offset] = this->create_publisher<std_msgs::msg::Byte>(
                    param_.prefix_lighting + indicators::getTopicName(topic), rclcpp::SensorDataQoS());
        }

        RCLCPP_INFO(this->get_logger(),
                "setting up listener on %s", param_.ackermann_topic_from.c_str());
        steeringInputSubscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                    param_.ackermann_topic_from, rclcpp::SensorDataQoS(), std::bind(
                      &AckermannToLightingNode::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        // TODO
        // RCLCPP_INFO(this->get_logger(), "%f %f", msg->drive.speed, msg->drive.steering_angle);

        std::array<uint8_t, indicators::topics.size()> new_state = last_state_;

        // brakes
        if (msg->drive.speed < last_msg_.drive.speed || msg->drive.speed == 0)
        {
            // TODO: Instead of last_msg_, compare against actual odometry
            // RCLCPP_INFO(this->get_logger(), "brake light on");
            // TODO: Intensity in config
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = 0xF0;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "brake light off");
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = 0x00;
        }


        // turning signals
        const bool warnblink_on_backwards = true;   // TODO: Make Parameter
        if (msg->drive.steering_angle > param_.turning_threshold_rad ||
            (warnblink_on_backwards && msg->drive.speed < -0.1))
        {
            // RCLCPP_INFO(this->get_logger(), "blink right");
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = 0xC0;
        }
        else
        {
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = 0x00;
        }

        if (msg->drive.steering_angle < -param_.turning_threshold_rad ||
            (warnblink_on_backwards && msg->drive.speed < -0.1))
        {
            // RCLCPP_INFO(this->get_logger(), "blink left");
            new_state[getOffsetFromTopic(indicators::Topic::indicatorLeft)] = 0xC0;
        }
        else
        {
            new_state[getOffsetFromTopic(indicators::Topic::indicatorLeft)] = 0x00;
        }

        // TODO: Headlight when dark? Aufblenden?

        // Apply changes
        for (const auto& topic : indicators::topics)
        {
            const auto& offset = getOffsetFromTopic(topic);
            if (new_state[offset] != last_state_[offset])
            {
                auto message = std_msgs::msg::Byte();
                message.data = new_state[offset];
                lightingPublishers_[offset]->publish(message);
                last_state_[offset] = new_state[offset];
            }
        }

        last_msg_ = *msg;
    }

    static constexpr
    std::underlying_type_t<indicators::Topic>
    getOffsetFromTopic(const indicators::Topic& topic)
    {
        return static_cast<std::underlying_type_t<indicators::Topic>>(topic);
    }

    std::vector<rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr> lightingPublishers_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steeringInputSubscription_;

    std::array<uint8_t, indicators::topics.size()> last_state_;
    ackermann_msgs::msg::AckermannDriveStamped last_msg_;

    // todo: Use with timeout to enable hazard light if too long no commands
    Time last_publish_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannToLightingNode>());
  rclcpp::shutdown();
  return 0;
}
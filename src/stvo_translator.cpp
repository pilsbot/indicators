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
        double brakelight_diff_m_s;

        bool tagfahrlicht;

        uint8_t brake_intensity;
        uint8_t indicator_intensity;
        uint8_t tagfahrlicht_intensity;
        uint8_t headlights_intensity;

    } param_;

    using Time = std::chrono::time_point<std::chrono::system_clock>;

  public:
    AckermannToLightingNode()
    : Node("pilsbot_stvo_converter"), last_state_({0}), last_publish_(Time::min())
    {
        this->declare_parameter<std::string>("ackermann_topic_from", "pilsbot_velocity_controller/cmd_vel");
        this->declare_parameter<std::string>("prefix_lighting", "");

        this->declare_parameter<double>("turning_threshold_rad", .31415);
        this->declare_parameter<bool>("hazard_when_backing_up", true);
        this->declare_parameter<double>("backwards_hazard_m_s", .5);
        this->declare_parameter<double>("brakelight_diff_m_s", .5);

        this->declare_parameter<bool>("tagfahrlicht", true);

        this->declare_parameter<uint8_t>("brake_intensity", 0xF0);
        this->declare_parameter<uint8_t>("indicator_intensity", 0xB0);
        this->declare_parameter<uint8_t>("tagfahrlicht_intensity", 0x80);
        this->declare_parameter<uint8_t>("headlights_intensity", 0xA0);

        param_.ackermann_topic_from = this->get_parameter("ackermann_topic_from").as_string();
        param_.prefix_lighting = this->get_parameter("prefix_lighting").as_string();

        param_.turning_threshold_rad = this->get_parameter("turning_threshold_rad").as_double();
        param_.hazard_when_backing_up = this->get_parameter("hazard_when_backing_up").as_bool();
        param_.backwards_hazard_m_s = this->get_parameter("backwards_hazard_m_s").as_double();
        param_.brakelight_diff_m_s = this->get_parameter("brakelight_diff_m_s").as_double();
        param_.tagfahrlicht = this->get_parameter("tagfahrlicht").as_bool();

        param_.brake_intensity = this->get_parameter("brake_intensity").as_int();
        param_.indicator_intensity = this->get_parameter("indicator_intensity").as_int();
        param_.tagfahrlicht_intensity = this->get_parameter("tagfahrlicht_intensity").as_int();
        param_.headlights_intensity = this->get_parameter("headlights_intensity").as_int();

        lightingPublishers_.reserve(indicators::topics.size());
        for (const auto& topic : indicators::topics)
        {
            const auto& offset = getOffsetFromTopic(topic);
            RCLCPP_INFO(this->get_logger(),
                "setting up publisher on " + param_.prefix_lighting + indicators::getTopicName(topic));
            // the following might throw because something something reverse ion thrusters
            // https://github.com/ros2/rcl/issues/1118
            lightingPublishers_[offset] = this->create_publisher<std_msgs::msg::Byte>(
                    param_.prefix_lighting + indicators::getTopicName(topic), rclcpp::SensorDataQoS());
        }

        // Initial publish to default values
        topic_callback(std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>(last_msg_));

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
        if ((std::abs(msg->drive.speed) + param_.brakelight_diff_m_s) < std::abs(last_msg_.drive.speed)
            || msg->drive.speed == 0)
        {
            // TODO: Instead of last_msg_, compare against actual odometry
            // RCLCPP_INFO(this->get_logger(), "brake light on");
            // TODO: Intensity in config
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = param_.brake_intensity;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "brake light off");
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = 0x00;
        }


        // turning signal right
        if (msg->drive.steering_angle > param_.turning_threshold_rad ||
            (param_.hazard_when_backing_up && msg->drive.speed < -0.1))
        {
            // RCLCPP_INFO(this->get_logger(), "blink right");
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = param_.indicator_intensity;
        }
        else
        {
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = 0x00;
        }
        // turning signal left
        if (msg->drive.steering_angle < -param_.turning_threshold_rad ||
            (param_.hazard_when_backing_up && msg->drive.speed < -0.1))
        {
            // RCLCPP_INFO(this->get_logger(), "blink left");
            new_state[getOffsetFromTopic(indicators::Topic::indicatorLeft)] = param_.indicator_intensity;
        }
        else
        {
            new_state[getOffsetFromTopic(indicators::Topic::indicatorLeft)] = 0x00;
        }

        if (param_.tagfahrlicht)
        {
            new_state[getOffsetFromTopic(indicators::Topic::headlight)] = param_.tagfahrlicht_intensity;
        }
        if (false ) // Headlight when dark? Und assi BMW Aufblenden?
        {
            new_state[getOffsetFromTopic(indicators::Topic::headlight)] = param_.headlights_intensity;
        }

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
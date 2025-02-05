#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <pilsbot_indicators/topics.hpp>

#include <chrono>
#include <functional>
#include <optional>
#include <memory>
#include <string>
#include <vector>

class AckermannToLightingNode : public rclcpp::Node
{
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

    struct Parameter
    {
        std::string ackermann_topic_from;
        std::string light_teleop_prefix;
        std::string prefix_lighting_publish;

        double turning_threshold_rad;
        bool hazard_when_backing_up;
        double backwards_hazard_m_s;
        std::chrono::duration<double> min_brake_time;
        double brakelight_diff_m_s;

        // these are (optional) requests coming in from teleop acker joy
        bool tagfahrlicht;
        bool turn_left;
        bool turn_right;
        bool flash;

        uint8_t brake_intensity;
        uint8_t indicator_intensity;
        uint8_t tagfahrlicht_intensity;
        uint8_t headlights_intensity;

    } param_;

public:
    AckermannToLightingNode()
    : Node("pilsbot_stvo_converter"), last_state_({0})
    {
        this->declare_parameter<std::string>("ackermann_topic_from", "pilsbot_velocity_controller/cmd_vel");
        this->declare_parameter<std::string>("light_teleop_prefix", "cmd/");
        this->declare_parameter<std::string>("prefix_lighting_publish", "");

        this->declare_parameter<double>("turning_threshold_rad", .31415);
        this->declare_parameter<bool>("hazard_when_backing_up", true);
        this->declare_parameter<double>("backwards_hazard_m_s", .5);
        this->declare_parameter<double>("min_brake_time_s", 1.);
        this->declare_parameter<double>("brakelight_diff_m_s", .5);

        this->declare_parameter<bool>("tagfahrlicht", true);

        this->declare_parameter<uint8_t>("brake_intensity", 0xE0);
        this->declare_parameter<uint8_t>("indicator_intensity", 0xB0);
        this->declare_parameter<uint8_t>("tagfahrlicht_intensity", 0x80);
        this->declare_parameter<uint8_t>("headlights_intensity", 0xA0);

        param_.ackermann_topic_from = this->get_parameter("ackermann_topic_from").as_string();
        param_.light_teleop_prefix = this->get_parameter("light_teleop_prefix").as_string();
        param_.prefix_lighting_publish = this->get_parameter("prefix_lighting_publish").as_string();

        param_.turning_threshold_rad = this->get_parameter("turning_threshold_rad").as_double();
        param_.hazard_when_backing_up = this->get_parameter("hazard_when_backing_up").as_bool();
        param_.backwards_hazard_m_s = this->get_parameter("backwards_hazard_m_s").as_double();
        param_.min_brake_time = std::chrono::duration<double>{this->get_parameter("min_brake_time_s").as_double()};
        param_.brakelight_diff_m_s = this->get_parameter("brakelight_diff_m_s").as_double();
        param_.tagfahrlicht = this->get_parameter("tagfahrlicht").as_bool();

        param_.brake_intensity = this->get_parameter("brake_intensity").as_int();
        param_.indicator_intensity = this->get_parameter("indicator_intensity").as_int();
        param_.tagfahrlicht_intensity = this->get_parameter("tagfahrlicht_intensity").as_int();
        param_.headlights_intensity = this->get_parameter("headlights_intensity").as_int();

        for (const auto& topic : indicators::topics)
        {
            const auto& offset = getOffsetFromTopic(topic);
            RCLCPP_INFO(this->get_logger(),
                "setting up publisher on " + param_.prefix_lighting_publish + indicators::getTopicName(topic));
            // the following might throw because something something reverse ion thrusters
            // https://github.com/ros2/rcl/issues/1118
            lightingPublishers_[offset] = this->create_publisher<std_msgs::msg::Byte>(
                    param_.prefix_lighting_publish + indicators::getTopicName(topic), rclcpp::SensorDataQoS());
        }

        RCLCPP_INFO(this->get_logger(),
                "Brake-time: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(param_.min_brake_time).count()) + "ms");

        // Initial publish to default values
        cmd_callback(std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>(last_msg_));

        RCLCPP_INFO(this->get_logger(),
                "setting up listener on %s", param_.ackermann_topic_from.c_str());
        steeringInputSubscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                    param_.ackermann_topic_from, rclcpp::SensorDataQoS(), std::bind(
                      &AckermannToLightingNode::cmd_callback, this, std::placeholders::_1));


        // from teleop_acker_joy.cpp, make sure these are the same
        // TODO: Perhaps individual message?
        addLightingSubscription(param_.tagfahrlicht, param_.light_teleop_prefix + "headlight");;
        addLightingSubscription(param_.turn_left, param_.light_teleop_prefix + "turn_left");;
        addLightingSubscription(param_.turn_right, param_.light_teleop_prefix + "turn_right");
        addLightingSubscription(param_.flash, param_.light_teleop_prefix + "flash");
    }

  private:
    void cmd_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "%f %f", msg->drive.speed, msg->drive.steering_angle);

        auto new_state = last_state_;

        const bool brake_on = (param_.brakelight_diff_m_s + std::abs(msg->drive.speed)) < std::abs(last_msg_.drive.speed)
                                || msg->drive.speed == 0;

        if (brake_on)
        {
            start_brake_ = std::chrono::system_clock::now();
        }

        const bool brake_still_on = (start_brake_ && (std::chrono::system_clock::now() < *start_brake_ + param_.min_brake_time));

        if (!brake_still_on)
        {
            start_brake_ = std::nullopt;
        }

        // brakes
        if (brake_on || brake_still_on)
        {
            // TODO: Instead of last_msg_, compare against actual odometry
            // RCLCPP_INFO(this->get_logger(), "brake light on");
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = param_.brake_intensity;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "brake light off");
            new_state[getOffsetFromTopic(indicators::Topic::brake)] = 0x00;
        }

        // turning signals
        if (param_.turn_right ||
            msg->drive.steering_angle > param_.turning_threshold_rad ||
            (param_.hazard_when_backing_up && msg->drive.speed < -param_.backwards_hazard_m_s))
        {
            // RCLCPP_INFO(this->get_logger(), "blink right");
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = param_.indicator_intensity;
        }
        else
        {
            new_state[getOffsetFromTopic(indicators::Topic::indicatorRight)] = 0x00;
        }

        if (param_.turn_left ||
            msg->drive.steering_angle < -param_.turning_threshold_rad ||
            (param_.hazard_when_backing_up && msg->drive.speed < -param_.backwards_hazard_m_s))
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
        if (param_.flash)
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

    void addLightingSubscription(bool& which_field, std::string name)
    {
        RCLCPP_INFO(this->get_logger(),
            "setting up listener on %s", name.c_str());
        auto subs = this->create_subscription<std_msgs::msg::Bool>(
                name,
                rclcpp::SensorDataQoS(),
                [&which_field](const std_msgs::msg::Bool::SharedPtr msg)
                {
                    which_field = msg->data;
                }
                );
        lightingInputSubscriptions_.push_back(std::move(subs));
    }

    std::vector<rclcpp::Publisher<std_msgs::msg::Byte>::SharedPtr> lightingPublishers_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr steeringInputSubscription_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> lightingInputSubscriptions_;

    std::array<uint8_t, indicators::topics.size()> last_state_;
    ackermann_msgs::msg::AckermannDriveStamped last_msg_;

    // todo: hazard light if too long no commands?

    std::optional<TimePoint> start_brake_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannToLightingNode>());
  rclcpp::shutdown();
  return 0;
}

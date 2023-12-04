#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte.hpp"

#include "../rp2040_firmware/include/protocol.hpp"
#include <pilsbot_indicators/topics.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <arpa/inet.h>  //ntoh

using namespace std::chrono_literals;
using namespace indicators;

class IndicatorBridge
{
public:
    struct Parameter {
        std::string devicename;
        unsigned baud_rate;
    };

    IndicatorBridge(const Parameter& param) :
        serial_fd_(-1), params_(param),
        remote_state({0}), remote_state_was_initialized({false})
    {

    };

    void
    setParameter(const Parameter& param)
    {
        params_ = param;
    }

    ~IndicatorBridge()
    {
        stop();
    }

    void
    stop() {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }

    bool
    open_serial_port() {
        if ((serial_fd_ = ::open(params_.devicename.c_str(), O_RDWR | O_NOCTTY)) < 0) {
            return false;
        }
        return true;
    }

    bool
    set_serial_properties() {
        tcflush(serial_fd_, TCIOFLUSH); // flush previous bytes

        struct termios tio;
        if(tcgetattr(serial_fd_, &tio) < 0){
            perror("tcgetattr");
            return false;
        }

        tio.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
        tio.c_oflag &= ~(ONLCR | OCRNL);
        tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

        switch (params_.baud_rate)
        {
        case 9600:   cfsetospeed(&tio, B9600);   break;
        case 19200:  cfsetospeed(&tio, B19200);  break;
        case 38400:  cfsetospeed(&tio, B38400);  break;
        case 115200: cfsetospeed(&tio, B115200); break;
        case 230400: cfsetospeed(&tio, B230400); break;
        case 460800: cfsetospeed(&tio, B460800); break;
        case 500000: cfsetospeed(&tio, B500000); break;
        default:
            std::cerr << "Baudrate of " << params_.baud_rate <<
                " not supported, using 115200!" << std::endl;
            cfsetospeed(&tio, B115200);
            break;
        }
        cfsetispeed(&tio, cfgetospeed(&tio));

        if(tcsetattr(serial_fd_, TCSANOW, &tio) < 0) {
            std::cerr << "Could not set terminal attributes!" << std::endl;
            perror("tcsetattr");
            return false;
        }

        return true;
    }

    /**
     * \param[in] value
     *  intensity. Zero means "off".
    */
    bool
    sendCommand(const Topic& topic, ColorIntensity value)
    {
        const auto& offset = static_cast<std::underlying_type_t<Topic>>(topic);
        bool& was_initialized = remote_state_was_initialized[offset];
        auto& previous_value = remote_state[offset];

        if (!was_initialized || value != previous_value)
        {
            Command cmd;
            switch (topic)
            {
                case Topic::indicatorLeft:
                    cmd = cmd::IndicatorLeft(value);
                    break;
                case Topic::indicatorRight:
                    cmd = cmd::IndicatorRight(value);
                    break;
                case Topic::brake:
                    cmd = cmd::Brake(value);
                    break;
                case Topic::headlight:
                    cmd = cmd::Headlight(value);
                    break;
                case Topic::party:
                    cmd = cmd::Party(value);
                    break;
                default:
                    return false;
            }
            was_initialized = true;
            previous_value = value;
            return ::write(serial_fd_, &cmd, sizeof(decltype(cmd))) ==  sizeof(decltype(cmd));
        }
        // already sent this value
        return true;
    }

private:
    int serial_fd_;
    Parameter params_;
    std::array<ColorIntensity, indicators::topics.size()> remote_state;
    std::array<bool, indicators::topics.size()> remote_state_was_initialized;
};

class IndicatorBridgeNode : public rclcpp::Node
{
    std::unique_ptr<IndicatorBridge> bridge_;
public:
    IndicatorBridgeNode()
    : Node("pilsbot_indicator_bridge")
    {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        IndicatorBridge::Parameter params;
        params.devicename = this->get_parameter("serial_port").as_string();
        params.baud_rate = this->get_parameter("baud_rate").as_int();

        bridge_ = std::make_unique<IndicatorBridge>(params);
        if (!bridge_->open_serial_port())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open port %s at %d baud",
                params.devicename.c_str(), params.baud_rate);
            throw std::runtime_error("Could not open port " + params.devicename);
        }

        if (!bridge_->set_serial_properties())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open set up serial properties for %s",
                params.devicename);
            throw std::runtime_error("Could not open set up serial properties for " + params.devicename);
        }

        RCLCPP_INFO(this->get_logger(),
            "setup of %s at %d baud successful", params.devicename.c_str(), params.baud_rate);

        subscriptions_.reserve(topics.size());

        for (const auto& topic : topics)
        {
            RCLCPP_INFO(this->get_logger(),
                "setting up listener on %s", getTopicName(topic));

            // https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
            std::function<void(const std_msgs::msg::Byte::SharedPtr msg)> fcn =
                std::bind(&IndicatorBridgeNode::topic_callback, this, topic, std::placeholders::_1);
            subscriptions_.emplace_back(
                this->create_subscription<std_msgs::msg::Byte>(
                    getTopicName(topic), rclcpp::SensorDataQoS(), fcn));

        }
    }

private:
    void topic_callback(const Topic type, const std_msgs::msg::Byte::SharedPtr msg) const
    {
        // RCLCPP_INFO(this->get_logger(),
        //         "got command at %s", getTopicName(type));
        bridge_->sendCommand(type, msg->data);
    }

    std::vector<rclcpp::Subscription<std_msgs::msg::Byte>::SharedPtr> subscriptions_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IndicatorBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
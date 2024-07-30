#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"

namespace canedudev_interface
{
class VehicleReport : public rclcpp::Node {
public:
    VehicleReport();
    ~VehicleReport() = default;

private:
    void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void timer_callback();

    std::string frame_id_;
    double loop_rate_;
    float steer_angle_;
    float throttle_value_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_mode_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

VehicleReport::VehicleReport() : Node("canedudev_vehicle_report_node")
{
    frame_id_  = declare_parameter("frame_id", "base_link");
    loop_rate_ = declare_parameter("loop_rate", 50.0);

    // Subscription
    throttle_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/input/throttle", 10, std::bind(&VehicleReport::throttle_callback, this, std::placeholders::_1));
    steering_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/input/steering", 10, std::bind(&VehicleReport::steering_callback, this, std::placeholders::_1));
    
    steering_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));
    velocity_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));
    battery_report_pub_  = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS(1));
    gear_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS(1));
    control_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1));

    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / loop_rate_)), std::bind(&VehicleReport::timer_callback, this));
}

void VehicleReport::throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    throttle_value_ = msg->data;
}

void VehicleReport::steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    steer_angle_ = msg->data;
}

void VehicleReport::timer_callback()
{
    // Publish steering report
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = get_clock()->now();
    steer_msg.steering_tire_angle = steer_angle_ * M_PI / 180.0; // Convert to radians
    steering_report_pub_->publish(steer_msg);

    // Publish velocity report
    autoware_vehicle_msgs::msg::VelocityReport vel_report_msg;
    vel_report_msg.header.stamp = get_clock()->now();
    vel_report_msg.longitudinal_velocity = (throttle_value_ - 50) * (30.0 / 50.0); // Convert to m/s
    vel_report_msg.lateral_velocity      = 0;
    vel_report_msg.heading_rate          = 0;
    vel_report_msg.header.frame_id       = frame_id_;
    velocity_report_pub_->publish(vel_report_msg);

    // Publish control mode report
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_mode_report_pub_->publish(control_mode_report);
}

} // namespace canedudev_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<canedudev_interface::VehicleReport>());
  rclcpp::shutdown();
  return 0;
}

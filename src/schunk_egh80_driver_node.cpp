#include "schunk_control_egh80/schunk_egh80_driver.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("schunk_egh80_driver_node");

    
    rclcpp::Parameter ip_address_parameter;
    rclcpp::Parameter device_port_parameter;
    rclcpp::Parameter gripper_stroke_parameter;
    rclcpp::Parameter joint_state_topic_parameter;

    std::string ip_address; 
    std::string joint_state_topic;

    int device_port;
    float gripper_stroke;
    std::shared_ptr<SchunkEGH80ModbusTCP> egh80_driver;

    try
    {
        //obtain parameters
        node->declare_parameter<std::string>("ip_address", "192.168.1.254");
        node->declare_parameter<int>("device_port", 1);
        node->declare_parameter<double>("gripper_stroke", 40.683074951171875);
        node->declare_parameter<std::string>("gripper_joint_state_topic", "schunk/schunk_egh80/joint_states");

        node->get_parameter("ip_address", ip_address_parameter);
        node->get_parameter("device_port", device_port_parameter);
        node->get_parameter("gripper_stroke", gripper_stroke_parameter);
        node->get_parameter("gripper_joint_state_topic", joint_state_topic_parameter);

        ip_address = ip_address_parameter.as_string();
        device_port = device_port_parameter.as_int();
        gripper_stroke = gripper_stroke_parameter.as_double();
        joint_state_topic = joint_state_topic_parameter.as_string();

        //instantiate modbus TCP client
        egh80_driver = std::make_shared<SchunkEGH80ModbusTCP>(node, ip_address, device_port, gripper_stroke, joint_state_topic);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), e.what());
        return 1;
    }

    //timers
    rclcpp::TimerBase::SharedPtr watchdog_timer =
        node->create_wall_timer(100ms, std::bind(&SchunkEGH80ModbusTCP::timerCallback, egh80_driver));

    //services
    rclcpp::Service<schunk_control_egh80::srv::ControlEGH80>::SharedPtr control_egh80_srv =
        node->create_service<schunk_control_egh80::srv::ControlEGH80>
                ("/schunk/control/schunk_egh80", std::bind(&SchunkEGH80ModbusTCP::controlCallback, 
                                                egh80_driver, std::placeholders::_1, std::placeholders::_2));
  
    rclcpp::spin(node);
}
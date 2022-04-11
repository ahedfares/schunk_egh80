#include "schunk_control_egh80/schunk_egh80_driver.hpp"
#include <bitset>

typedef schunk_control_egh80::srv::ControlEGH80::Request control_request;

SchunkEGH80ModbusTCP::SchunkEGH80ModbusTCP(rclcpp::Node::SharedPtr node, const std::string& ip_address, 
                                            int device_port, double gripper_stroke, const std::string& joint_state_topic) : 
    node_(node),
    ip_address_(ip_address),
    device_port_(device_port),
    gripper_stroke_(gripper_stroke)
{
    //initialize modbus communication
    RCLCPP_INFO(node_->get_logger(), "Connecting to egh80 modbus TCP server through: '%s', port: '%d', please wait ...", ip_address.c_str(), device_port);
    client_ = std::make_shared<modbus>(ip_address, device_port); //502: standard port for modbusTCP
    client_->modbus_set_slave_id(device_port);
    if (client_->modbus_connect())
    {
        RCLCPP_INFO(node_->get_logger(), "Connection to Modbus TCP server is successfull!");
    }else{
        RCLCPP_ERROR(node_->get_logger(), "Connection to the egh80 gripper failed: incorrect ip_adress '%s' or device is not connected!", ip_address.c_str());
        RCLCPP_ERROR(node_->get_logger(), "Terminating...");
        throw std::runtime_error("Connection to schunk gripper modbus TCP server failed!");
    }

    //publishers
    joint_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
    status_pub_ = node->create_publisher<std_msgs::msg::UInt8>("/schunk/schunk_egh80/status", 10);

    //prepare gripper
    status_ = getStatus();
    timeout(0);
    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    acknowledge();
    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    reference();
}

SchunkEGH80ModbusTCP::~SchunkEGH80ModbusTCP()
{
    client_->modbus_close();
}

bool SchunkEGH80ModbusTCP::controlCallback(std::shared_ptr<schunk_control_egh80::srv::ControlEGH80::Request> req,
                        std::shared_ptr<schunk_control_egh80::srv::ControlEGH80::Response> res)
{
    
    switch (req->command)
    {
        case control_request::CONNECT:
        {
            RCLCPP_INFO(node_->get_logger(), "Connecting to egh80 modbus TCP server through: '%s', port: '%d', please wait ...", ip_address_.c_str(), device_port_);
            client_ = std::make_shared<modbus>(ip_address_, device_port_); //502: standard port for modbusTCP
            client_->modbus_set_slave_id(device_port_);
            if (client_->modbus_connect())
            {
                RCLCPP_INFO(node_->get_logger(), "Connection to Modbus TCP server is successfull!");
                res->status = true;
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Connection to the egh80 gripper failed: incorrect ip_adress '%s' or device is not connected!", ip_address_.c_str());
                RCLCPP_ERROR(node_->get_logger(), "Terminating...");
                res->status = false;
            }
            break;
        }

        case control_request::SET_POSITION:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'setPosition' Cmd recieved");
            if (req->position >= 0 && req->position <= 100) //req->position is in percentage
            {
                if (!setPosition(req->position))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set gripper to the given position...");
                    res->status = false;
                }else
                {
                    RCLCPP_INFO(node_->get_logger(), "'setPosition' successfull.");
                    res->status = true;
                }
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Goal position is out of range! terminating execution...");
                res->status = false;
            }
            break;
        }

        case control_request::ACKNOWLEDGE:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'acknowledge' Cmd recieved");
            if (acknowledge())
            {
                RCLCPP_INFO(node_->get_logger(), "'acknowledge' successfull.");
                res->status = true;
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to perform 'acknowledge'!!");
                res->status = false;
            }
            break;
        }

        case control_request::GET_POSITION:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'getPosition' Cmd recieved");
            if (getPosition())
            {
                RCLCPP_INFO(node_->get_logger(), "'getPosition' successfull.");
                res->current_position = current_position_;
                res->status = true;
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to perform 'getPosition'!!");
                res->status = false;
            }
            break;
        }

        case control_request::GET_STATUS:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'getState' Cmd recieved");
            status_ = getStatus();
            res->error_code = status_;
            res->status = true;
            break;
        }

        case control_request::STOP_GRIPPER:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'stop' Cmd recieved");
            if (stop())
            {
                RCLCPP_INFO(node_->get_logger(), "'stop' successfull.");
                res->status = true;
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to perform 'stop'!!");
                res->status = false;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
            getPosition();  //record the position when stop is executed
            break;
        }

        case control_request::REFERENCE:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'reference' Cmd recieved ");
            if (reference())
            {
                RCLCPP_INFO(node_->get_logger(), "'Reference' successfull.");
                res->status = true;
            }else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to perform 'Reference'!!");
                res->status = false;
            }
            break;
        }

        case control_request::DISCONNECT:
        {
            RCLCPP_INFO(node_->get_logger(), "EGH80: 'Disconnect' Cmd recieved ");
            client_->modbus_close();
            res->status = true;
            break;
        }

        default:
        {
            RCLCPP_ERROR(node_->get_logger(), "Command not recognized!");
            res->status = false;
            break;
        }
    }
    
    return true;
}

void SchunkEGH80ModbusTCP::timerCallback()
{
    


    if (getPosition())
    {
        float position_mm;
        position_mm = float(current_position_) * gripper_stroke_ / 100.0;

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = node_->get_clock()->now();

        joint_state.name.push_back("gripper_right_finger_joint_mm");
        joint_state.position.push_back(position_mm);

        joint_state.name.push_back("gripper_left_finger_joint_mm");
        joint_state.position.push_back(-position_mm);

        joint_state.name.push_back("percentage");
        joint_state.position.push_back(current_position_);

        joint_pub_->publish(joint_state);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to obtain current position, joint_state publisher failed!!");
    }    

    std_msgs::msg::UInt8 status;
    status.data = status_;
    status_pub_->publish(status);
}

void SchunkEGH80ModbusTCP::timeout(int value)
{
    client_->modbus_write_register(EGH80_REG_WATCHDOG, value);
    // self.client.write_register(0x1120, value)
}

bool SchunkEGH80ModbusTCP::acknowledge()
{
    bool status = true;
    int status_modbus;
    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x0100);
    if (status_modbus != 0)
    {
        status = false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    
    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x8100);
    if (status_modbus != 0)
    {
        status = false;
    }

    return status;
}

bool SchunkEGH80ModbusTCP::reference()
{
    bool status = true;
    int status_modbus;

    handleErrors();
    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x0200);
    if (status_modbus != 0)
    {
        status = false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x8200);
    if (status_modbus != 0)
    {
        status = false;
    }

    return status;
}

void SchunkEGH80ModbusTCP::measureStroke()
{
    handleErrors();
    client_->modbus_write_register(EGH80_REG_CONTROL, 0x0700);
    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    client_->modbus_write_register(EGH80_REG_CONTROL, 0x8700);
}

void SchunkEGH80ModbusTCP::calibrate()
{
    handleErrors();
    client_->modbus_write_register(EGH80_REG_CONTROL, 0x0900);
    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    client_->modbus_write_register(EGH80_REG_CONTROL, 0x8900);
}

void SchunkEGH80ModbusTCP::grip(uint16_t force)
{

    handleErrors();
    if (!(force >= 1 && force <= 4))
    {
        std::runtime_error("Force value out of range (Must be between 1 and 4)");
    }
    uint16_t values[4] = {0x0400, (4 - force) * 0x0100, 0, 0};
    client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values);
    rclcpp::sleep_for(std::chrono::milliseconds(50)); //0.05 seconds

    uint16_t values_new[4] = {0x8400, (4 - force) * 0x0100, 0, 0};
    client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values_new);
    waitProcessCommand();
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    while(!success())
        rclcpp::sleep_for(std::chrono::milliseconds(50)); //0.05 seconds
}

void SchunkEGH80ModbusTCP::release()
{
    handleErrors();
    uint16_t values[4] = {0x0300, 0, 0, 0};
    client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values);
    rclcpp::sleep_for(std::chrono::milliseconds(50));

    uint16_t values_new[4] = {0x8300, 0, 0, 0};
    client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values_new);
    waitProcessCommand();
    rclcpp::sleep_for(std::chrono::seconds(1));
}

bool SchunkEGH80ModbusTCP::setPosition(int target_position)
{
    bool status = true;
    int status_modbus;

    handleErrors();
    if (!(target_position >=0 && target_position <= 100))
    {
        std::runtime_error("Target position value out of range (0-100%)");
    }else if (target_position < 1)
    {
        grip();
    }else if (target_position > 99)
    {
        release();
    }
    
    float position_mm = ((100 - float(target_position)) / 100.) * gripper_stroke_;    
    uint position_bin = float_to_IEEE_754(position_mm);
    uint16_t register_left = position_bin >> 16;
    uint16_t register_right = position_bin & 0x0000FFFF;

    uint16_t values[4] = {0x0500, 0, register_left, register_right};
    status_modbus = client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values);
    if (status_modbus != 0)
    {
        status = false;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));

    uint16_t values_new[4] = {0x8500, 0, register_left, register_right};
    status_modbus = client_->modbus_write_registers(EGH80_REG_CONTROL, 4, values_new);
    if (status_modbus != 0)
    {
        status = false;
    }
    waitProcessCommand(); 
    
    return status;
}

void SchunkEGH80ModbusTCP::fastStop()
{
    if (getStatus() == STATUS_ERROR)
    {
        acknowledge();
    }
    client_->modbus_write_register(EGH80_REG_CONTROL, 0x0000);
}

bool SchunkEGH80ModbusTCP::stop()
{
    int status_modbus;
    bool status = true;
    handleErrors();

    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x0800);
    if (status_modbus != 0)
    {
        status = false;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100)); //0.1 seconds
    status_modbus = client_->modbus_write_register(EGH80_REG_CONTROL, 0x8800);
    if (status_modbus != 0)
    {
        status = false;
    }

    return status;
}

void SchunkEGH80ModbusTCP::disconnect()
{
    client_->modbus_close();
}

SchunkEGH80ModbusTCP::Status SchunkEGH80ModbusTCP::getStatus()
{
    uint16_t value;
    client_->modbus_read_input_registers(EGH80_REG_STATUS, 1, &value);
    Status status = STATUS_ERROR;

    if ((value & 0x0000) == 0x0000)    //masking with status bits p.24
        status = STATUS_ERROR;
    else if ((value & 0x0100) == 0x0100)
        status = STATUS_OOS;
    else if ((value & 0x0200) == 0x0200)
        status = STATUS_MAINTENANCE;
    else if ((value & 0x0300) == 0x0300)
        status = STATUS_READY;

    return status;
}

int SchunkEGH80ModbusTCP::success()
{
    uint16_t value;
    client_->modbus_read_input_registers(EGH80_REG_STATUS, 1, &value);
    if (!((value & 0x0800) == 0x0800)) //masking the last bit (success bit of the gripper p.24)
    {
        return false;
    }
    
    return true;
}

bool SchunkEGH80ModbusTCP::getPosition()
{
    int status_modbus;
    bool status = true;
    uint16_t values[2];

    status_modbus = client_->modbus_read_input_registers(EGH80_REG_POSITION, 2, values); 
    if (status_modbus == 0)
    {
        float position_mm = IEEE754_to_float(values[0], values[1]);
        int position_perc = int(((gripper_stroke_ - position_mm) / gripper_stroke_) * 100);
        current_position_ =  position_perc;

    }else
    {
        status = false;
    }

    return status;
}


void SchunkEGH80ModbusTCP::waitProcessCommand()
{
    for(int i = 0; i < 1000; ++i)
    {
        uint16_t value;
        client_->modbus_read_input_registers(EGH80_REG_STATUS, 1, &value);
        if ((value & 0x8000) == 0x8000)    //masking last bit of the status register
            return;
    }

    timeout(1);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    timeout(0);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    acknowledge();
}

void SchunkEGH80ModbusTCP::handleErrors()
{
    status_ = getStatus();
    if (status_ == STATUS_ERROR)
    {
        acknowledge();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }else if (status_ == STATUS_OOS)
    {
        timeout(1);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        timeout(0);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        acknowledge();
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

float SchunkEGH80ModbusTCP::IEEE754_to_float(uint16_t left_register, uint16_t right_register)
{
    uint32_t fraction_bits = (static_cast<uint32_t>(left_register) << 16) + static_cast<uint32_t>(right_register) ;

    //calculate exponent and sign
    bool negative  = !!(fraction_bits & 0x80000000);
    int  exponent  =   (fraction_bits & 0x7f800000) >> 23;    
    int sign = negative ? -1 : 1;
    exponent -= 127;
    std::bitset<23> mantissa_bit(fraction_bits & 0x007FFFFF);

    // calculate mantissa
    int power = -1;
    float total = 0.0;
    for ( int i = 22; i >= 0; --i )
    {
        total += (float)mantissa_bit[i] * (float)pow(2.0, power);
        --power;
    }
    total += 1.0;
    float value = sign * (float)pow(2.0, exponent) * total; 

    return value;
}

uint SchunkEGH80ModbusTCP::float_to_IEEE_754(float value)
{
    union
    {
         float input;   // assumes sizeof(float) == sizeof(int)
         int   output;
    }    data;
 
    data.input = value;
 
    std::bitset<32> bits(data.output);
    uint IEEE_int = bits.to_ulong();

    return IEEE_int;
}
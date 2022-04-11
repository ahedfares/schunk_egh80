#ifndef SCHUNK_EGH80_DRIVER_H
#define SCHUNK_EGH80_DRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <modbuspp/modbus.h>    //https://github.com/fanzhe98/modbuspp
#include <stdexcept>

//ROS Msgs & Srvs
#include <schunk_control_egh80/srv/control_egh80.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/u_int8.hpp>



class SchunkEGH80ModbusTCP
{  
private:

    enum Registers : int16_t{
        EGH80_REG_STATUS = 0x0001,
        EGH80_REG_POSITION = 0x0003,
        EGH80_REG_CONTROL = 0x0801,
        EGH80_REG_WATCHDOG = 0x1120
    };

    enum Status : int8_t   //p.24
    {
        STATUS_ERROR = 0,        // Error
        STATUS_OOS = 1,          // Out of specification
        STATUS_MAINTENANCE = 2,  // Maintenance required
        STATUS_READY = 3         //Ready for operation
    };

    enum Commands : uint16_t
    {
        CMD_ACKNOWLEDGE = 0x0100,
        CMD_REFERENCE = 0x0200,
        CMD_MEASURE_STROKE = 0x0700,
        CMD_CALIBRATE = 0x0900,
        CMD_GRIP = 0x0400,
        CMD_FAST_STOP = 0X0000,
        CMD_STOP = 0x0800
    };

    rclcpp::Node::SharedPtr node_;
    std::string ip_address_;
    int device_port_;
    double gripper_stroke_;
    std::shared_ptr<modbus> client_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;

    Status status_;         //status of the gripper (p.24)
    int current_position_; //percentage (0 - 100%)

public:

    /**
     * \brief Constructor
     * \param node ROS2 node handle
     * \param ip_address ip_address of the gripper (master iol device)
     * \param device_port the modbus TCP port of the gripper, default = 502
     * \param gripper_stroke gripper stroke in mm, default = 40.683074951171875
     * \param joint_state_topic topic at which the gripper joint states are published
     * \return value in decimal format
     */
    SchunkEGH80ModbusTCP(rclcpp::Node::SharedPtr node, 
                            const std::string& ip_address = "192.168.1.253",
                            int device_port = 502, 
                            double gripper_stroke = 40.683074951171875,
                            const std::string& joint_state_topic = "egh80_joint_states");

    /**
     * \brief Destructor */     
    ~SchunkEGH80ModbusTCP();

    /** \brief control gripper service callback */
    bool controlCallback( std::shared_ptr<schunk_control_egh80::srv::ControlEGH80::Request> req,
                            std::shared_ptr<schunk_control_egh80::srv::ControlEGH80::Response> res);

  
    /** \brief Timer callback to read serial input buffer periodically */
    // void timerCallback(const ros::TimerEvent &event);  
    void timerCallback();  
  
private:

    /**
     * \brief Converts 2 x 16bit registers - (32bit register IEEE754 float) to a decimal value.
     * \param left_register first 16bit register 
     * \param right_register second 16bit register 
     * \return value in decimal format
     */
    float IEEE754_to_float(uint16_t left_register, uint16_t right_register);

    /**
     * \brief Converts a decimal value to 2 x 16bit registers - (32bit register IEEE754 float). 
     * \param value float value to be converted
     * \return value in float IEEE754 format
     */
    uint float_to_IEEE_754(float value);

    /**
     * \brief Does a time out on the gripper by specifying the duration value.
     *            By default, the value is 0, which means the Modbus connection will
     *            never be closed
     * \param value argument to define after which time (in seconds) of inactivity
     *                     a Modbus connection is closed through a Disconnect.
     */
    void timeout(int value);

    /**
    * \brief After an error has been rectified, the gripper is set to the normal
    *            operating status by acknowledging the error from the error status.
    *            The actuator remains de-energized until the next command.
    */
    bool acknowledge();

    /**
    * \brief The zero position is set during the referencing process. The gripper
    *            moves (Parameter [} 26]) to the mechanical end stop in the
    *            referencing direction set.
    */
    bool reference();

    /**
    * \brief Stroke measurement is an optional function. During the stroke
    *           measurement, the maximum stroke of the gripper is set relative to
    *           the referencing position. A stroke measurement should be
    *           performed if the stroke of the base jaws is limited, for instance, by
    *           specific gripper finger shapes.
    */
    void measureStroke();

    /**
    * \brief Calibration is an optional function. For calibration, the functions
    *            "reference" and "measureStroke" are performed one after
    *            the other. For modules with an absolute measuring system, the
    *            offset and slope are determined.
    */
    void calibrate();

    /**
    * \brief When gripping, movement follows the gripping direction to the
    *            stop and the workpiece is held. With electric grippers, the
    *            workpiece is held with the gripping force set.
    * \param force optional argument to define the gripping force.
    *                     Must be between 1 (weakest) and 4 (strongest), by default 4.
    */
    void grip(uint16_t force = 4);

    /**
    * \brief When releasing, movement occurs in the opposite direction to
    *            gripping, up until the end stop. The command signals success
    *            when the end stop is reached. The smallest gripping force
    *            adjustment is set for the releasing process.
    */
    void release();

    /**
    * \brief The gripper moves to the position that was specified under
    *            "target_position". If the run is interrupted by a blockage,
    *            the drive switches off. An error message requiring acknowledgment
    *            is generated. The actuator remains de-energized until the next run
    *            command.
    *
    *\param target_position define sthe relative position of the fingers in percent.
    *                               Must be between 0 (fingers closed) and 100 (fingers opened).
    */
    bool setPosition(int target_position);

    /**
    * \brief The electrical power supply to the actuator is interrupted
    *            immediately, the gripper is stopped uncontrolled. A FastStop
    *            occurs independently of the status change of the "Execution
    *            command" bit.
    *            An error message requiring acknowledgment is generated. A
    *            FastStop does not increase the error count and is not saved as the
    *            most recent error.
    */
    void fastStop();

    /**
    * \brief The gripper is brought to a controlled standstill. The gripper
    *            remains in a controlled standstill while retaining the force
    *            provided in the previous command.
    */
    bool stop();

    /**
    * \brief Disconnects the Modbus TCP Client
    */
    void disconnect();

    /**
    * \brief Obtains the operating status of the gripper
    *            0: Error
    *            1: Out of specification
    *            2: Maintenance required
    *            3: Ready for operation
    * \return the current operating status
    */
    Status getStatus();

    /**
    * \brief Reads the success bit of the gripper. When a new command is executed,
    *            the "Success" bit is reset to 0. If the command is successful, the bit
    *            is set to 1.
    * \return the success bit of the gripper
    */
    int success();

    /**
    * \brief obtains the relative position in percent of the fingers.
    * \return the relative position in percent
    */
    bool getPosition();

    /**
    * \brief Waits until the current command has been processed. Process command = 1 if
    *        the execute command is 1 and the process data has been processed.
    *        Process command = 0 if the execute command changes to 0.
    */
    void waitProcessCommand();

    /**
    * \brief Handles errors of the gripper by acknowledging the error. If the error is
    *            of type "Out of specification", the gripper is timed out (rebooted) and
    *            then acknowledged.
    */
    void handleErrors();






};

#endif //SCHUNK_EGH80_DRIVER_H

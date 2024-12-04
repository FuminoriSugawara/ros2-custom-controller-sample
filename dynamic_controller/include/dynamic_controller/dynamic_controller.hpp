#ifndef DYNAMIC_CONTROLLER_HPP_
#define DYNAMIC_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "forward_command_controller/multi_interface_forward_command_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"

namespace dynamic_controller
{

    typedef union ControlWord {
        uint16_t word;
        struct
        {
            bool enable : 1;
            bool reserved_1 : 3;
            uint8_t control_mode : 2;
            uint8_t reserved_2 : 2;
            uint8_t control_state : 4;
            uint8_t module_id : 4;
        } bits;

    } ControlWord;

    typedef union StatusWord {
        uint16_t word;
        struct
        {
            bool enable : 1;
            bool brake : 1;
            bool error : 1;
            bool reserved_1 : 1;
            uint8_t control_mode : 2;
            uint8_t reserved_2 : 2;
            uint8_t control_state : 4;
            uint8_t reserved_3 : 4;
        } bits;
    } StatusWord;


    class DynamicController : public forward_command_controller::MultiInterfaceForwardCommandController
    {
    public:
        DynamicController() = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        enum class ControllerState
        {
            INITIALIZING = 0,
            INITIAL_MODE_SETTING = 1,
            IDLING = 2,
            HOMING = 3,
            CONTROLLING = 4,
            MODE_CHANGING = 5,
            STOPPING = 6,
            ENABLING = 7,
        };
        enum class ControlMode
        {
            OPEN = 0,
            EFFORT = 1,
            VELOCITY = 2,
            POSITION = 3,
        };
        struct JointConfig
        {
            std::string name;
            int module_id;
            double position_control_amplitude;
            double position_control_frequency;
            double velocity_control_amplitude;
            double velocity_control_frequency;
            double effort_control_amplitude;
            double effort_control_frequency;
            double phase;
            bool is_target;
            bool is_online;
            double current_position;
            double current_velocity;
            double current_effort;
            StatusWord current_status;
            double stop_initial_command;  // 停止開始時の指令値
            double deceleration_time;      // 減速に要する時間
            double elapsed_stop_time;      // 停止開始からの経過時間
        };
        std::vector<JointConfig> joint_configs_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> state_interface_types_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> interface_types_;
        std::vector<std::vector<double>> joint_commands_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_status_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_effort_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_control_word_command_interface_;
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_ = {
                {"position", &joint_position_state_interface_},
                {"velocity", &joint_velocity_state_interface_},
                {"effort", &joint_effort_state_interface_},
                {"status", &joint_status_interface_}
            };
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            command_interface_map_ = {
                {"position", &joint_position_command_interface_},
                {"velocity", &joint_velocity_command_interface_},
                {"effort", &joint_effort_command_interface_},
                {"control_word", &joint_control_word_command_interface_}
            };
        realtime_tools::RealtimeBuffer<bool> is_running_;
        realtime_tools::RealtimeBuffer<ControllerState> controller_state_;
        realtime_tools::RealtimeBuffer<ControlMode> control_mode_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr controller_state_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr control_mode_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
        double deceleration_duration_ = 0.25;  // 停止までの目標時間(秒)
        double velocity_tolerance_ = 100.0;    // 速度がこの値以下になったらホーミングへ移行
        double homing_velocity_;    // ゼロ位置への移動速度
        double position_tolerance_; // ゼロ位置の許容誤差
        // last_log_time_ は、前回ログを出力した時間を保持する
        rclcpp::Time last_log_time_;
    };

} // namespace dynamic_controller

#endif // DYNAMIC_CONTROLLER_HPP_
#ifndef CUSTOM_POSITION_CONTROLLER_HPP_
#define CUSTOM_POSITION_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "position_controllers/joint_group_position_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "constants.hpp"

namespace custom_position_controller
{
    class CustomPositionController : public position_controllers::JointGroupPositionController
    {
    public:
        CustomPositionController() = default;

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
        // コントローラの状態を管理する列挙型
        enum class ControllerState
        {
            IDLE,        // 停止状態
            HOMING,      // ゼロ位置への移動中
            RUNNING_SINE // Sin波出力中
        };
        struct JointConfig
        {
            std::string name;
            double amplitude;
            double frequency;
            double phase;
            bool is_target;
            double current_position;
            double current_velocity;
            double current_effort;
            int16_t encoder_diff;
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
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_encoder_diff_state_interface_;
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
                {"encoder_diff", &joint_encoder_diff_state_interface_}};
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
            command_interface_map_ = {
                {"position", &joint_position_command_interface_},
                {"velocity", &joint_velocity_command_interface_},
                {"effort", &joint_effort_command_interface_},
                {"control_word", &joint_control_word_command_interface_}};
        realtime_tools::RealtimeBuffer<bool> is_running_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
        ControllerState controller_state_;
        double homing_velocity_;    // ゼロ位置への移動速度
        double position_tolerance_; // ゼロ位置の許容誤差
        rclcpp::Time last_log_time_;
    };

} // namespace custom_position_controller

#endif // CUSTOM_POSITION_CONTROLLER_HPP_
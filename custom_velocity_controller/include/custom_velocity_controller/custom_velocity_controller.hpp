#ifndef CUSTOM_VELOCITY_CONTROLLER_HPP_
#define CUSTOM_VELOCITY_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "velocity_controllers/joint_group_velocity_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace custom_velocity_controller
{

    class CustomVelocityController : public velocity_controllers::JointGroupVelocityController
    {
    public:
        CustomVelocityController() = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        enum class ControllerState
        {
            IDLE,
            HOMING,
            RUNNING_SINE,
            STOPPING_SINE
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
            double stop_initial_command;  // 停止開始時の指令値
            double deceleration_time;      // 減速に要する時間
            double elapsed_stop_time;      // 停止開始からの経過時間
        };
        std::vector<JointConfig> joint_configs_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> state_interface_types_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_effort_state_interface_;
        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
            state_interface_map_ = {
                {"position", &joint_position_state_interface_},
                {"velocity", &joint_velocity_state_interface_},
                {"effort", &joint_effort_state_interface_},
            };
        realtime_tools::RealtimeBuffer<bool> is_running_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
        ControllerState controller_state_;
        double deceleration_duration_ = 0.25;  // 停止までの目標時間(秒)
        double velocity_tolerance_ = 100.0;    // 速度がこの値以下になったらホーミングへ移行
        double homing_velocity_;    // ゼロ位置への移動速度
        double position_tolerance_; // ゼロ位置の許容誤差
    };

} // namespace custom_velocity_controller

#endif // CUSTOM_VELOCITY_CONTROLLER_HPP_
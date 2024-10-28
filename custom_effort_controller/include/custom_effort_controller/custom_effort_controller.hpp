#ifndef CUSTOM_EFFORT_CONTROLLER_HPP_
#define CUSTOM_EFFORT_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "effort_controllers/joint_group_effort_controller.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace custom_effort_controller
{

    class CustomEffortController : public effort_controllers::JointGroupEffortController
    {
    public:
        CustomEffortController() = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
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
    };

} // namespace custom_effort_controller

#endif // CUSTOM_EFFORT_CONTROLLER_HPP_
#include "controller_interface/controller_interface.hpp"
#include "custom_position_controller/custom_position_controller.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace custom_position_controller
{

    controller_interface::CallbackReturn CustomPositionController::on_init()
    {
        auto ret = position_controllers::JointGroupPositionController::on_init();
        if (ret != controller_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        state_interface_types_ =
            auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
        frequency_ = auto_declare<double>("frequency", frequency_);
        amplitude_ = auto_declare<double>("amplitude", amplitude_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CustomPositionController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        auto ret = position_controllers::JointGroupPositionController::on_configure(previous_state);
        if (ret != controller_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        joint_names_ = get_node()->get_parameter("joints").as_string_array();

        if (joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
            return controller_interface::CallbackReturn::ERROR;
        }

        amplitude_ = get_node()->get_parameter("amplitude").as_double();
        frequency_ = get_node()->get_parameter("frequency").as_double();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CustomPositionController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::CallbackReturn CustomPositionController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_effort_state_interface_.clear();

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Interface: %s", interface.get_interface_name().c_str());
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CustomPositionController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CustomPositionController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // ForwardControllersBase::update method from
        auto joint_commands = rt_command_ptr_.readFromRT();

        // no command received yet
        if (!joint_commands || !(*joint_commands))
        {
            return controller_interface::return_type::OK;
        }

        if ((*joint_commands)->data.size() != command_interfaces_.size())
        {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                "command size (%zu) does not match number of interfaces (%zu)",
                (*joint_commands)->data.size(), command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }

        phase_ += 2.0 * M_PI * frequency_ * period.seconds();
        double command = amplitude_ * sin(phase_);

        for (auto index = 0ul; index < command_interfaces_.size(); ++index)
        {
            //command_interfaces_[index].set_value((*joint_commands)->data[index]);
            command_interfaces_[index].set_value(command);
        }
        // ForwardControllersBase::update method until here


        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto &joint_position = joint_position_state_interface_[i].get();
            auto &joint_velocity = joint_velocity_state_interface_[i].get();
            auto &joint_effort = joint_effort_state_interface_[i].get();
            auto &joint_name = joint_names_[i];

            RCLCPP_INFO(get_node()->get_logger(), "Joint %s, position: %f, velocity: %f, effort: %f",
                        joint_name.c_str(), joint_position.get_value(), joint_velocity.get_value(), joint_effort.get_value());
        }

        return controller_interface::return_type::OK;
    }

} // namespace custom_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    custom_position_controller::CustomPositionController, controller_interface::ControllerInterface)
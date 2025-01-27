#include "controller_interface/controller_interface.hpp"
#include "custom_effort_controller/custom_effort_controller.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

using ros2_custom_controller_sample::torque_coefficients;

namespace custom_effort_controller
{

    controller_interface::CallbackReturn CustomEffortController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("target_joints", std::vector<std::string>());
            auto_declare<std::vector<double>>("amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("frequencies", std::vector<double>());
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CustomEffortController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        auto joints = get_node()->get_parameter("joints").as_string_array();
        auto target_joints = get_node()->get_parameter("target_joints").as_string_array();
        auto amplitudes = get_node()->get_parameter("amplitudes").as_double_array();
        auto frequencies = get_node()->get_parameter("frequencies").as_double_array();
        homing_effort_ = get_node()->get_parameter("homing_effort").as_double();
        position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
        command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
        command_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::SystemDefaultsQoS());
        joint_names_ = joints;
        controller_state_ = ControllerState::IDLE;

        if (amplitudes.size() != target_joints.size() ||
            frequencies.size() != target_joints.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "The number of amplitudes and frequencies must match the number of target joints");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
            return controller_interface::CallbackReturn::ERROR;
        }

        // ジョイント設定の初期化
        joint_configs_.clear();

        for (const auto &joint : joints)
        {
            JointConfig config;
            config.name = joint;
            config.is_target = false;
            config.amplitude = 0.0;
            config.frequency = 0.0;
            config.phase = 0.0;

            // 制御対象のジョイントかチェック
            auto it = std::find(target_joints.begin(), target_joints.end(), joint);
            if (it != target_joints.end())
            {
                size_t idx = std::distance(target_joints.begin(), it);
                config.is_target = true;
                config.amplitude = amplitudes[idx];
                config.frequency = frequencies[idx];
            }

            joint_configs_.push_back(config);
        }

        // Sin波の開始/停止用サブスクライバの設定
        command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
            "~/enable", 1,
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            {
                is_running_.writeFromNonRT(msg->data);
            });

        // 初期状態は停止
        is_running_.writeFromNonRT(false);

        last_log_time_ = get_node()->get_clock()->now();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration CustomEffortController::state_interface_configuration() const
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

    controller_interface::InterfaceConfiguration CustomEffortController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // 各ジョイントに対して指定されたすべてのcommand_interfacesを追加
        for (const auto &joint : joint_names_)
        {
            for (const auto &interface : command_interface_types_)
            {
                RCLCPP_INFO(get_node()->get_logger(), "command interface: %s", (joint + "/" + interface).c_str());
                command_interfaces_config.names.push_back(joint + "/" + interface);
            }
        }

        return command_interfaces_config;
        
    }

    controller_interface::CallbackReturn CustomEffortController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_effort_state_interface_.clear();
        joint_encoder_diff_state_interface_.clear();

        // assign state interfaces
        for (auto &state_interface : state_interfaces_)
        {
            state_interface_map_[state_interface.get_interface_name()]->push_back(state_interface);
        }

        // assign command interfaces
        for (auto &command_interface : command_interfaces_)
        {
            command_interface_map_[command_interface.get_interface_name()]->push_back(command_interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CustomEffortController::on_deactivate(const rclcpp_lifecycle::State &)
    {
         // Set effort to zero
        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            joint_effort_command_interface_[i].get().set_value(0.0);
        }
        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CustomEffortController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            joint_configs_[i].current_position = joint_position_state_interface_[i].get().get_value();
            joint_configs_[i].current_velocity = joint_velocity_state_interface_[i].get().get_value();
            joint_configs_[i].current_effort = joint_effort_state_interface_[i].get().get_value();
            joint_configs_[i].encoder_diff = joint_encoder_diff_state_interface_[i].get().get_value();
        }
        // 1秒ごとにジョイントの状態を表示
        if (time - last_log_time_ >= rclcpp::Duration(1, 0))
        {
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                double diff = joint_configs_[i].encoder_diff;
                double torque = torque_coefficients.at(joint_configs_[i].name).a1 * diff +
                                torque_coefficients.at(joint_configs_[i].name).a2 * pow(diff, 2) +
                                torque_coefficients.at(joint_configs_[i].name).a3 * pow(diff, 3);
                RCLCPP_INFO(get_node()->get_logger(), "Joint: %s, Position: %f, Velocity: %f, Effort: %f, Diff: %f, Torque: %f",
                            joint_configs_[i].name.c_str(), joint_configs_[i].current_position,
                            joint_configs_[i].current_velocity, joint_configs_[i].current_effort,
                            diff,
                            torque);
            }
            last_log_time_ = time;
        }

        switch (controller_state_)
        {
        case ControllerState::IDLE:
        {
            // Effortをゼロに設定
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                joint_effort_command_interface_[i].get().set_value(0.0);
                // ControlWordを設定
                bool online = static_cast<int>(joint_configs_[i].current_position) != 0;
                uint16_t control_word = online ? 1 : 0;
                joint_control_word_command_interface_[i].get().set_value(control_word);
            }

            if (*is_running_.readFromRT())
            {
                // Sin波開始コマンドを受信したらHOMINGに移行
                controller_state_ = ControllerState::HOMING;
                
            }
            break;
        }
        case ControllerState::HOMING:
        {
            // ホーミング処理
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target)
                {
                    double command;
                    if (joint_configs_[i].current_position > 0.0)
                    {
                        command = -homing_effort_;
                    }
                    else
                    {
                        command = homing_effort_;
                    }
                    if (std::abs(joint_configs_[i].current_position) < position_tolerance_)
                    {
                        command = 0.0;
                        controller_state_ = ControllerState::RUNNING_SINE;
                        RCLCPP_INFO(get_node()->get_logger(), "Starting sine wave");
                    }

                    joint_effort_command_interface_[i].get().set_value(command);
                }
            }
            break;
        }
        case ControllerState::RUNNING_SINE:
        {
            if (!(*is_running_.readFromRT()))
            {
                // 停止コマンドを受信したら停止
                controller_state_ = ControllerState::IDLE;
                RCLCPP_INFO(get_node()->get_logger(), "Stopping sine wave");
                break;
            }

            // Sin波の生成と出力
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target)
                {
                    joint_configs_[i].phase +=
                        2.0 * M_PI * joint_configs_[i].frequency * period.seconds();
                    double command =
                        joint_configs_[i].amplitude * std::sin(joint_configs_[i].phase);
                    joint_effort_command_interface_[i].get().set_value(command);
                }
            }
            break;
        }
        }

        std_msgs::msg::Float64MultiArray command_msg;
        command_msg.data.resize(command_interfaces_.size());
        
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            command_msg.data[i] = command_interfaces_[i].get_value();
        }
        
        command_pub_->publish(command_msg);


        return controller_interface::return_type::OK;
    }

} // namespace custom_effort_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    custom_effort_controller::CustomEffortController, controller_interface::ControllerInterface)
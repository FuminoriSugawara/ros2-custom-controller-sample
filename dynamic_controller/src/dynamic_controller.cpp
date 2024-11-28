#include "controller_interface/controller_interface.hpp"
#include "dynamic_controller/dynamic_controller.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace dynamic_controller
{

    controller_interface::CallbackReturn DynamicController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("target_joints", std::vector<std::string>());
            auto_declare<std::vector<double>>("amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("frequencies", std::vector<double>());
            //state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
            //command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DynamicController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        auto joints = get_node()->get_parameter("joints").as_string_array();
        auto target_joints = get_node()->get_parameter("target_joints").as_string_array();
        //auto amplitudes = get_node()->get_parameter("amplitudes").as_double_array();
        //auto frequencies = get_node()->get_parameter("frequencies").as_double_array();
        //homing_velocity_ = get_node()->get_parameter("homing_velocity").as_double();
        //position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
        command_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::SystemDefaultsQoS());
        joint_names_ = joints;
        controller_state_ = ControllerState::IDLE;
        command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();

        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
        RCLCPP_INFO(get_node()->get_logger(), "==========Configuring==========");   
        for (const auto &state_interface : state_interface_types_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "state interface: %s", state_interface.c_str());
        }

        for (const auto &command_interface : command_interface_types_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "command interface: %s", command_interface.c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(), "==============================");

        //if (amplitudes.size() != target_joints.size() ||
        //    frequencies.size() != target_joints.size())
        //{
        //    RCLCPP_ERROR(get_node()->get_logger(),
        //                 "The number of amplitudes and frequencies must match the number of target joints");
        //    return controller_interface::CallbackReturn::ERROR;
        //}

        if (joint_names_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No joints provided");
            return controller_interface::CallbackReturn::ERROR;
        }

        // ジョイント設定の初期化
        joint_configs_.clear();
        //command_interface_types_.clear();

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
                //config.amplitude = amplitudes[idx];
                //config.frequency = frequencies[idx];
            }

            joint_configs_.push_back(config);
            //command_interface_types_.push_back(
            //    joint + "/" + hardware_interface::HW_IF_VELOCITY);
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

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DynamicController::state_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration"); 
        for (const auto &state_interface : state_interface_types_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "state interface: %s", state_interface.c_str());
        }
        controller_interface::InterfaceConfiguration conf = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name : joint_names_)
        {
            for (const auto &interface_type : state_interface_types_)
            {
                RCLCPP_INFO(get_node()->get_logger(), "state interface: %s", (joint_name + "/" + interface_type).c_str());
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration DynamicController::command_interface_configuration() const
    {
        RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration");
        for (const auto &command_interface : command_interface_types_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "command interface: %s", command_interface.c_str());
        }
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

    controller_interface::CallbackReturn DynamicController::on_activate(const rclcpp_lifecycle::State &)
    {
        // clear out vectors in case of restart
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();
        joint_effort_state_interface_.clear();
        joint_status_interface_.clear();

        // assign state interfaces
        for (auto &interface : state_interfaces_)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Interface: %s", interface.get_interface_name().c_str());
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }
        RCLCPP_INFO(get_node()->get_logger(), "Activated");

        auto i = 1;

        RCLCPP_INFO(get_node()->get_logger(), "command_interfaces_ size: %d", command_interfaces_.size());
        
        for (auto &command_interface : command_interfaces_)
        {
            
            // control_word を送信する
            if (command_interface.get_name().find("control_word") != std::string::npos)
            {
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = true;
                control_word.bits.control_mode = 0x03; // 位置制御モード
                control_word.bits.module_id = i;
                command_interface.set_value(control_word.word);
                RCLCPP_INFO(get_node()->get_logger(), "control_word: %d", control_word.word);
                i++;
            }
        }
        


        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DynamicController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Set velocity to zero
        for (auto &command_interface : command_interfaces_)
        {
            command_interface.set_value(0.0);
        }

        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DynamicController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {


        //// control_word を送信する
        //uint8_t count = 25;
        //for (auto &command_interface : command_interfaces_)
        //{
        //    command_interface.set_value(count);
        //    count++;
        //}
        


        
        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            joint_configs_[i].current_position = joint_position_state_interface_[i].get().get_value();
            joint_configs_[i].current_velocity = joint_velocity_state_interface_[i].get().get_value();
            joint_configs_[i].current_effort = joint_effort_state_interface_[i].get().get_value();
            joint_configs_[i].current_status.word = joint_status_interface_[i].get().get_value();
        }
        // 1秒ごとにログを出力

        if (time.seconds() - last_log_time_.seconds() > 1.0)
        {
            last_log_time_ = time;
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Joint %s: Position: %f, Velocity: %f, Effort: %f, Status: 0x%04x",
                            joint_configs_[i].name.c_str(), joint_configs_[i].current_position,
                            joint_configs_[i].current_velocity, joint_configs_[i].current_effort,
                            static_cast<uint16_t>(joint_configs_[i].current_status.word));

            }
        }

        //switch (controller_state_)
        //{
        //case ControllerState::IDLE:
        //{
        //    // 速度をゼロに設定
        //    for (auto &command_interface : command_interfaces_)
        //    {
        //        command_interface.set_value(0.0);
        //    }

        //    if (*is_running_.readFromRT())
        //    {
        //        // Sin波開始コマンドを受信したらHOMINGに移行
        //        controller_state_ = ControllerState::RUNNING_SINE;
        //        RCLCPP_INFO(get_node()->get_logger(), "Starting sine wave");
        //    } else {
        //        // ホーミングコマンドを受信したらHOMINGに移行
        //        controller_state_ = ControllerState::HOMING;
        //    }
        //    break;
        //}
        //case ControllerState::HOMING:
        //{
        //    // ホーミング処理
        //    for (size_t i = 0; i < joint_configs_.size(); ++i)
        //    {
        //        if (joint_configs_[i].is_target)
        //        {
        //            double command;
        //            if (joint_configs_[i].current_position > 0.0)
        //            {
        //                command = -homing_velocity_;
        //            }
        //            else
        //            {
        //                command = homing_velocity_;
        //            }
        //            if (std::abs(joint_configs_[i].current_position) < position_tolerance_)
        //            {
        //                command = 0.0;
        //                controller_state_ = ControllerState::IDLE;
        //            }

        //            command_interfaces_[i].set_value(command);
        //        }
        //    }
        //    break;
        //}
        //case ControllerState::RUNNING_SINE:
        //{
        //    if (!(*is_running_.readFromRT()))
        //    {
        //        // 停止コマンドを受信したら停止
        //        controller_state_ = ControllerState::STOPPING_SINE;
        //        // 各ジョイントの現在の速度を保存し、減速時間を設定
        //        for (auto &config : joint_configs_)
        //        {
        //            if (config.is_target)
        //            {
        //                config.stop_initial_command = config.amplitude * std::sin(config.phase);
        //                config.deceleration_time = deceleration_duration_;
        //                config.elapsed_stop_time = 0.0;
        //                config.phase = 0.0;
        //            }
        //        }
        //        
        //        RCLCPP_INFO(get_node()->get_logger(), "Starting smooth deceleration");
        //        break;
        //    }

        //    // Sin波の生成と出力
        //    for (size_t i = 0; i < joint_configs_.size(); ++i)
        //    {
        //        if (joint_configs_[i].is_target)
        //        {
        //            joint_configs_[i].phase +=
        //                2.0 * M_PI * joint_configs_[i].frequency * period.seconds();
        //            double command =
        //                joint_configs_[i].amplitude * std::sin(joint_configs_[i].phase);
        //            command_interfaces_[i].set_value(command);
        //        }
        //    }
        //    break;
        //}
        //case ControllerState::STOPPING_SINE:
        //{
        //    bool all_stopped = true;
        //    
        //    for (size_t i = 0; i < joint_configs_.size(); ++i)
        //    {
        //        if (joint_configs_[i].is_target)
        //        {
        //            // 経過時間を更新
        //            joint_configs_[i].elapsed_stop_time += period.seconds();
        //            
        //            // 線形減速の計算
        //            double progress = std::min(joint_configs_[i].elapsed_stop_time / 
        //                                    joint_configs_[i].deceleration_time, 1.0);
        //            
        //            // 初期速度から線形に減速
        //            double command = joint_configs_[i].stop_initial_command * (1.0 - progress);

        //            RCLCPP_INFO(get_node()->get_logger(), "Progress: %f, Command: %f", progress, command);
        //            
        //            command_interfaces_[i].set_value(command);
        //            
        //            // 速度が十分小さくなったかチェック
        //            if (std::abs(command) > velocity_tolerance_)
        //            {
        //                all_stopped = false;
        //            }
        //        }
        //    }

        //    if (all_stopped)
        //    {
        //        // すべてのジョイントの速度が十分小さくなったらホーミングへ移行
        //        controller_state_ = ControllerState::HOMING;
        //        RCLCPP_INFO(get_node()->get_logger(), "Transitioning to homing");
        //    }
        //    break;
        //}
        //}

        std_msgs::msg::Float64MultiArray command_msg;
        command_msg.data.resize(command_interfaces_.size());
        
        for (size_t i = 0; i < command_interfaces_.size(); ++i)
        {
            command_msg.data[i] = command_interfaces_[i].get_value();
        }
        
        command_pub_->publish(command_msg);

        return controller_interface::return_type::OK;
    }

} // namespace dynamic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dynamic_controller::DynamicController, controller_interface::ControllerInterface)
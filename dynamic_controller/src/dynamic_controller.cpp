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
            auto_declare<std::vector<double>>("position_control_amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("position_control_frequencies", std::vector<double>());
            auto_declare<std::vector<double>>("velocity_control_amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("velocity_control_frequencies", std::vector<double>());
            auto_declare<std::vector<double>>("effort_control_amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("effort_control_frequencies", std::vector<double>());
            auto_declare<std::vector<int>>("module_ids", std::vector<int>());
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
        auto module_ids = get_node()->get_parameter("module_ids").as_integer_array();
        auto position_control_amplitudes = get_node()->get_parameter("position_control_amplitudes").as_double_array();
        auto position_control_frequencies = get_node()->get_parameter("position_control_frequencies").as_double_array();
        auto velocity_control_amplitudes = get_node()->get_parameter("velocity_control_amplitudes").as_double_array();
        auto velocity_control_frequencies = get_node()->get_parameter("velocity_control_frequencies").as_double_array();
        auto effort_control_amplitudes = get_node()->get_parameter("effort_control_amplitudes").as_double_array();
        auto effort_control_frequencies = get_node()->get_parameter("effort_control_frequencies").as_double_array();
        homing_velocity_ = get_node()->get_parameter("homing_velocity").as_double();
        position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
        command_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/command", rclcpp::SystemDefaultsQoS());
        joint_names_ = joints;
        controller_state_.writeFromNonRT(ControllerState::STARTING);
        control_mode_.writeFromNonRT(ControlMode::POSITION);
        command_interface_types_ = get_node()->get_parameter("command_interfaces").as_string_array();
        state_interface_types_ = get_node()->get_parameter("state_interfaces").as_string_array();
        
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
            config.module_id = 0;
            config.is_target = false;
            config.is_online = false;
            config.position_control_amplitude = 0.0;
            config.position_control_frequency = 0.0;
            config.velocity_control_amplitude = 0.0;
            config.velocity_control_frequency = 0.0;
            config.effort_control_amplitude = 0.0;
            config.effort_control_frequency = 0.0;
            config.phase = 0.0;

            // 制御対象のジョイントかチェック
            auto it = std::find(target_joints.begin(), target_joints.end(), joint);
            if (it != target_joints.end())
            {
                size_t idx = std::distance(target_joints.begin(), it);
                config.is_target = true;
                config.module_id = module_ids[idx];
                config.position_control_amplitude = position_control_amplitudes[idx];
                config.position_control_frequency = position_control_frequencies[idx];
                config.velocity_control_amplitude = velocity_control_amplitudes[idx];
                config.velocity_control_frequency = velocity_control_frequencies[idx];
                config.effort_control_amplitude = effort_control_amplitudes[idx];
                config.effort_control_frequency = effort_control_frequencies[idx];
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
        
        controller_state_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int8>(
            "~/controller_state", 1,
            [this](const std_msgs::msg::Int8::SharedPtr msg)
            {
                RCLCPP_INFO(get_node()->get_logger(), "controller_state: %d", msg->data);
                controller_state_.writeFromNonRT(static_cast<ControllerState>(msg->data));
            });

        control_mode_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int8>(
            "~/control_mode", 1,
            [this](const std_msgs::msg::Int8::SharedPtr msg)
            {
                RCLCPP_INFO(get_node()->get_logger(), "control_mode: %d", msg->data);
                control_mode_.writeFromNonRT(static_cast<ControlMode>(msg->data));
                controller_state_.writeFromNonRT(ControllerState::MODE_CHANGING);
            });

        // 初期状態は停止
        is_running_.writeFromNonRT(false);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration DynamicController::state_interface_configuration() const
    {
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

    controller_interface::CallbackReturn DynamicController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Set velocity to zero
        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            // 速度をゼロに設定
            joint_velocity_command_interface_[i].get().set_value(0.0);
            // 電流をゼロに設定
            joint_effort_command_interface_[i].get().set_value(0.0);
        }


        release_interfaces();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DynamicController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        ControllerState controller_state = *controller_state_.readFromRT();

        for (size_t i = 0; i < joint_configs_.size(); ++i)
        {
            joint_configs_[i].current_position = joint_position_state_interface_[i].get().get_value();
            joint_configs_[i].current_velocity = joint_velocity_state_interface_[i].get().get_value();
            joint_configs_[i].current_effort = joint_effort_state_interface_[i].get().get_value();
            joint_configs_[i].current_status.word = joint_status_interface_[i].get().get_value();
        }

        switch (controller_state)
        {
        case ControllerState::STARTING:
        {
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(ControlMode::POSITION);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::STARTING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            controller_state_.writeFromNonRT(ControllerState::INITIALIZING);
            break;
        }
        case ControllerState::INITIALIZING:
        {
            // 初回実行時の初期化
            if (online_sent_.empty())
            {
                online_sent_.resize(joint_configs_.size(), false);
                last_online_sent_time_ = time;
            }

            if (!online_sent_[current_joint_index_] && (time - last_online_sent_time_).seconds() >= 0.5)
            {

                JointConfig &config = joint_configs_[current_joint_index_];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(ControlMode::POSITION);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::INITIALIZING);
                control_word.bits.module_id = config.module_id;

                joint_control_word_command_interface_[current_joint_index_].get().set_value(control_word.word);

                RCLCPP_INFO(get_node()->get_logger(), "Sending Online to joint %zu", current_joint_index_);

                online_sent_[current_joint_index_] = true;
                last_online_sent_time_ = time;
                current_joint_index_++;
            }

            if (current_joint_index_ >= joint_configs_.size())
            {
                online_sent_.clear();
                current_joint_index_ = 0;
                controller_state_.writeFromNonRT(ControllerState::CONNECTION_WAITING);
                last_online_check_time_ = time;
            } else {
                controller_state_.writeFromNonRT(ControllerState::STARTING);
            }
            break;
        }
        case ControllerState::CONNECTION_WAITING:
        {
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(static_cast<uint8_t>(ControlMode::POSITION));
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::CONNECTION_WAITING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            
            // 1秒経過したかチェック
            if ((time - last_online_check_time_).seconds() >= 1.0)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Checking connection...");

                // 接続確認
                bool all_joints_connected = true;
                for (size_t i = 0; i < joint_configs_.size(); ++i)
                {
                    if (joint_configs_[i].is_target &&
                        !joint_configs_[i].current_status.bits.enable)
                    {
                        all_joints_connected = false;
                        break;
                    }
                }

                if (all_joints_connected)
                {
                    // 接続成功
                    RCLCPP_INFO(get_node()->get_logger(), "All joints connected successfully");
                    online_check_retry_count_ = 0;
                    controller_state_.writeFromNonRT(ControllerState::INITIAL_MODE_SETTING);
                }
                else
                {
                    // 接続失敗
                    online_check_retry_count_++;
                    if (online_check_retry_count_ >= online_check_max_retry_)
                    {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "Connection failed after %d attempts, reinitializing...", online_check_max_retry_);
                        online_check_retry_count_ = 0;
                        controller_state_.writeFromNonRT(ControllerState::STARTING);
                    }
                }

                last_online_check_time_ = time;
            }
            break;
        }
        case ControllerState::INITIAL_MODE_SETTING:
        {
            control_mode_.writeFromNonRT(ControlMode::POSITION);
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(static_cast<uint8_t>(ControlMode::POSITION));
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::INITIAL_MODE_SETTING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            // control mode が POSITION になったかチェック
            bool all_joints_in_position_control_mode = true;
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target &&
                    joint_configs_[i].current_status.bits.control_mode != static_cast<uint8_t>(ControlMode::POSITION))
                {
                    all_joints_in_position_control_mode = false;
                    break;
                }
            }

            // すべてのジョイントが位置制御モードになったら IDLING に移行
            if (all_joints_in_position_control_mode)
            {
                controller_state_.writeFromNonRT(ControllerState::IDLING);
                RCLCPP_INFO(get_node()->get_logger(), "IDLING");
            }
            break;
        }
        case ControllerState::IDLING:
        {
            ControlMode control_mode = *control_mode_.readFromRT();
            // ControlWord を送信する
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(control_mode);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::IDLING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            // なにもしない
            break;
        }
        case ControllerState::HOMING:
        {
            RCLCPP_INFO(get_node()->get_logger(), "HOMING");
            control_mode_.writeFromNonRT(ControlMode::POSITION);
            // ホーミング処理
            bool all_joints_homed = true;
            ControlMode control_mode = *control_mode_.readFromRT();
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(control_mode);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::HOMING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);

                if (config.current_status.bits.enable)
                {
                    double current_position = config.current_position;
                    double command = 0.0;
                    if (std::abs(current_position) > position_tolerance_)
                    {
                        if (current_position > 0.0)
                        {
                            command = std::max(0.0, current_position - homing_velocity_);
                        }
                        else
                        {
                            command = std::min(0.0, current_position + homing_velocity_);
                        }
                        all_joints_homed = false;
                    }
                    joint_position_command_interface_[i].get().set_value(command);
                }
            }
            // すべてのジョイントがホーミング完了したら IDLING に移行
            if (all_joints_homed)
            {
                controller_state_.writeFromNonRT(ControllerState::IDLING);
                RCLCPP_INFO(get_node()->get_logger(), "IDLING");
            }
            break;
        }
        case ControllerState::CONTROLLING:
        {
            RCLCPP_INFO(get_node()->get_logger(), "CONTROLLING");
            // ここでSin波の処理
            ControlMode control_mode = *control_mode_.readFromRT();
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(control_mode);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::CONTROLLING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
                if (config.is_target)
                {
                    switch (control_mode)
                    {
                    case ControlMode::POSITION:
                    {
                        config.phase +=
                            2.0 * M_PI * config.position_control_frequency * period.seconds();
                        double command =
                            config.position_control_amplitude * std::sin(config.phase);
                        joint_position_command_interface_[i].get().set_value(command);
                        break;
                    }
                    case ControlMode::VELOCITY:
                    {
                        config.phase +=
                            2.0 * M_PI * config.velocity_control_frequency * period.seconds();
                        double command =
                            config.velocity_control_amplitude * std::sin(config.phase);
                        joint_velocity_command_interface_[i].get().set_value(command);
                        break;
                    }
                    case ControlMode::EFFORT:
                    {
                        config.phase +=
                            2.0 * M_PI * config.effort_control_frequency * period.seconds();
                        double command =
                            config.effort_control_amplitude * std::sin(config.phase);
                        joint_effort_command_interface_[i].get().set_value(command);
                        break;
                    }
                    }
                }
            }

            break;
        }
        case ControllerState::MODE_CHANGING:
        {
            // モード変更処理
            // RCLCPP_INFO(get_node()->get_logger(), "MODE_CHANGING");
            ControlMode control_mode = *control_mode_.readFromRT();
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(control_mode);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::MODE_CHANGING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            // すべてのジョイントのモードが変更されたかをチェック
            bool all_joints_mode_changed = true;
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target &&
                    joint_configs_[i].current_status.bits.control_mode != static_cast<uint8_t>(control_mode))
                {
                    all_joints_mode_changed = false;
                    break;
                }
            }
            // すべてのジョイントのモードが変更されたら ENABLING に移行
            if (all_joints_mode_changed)
            {
                controller_state_.writeFromNonRT(ControllerState::ENABLING);
                RCLCPP_INFO(get_node()->get_logger(), "ENABLING");
            }

            break;
        }
        case ControllerState::STOPPING:
        {
            RCLCPP_INFO(get_node()->get_logger(), "STOPPING");
            ControlMode control_mode = *control_mode_.readFromRT();
            // 停止処理
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                joint_configs_[i].phase = 0.0;
                if (joint_configs_[i].is_target)
                {
                    switch (control_mode)
                    {
                    case ControlMode::VELOCITY:
                    {
                        joint_velocity_command_interface_[i].get().set_value(0.0);
                        break;
                    }
                    case ControlMode::EFFORT:
                    {
                        joint_effort_command_interface_[i].get().set_value(0.0);
                        break;
                    }
                    case ControlMode::POSITION:
                    {
                        break;
                    }
                    case ControlMode::OPEN:
                    {
                        break;
                    }
                    }
                }
            }

            break;
        }
        case ControllerState::ENABLING:
        {
            // 有効化処理
            ControlMode control_mode = *control_mode_.readFromRT();
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                JointConfig &config = joint_configs_[i];
                ControlWord control_word;
                control_word.word = 0;
                control_word.bits.enable = config.is_target;
                control_word.bits.control_mode = static_cast<uint8_t>(control_mode);
                control_word.bits.control_state = static_cast<uint8_t>(ControllerState::ENABLING);
                control_word.bits.module_id = config.module_id;
                joint_control_word_command_interface_[i].get().set_value(control_word.word);
            }
            // IDLING に移行
            controller_state_.writeFromNonRT(ControllerState::IDLING);
            RCLCPP_INFO(get_node()->get_logger(), "IDLING");
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

        // 1秒ごとにログを出力

        // if (time.seconds() - last_log_time_.seconds() > 1.0)
        //{
        //     last_log_time_ = time;
        //     for (size_t i = 0; i < joint_configs_.size(); ++i)
        //     {
        //         RCLCPP_INFO(get_node()->get_logger(), "Joint %s: Module Id 0x%02X, Position: %f, Velocity: %f, Effort: %f, Status: 0x%04x",
        //                     joint_configs_[i].name.c_str(),
        //                     joint_configs_[i].module_id,
        //                     joint_configs_[i].current_position,
        //                     joint_configs_[i].current_velocity,
        //                     joint_configs_[i].current_effort,
        //                     static_cast<uint16_t>(joint_configs_[i].current_status.word));
        //     }
        // }

        return controller_interface::return_type::OK;
    }

} // namespace dynamic_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dynamic_controller::DynamicController, controller_interface::ControllerInterface)

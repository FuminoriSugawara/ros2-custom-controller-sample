#include "controller_interface/controller_interface.hpp"
#include "custom_position_controller/custom_position_controller.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace custom_position_controller
{

    controller_interface::CallbackReturn CustomPositionController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("target_joints", std::vector<std::string>());
            auto_declare<std::vector<double>>("amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("frequencies", std::vector<double>());
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
            auto_declare<double>("homing_velocity", 0.1);     // ゼロ位置への移動速度
            auto_declare<double>("position_tolerance", 0.01); // 位置許容誤差
        }
        catch (const std::exception &e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn CustomPositionController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        // パラメータの取得
        auto joints = get_node()->get_parameter("joints").as_string_array();
        auto target_joints = get_node()->get_parameter("target_joints").as_string_array();
        auto amplitudes = get_node()->get_parameter("amplitudes").as_double_array();
        auto frequencies = get_node()->get_parameter("frequencies").as_double_array();
        homing_velocity_ = get_node()->get_parameter("homing_velocity").as_double();
        position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
        controller_state_ = ControllerState::IDLE;

        // 設定のバリデーション
        if (amplitudes.size() != target_joints.size() ||
            frequencies.size() != target_joints.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "The number of amplitudes and frequencies must match the number of target joints");
            return controller_interface::CallbackReturn::ERROR;
        }

        // ジョイント設定の初期化
        joint_configs_.clear();
        command_interface_types_.clear();

        // 全ジョイントの設定を作成
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
            command_interface_types_.push_back(
                joint + "/" + hardware_interface::HW_IF_POSITION);
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
        for (size_t i = 0; i < joint_configs_.size(); ++i) {
            joint_configs_[i].current_position = joint_position_state_interface_[i].get().get_value();
            joint_configs_[i].current_velocity = joint_velocity_state_interface_[i].get().get_value();
            joint_configs_[i].current_effort = joint_effort_state_interface_[i].get().get_value();
        }

        // コントローラの状態に応じた処理
        switch (controller_state_)
        {
        case ControllerState::IDLE:
        {
            if (*is_running_.readFromRT())
            {
                // Sin波開始コマンドを受信したらHOMINGに移行
                controller_state_ = ControllerState::HOMING;
                RCLCPP_INFO(get_node()->get_logger(), "Starting homing sequence");
            }
            break;
        }

        case ControllerState::HOMING:
        {
            bool all_joints_homed = true;

            // 各ジョイントをゼロ位置に移動
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (!joint_configs_[i].is_target)
                {
                    continue;
                }

                double current_pos = joint_configs_[i].current_position;
                if (std::abs(current_pos) > position_tolerance_)
                {
                    // ゼロ位置までの移動指令を生成
                    double command = current_pos > 0 ? current_pos - homing_velocity_ * period.seconds() : current_pos + homing_velocity_ * period.seconds();

                    command_interfaces_[i].set_value(command);
                    all_joints_homed = false;
                }
            }

            // すべてのジョイントがゼロ位置に到達したらSin波出力開始
            if (all_joints_homed)
            {
                controller_state_ = ControllerState::RUNNING_SINE;
                // 位相をリセット
                for (auto &config : joint_configs_)
                {
                    config.phase = 0.0;
                }
                RCLCPP_INFO(get_node()->get_logger(), "Homing complete, starting sine wave");
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
                    command_interfaces_[i].set_value(command);
                }
            }
            break;
        }
        }

        return controller_interface::return_type::OK;
    }

} // namespace custom_position_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    custom_position_controller::CustomPositionController, controller_interface::ControllerInterface)

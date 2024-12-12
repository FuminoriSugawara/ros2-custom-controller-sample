#include "controller_interface/controller_interface.hpp"
#include "custom_effort_controller/custom_effort_controller.hpp"

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "custom_effort_controller/kkwTABC.h"


namespace custom_effort_controller
{


	kkwTABC tabc[7];

    controller_interface::CallbackReturn CustomEffortController::on_init()
    {
        try
        {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("target_joints", std::vector<std::string>());
            auto_declare<std::vector<double>>("amplitudes", std::vector<double>());
            auto_declare<std::vector<double>>("frequencies", std::vector<double>());
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
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
        command_interface_types_.clear();

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
                joint + "/" + hardware_interface::HW_IF_EFFORT);
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
		for(int i=0;i<7;i++) tabc[i].reset(); //KIKUUWE

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

    controller_interface::CallbackReturn CustomEffortController::on_activate(const rclcpp_lifecycle::State &)
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

    controller_interface::CallbackReturn CustomEffortController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        // Set effort to zero
        for (auto &command_interface : command_interfaces_)
        {
            command_interface.set_value(0.0);
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
        }

        switch (controller_state_)
        {
        case ControllerState::IDLE:
        {
            // Effortをゼロに設定
            for (auto &command_interface : command_interfaces_)
            {
                command_interface.set_value(0.0);
            }

            if (*is_running_.readFromRT())
            {
                // Sin波開始コマンドを受信したらRUNNING_SINEに移行
                controller_state_ = ControllerState::RUNNING_SINE;
                RCLCPP_INFO(get_node()->get_logger(), "Starting sine wave");
            //} else {
                // ホーミングコマンドを受信したらHOMINGに移行
                // controller_state_ = ControllerState::HOMING;
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
                        controller_state_ = ControllerState::IDLE;
                    }

                    command_interfaces_[i].set_value(command);
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
#if 0
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
#else
			//KIKUUWE
            {
                static int CNTR=0;
                if(CNTR==0) for(int i=0;i<7;i++) tabc[i].first_use = 1;
                CNTR =1;
            }
            static int cntr = 0; cntr++;
            double t = 0.002 * cntr;
			for(int i=0;i<7;i++) tabc[i].T  = 0.002;   // sampling interval in s.  
			for(int i=0;i<7;i++) tabc[i].M  = 0;       // for inertial compensation, in kg.m^2. Can be zero.
			for(int i=0;i<7;i++) tabc[i].secB = 1;  // safer to set it to be 1. Kikuuwe's 2018 TRO paper, "Modification B"         
			for(int i=0;i<7;i++) tabc[i].secC = 0;  // Kikuuwe's 2018 TRO paper, "Modification C". Not effective when M=0. 
			// K  : Proportional gain in Nm/rad. Must not be zero.
			// B  : Derivative gain in Nms/rad. Must not be zero.
			// L  : Integral gain in Nm/rad/s. Can be small but should not be zero.
			// F  : torque limit in Nm
			// MV : inertia in kg.m^2. Must not be zero.
			// BV : viscosity in Nms/rad. Must not be zero.
			// KV : stiffness in Nm/rad. Can be zero.
			// FV : friction in Nm. Can be zero.
            // 0-1: WHJ-60
            // 2-3: WHJ-30
            // 4-6: WHJ-10  <= could not be tuned enough.
            
			tabc[0].K = 400 ; tabc[1].K = 400 ; tabc[2].K = 400 ; tabc[3].K = 400 ; tabc[4].K = 100 ; tabc[5].K = 100 ; tabc[6].K = 100 ;
			tabc[0].B = 8   ; tabc[1].B = 8   ; tabc[2].B = 8   ; tabc[3].B = 8   ; tabc[4].B = 2   ; tabc[5].B = 2   ; tabc[6].B = 2   ;
			tabc[0].L = 0.01; tabc[1].L = 0.01; tabc[2].L = 0.01; tabc[3].L = 0.01; tabc[4].L = 0.01; tabc[5].L = 0.01; tabc[6].L = 0.01;
			tabc[0].F = 2   ; tabc[1].F = 4   ; tabc[2].F = 2   ; tabc[3].F = 2   ; tabc[4].F = 1   ; tabc[5].F = 1   ; tabc[6].F = 1   ;
			tabc[0].MV= 0.1 ; tabc[1].MV= 0.1 ; tabc[2].MV= 0.1 ; tabc[3].MV= 0.1 ; tabc[4].MV= 0.1 ; tabc[5].MV= 0.1 ; tabc[6].MV= 0.1 ;
			tabc[0].BV= 2   ; tabc[1].BV= 2   ; tabc[2].BV= 2   ; tabc[3].BV= 2   ; tabc[4].BV= 2   ; tabc[5].BV= 2   ; tabc[6].BV= 2   ;
			tabc[0].KV= 5.  ; tabc[1].KV= 5.  ; tabc[2].KV= 5   ; tabc[3].KV= 5   ; tabc[4].KV= 5   ; tabc[5].KV= 5   ; tabc[6].KV= 5   ;
			tabc[0].FV= 0   ; tabc[1].FV= 0   ; tabc[2].FV= 0   ; tabc[3].FV= 0   ; tabc[4].FV= 0   ; tabc[5].FV= 0   ; tabc[6].FV= 0   ;

            const double mA2Nm[7]= {0.002,0.002,0.001,0.001,0.001,0.001,0.001} ; // TEMPORARY 
            const double tauSDZ[7]= {0.5,0.5,0.5,0.5,0.5,0.5,0.5} ; // TEMPORARY 

            double tauD[7];
            double tauS[7];
            double angD[7];
            double angS[7];
            angD[0] = (M_PI/180.)*(  0.+  0.*sin(2.*M_PI*t/5));
            angD[1] = (M_PI/180.)*(  0.+  0.*sin(2.*M_PI*t/10));
            angD[2] = (M_PI/180.)*(  0.+ 20.*sin(2.*M_PI*t/3));
            angD[3] = (M_PI/180.)*(  0.+ 20.*sin(2.*M_PI*t/7));
            angD[4] = (M_PI/180.)*(  0.+  0.*sin(2.*M_PI*t/4));
            angD[5] = (M_PI/180.)*(  0.+  0.*sin(2.*M_PI*t/5));
            angD[6] = (M_PI/180.)*(  0.+  0.*sin(2.*M_PI*t/5));
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target)
                {
                    tauD[i] = 0;
                    tauS[i] = kkwTABC::dzn( tauSDZ[i] ,  - mA2Nm[i] * joint_configs_[i].current_effort  ) ;
                    angS[i] = ((M_PI/180.)*0.0001)* joint_configs_[i].current_position;
                    if(i==0) tauS[i]=0 ;
                    if(i==1) tauS[i]=0 ;
                    if(i==2) tauS[i]=0 ;
                    if(i==3) tauS[i]=0 ;
                    if(i==4) tauS[i]=0 ;
                    if(i==5) tauS[i]=0 ;
                    if(i==6) tauS[i]=0 ;
                }
            }

            double tauMG[7];
            for(int i=0;i<7;i++) tauMG[i]=0;
            tauMG[1] = -2.* sin(angS[1]);      // rough gravity compensation
            tauMG[3] = -1.* sin(angS[1]+angS[3]);  // rough gravity compensation
            tauMG[5] = -0.5* sin(angS[1]+angS[3]+angS[5]); // rough gravity compensation
            double tauM[7];
            double angX[7];
            for (size_t i = 0; i < joint_configs_.size(); ++i)
            {
                if (joint_configs_[i].is_target)
                {
                    angX[i] = 0;
                    //if(i==2) RCLCPP_INFO(get_node()->get_logger(),"%d : qs1: %f ,  angS: %f", tabc[i].first_use, tabc[i].qs1, angS);
    				tauM[i] = tabc[i].work(tauD[i], angD[i], tauS[i], angS[i], &angX[i]);
                    tauM[i] += tauMG[i];
    				//if(i==5) tauM[i]=0;
    				if(i==6) tauM[i]=0;
                    command_interfaces_[i].set_value(tauM[i]/mA2Nm[i]);
                }
            }
#endif
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
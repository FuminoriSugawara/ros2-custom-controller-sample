#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_
namespace ros2_custom_controller_sample
{
    typedef struct
    {
        double a1;
        double a2;
        double a3;
    } TorqueCoefficients;

    static const std::map<std::string, TorqueCoefficients> torque_coefficients = {
        {"joint_1", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"joint_2", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"joint_3", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"joint_4", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"joint_5", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"joint_6", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"joint_7", {5.1807E-02, -7.7299E-05, 2.4623E-06}},  // WHJ10-80
        {"left_joint_1", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"left_joint_2", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"left_joint_3", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"left_joint_4", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"left_joint_5", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"left_joint_6", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"left_joint_7", {5.1807E-02, -7.7299E-05, 2.4623E-06}},  // WHJ10-80
        {"right_joint_1", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"right_joint_2", {2.1492E-01, 3.1069E-04, 8.8751E-06}},  // WHJ60-100
        {"right_joint_3", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"right_joint_4", {1.4302E-01, 1.1097E-04, 4.2523E-06}},  // WHJ30-80
        {"right_joint_5", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"right_joint_6", {5.1807E-02, -7.7299E-05, 2.4623E-06}}, // WHJ10-80
        {"right_joint_7", {5.1807E-02, -7.7299E-05, 2.4623E-06}}  // WHJ10-80
    };

}

#endif
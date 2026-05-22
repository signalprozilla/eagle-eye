#ifndef ANATOMY_H
#define ANATOMY_H

#include <Eigen/Dense>

// Standard ISO C++ Compile-Time Constants
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

struct SegmentFrame {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Matrix3d axes = Eigen::Matrix3d::Identity(); 

    const Eigen::Matrix3d& rotation() const { return axes; }
};

struct JointKinematics {
    double elbow_flexion_deg = 0.0;
    double elbow_valgus_carrying_deg = 0.0;
    double forearm_pronation_supination_deg = 0.0;
    double shoulder_external_rotation_deg = 0.0;
    double shoulder_horizontal_abduction_deg = 0.0;
    double shoulder_elevation_deg = 0.0;
};

#endif // ANATOMY_H
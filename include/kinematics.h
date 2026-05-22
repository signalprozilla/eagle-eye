// #ifndef KINEMATICS_H
// #define KINEMATICS_H
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/Dense> // Explicit header hygiene dependency insulation
// #include "anatomy.h"


#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "anatomy.h" // Pulls in the original SegmentFrame and JointKinematics definitions safely

// Exposed calculation functions
SegmentFrame computeForearmFrame(const Eigen::Vector3d& melb, const Eigen::Vector3d& elb, const Eigen::Vector3d& wrb);
SegmentFrame computeHumerusFrame(const Eigen::Vector3d& sho, const Eigen::Vector3d& melb, const Eigen::Vector3d& elb);
JointKinematics calculateRelativeAngles(const Eigen::Matrix3d& R_parent, const Eigen::Matrix3d& R_child);

// Pure physics engine node for rotational inertia math
double computeRotationalMoment(double I_com_kgm2, double alpha_rad_s2);


double computeElbowVarusTorque(double vel_deg_s, double accel_deg_s2, 
                               double segment_length_meters, double body_mass_kg);

#endif // KINEMATICS_H

// SegmentFrame computeForearmFrame(const Eigen::Vector3d& elbowM, 
//                                  const Eigen::Vector3d& elbowL, 
//                                  const Eigen::Vector3d& wristM);

// JointKinematics calculateRelativeAngles(const Eigen::Matrix3d& R_humerus, const Eigen::Matrix3d& R_forearm);

// SegmentFrame computeTorsoFrame(const Eigen::Vector3d& ij,
//                                const Eigen::Vector3d& px,
//                                const Eigen::Vector3d& c7,
//                                const Eigen::Vector3d& t8);

// double computeElbowVarusTorque(double flexion_deg, double vel_deg_s, double accel_deg_s2, 
//                                double segment_length_meters, double body_mass_kg);



// --------------


// // Keeps track of our relative joint angles
// struct JointKinematics {
//     double elbow_flexion_deg;
//     double elbow_varus_deg;
// };

// // Holds our 3D coordinate system basis vectors
// struct SegmentFrame {
//     Eigen::Vector3d origin;
//     Eigen::Matrix3d axes; // Verify this is named 'axes' and matches your codebase!
    
//     Eigen::Matrix3d rotation() const { return axes; }
// };

// // Updated function declarations
// SegmentFrame computeForearmFrame(const Eigen::Vector3d& melb, const Eigen::Vector3d& elb, const Eigen::Vector3d& wrb);
// SegmentFrame computeHumerusFrame(const Eigen::Vector3d& sho, const Eigen::Vector3d& melb, const Eigen::Vector3d& elb);
// JointKinematics calculateRelativeAngles(const Eigen::Matrix3d& R_parent, const Eigen::Matrix3d& R_child);

// double computeElbowVarusTorque(double vel_deg_s, double accel_deg_s2, 
//                                double segment_length_meters, double body_mass_kg);

// #endif // KINEMATICS_H
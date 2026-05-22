#include "kinematics.h"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>

// Explicit definition of local constants if not present in header
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
extern const double kRadToDeg; // Inherited from anatomy/kinematics headers

// Forearm Segment Frame Builder
SegmentFrame computeForearmFrame(const Eigen::Vector3d& elbowM, 
                                 const Eigen::Vector3d& elbowL, 
                                 const Eigen::Vector3d& wristM) {
    SegmentFrame frame;
    frame.origin = 0.5 * (elbowM + elbowL);
    
    Eigen::Vector3d z_axis = (frame.origin - wristM).normalized();
    Eigen::Vector3d elbow_axis = (elbowL - elbowM).normalized();
    
    // Guard against degenerate frames from dropped/noisy markers
    Eigen::Vector3d cross_product = elbow_axis.cross(z_axis);
    assert(cross_product.norm() > 1e-4 && "Degenerate frame detected: markers are collinear!");
    
    Eigen::Vector3d x_axis = cross_product.normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();
    
    frame.axes.col(0) = x_axis;
    frame.axes.col(1) = y_axis;
    frame.axes.col(2) = z_axis;
    
    return frame;
}

// Relative Joint Angle Kinematics Engine
JointKinematics calculateRelativeAngles(const Eigen::Matrix3d& R_parent, const Eigen::Matrix3d& R_child) {
    JointKinematics kinematics;
    Eigen::Matrix3d R_rel = R_parent.transpose() * R_child;
    
    // 1. ELBOW KINEMATICS (Strict Intrinsic Y-X-Z Cardan Sequence)
    double clamped_elbow = std::clamp(R_rel(1, 2), -1.0, 1.0);
    double valgusRad = std::asin(-clamped_elbow); 
    double flexionRad = std::atan2(R_rel(0, 2), R_rel(2, 2));
    double rotationRad = std::atan2(R_rel(1, 0), R_rel(1, 1));
    
    kinematics.elbow_flexion_deg             = flexionRad * kRadToDeg;
    kinematics.elbow_valgus_carrying_deg     = valgusRad * kRadToDeg;
    kinematics.forearm_pronation_supination_deg = rotationRad * kRadToDeg;
    
    // 2. SHOULDER KINEMATICS (Strict ISB Y-X'-Y'' Joint Coordinate System)
    double cos_elevation = std::clamp(R_rel(2, 2), -1.0, 1.0);
    double shoulder_elevation_rad = std::acos(cos_elevation);
    
    double shoulder_plane_rad = 0.0;
    double shoulder_rotation_rad = 0.0;
    
    if (std::abs(cos_elevation) < 0.9999) {
        shoulder_plane_rad = std::atan2(R_rel(0, 2), -R_rel(1, 2));
        shoulder_rotation_rad = std::atan2(R_rel(2, 0), R_rel(2, 1));
    } else {
        shoulder_plane_rad = 0.0;
        shoulder_rotation_rad = std::atan2(-R_rel(1, 0), R_rel(0, 0));
    }
    
    kinematics.shoulder_elevation_deg            = shoulder_elevation_rad * kRadToDeg;
    kinematics.shoulder_horizontal_abduction_deg = shoulder_plane_rad * kRadToDeg;
    kinematics.shoulder_external_rotation_deg    = shoulder_rotation_rad * kRadToDeg;
    
    return kinematics;
}

// Torso/Thorax Segment Frame Builder (ISB Standard)
SegmentFrame computeTorsoFrame(const Eigen::Vector3d& ij,
                               const Eigen::Vector3d& px,
                               const Eigen::Vector3d& c7,
                               const Eigen::Vector3d& t8) {
    SegmentFrame frame;
    frame.origin = ij;
    
    Eigen::Vector3d upper_thorax = 0.5 * (ij + c7);
    Eigen::Vector3d lower_thorax = 0.5 * (px + t8);
    
    // Z-Axis: Longitudinal axis pointing upward
    Eigen::Vector3d z_axis = (upper_thorax - lower_thorax).normalized();
    
    // Establish a temporary back-to-front anterior vector line
    Eigen::Vector3d thorax_back = 0.5 * (c7 + t8);
    Eigen::Vector3d thorax_front = 0.5 * (ij + px);
    Eigen::Vector3d temp_anterior = (thorax_front - thorax_back).normalized();
    
    // Y-Axis: Lateral axis pointing rightward
    Eigen::Vector3d cross_product = z_axis.cross(temp_anterior);
    assert(cross_product.norm() > 1e-4 && "Degenerate torso frame detected!");
    Eigen::Vector3d y_axis = cross_product.normalized();
    
    // X-Axis: True clean anterior axis pointing forward
    Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();
    
    frame.axes.col(0) = x_axis;
    frame.axes.col(1) = y_axis;
    frame.axes.col(2) = z_axis;
    
    return frame;
}

// Humerus Segment Frame Builder (Aligned with .axes and .origin structs)
SegmentFrame computeHumerusFrame(const Eigen::Vector3d& sho, const Eigen::Vector3d& melb, const Eigen::Vector3d& elb) {
    SegmentFrame frame;
    // Set explicit segment origin at the center of the elbow joint
    frame.origin = 0.5 * (melb + elb);
    
    // Longitudinal Axis (Y-axis pointing from elbow center up to shoulder)
    Eigen::Vector3d y_axis = (sho - frame.origin).normalized();
    
    // Temporary mediolateral line to establish plane orthogonalization
    Eigen::Vector3d ml_line = melb - elb;
    
    // Anteroposterior Axis (X-axis perpendicular to arm plane)
    Eigen::Vector3d x_axis = y_axis.cross(ml_line).normalized();
    
    // Clean Mediolateral Axis (Z-axis perpendicular to X and Y)
    Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
    
    // Store in unified .axes member variable
    frame.axes.col(0) = x_axis;
    frame.axes.col(1) = y_axis;
    frame.axes.col(2) = z_axis;
    
    return frame;
}

// Pure physics engine node - isolated and trivial to unit test
double computeRotationalMoment(double I_com_kgm2, double alpha_rad_s2) {
    return I_com_kgm2 * alpha_rad_s2;
}

// Anthropometric wrapper handling segment mechanics
double computeElbowVarusTorque(double vel_deg_s, double accel_deg_s2, 
                               double segment_length_meters, double body_mass_kg) {
    // 1. de Leva (1996) forearm parameters
    double m_segment = body_mass_kg * 0.0221;               // 2.21% of body mass
    double k_g = segment_length_meters * 0.32;               // Radius of gyration (32% from proximal)
    double I_com = m_segment * (k_g * k_g);                  // Segment Moment of Inertia (kg*m²)
    
    // 2. Convert kinematic acceleration from degrees to radians
    double alpha_rad = accel_deg_s2 * (M_PI / 180.0);
    
    // 3. Delegate to pure physics calculator
    double M_com = computeRotationalMoment(I_com, alpha_rad);
    
    return std::abs(M_com); 
}

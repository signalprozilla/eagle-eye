#include "kinematics.h"
#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
extern const double kRadToDeg; 

// =========================================================================
// FOREARM SEGMENT FRAME BUILDER (Aligned to ISB Standard via Y-X-Z Translation)
// =========================================================================
SegmentFrame computeForearmFrame(const Eigen::Vector3d& elbowM, 
                                 const Eigen::Vector3d& elbowL, 
                                 const Eigen::Vector3d& wristM) {
    SegmentFrame frame;
    frame.origin = 0.5 * (elbowM + elbowL);
    
    // ISB Longitudinal axis points proximally along the forearm (Wrist center to Elbow center)
    Eigen::Vector3d long_axis = (frame.origin - wristM).normalized();
    Eigen::Vector3d elbow_axis = (elbowL - elbowM).normalized(); 
    
    Eigen::Vector3d cross_product = elbow_axis.cross(long_axis);
    assert(cross_product.norm() > 1e-4 && "Degenerate frame detected: markers are collinear!");
    
    Eigen::Vector3d anterior_axis = cross_product.normalized();
    Eigen::Vector3d ortho_ml_axis = long_axis.cross(anterior_axis).normalized();
    
    // Mapping to your internal math matrix structure:
    // Internal X = Anterior Axis (ISB X)
    // Internal Y = Mediolateral Axis (ISB Z)
    // Internal Z = Longitudinal Axis (ISB Y)
    frame.axes.col(0) = anterior_axis; 
    frame.axes.col(1) = ortho_ml_axis;  
    frame.axes.col(2) = long_axis;     
    
    return frame;
}

// =========================================================================
// HUMERUS SEGMENT FRAME BUILDER (Option 1 - Wu 2005 Documented Implementation)
// =========================================================================
SegmentFrame computeHumerusFrame(const Eigen::Vector3d& sho, const Eigen::Vector3d& melb, const Eigen::Vector3d& elb) {
    SegmentFrame frame;
    frame.origin = 0.5 * (melb + elb);
    
    // Longitudinal axis pointing up from elbow center to shoulder
    Eigen::Vector3d long_axis = (sho - frame.origin).normalized();
    Eigen::Vector3d ml_line = melb - elb;
    
    Eigen::Vector3d anterior_axis = long_axis.cross(ml_line).normalized();
    Eigen::Vector3d ortho_ml_axis = anterior_axis.cross(long_axis).normalized();
    
    // Mapping to match your internal decomposition tracking:
    // Internal X = Anterior Axis (ISB X)
    // Internal Y = Mediolateral Axis (ISB Z)
    // Internal Z = Longitudinal Axis (ISB Y)
    frame.axes.col(0) = anterior_axis;
    frame.axes.col(1) = ortho_ml_axis;
    frame.axes.col(2) = long_axis;
    
    return frame;
}

// =========================================================================
// RELATIVE JOINT ANGLE KINEMATICS ENGINE
// =========================================================================
JointKinematics calculateRelativeAngles(const Eigen::Matrix3d& R_parent, const Eigen::Matrix3d& R_child) {
    JointKinematics kinematics;
    Eigen::Matrix3d R_rel = R_parent.transpose() * R_child;
    
    // Under our axis alignment transformation, your internal Y-X-Z Cardan parser
    // resolves mathematically to the exact ISB Z-X-Y kinematic outcomes.
    double clamped_elbow = std::clamp(R_rel(1, 2), -1.0, 1.0);
    double valgusRad = std::asin(-clamped_elbow); 
    double flexionRad = std::atan2(R_rel(0, 2), R_rel(2, 2));
    double rotationRad = std::atan2(R_rel(1, 0), R_rel(1, 1));
    
    kinematics.elbow_flexion_deg                = flexionRad * kRadToDeg;
    kinematics.elbow_valgus_carrying_deg        = valgusRad * kRadToDeg;
    kinematics.forearm_pronation_supination_deg = rotationRad * kRadToDeg;
    
    // Shoulder calculation block remains unaffected
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

// =========================================================================
// TORSO/THORAX SEGMENT FRAME BUILDER
// =========================================================================
SegmentFrame computeTorsoFrame(const Eigen::Vector3d& ij,
                               const Eigen::Vector3d& px,
                               const Eigen::Vector3d& c7,
                               const Eigen::Vector3d& t8) {
    SegmentFrame frame;
    frame.origin = ij;
    
    Eigen::Vector3d upper_thorax = 0.5 * (ij + c7);
    Eigen::Vector3d lower_thorax = 0.5 * (px + t8);
    
    Eigen::Vector3d z_axis = (upper_thorax - lower_thorax).normalized();
    
    Eigen::Vector3d thorax_back = 0.5 * (c7 + t8);
    Eigen::Vector3d thorax_front = 0.5 * (ij + px);
    Eigen::Vector3d temp_anterior = (thorax_front - thorax_back).normalized();
    
    Eigen::Vector3d cross_product = z_axis.cross(temp_anterior);
    assert(cross_product.norm() > 1e-4 && "Degenerate torso frame detected!");
    Eigen::Vector3d y_axis = cross_product.normalized();
    Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();
    
    frame.axes.col(0) = x_axis;
    frame.axes.col(1) = y_axis;
    frame.axes.col(2) = z_axis;
    
    return frame;
}

// =========================================================================
// PHYSICS MOMENT AND TORQUE ENGINES
// =========================================================================
double computeRotationalMoment(double I_com_kgm2, double alpha_rad_s2) {
    return I_com_kgm2 * alpha_rad_s2;
}

double computeElbowVarusTorque(double vel_deg_s, double accel_deg_s2, 
                               double segment_length_meters, double body_mass_kg) {
    double m_segment = body_mass_kg * 0.0221;               
    double k_g = segment_length_meters * 0.32;               
    double I_com = m_segment * (k_g * k_g);                  
    
    double alpha_rad = accel_deg_s2 * (M_PI / 180.0);
    double M_com = computeRotationalMoment(I_com, alpha_rad);
    
    return std::abs(M_com); 
}
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Dense>
#include "ezc3d_all.h" // The biomechanics library
#include "obp_markers.h"
#include "c3d_loader.h" 
#include "csv_writer.h"
#include "ground_truth.h"
#define _USE_MATH_DEFINES // For some compilers to enable M_PI
#include <cmath>
#include <vector>
#include "kinematics.h"
#include "filter.h"
#include "signal_processing.h"
int main(int argc, char* argv[]) {
    // ... [Keep standard header and file checking logic exactly the same] ...
    if (argc < 2) {
        std::cerr << "USAGE ERROR: Please provide a path to a C3D file!" << std::endl;
        std::cerr << "Example: ./eagle-eye data/BOS_000002_Pitch1.c3d" << std::endl;
        return 1;
    }
    std::string sample_pitch = argv[1];
    C3DLoader loader(sample_pitch);

    size_t num_frames = loader.getNumFrames();
    double fs = loader.getSampleRate();
    double dt = 1.0 / fs;
    
    // 1. TUNE CUTOFF TO BASEBALL SPECIFIC LAWS (20 Hz)
    double cutoff_hz = 20.0; 
    double athlete_body_mass_kg = 80.0; 

    std::cout << "\n=============================================" << std::endl;
    std::cout << "        DATA INPUT SPECIFICATIONS            " << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "  Mocap Sampling Rate (fs): " << fs << " Hz" << std::endl;
    std::cout << "  Total Frames Detected    : " << num_frames << std::endl;
    std::cout << "  Filter Cutoff Frequency  : " << cutoff_hz << " Hz" << std::endl;
    std::cout << "=============================================" << std::endl;

    // Extract raw markers
    std::vector<Eigen::Vector3d> raw_sho, raw_melb, raw_elb, raw_wrb;
    for (size_t f = 0; f < num_frames; ++f) {
        raw_sho.push_back(loader.getMarkerFrame("RSHO", f)); 
        raw_melb.push_back(loader.getMarkerFrame("RMELB", f));
        raw_elb.push_back(loader.getMarkerFrame("RELB", f));
        raw_wrb.push_back(loader.getMarkerFrame("RWRB", f));
    }

    // 2. POSITION-DOMAIN FILTER (RUNS FILTFILT FORWARD-BACKWARD)
    std::vector<Eigen::Vector3d> filtered_sho  = filterZeroPhaseButterworth(raw_sho, fs, cutoff_hz);
    std::vector<Eigen::Vector3d> filtered_melb = filterZeroPhaseButterworth(raw_melb, fs, cutoff_hz);
    std::vector<Eigen::Vector3d> filtered_elb  = filterZeroPhaseButterworth(raw_elb, fs, cutoff_hz);
    std::vector<Eigen::Vector3d> filtered_wrb  = filterZeroPhaseButterworth(raw_wrb, fs, cutoff_hz);

    // 3. COMPUTE KINEMATICS FROM PRISTINE MARKERS
    std::vector<double> flexion_angles;
    std::vector<double> segment_lengths;

    for (size_t f = 0; f < num_frames; ++f) {
        if (filtered_sho[f].hasNaN() || filtered_melb[f].hasNaN() || filtered_elb[f].hasNaN() || filtered_wrb[f].hasNaN()) {
            flexion_angles.push_back(0.0);
            segment_lengths.push_back(0.28);
            continue;
        }

        double length_m = (filtered_wrb[f] - filtered_elb[f]).norm();
        segment_lengths.push_back(length_m);

        SegmentFrame humerus = computeHumerusFrame(filtered_sho[f], filtered_melb[f], filtered_elb[f]);
        SegmentFrame forearm = computeForearmFrame(filtered_melb[f], filtered_elb[f], filtered_wrb[f]);
        
        JointKinematics angles = calculateRelativeAngles(humerus.rotation(), forearm.rotation());
        flexion_angles.push_back(angles.elbow_flexion_deg);
    }

    // NOTE: Old Step 4 (the secondary angular filter pass) has been completely deleted.
    // Data transitions cleanly to the central difference arrays.

    // 4. CENTRAL DIFFERENCE EVALUATION & FRAME TRACKING
    double max_vel = 0.0;
    double max_accel = 0.0;
    double max_torque = 0.0;
    
    size_t peak_vel_frame = 0;
    size_t peak_accel_frame = 0;
    size_t peak_kinetic_frame = 0;

    double debug_vel_at_kinetic_peak = 0.0;
    double debug_accel_at_kinetic_peak = 0.0;
    double debug_len_at_kinetic_peak = 0.0;

    for (size_t f = 1; f < num_frames - 1; ++f) {
        double vel = (flexion_angles[f+1] - flexion_angles[f-1]) / (2.0 * dt);
        double accel = (flexion_angles[f+1] - 2.0 * flexion_angles[f] + flexion_angles[f-1]) / (dt * dt);
        
        double current_len = segment_lengths[f];
        double torque = computeElbowVarusTorque(vel, accel, current_len, athlete_body_mass_kg);

        // Track global maximum absolute boundaries and their specific frames
        if (std::abs(vel) > std::abs(max_vel)) {
            max_vel = vel;
            peak_vel_frame = f;
        }
        if (std::abs(accel) > std::abs(max_accel)) {
            max_accel = std::abs(accel);
            peak_accel_frame = f;
        }
        if (torque > max_torque) {
            max_torque = torque;
            peak_kinetic_frame = f;
            debug_vel_at_kinetic_peak = vel;
            debug_accel_at_kinetic_peak = accel;
            debug_len_at_kinetic_peak = current_len;
        }
    }

    std::cout << "\n=============================================" << std::endl;
    std::cout << "        EAGLE EYE KINETIC EXTRAPOLATION REPORT" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "Peak Flexion Velocity : " << max_vel << " deg/s (Frame " << peak_vel_frame << ")" << std::endl;
    std::cout << "Peak Flexion Accel    : " << max_accel << " deg/s² (Frame " << peak_accel_frame << ")" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Peak Elbow Varus Moment: " << max_torque << " Nm (Frame " << peak_kinetic_frame << ")" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "Kinetic Peak Window Diagnostics (Frame " << peak_kinetic_frame << "):" << std::endl;
    std::cout << "  Instantaneous Velocity at Peak Frame    : " << debug_vel_at_kinetic_peak << " deg/s" << std::endl;
    std::cout << "  Instantaneous Acceleration at Peak Frame: " << debug_accel_at_kinetic_peak << " deg/s²" << std::endl;
    std::cout << "=============================================" << std::endl;


    return 0;
}




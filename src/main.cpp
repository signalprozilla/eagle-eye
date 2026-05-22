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




// int main() {

    // take 1:
    // Eigen::Vector3d elbow(1.0, 2.0, 3.0);
    // Eigen::Vector3d shoulder(0.0, 1.0, 5.0);

    // double dot = elbow.dot(shoulder);
    // Eigen::Vector3d cross = elbow.cross(shoulder);

    // std::cout << "Eagle Eye Math Foundation Ready!" << std::endl;
    // std::cout << "Dot Product: " << dot << std::endl;
    // std::cout << "Cross Product: " << cross.transpose() << std::endl;

    // take 2: 

    // 1. Keep the Eigen math test to ensure math foundation is still live
    // Eigen::Vector3d elbow(1.0, 2.0, 3.0);
    // std::cout << "Math Foundation: " << elbow.transpose() << " is active." << std::endl;

    // // 2. Try to open a C3D file (even if it doesn't exist yet)
    // try {
    //     // We'll point this to a real file in the next step
    //     ezc3d::c3d c3d("data/test_pitch.c3d");
    //     std::cout << "Successfully opened C3D file!" << std::endl;
        
    //     // Print how many points (markers) are in the file
    //     size_t nMarkers = c3d.header().nb3dPoints();
    //     std::cout << "Number of markers detected: " << nMarkers << std::endl;
    // } 
    // catch (const std::exception& e) {
    //     // It WILL hit this right now because you don't have a file in data/ yet
    //     std::cout << "C3D system active, but: " << e.what() << " (Expected until we add data)" << std::endl;
    // }

    // take 3: 
    // std::cout << "--- Eagle Eye Initialized ---" << std::endl;
    // std::cout << "Targeting Marker: " << obp::R_ELBOW_M << " (Medial Epicondyle)" << std::endl;

    // try {
    //     // Pointing to one of the files you just cloned to see if it opens
    //     ezc3d::c3d c3d("data/obp_repo/baseball_hitting/data/c3d/000004/marker_data.c3d");
    //     std::cout << "C3D File Linked! Markers found: " << c3d.header().nb3dPoints() << std::endl;
    // } 
    // catch (const std::exception& e) {
    //     std::cout << "C3D Status: System active, waiting for specific file path." << std::endl;
    // }

    // take 4: 
//    try {
//     std::string testFile = "/Users/chittibook/eagle-eye/data/obp_repo/baseball_pitching/data/c3d/000787/000787_002990_73_183_010_FF_814.c3d"; 
//     C3DLoader loader(testFile);
    
//     int nFrames = loader.getMarkerTimeSeries(std::string(obp::R_ELBOW_M)).size(); 
//     double sampleRate = loader.getSampleRate();
//     double dt = 1.0 / sampleRate; // Time delta between frames

//     std::vector<double> angles(nFrames, 0.0);
//     std::vector<double> velocities(nFrames - 1, 0.0);
//     std::vector<double> accelerations(nFrames - 2, 0.0);

//     // Phase 1: Compute all angles across the entire pitch
//     for (int i = 0; i < nFrames; ++i) { 
//         Eigen::Vector3d shoulder = loader.getMarkerFrame(std::string(obp::R_SHOULDER), i);
//         Eigen::Vector3d elbow    = loader.getMarkerFrame(std::string(obp::R_ELBOW_M), i);
//         Eigen::Vector3d wrist    = loader.getMarkerFrame(std::string(obp::R_WRIST_M), i);

//         Eigen::Vector3d humerus = shoulder - elbow; 
//         Eigen::Vector3d forearm = wrist - elbow;

//         double dot = humerus.dot(forearm);
//         double mags = humerus.norm() * forearm.norm();
        
//         if (mags > 0) {
//             angles[i] = std::acos(dot / mags) * (180.0 / M_PI);
//         }
//     }

//     // Phase 2: Compute Angular Velocity (deg/sec)
//     for (size_t i = 0; i < nFrames - 1; ++i) {
//         velocities[i] = (angles[i + 1] - angles[i]) / dt;
//     }

//     // Phase 3: Compute Angular Acceleration (deg/sec^2) and find the peak load
//     double maxAcceleration = 0.0;
//     int peakTorqueFrame = 0;

//     for (size_t i = 0; i < velocities.size() - 1; ++i) {
//         accelerations[i] = (velocities[i + 1] - velocities[i]) / dt;
        
//         // We look for absolute value because both violent flexion and extension exert torque
//         if (std::abs(accelerations[i]) > std::abs(maxAcceleration)) {
//             maxAcceleration = accelerations[i];
//             peakTorqueFrame = i + 1; // Aligning back to the approximate frame
//         }
//     }

//     // Phase 4: Estimate Varus Torque using a generic scaling factor for forearm inertia
//     // In full inverse dynamics, this uses segment mass. We will scale it to approximate Newton-meters (Nm)
//     double standardForearmInertia = 0.005; // kg*m^2 (approximate human forearm asset value)
//     double maxAccelerationRad = maxAcceleration * (M_PI / 180.0); // Physics requires radians
//     double estimatedPeakTorque = std::abs(maxAccelerationRad * standardForearmInertia);

//     std::cout << "\n=============================================" << std::endl;
//     std::cout << "         EAGLE EYE TORQUE ENGINE ONLINE      " << std::endl;
//     std::cout << "=============================================" << std::endl;
//     printf("Capture Frame Rate       : %.1f Hz\n", sampleRate);
//     printf("Time Step (dt)           : %.5f seconds\n", dt);
//     printf("Peak Angular Acceleration: %.2f deg/s²\n", maxAcceleration);
//     printf("Calculated Torque Window : Frame %03d\n", peakTorqueFrame);
//     printf("Estimated Joint Stress   : %.2f Nm\n", estimatedPeakTorque);
//     std::cout << "=============================================" << std::endl;

// } catch (const std::exception& e) {
//     std::cerr << "Error: " << e.what() << std::endl;
// }

//// separation line ___________________________________________________-everything above is isolated

// try {
//     std::string testFile = "/Users/chittibook/eagle-eye/data/obp_repo/baseball_pitching/data/c3d/000787/000787_002990_73_183_010_FF_814.c3d"; 
//     ezc3d::c3d c3dFile(testFile);
    
//     // Grab the exact strings stored inside the file's header metadata
//     const auto& labels = c3dFile.parameters().group("POINT").parameter("LABELS").valuesAsString();
    
//     std::cout << "--- Available Markers in this C3D File ---" << std::endl;
//     for (const auto& label : labels) {
//         // Let's filter for anything starting with 'R' to narrow down right-arm markers
//         if (!label.empty() && label[0] == 'R') {
//             std::cout << "  " << label << std::endl;
//         }
//     }

// } catch (const std::exception& e) {
//     std::cerr << "Error: " << e.what() << std::endl;
// }
//     return 0;
// }


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
// #include <iostream>
// #include <vector>
// #include "c3d_loader.h"
// #include "kinematics.h"

// int main(int argc, char* argv[]) {
//     std::cout << "=== Running Kinematic Sanity Check on Live Pitch ===" << std::endl;

//     // Default path if no argument is passed, but fallback to relative parent root
//     std::string sample_pitch = "../data/sample_pitch.c3d"; 
    
//     // If you pass a path in the terminal, use that instead!
//     if (argc > 1) {
//         sample_pitch = argv[1];
//     }

//     std::cout << "[*] Attempting to open file at path: " << sample_pitch << std::endl;
    
//     // 1. Initialize your custom Phase 0 Loader by passing the file path immediately
//     C3DLoader loader(sample_pitch);
    
//     // Check total frames using your native method
//     size_t num_frames = loader.getNumFrames(); 
//     std::cout << "[+] Loaded " << num_frames << " frames from trial." << std::endl;

//     double max_flexion = -999.0;
//     double min_flexion = 999.0;
//     size_t max_frame_idx = 0;

//     // Assume an identity matrix for the humerus parent for this initial baseline
//     Eigen::Matrix3d R_humerus_placeholder = Eigen::Matrix3d::Identity();

//     // 2. Stream through time-series frames
//     for (size_t f = 0; f < num_frames; ++f) {
//         // Fetch raw 3D point vectors using your exact public signatures: (name, frame)
//         Eigen::Vector3d r_melb = loader.getMarkerFrame("RMELB", f);
//         Eigen::Vector3d r_elb  = loader.getMarkerFrame("RELB", f);
//         Eigen::Vector3d r_wrb  = loader.getMarkerFrame("RWRB", f);

//         // Skip any invalid/dropped frames where tracking was temporarily obscured
//         if (r_melb.hasNaN() || r_elb.hasNaN() || r_wrb.hasNaN()) {
//             continue;
//         }

//         // 3. Build the local Segment Frame
//         SegmentFrame forearm = computeForearmFrame(r_melb, r_elb, r_wrb);

//         // 4. Extract relative clinical joint angles
//         JointKinematics angles = calculateRelativeAngles(R_humerus_placeholder, forearm.rotation());

//         double current_flexion = angles.elbow_flexion_deg;
        
//         if (current_flexion > max_flexion) {
//             max_flexion = current_flexion;
//             max_frame_idx = f;
//         }
//         if (current_flexion < min_flexion) {
//             min_flexion = current_flexion;
//         }

//         // Print a strategic diagnostic block every 20 frames
//         if (f % 20 == 0) {
//             std::cout << "Frame " << f << " -> Elbow Flexion: " << current_flexion << "°" << std::endl;
//         }
//     }

//     std::cout << "\n=============================================" << std::endl;
//     std::cout << "[+] SANITY METRIC RESULTS:" << std::endl;
//     std::cout << "    Minimum Extracted Flexion: " << min_flexion << "°" << std::endl;
//     std::cout << "    Maximum Extracted Flexion: " << max_flexion << "° at Frame " << max_frame_idx << std::endl;
//     std::cout << "=============================================" << std::endl;

//     if (max_flexion > 160.0 || max_flexion < 45.0 || std::isnan(max_flexion)) {
//         std::cout << "[-] WARNING: Flexion peaks are out of normal bounds. Verify marker mapping signs!" << std::endl;
//     } else {
//         std::cout << "[+] SUCCESS: Flexion curve falls into standard clinical constraints." << std::endl;
//     }

//     return 0;
// }
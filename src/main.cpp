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

int main() {

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
   try {
    std::string testFile = "/Users/chittibook/eagle-eye/data/obp_repo/baseball_pitching/data/c3d/000787/000787_002990_73_183_010_FF_814.c3d"; 
    C3DLoader loader(testFile);
    
    int nFrames = loader.getMarkerTimeSeries(std::string(obp::R_ELBOW_M)).size(); 
    double sampleRate = loader.getSampleRate();
    double dt = 1.0 / sampleRate; // Time delta between frames

    std::vector<double> angles(nFrames, 0.0);
    std::vector<double> velocities(nFrames - 1, 0.0);
    std::vector<double> accelerations(nFrames - 2, 0.0);

    // Phase 1: Compute all angles across the entire pitch
    for (int i = 0; i < nFrames; ++i) { 
        Eigen::Vector3d shoulder = loader.getMarkerFrame(std::string(obp::R_SHOULDER), i);
        Eigen::Vector3d elbow    = loader.getMarkerFrame(std::string(obp::R_ELBOW_M), i);
        Eigen::Vector3d wrist    = loader.getMarkerFrame(std::string(obp::R_WRIST_M), i);

        Eigen::Vector3d humerus = shoulder - elbow; 
        Eigen::Vector3d forearm = wrist - elbow;

        double dot = humerus.dot(forearm);
        double mags = humerus.norm() * forearm.norm();
        
        if (mags > 0) {
            angles[i] = std::acos(dot / mags) * (180.0 / M_PI);
        }
    }

    // Phase 2: Compute Angular Velocity (deg/sec)
    for (size_t i = 0; i < nFrames - 1; ++i) {
        velocities[i] = (angles[i + 1] - angles[i]) / dt;
    }

    // Phase 3: Compute Angular Acceleration (deg/sec^2) and find the peak load
    double maxAcceleration = 0.0;
    int peakTorqueFrame = 0;

    for (size_t i = 0; i < velocities.size() - 1; ++i) {
        accelerations[i] = (velocities[i + 1] - velocities[i]) / dt;
        
        // We look for absolute value because both violent flexion and extension exert torque
        if (std::abs(accelerations[i]) > std::abs(maxAcceleration)) {
            maxAcceleration = accelerations[i];
            peakTorqueFrame = i + 1; // Aligning back to the approximate frame
        }
    }

    // Phase 4: Estimate Varus Torque using a generic scaling factor for forearm inertia
    // In full inverse dynamics, this uses segment mass. We will scale it to approximate Newton-meters (Nm)
    double standardForearmInertia = 0.005; // kg*m^2 (approximate human forearm asset value)
    double maxAccelerationRad = maxAcceleration * (M_PI / 180.0); // Physics requires radians
    double estimatedPeakTorque = std::abs(maxAccelerationRad * standardForearmInertia);

    std::cout << "\n=============================================" << std::endl;
    std::cout << "         EAGLE EYE TORQUE ENGINE ONLINE      " << std::endl;
    std::cout << "=============================================" << std::endl;
    printf("Capture Frame Rate       : %.1f Hz\n", sampleRate);
    printf("Time Step (dt)           : %.5f seconds\n", dt);
    printf("Peak Angular Acceleration: %.2f deg/s²\n", maxAcceleration);
    printf("Calculated Torque Window : Frame %03d\n", peakTorqueFrame);
    printf("Estimated Joint Stress   : %.2f Nm\n", estimatedPeakTorque);
    std::cout << "=============================================" << std::endl;

} catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
}
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
    return 0;
}


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
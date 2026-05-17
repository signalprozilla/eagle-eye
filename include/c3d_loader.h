#ifndef C3D_LOADER_H
#define C3D_LOADER_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include "ezc3d_all.h"

class C3DLoader {
public:
    // Constructor opens the file
    C3DLoader(const std::string& filePath);

    // Returns a vector of 3D points (one per frame) for a specific marker
    std::vector<Eigen::Vector3d> getMarkerTimeSeries(const std::string& markerName);
    Eigen::Vector3d getMarkerFrame(const std::string& markerName, size_t frameIdx);
    // Meta-data helpers
    double getSampleRate() const;
    size_t getNumFrames() const;

private:
    ezc3d::c3d m_c3d; // The internal library object
};

#endif
#include "c3d_loader.h"
#include <stdexcept>


C3DLoader::C3DLoader(const std::string& filePath) : m_c3d(filePath) {
    // ezc3d automatically throws an error if the file doesn't exist
}

std::vector<Eigen::Vector3d> C3DLoader::getMarkerTimeSeries(const std::string& markerName) {
    std::vector<Eigen::Vector3d> series;
    
    // 1. Find the index of the marker name
    int markerIdx = -1;
    const auto& labels = m_c3d.parameters().group("POINT").parameter("LABELS").valuesAsString();
    
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == markerName) {
            markerIdx = i;
            break;
        }
    }

    if (markerIdx == -1) {
        throw std::runtime_error("Marker not found: " + markerName);
    }

    // 2. Loop through frames and extract (x, y, z)
    size_t nFrames = m_c3d.header().nb3dPoints(); // Total 3D data points
    for (size_t f = 0; f < m_c3d.data().nbFrames(); ++f) {
        const auto& p = m_c3d.data().frame(f).points().point(markerIdx);
        series.push_back(Eigen::Vector3d(p.x(), p.y(), p.z()));
    }

    return series;
}

double C3DLoader::getSampleRate() const {
    return m_c3d.header().frameRate();
}

// Eigen::Vector3d C3DLoader::getMarkerFrame(const std::string& markerName, size_t frameIdx) {
//     // Assuming m_c3d is your ezc3d object
//     const auto& point = m_c3d.data().point(markerName);
//     const auto& frame = point.frame(frameIdx);
//     return Eigen::Vector3d(frame.x(), frame.y(), frame.z());
// }

Eigen::Vector3d C3DLoader::getMarkerFrame(const std::string& markerName, size_t frameIdx) {
    // 1. Find the index of the marker name using the exact same logic you wrote above
    int markerIdx = -1;
    const auto& labels = m_c3d.parameters().group("POINT").parameter("LABELS").valuesAsString();
    
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == markerName) {
            markerIdx = i;
            break;
        }
    }

    if (markerIdx == -1) {
        throw std::runtime_error("Marker not found: " + markerName);
    }

    // 2. Bounds check for the requested frame index
    if (frameIdx >= m_c3d.data().nbFrames()) {
        throw std::runtime_error("Requested frame index out of bounds.");
    }

    // 3. Extract using ezc3d's true data pattern: data -> frame -> points -> point
    const auto& p = m_c3d.data().frame(frameIdx).points().point(markerIdx);
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}
size_t C3DLoader::getNumFrames() const {
    return m_c3d.data().nbFrames();
}
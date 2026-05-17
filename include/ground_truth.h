#ifndef GROUND_TRUTH_H
#define GROUND_TRUTH_H

#include <string>
#include <unordered_map>

struct PitcherGroundTruth {
    std::string sessionPitchId;
    double peakElbowVarusMoment; // The "Gold Standard" we'll compare against
    double peakShoulderIRTorque; 
};

// A simple manager to load and store these values
class GroundTruthManager {
public:
    void loadFromCSV(const std::string& filePath);
    PitcherGroundTruth getForPitch(const std::string& pitchId) const;

private:
    std::unordered_map<std::string, PitcherGroundTruth> m_data;
};

#endif
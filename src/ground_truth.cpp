#include "ground_truth.h"
#include <fstream>
#include <sstream>
#include <iostream>

void GroundTruthManager::loadFromCSV(const std::string& filePath) {
    std::ifstream file(filePath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Oracle Error: Cannot open " << filePath << std::endl;
        return;
    }

    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }

        if (row.size() > 10) {
            PitcherGroundTruth gt;
            gt.sessionPitchId = row[0];
            // Adjust indices based on the OBP data dictionary
            gt.peakElbowVarusMoment = std::stod(row[5]); 
            gt.peakShoulderIRTorque = std::stod(row[8]);
            
            m_data[gt.sessionPitchId] = gt;
        }
    }
    std::cout << "Oracle Active: Loaded " << m_data.size() << " ground truth records." << std::endl;
}

PitcherGroundTruth GroundTruthManager::getForPitch(const std::string& pitchId) const {
    auto it = m_data.find(pitchId);
    if (it != m_data.end()) {
        return it->second;
    }
    // Return an empty/default struct if not found
    return PitcherGroundTruth(); 
}
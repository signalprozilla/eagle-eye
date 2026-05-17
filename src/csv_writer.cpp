#include "csv_writer.h"
#include <fstream>
#include <iostream>

void CSVWriter::writeMarkerData(const std::string& filename, 
                               const std::string& markerName, 
                               const std::vector<Eigen::Vector3d>& data) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Could not open CSV file for writing: " + filename);
    }

    // Header row
    file << "frame,x,y,z\n";

    // Data rows
    for (size_t i = 0; i < data.size(); ++i) {
        file << i << "," 
             << data[i].x() << "," 
             << data[i].y() << "," 
             << data[i].z() << "\n";
    }

    file.close();
    std::cout << "Successfully exported " << markerName << " data to " << filename << std::endl;
}
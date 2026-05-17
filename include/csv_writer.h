#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <string>
#include <vector>
#include <Eigen/Dense>

class CSVWriter {
public:
    static void writeMarkerData(const std::string& filename, 
                               const std::string& markerName, 
                               const std::vector<Eigen::Vector3d>& data);
};

#endif
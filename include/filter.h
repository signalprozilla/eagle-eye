#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <Eigen/Dense>

struct FilterCoefficients {
    double b0 = 0.0, b1 = 0.0, b2 = 0.0;
    double a1 = 0.0, a2 = 0.0;
};

// Generates 2nd-order Butterworth low-pass coefficients
FilterCoefficients computeButterworthCoefficients(double sampleRate, double cutoffFrequency);

// Filters an entire time-series vector of 3D positions using a zero-phase bidirectional pass
std::vector<Eigen::Vector3d> filterSignalDualPass(const std::vector<Eigen::Vector3d>& signal, 
                                                  const FilterCoefficients& coeffs);

#endif // FILTER_H
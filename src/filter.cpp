#include "filter.h"
#include <cmath>
#include <algorithm>

FilterCoefficients computeButterworthCoefficients(double sampleRate, double cutoffFrequency) {
    FilterCoefficients coeffs;
    
    // Constant Pi value
    const double pi = 3.14159265358979323846;
    
    // Frequency pre-warping parameter calculation
    double k = std::tan(pi * cutoffFrequency / sampleRate);
    double k2 = k * k;
    double sqrt2 = std::sqrt(2.0);
    
    double gamma = 1.0 + sqrt2 * k + k2;
    
    coeffs.b0 = k2 / gamma;
    coeffs.b1 = 2.0 * k2 / gamma;
    coeffs.b2 = coeffs.b0; // Symmetric for Butterworth designs
    
    coeffs.a1 = 2.0 * (k2 - 1.0) / gamma;
    coeffs.a2 = (1.0 - sqrt2 * k + k2) / gamma;
    
    return coeffs;
}

std::vector<Eigen::Vector3d> filterSignalDualPass(const std::vector<Eigen::Vector3d>& signal, 
                                                  const FilterCoefficients& coeffs) {
    size_t n = signal.size();
    if (n < 5) return signal; // Return raw data if too short to filter reasonably
    
    std::vector<Eigen::Vector3d> forward_pass(n, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> backward_pass(n, Eigen::Vector3d::Zero());
    
    // -------------------------------------------------------------------------
    // 1. FORWARD PASS
    // -------------------------------------------------------------------------
    // Initialize filter state by clamping to the initial frame position
    forward_pass[0] = signal[0];
    forward_pass[1] = signal[1];
    
    for (size_t i = 2; i < n; ++i) {
        forward_pass[i] = coeffs.b0 * signal[i] + 
                          coeffs.b1 * signal[i-1] + 
                          coeffs.b2 * signal[i-2] - 
                          coeffs.a1 * forward_pass[i-1] - 
                          coeffs.a2 * forward_pass[i-2];
    }
    
    // -------------------------------------------------------------------------
    // 2. BACKWARD PASS (Eliminates Phase Lag entirely)
    // -------------------------------------------------------------------------
    // Initialize backward pass state starting from the final frame position
    backward_pass[n-1] = forward_pass[n-1];
    backward_pass[n-2] = forward_pass[n-2];
    
    for (int i = static_cast<int>(n) - 3; i >= 0; --i) {
        backward_pass[i] = coeffs.b0 * forward_pass[i] + 
                           coeffs.b1 * forward_pass[i+1] + 
                           coeffs.b2 * forward_pass[i+2] - 
                           coeffs.a1 * backward_pass[i+1] - 
                           coeffs.a2 * backward_pass[i+2];
    }
    
    return backward_pass;
}
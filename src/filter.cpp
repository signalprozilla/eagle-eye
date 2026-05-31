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

// Helper function to run a single, stable forward pass of the difference equation
std::vector<Eigen::Vector3d> runForwardPass(const std::vector<Eigen::Vector3d>& input, 
                                            const FilterCoefficients& coeffs) {
    size_t n = input.size();
    std::vector<Eigen::Vector3d> output(n, Eigen::Vector3d::Zero());
    
    if (n < 3) return input;
    
    // Clamp initial states to prevent transient edge spikes
    output[0] = input[0];
    output[1] = input[1];
    
    for (size_t i = 2; i < n; ++i) {
        output[i] = coeffs.b0 * input[i] + 
                    coeffs.b1 * input[i-1] + 
                    coeffs.b2 * input[i-2] - 
                    coeffs.a1 * output[i-1] - 
                    coeffs.a2 * output[i-2];
    }
    return output;
}

std::vector<Eigen::Vector3d> filterSignalDualPass(const std::vector<Eigen::Vector3d>& signal, 
                                                  const FilterCoefficients& coeffs) {
    size_t n = signal.size();
    if (n < 5) return signal; // Return raw data if too short to filter reasonably
    
    // 1. Pass Forward
    std::vector<Eigen::Vector3d> forward_pass = runForwardPass(signal, coeffs);
    
    // 2. Reverse the intermediate stream
    std::reverse(forward_pass.begin(), forward_pass.end());
    
    // 3. Pass Forward again over the inverted time sequence
    std::vector<Eigen::Vector3d> backward_pass = runForwardPass(forward_pass, coeffs);
    
    // 4. Reverse back to restore native chronological timeline
    std::reverse(backward_pass.begin(), backward_pass.end());
    
    return backward_pass;
}
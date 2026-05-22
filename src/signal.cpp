#include "signal_processing.h" 
#include <cmath>
#include <vector>

std::vector<Eigen::Vector3d> filterZeroPhaseButterworth(const std::vector<Eigen::Vector3d>& raw_signal, double fs, double fc) {
    size_t N = raw_signal.size();
    if (N < 5) return raw_signal; // Defensive check for empty/short sequences

    // 1. Calculate warped Butterworth coefficients
    double K = std::tan(M_PI * fc / fs);
    double norm = 1.0 / (1.0 + std::sqrt(2.0) * K + K * K);
    
    double b0 = K * K * norm;
    double b1 = 2.0 * b0;
    double b2 = b0;
    double a1 = 2.0 * (K * K - 1.0) * norm;
    double a2 = (1.0 - std::sqrt(2.0) * K + K * K) * norm;

    // Temporary allocation for forward pass
    std::vector<Eigen::Vector3d> forward_pass(N, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> clean_signal(N, Eigen::Vector3d::Zero());

    // 2. Forward Pass (Introduces phase lag)
    // Pad boundary conditions using the initial frame to prevent edge transients
    forward_pass[0] = raw_signal[0];
    forward_pass[1] = raw_signal[1];
    for (size_t i = 2; i < N; ++i) {
        forward_pass[i] = b0 * raw_signal[i]     + b1 * raw_signal[i-1] + b2 * raw_signal[i-2]
                        - a1 * forward_pass[i-1] - a2 * forward_pass[i-2];
    }

    // 3. Backward Pass (Cancels phase lag completely)
    clean_signal[N-1] = forward_pass[N-1];
    clean_signal[N-2] = forward_pass[N-2];
    for (int i = static_cast<int>(N) - 3; i >= 0; --i) {
        clean_signal[i] = b0 * forward_pass[i]   + b1 * forward_pass[i+1] + b2 * forward_pass[i+2]
                        - a1 * clean_signal[i+1] - a2 * clean_signal[i+2];
    }

    return clean_signal;
}
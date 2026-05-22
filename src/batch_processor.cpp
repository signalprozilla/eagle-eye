#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <map>
#include <chrono>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <Eigen/Dense>

#include "c3d_loader.h"
#include "kinematics.h"
#include "anatomy.h"

// Note: If computeElbowVarusTorque lives in a header like kinetics.h, 
// make sure it matches the signature called below.

namespace fs = std::filesystem;

// Helper to parse the OBP filename into the CSV join key
std::string deriveSessionPitchKey(const std::string& filename) {
    std::vector<std::string> tokens;
    std::string temp = "";
    for (char c : filename) {
        if (c == '_') {
            if (!temp.empty()) tokens.push_back(temp);
            temp = "";
        } else if (c == '.') {
            if (!temp.empty()) tokens.push_back(temp);
            break;
        } else {
            temp += c;
        }
    }
    if (!temp.empty() && temp != "c3d") tokens.push_back(temp);

    if (tokens.size() < 5) return "";

    std::string session_str = tokens[1];
    session_str.erase(0, std::min(session_str.find_first_not_of('0'), session_str.size() - 1));

    std::string trial_str = tokens[4];
    trial_str.erase(0, std::min(trial_str.find_first_not_of('0'), trial_str.size() - 1));

    return session_str + "_" + trial_str;
}

// Simple CSV parser for ground truth moments
std::map<std::string, double> loadGroundTruthMoments(const std::string& csv_path) {
    std::map<std::string, double> ground_truth;
    std::ifstream file(csv_path);
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "[!] CRITICAL: Could not open CSV file at path: " << csv_path << std::endl;
        return ground_truth;
    }
    
    if (!std::getline(file, line)) return ground_truth;
    std::stringstream header_ss(line);
    std::string header_item;
    int session_pitch_col = -1;
    int varus_moment_col = -1;
    int current_col = 0;

    while (std::getline(header_ss, header_item, ',')) {
        header_item.erase(std::remove(header_item.begin(), header_item.end(), '\r'), header_item.end());
        header_item.erase(std::remove(header_item.begin(), header_item.end(), '\n'), header_item.end());

        if (header_item == "session_pitch") session_pitch_col = current_col;
        else if (header_item == "elbow_varus_moment") varus_moment_col = current_col;
        current_col++;
    }

    if (session_pitch_col == -1 || varus_moment_col == -1) {
        std::cerr << "[!] CRITICAL CSV ERROR: Required headers missing!" << std::endl;
        return ground_truth;
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        std::string session_pitch = "";
        double moment = -1.0;
        int col_idx = 0;

        while (std::getline(ss, item, ',')) {
            if (col_idx == session_pitch_col) session_pitch = item;
            if (col_idx == varus_moment_col) {
                try { moment = std::stod(item); } catch (...) { moment = -1.0; }
            }
            col_idx++;
        }
        if (!session_pitch.empty() && moment >= 0.0) {
            ground_truth[session_pitch] = moment;
        }
    }
    return ground_truth;
}

int main() {
    std::string c3d_dir = "/Users/chittibook/eagle-eye/data/obp_repo/baseball_pitching/data/c3d/";
    std::string csv_path = "/Users/chittibook/eagle-eye/data/obp_repo/baseball_pitching/data/poi/poi_metrics.csv";
    std::string output_path = "outputs/mocap_predictions.csv";

    auto start_time = std::chrono::high_resolution_clock::now();

    std::map<std::string, double> obp_truths = loadGroundTruthMoments(csv_path);
    fs::create_directories("outputs");
    std::ofstream out_file(output_path);
    out_file << "pitcher_id,session_pitch,your_peak_varus,obp_peak_varus,absolute_pct_error,pct_gap\n";

    std::cout << "[*] Starting Serial Batch Engine across OBP Repository..." << std::endl;
    size_t processed_count = 0;
    std::vector<double> absolute_errors;
    std::vector<double> raw_gaps;

    for (const auto& entry : fs::recursive_directory_iterator(c3d_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".c3d" && 
            entry.path().string().find("model") == std::string::npos) {
            
            std::string filename = entry.path().filename().string();
            std::string session_pitch_key = deriveSessionPitchKey(filename);
            
            if (obp_truths.find(session_pitch_key) == obp_truths.end()) continue;
            double obp_moment = obp_truths[session_pitch_key];

            try {
                C3DLoader loader(entry.path().string());
                size_t num_frames = loader.getNumFrames();
                double fs_rate = loader.getSampleRate();
                double dt = 1.0 / fs_rate;

                // DYNAMIC ANTHROPOMETRIC PARSING
                double body_mass = 85.0; 
                std::vector<std::string> path_tokens;
                std::stringstream ss_path(filename);
                std::string token;
                
                while (std::getline(ss_path, token, '_')) {
                    path_tokens.push_back(token);
                }

                if (path_tokens.size() >= 4) {
                    try {
                        double weight_lbs = std::stod(path_tokens[3]);
                        body_mass = weight_lbs * 0.45359237; 
                    } catch (...) {
                        body_mass = 85.0;
                    }
                }

                std::vector<double> elbow_flexion_series(num_frames, 0.0);
                double total_forearm_len_accum = 0.0;

                // 4. Frame-by-Frame Kinematic Array Generation
                for (size_t f = 0; f < num_frames; ++f) {
                    Eigen::Vector3d sho  = loader.getMarkerFrame("RSHO", f);
                    Eigen::Vector3d melb = loader.getMarkerFrame("RMELB", f);
                    Eigen::Vector3d elb  = loader.getMarkerFrame("RELB", f);
                    Eigen::Vector3d rwra = loader.getMarkerFrame("RWRA", f);
                    Eigen::Vector3d rwrb = loader.getMarkerFrame("RWRB", f);
                    
                    Eigen::Vector3d wrisM = (rwra + rwrb) / 2.0;
                    Eigen::Vector3d elbow_center = (melb + elb) / 2.0;
                    total_forearm_len_accum += (wrisM - elbow_center).norm();

                    SegmentFrame humerus = computeHumerusFrame(sho, melb, elb);
                    SegmentFrame forearm = computeForearmFrame(melb, elb, wrisM);

                    JointKinematics jk = calculateRelativeAngles(humerus.rotation(), forearm.rotation());
                    elbow_flexion_series[f] = jk.elbow_flexion_deg;
                }

                double forearm_length = (total_forearm_len_accum / num_frames);
                if (forearm_length > 5.0) {
                    forearm_length /= 1000.0; 
                }

                std::vector<double>& processed_angles = elbow_flexion_series;
                double max_torque = 0.0;

                // 5. Kinetic central difference tracking loop
                for (size_t f = 2; f < processed_angles.size() - 2; ++f) {
                    double vel_deg_s = (processed_angles[f+1] - processed_angles[f-1]) / (2.0 * dt);
                    double accel_deg_s2 = (processed_angles[f+1] - 2.0 * processed_angles[f] + processed_angles[f-1]) / (dt * dt);

                    double current_torque = computeElbowVarusTorque(vel_deg_s, accel_deg_s2, forearm_length, body_mass);
                    if (current_torque > max_torque) {
                        max_torque = current_torque;
                    }
                }

                double your_moment = max_torque;
                double pct_gap = (obp_moment - your_moment) / obp_moment;
                double abs_pct_err = std::abs(pct_gap) * 100.0;
                
                std::string pitcher_id = filename.substr(0, 6);
                out_file << pitcher_id << "," << session_pitch_key << ","
                         << your_moment << "," << obp_moment << ","
                         << abs_pct_err << "," << (pct_gap * 100.0) << "\n";

                absolute_errors.push_back(abs_pct_err);
                raw_gaps.push_back(pct_gap * 100.0);
                processed_count++;

            } catch (const std::exception& e) {
                std::cerr << "[!] Warning on file " << filename << ": " << e.what() << std::endl;
                continue;
            } catch (...) {
                continue;
            }
        }
    }

    out_file.close();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;

    if (processed_count == 0) {
        std::cerr << "[!] Error: Zero pitches processed. Verify data directory structural layout!" << std::endl;
        return 1;
    }

    std::sort(absolute_errors.begin(), absolute_errors.end());
    double median_mape = absolute_errors[absolute_errors.size() / 2];

    std::cout << "\n=============================================" << std::endl;
    std::cout << "          BATCH RECONCILIATION SUMMARY       " << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "  Total Pitchers Successfully Reconciled: " << processed_count << std::endl;
    std::cout << "  Execution Runtime Time                : " << duration.count() << " seconds!" << std::endl;
    std::cout << "  Median Absolute Percentage Error      : " << median_mape << "%" << std::endl;
    std::cout << "=============================================" << std::endl;

    return 0;
}
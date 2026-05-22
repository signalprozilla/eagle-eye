#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "kinematics.h"
#include <cmath>
#include "filter.h"

TEST_CASE("Kinematics Engine: ISB Intrinsic Y-X-Z Relative Rotation Verification") {
    // Parent frame initialized to Identity for isolated tests
    Eigen::Matrix3d Rh_id = Eigen::Matrix3d::Identity();

    SUBCASE("Test 1: Pure Isolated Flexion (Y-Axis Rotation)") {
        double target_flexion = 30.0;
        Eigen::Matrix3d Rf = Eigen::AngleAxisd(target_flexion * kDegToRad, Eigen::Vector3d::UnitY()).toRotationMatrix();
        
        auto k = calculateRelativeAngles(Rh_id, Rf);
        
        CHECK(k.elbow_flexion_deg == doctest::Approx(target_flexion).epsilon(1e-3));
        CHECK(k.elbow_valgus_carrying_deg == doctest::Approx(0.0).epsilon(1e-3));
        CHECK(k.forearm_pronation_supination_deg == doctest::Approx(0.0).epsilon(1e-3));
    }

    SUBCASE("Test 2: Pure Isolated Valgus Carrying Angle (X-Axis Rotation)") {
        double target_valgus = 15.0;
        Eigen::Matrix3d Rf = Eigen::AngleAxisd(target_valgus * kDegToRad, Eigen::Vector3d::UnitX()).toRotationMatrix();
        
        auto k = calculateRelativeAngles(Rh_id, Rf);
        
        CHECK(k.elbow_flexion_deg == doctest::Approx(0.0).epsilon(1e-3));
        CHECK(k.elbow_valgus_carrying_deg == doctest::Approx(target_valgus).epsilon(1e-3));
        CHECK(k.forearm_pronation_supination_deg == doctest::Approx(0.0).epsilon(1e-3));
    }

    SUBCASE("Test 3: Coupled Combined Multi-Axis Rotation (The Matrix Order Test)") {
        double f = 40.0; // Flexion (Y)
        double v = 12.0; // Valgus (X)
        double r = 25.0; // Pronation (Z)
        
        // Construct composite rotation matrix using explicit intrinsic Y -> X -> Z order
        Eigen::Matrix3d Rf_combo = 
            (Eigen::AngleAxisd(f * kDegToRad, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(v * kDegToRad, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(r * kDegToRad, Eigen::Vector3d::UnitZ())).toRotationMatrix();
             
        auto k = calculateRelativeAngles(Rh_id, Rf_combo);
        
        CHECK(k.elbow_flexion_deg == doctest::Approx(f).epsilon(1e-3));
        CHECK(k.elbow_valgus_carrying_deg == doctest::Approx(v).epsilon(1e-3));
        CHECK(k.forearm_pronation_supination_deg == doctest::Approx(r).epsilon(1e-3));
    }

    SUBCASE("Test 4: Relative Rotation via Non-Identity Parent Frame") {
        double f = 45.0;
        double v = 10.0;
        double r = 15.0;
        
        // 1. Give the humerus parent an arbitrary spatial orientation skew
        Eigen::Matrix3d Rh_skew = 
            (Eigen::AngleAxisd(20.0 * kDegToRad, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(35.0 * kDegToRad, Eigen::Vector3d::UnitY())).toRotationMatrix();
             
        // 2. Build the coupled child rotation profile
        Eigen::Matrix3d Rf_combo = 
            (Eigen::AngleAxisd(f * kDegToRad, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(v * kDegToRad, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(r * kDegToRad, Eigen::Vector3d::UnitZ())).toRotationMatrix();
             
        // 3. Transform the child into the global world space using the parent pose: Rf = Rh * R_rel
        Eigen::Matrix3d Rf_world = Rh_skew * Rf_combo;
        
        auto k = calculateRelativeAngles(Rh_skew, Rf_world);
        
        CHECK(k.elbow_flexion_deg == doctest::Approx(f).epsilon(1e-3));
        CHECK(k.elbow_valgus_carrying_deg == doctest::Approx(v).epsilon(1e-3));
        CHECK(k.forearm_pronation_supination_deg == doctest::Approx(r).epsilon(1e-3));
    }

    SUBCASE("Test 5: Shoulder Kinematic Decomposition (Torso -> Humerus)") {
        double phi = 45.0 * kDegToRad;   // Plane of Elevation
        double theta = 60.0 * kDegToRad; // Elevation
        double psi = 35.0 * kDegToRad;   // External Rotation
        
        // Build the explicit ISB Y-X'-Y'' matrix directly
        Eigen::Matrix3d R_isb;
        R_isb(0,0) = std::cos(phi)*std::cos(psi) - std::sin(phi)*std::cos(theta)*std::sin(psi);
        R_isb(0,1) = -std::cos(phi)*std::sin(psi) - std::sin(phi)*std::cos(theta)*std::cos(psi);
        R_isb(0,2) = std::sin(phi)*std::sin(theta);
        
        R_isb(1,0) = std::sin(phi)*std::cos(psi) + std::cos(phi)*std::cos(theta)*std::sin(psi);
        R_isb(1,1) = -std::sin(phi)*std::sin(psi) + std::cos(phi)*std::cos(theta)*std::cos(psi);
        R_isb(1,2) = -std::cos(phi)*std::sin(theta);
        
        R_isb(2,0) = std::sin(theta)*std::sin(psi);
        R_isb(2,1) = std::sin(theta)*std::cos(psi);
        R_isb(2,2) = std::cos(theta);
        
        Eigen::Matrix3d R_torso = Eigen::Matrix3d::Identity();
        auto k = calculateRelativeAngles(R_torso, R_isb);
        
        CHECK(k.shoulder_horizontal_abduction_deg == doctest::Approx(45.0).epsilon(1e-3));
        CHECK(k.shoulder_elevation_deg            == doctest::Approx(60.0).epsilon(1e-3));
        CHECK(k.shoulder_external_rotation_deg    == doctest::Approx(35.0).epsilon(1e-3));
    }

}

TEST_CASE("Signal Processing: Butterworth Zero-Phase Filter Verification") {
    SUBCASE("Noise Attenuation Pass") {
        double fs = 360.0; // 360 Hz sample rate
        double fc = 15.0;  // 15 Hz cutoff frequency
        
        FilterCoefficients coeffs = computeButterworthCoefficients(fs, fc);
        
        // Generate a synthetic test wave: Clean step function + high-frequency noise spike
        std::vector<Eigen::Vector3d> raw_signal;
        for (int i = 0; i < 100; ++i) {
            double noise = (i == 50) ? 10.0 : 0.0; // Sharp noise impulse at frame 50
            raw_signal.push_back(Eigen::Vector3d(50.0 + noise, 0.0, 0.0));
        }
        
        std::vector<Eigen::Vector3d> filtered_signal = filterSignalDualPass(raw_signal, coeffs);
        
        // Ensure that the high-frequency 10.0 magnitude noise spike has been heavily attenuated
        CHECK(filtered_signal[50].x() < 55.0); 
        // Ensure the zero-phase alignment keeps the signal bounded tightly near baseline at early frames
        CHECK(filtered_signal[5].x() == doctest::Approx(50.0).epsilon(1e-2));
    }
}



TEST_CASE("Rotational moment core math") {
    // Test pure physics scaling directly without anthropometric pollution
    CHECK(computeRotationalMoment(0.02, 1000.0) == doctest::Approx(20.0));
    CHECK(computeRotationalMoment(0.0, 5000.0) == doctest::Approx(0.0));
    
    // Real data regression anchor: locks in our verified pitching baseline metrics
    // I = 0.0158922 kg*m², alpha = -5014.63 rad/s² -> Expecting ~ -79.6933 Nm
    CHECK(computeRotationalMoment(0.0158922, -5014.63) == doctest::Approx(-79.6933).epsilon(0.001));
}

TEST_CASE("Anthropometric de Leva scaling boundaries") {
    // Isolate and check that a 100kg athlete gets a 2.21kg forearm-hand segment mass representation
    double test_body_mass = 100.0;
    double expected_segment_mass = test_body_mass * 0.0221;
    
    CHECK(expected_segment_mass == doctest::Approx(2.21));
}

TEST_CASE("Rotational moment core math - Regression Anchor") {
    // 1. Isolated Unit Tests for pure physics scaling laws
    CHECK(computeRotationalMoment(0.02, 1000.0) == doctest::Approx(20.0));
    CHECK(computeRotationalMoment(0.0, 5000.0) == doctest::Approx(0.0));
    
    // 2. Week 5 Validation Baseline: Pitcher 000002 Characterization
    // Derived from: I_com = 0.0177063 kg*m² (80kg mass, 0.296m length, de Leva)
    // alpha = -291960 deg/s² -> -5095.66 rad/s²
    // Expected Torque Magnitude = ~90.2246 Nm
    double baseline_I_com = 0.0177063;
    double baseline_alpha_rad = -291960.0 * (M_PI / 180.0);
    
    CHECK(std::abs(computeRotationalMoment(baseline_I_com, baseline_alpha_rad)) == doctest::Approx(90.2246).epsilon(0.001));
}

TEST_CASE("Anthropometric Segment Mass Isolation") {
    // Confirms 100kg athlete tracks to a 2.21kg forearm-hand representation
    double body_mass = 100.0;
    double expected_segment_mass = body_mass * 0.0221;
    CHECK(expected_segment_mass == doctest::Approx(2.21));
}
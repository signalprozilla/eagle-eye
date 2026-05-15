#ifndef OBP_MARKERS_H
#define OBP_MARKERS_H

#include <string_view>

namespace obp {
    // Verified against OBP raw C3D Point Labels
    // constexpr std::string_view R_SHOULDER = "RSHO"; 
    // constexpr std::string_view R_ELBOW    = "RELB"; // Lateral/Center Elbow
    // constexpr std::string_view R_ELBOW_M  = "RMELB"; // Medial Elbow (The UCL side!)
    // constexpr std::string_view R_HIP      = "RASI"; // Right Hip
    // const char* const R_WRIST_M = "RWRM"; // Right Wrist Medial
    // const char* const R_WRIST_L = "RWRL"; // Right Wrist Lateral
    
    const char* const R_SHOULDER = "RSHO";
    const char* const R_ELBOW_M  = "RMELB"; 
    const char* const R_WRIST_M  = "RWRB"; // Using RWRB for our wrist marker

}

#endif
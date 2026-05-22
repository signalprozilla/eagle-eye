cmake_minimum_required(VERSION 3.10)
project(eagle-eye)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. External Dependencies Setup
# ezc3d (The Biomechanics Binary Parser Library)
add_subdirectory(third_party/ezc3d)

# 2. Primary Target: Core Eagle Eye Application
add_executable(eagle-eye
    src/main.cpp 
    src/c3d_loader.cpp 
    src/csv_writer.cpp 
    src/ground_truth.cpp 
    src/kinematics.cpp
    src/filter.cpp
    src/signal.cpp
)

# Use explicit keyword scopes (Modern CMake)
target_include_directories(eagle-eye PRIVATE 
    include 
    third_party/eigen 
    third_party/ezc3d/include
)
target_link_libraries(eagle-eye PRIVATE ezc3d)


# 3. Secondary Target: Week 5 Kinematic Verification Test Suite
add_executable(test-kinematics 
    tests/test_kinematics.cpp
    src/kinematics.cpp
    src/filter.cpp
)

target_include_directories(test-kinematics PRIVATE 
    include 
    third_party
    third_party/eigen 
)

# ====================================================================
# 3. SEPARATE BATCH TARGET
# ====================================================================
add_executable(eagle-eye-batch 
    src/batch_processor.cpp 
    src/c3d_loader.cpp 
    src/csv_writer.cpp
    src/ground_truth.cpp
    src/kinematics.cpp
    src/filter.cpp
    src/signal.cpp
)

# TELL CMAKE WHERE THE HEADERS ARE FOR THE BATCH PROCESSOR (Fixes your fatal error)
target_include_directories(eagle-eye-batch PRIVATE 
    include 
    third_party/eigen 
    third_party/ezc3d/include
)

target_link_libraries(eagle-eye-batch PRIVATE ezc3d)
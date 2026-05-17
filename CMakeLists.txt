cmake_minimum_required(VERSION 3.10)
project(eagle-eye)

set(CMAKE_CXX_STANDARD 17)

# 1. Tell CMake where to find your own headers AND Eigen
include_directories(include)           # <--- ADD THIS LINE
include_directories(third_party/eigen)

# 2. ezc3d (The Biomechanics Library)
add_subdirectory(third_party/ezc3d)
include_directories(third_party/ezc3d/include)

# 3. Create your app
add_executable(eagle-eye src/main.cpp src/c3d_loader.cpp src/csv_writer.cpp src/ground_truth.cpp)

# 4. Link the libraries together
target_link_libraries(eagle-eye ezc3d)
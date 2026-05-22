#pragma once
#include <vector>
#include <Eigen/Dense>

std::vector<Eigen::Vector3d> filterZeroPhaseButterworth(const std::vector<Eigen::Vector3d>& raw_signal, double fs, double fc);
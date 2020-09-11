#ifndef CALIB_ZED_HPP
#define CALIB_ZED_HPP
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <sl/Camera.hpp>

std::vector<double> getDistortionCoefficient();

#endif
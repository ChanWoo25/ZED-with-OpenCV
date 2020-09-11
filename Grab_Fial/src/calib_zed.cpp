#include <calib_zed.hpp>

std::vector<double> getDistortionCoefficient()
{
    std::vector<double> ret;
    // sl::Camera zed;
    // // Set configuration parameters
    // sl::InitParameters init_params;
    // init_params.camera_resolution = sl::RESOLUTION::HD720;
    // init_params.camera_fps = 30;

    // // Open the camera
    // auto err = zed.open(init_params);
    // if (err != sl::ERROR_CODE::SUCCESS)
    //     exit(-1);

    // sl::CalibrationParameters calibration_params = zed.getCameraInformation().calibration_parameters;
    // // Focal length of the left eye in pixels
    // float focal_left_x = calibration_params.left_cam.fx;
    // // First radial distortion coefficient
    // float k1 = calibration_params.left_cam.disto[0];
    // // Translation between left and right eye on z-axis
    // float tz = calibration_params.T.z;
    // // Horizontal field of view of the left eye in degrees
    // float h_fov = calibration_params.left_cam.h_fov;

    // std::vector<double> ret;
    // auto disto = calibration_params.left_cam.disto;

    // for(int i=0 ;i<5; i++)
    //     ret.push_back(disto[i]);

    // std::cout   << "focal left x: " << focal_left_x << '\n'
    //             << "k1: " << k1 << '\n'
    //             << "tz: " << tz << '\n'
    //             << "h_fov" << h_fov << '\n';

    return ret;
}
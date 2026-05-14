#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <cmath>

#include "transform_utils/config.h"
#include "transform_utils/stereo_camera.h"
#include "transform_utils/camera_load.h"
#include "transform_utils/camera_model.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_stereo_camera");

    std::string config_path;
    if (argc > 1)
        config_path = argv[1];
    else
        config_path = ros::package::getPath("transform_utils") + "/config/front_stereo.yaml";

    std::cout << "[test_stereo] Using config file: " << config_path << std::endl;

    Config::setParameterFile(config_path);

    StereoCamera stereo_cam;

    std::cout << "\n[test_stereo] baseline = " << stereo_cam.baseline_ << std::endl;

    // =========================
    // Test 1: 已知左右像素点
    // =========================
    Vector2d pix_left(480.0, 360.0);

    // 期望深度 = 5m
    // disparity = fx * baseline / depth
    double expected_depth = 5.0;
    double disparity = stereo_cam.left_camera_.fx_ * stereo_cam.baseline_ / expected_depth;

    Vector2d pix_right(pix_left.x() - disparity, pix_left.y());

    std::cout << "\n========== Test 1: known disparity ==========" << std::endl;
    std::cout << "[test_stereo] pix_left  = " << pix_left.transpose() << std::endl;
    std::cout << "[test_stereo] pix_right = " << pix_right.transpose() << std::endl;
    std::cout << "[test_stereo] disparity = " << disparity << std::endl;

    // T_b_w = Identity
    SE3 T_b_w;

    Vector3d p_w_1 = stereo_cam.pixel2WorldFromDisparity(pix_left, pix_right, T_b_w);
    std::cout << "[test_stereo] pixel2WorldFromDisparity = " << p_w_1.transpose() << std::endl;

    // =========================
    // Test 2: 直接给 disparity
    // =========================
    std::cout << "\n========== Test 2: pixel2worldLeft ==========" << std::endl;
    Vector3d p_w_2 = stereo_cam.pixel2worldLeft(pix_left, disparity, T_b_w);
    std::cout << "[test_stereo] pixel2worldLeft = " << p_w_2.transpose() << std::endl;

    // =========================
    // Test 3: 带平移 + yaw 旋转
    // =========================
    std::cout << "\n========== Test 3: translated + rotated T_b_w ==========" << std::endl;

    Eigen::Matrix3d R_w_b;
    R_w_b = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d t_w_b(10.0, 2.0, 1.0);

    SE3 T_w_b(SO3(R_w_b), t_w_b);
    SE3 T_b_w_rot = T_w_b.inverse();

    Vector3d p_w_3 = stereo_cam.pixel2WorldFromDisparity(pix_left, pix_right, T_b_w_rot);
    std::cout << "[test_stereo] pixel2WorldFromDisparity (translated + rotated) = "
              << p_w_3.transpose() << std::endl;

    return 0;
}
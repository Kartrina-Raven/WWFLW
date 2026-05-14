#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <cmath>

#include "transform_utils/config.h"
#include "transform_utils/camera_load.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_camera_loader");

    std::string config_path;
    if (argc > 1)
        config_path = argv[1];
    else
        config_path = ros::package::getPath("transform_utils") + "/config/front_stereo.yaml";

    std::cout << "[test] Using config file: " << config_path << std::endl;

    Config::setParameterFile(config_path);

    Camera left_cam  = CameraLoader::loadFrontLeft();
    Camera right_cam = CameraLoader::loadFrontRight();

    std::cout << "\n[test] left camera:" << std::endl;
    std::cout << "  fx = " << left_cam.fx_ << std::endl;
    std::cout << "  fy = " << left_cam.fy_ << std::endl;
    std::cout << "  cx = " << left_cam.cx_ << std::endl;
    std::cout << "  cy = " << left_cam.cy_ << std::endl;
    std::cout << "  t_b_c = " << left_cam.t_b_c_.transpose() << std::endl;
    std::cout << "  R_b_c = \n" << left_cam.R_b_c_ << std::endl;

    std::cout << "\n[test] right camera:" << std::endl;
    std::cout << "  fx = " << right_cam.fx_ << std::endl;
    std::cout << "  fy = " << right_cam.fy_ << std::endl;
    std::cout << "  cx = " << right_cam.cx_ << std::endl;
    std::cout << "  cy = " << right_cam.cy_ << std::endl;
    std::cout << "  t_b_c = " << right_cam.t_b_c_.transpose() << std::endl;
    std::cout << "  R_b_c = \n" << right_cam.R_b_c_ << std::endl;

    // =========================
    // Camera usage test
    // =========================
    Vector2d pixel_center(480.0, 360.0);
    double depth = 5.0;

    std::cout << "\n========== Camera Usage Test ==========" << std::endl;
    std::cout << "[test] pixel center = " << pixel_center.transpose()
              << ", depth = " << depth << std::endl;

    // 1. pixel -> camera
    Vector3d p_c_left = left_cam.pixel2camera(pixel_center, depth);
    Vector3d p_c_right = right_cam.pixel2camera(pixel_center, depth);

    std::cout << "[left] pixel2camera = " << p_c_left.transpose() << std::endl;
    std::cout << "[right] pixel2camera = " << p_c_right.transpose() << std::endl;

    // 2. camera -> pixel
    Vector2d pix_left_back = left_cam.camera2pixel(p_c_left);
    Vector2d pix_right_back = right_cam.camera2pixel(p_c_right);

    std::cout << "[left] camera2pixel(back projection) = " << pix_left_back.transpose() << std::endl;
    std::cout << "[right] camera2pixel(back projection) = " << pix_right_back.transpose() << std::endl;

    // 3. pixel -> world
    // 这里先设 body 和 world 重合：T_b_w = Identity
    SE3 T_b_w;   // 默认单位位姿

    Vector3d p_w_left = left_cam.pixel2world(pixel_center, T_b_w, depth);
    Vector3d p_w_right = right_cam.pixel2world(pixel_center, T_b_w, depth);

    std::cout << "[left] pixel2world = " << p_w_left.transpose() << std::endl;
    std::cout << "[right] pixel2world = " << p_w_right.transpose() << std::endl;

    // =========================
    // Camera usage test 2: translated + rotated T_b_w
    // =========================
    std::cout << "\n========== Camera Usage Test 2 (translated + rotated T_b_w) ==========" << std::endl;

    Eigen::Matrix3d R_w_b;
    R_w_b = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d t_w_b(10.0, 2.0, 1.0);
    SE3 T_w_b(SO3(R_w_b), t_w_b);
    SE3 T_b_w_rot = T_w_b.inverse();

    Vector3d p_w_left_rot = left_cam.pixel2world(pixel_center, T_b_w_rot, depth);
    Vector3d p_w_right_rot = right_cam.pixel2world(pixel_center, T_b_w_rot, depth);

    std::cout << "[left] pixel2world (translated + rotated) = "
          << p_w_left_rot.transpose() << std::endl;
    std::cout << "[right] pixel2world (translated + rotated) = "
          << p_w_right_rot.transpose() << std::endl;

    return 0;
}
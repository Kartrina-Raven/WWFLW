#include "transform_utils/camera_load.h"
#include <cstdlib>
#include <iostream>

Eigen::Matrix3d CameraLoader::readRotation(const cv::FileNode& rotation_node)
{
    Eigen::Matrix3d R;
    R << (double)rotation_node["r00"], (double)rotation_node["r01"], (double)rotation_node["r02"],
         (double)rotation_node["r10"], (double)rotation_node["r11"], (double)rotation_node["r12"],
         (double)rotation_node["r20"], (double)rotation_node["r21"], (double)rotation_node["r22"];
    return R;
}

Eigen::Vector3d CameraLoader::readTranslation(const cv::FileNode& translation_node)
{
    return Eigen::Vector3d(
        (double)translation_node["x"],
        (double)translation_node["y"],
        (double)translation_node["z"]
    );
}


Camera CameraLoader::loadCamera(const cv::FileNode& camera_node, const std::string& name)
{
   
    cv::FileNode intrinsics_node = camera_node["intrinsics"];
    cv::FileNode extrinsics_node = camera_node["extrinsics_to_body"];

    double fx = (double)intrinsics_node["fx"];
    double fy = (double)intrinsics_node["fy"];
    double cx = (double)intrinsics_node["cx"];
    double cy = (double)intrinsics_node["cy"];

    Eigen::Vector3d t_b_c = readTranslation(extrinsics_node["translation"]);
    Eigen::Matrix3d R_b_c = readRotation(extrinsics_node["rotation"]);

    /*std::cout << "[CameraLoader] loaded " << name
              << " fx=" << fx << " fy=" << fy
              << " cx=" << cx << " cy=" << cy << std::endl;*/

    return Camera(name, fx, fy, cx, cy, R_b_c, t_b_c);
}

Camera CameraLoader::loadFrontLeft()
{
    cv::FileStorage& fs = Config::file();
    cv::FileNode first = fs.getFirstTopLevelNode();

    cv::FileNode stereo_node = fs["front_stereo"];
    cv::FileNode left_node = stereo_node["left"];

    return loadCamera(left_node, "front_left");
}

Camera CameraLoader::loadFrontRight()
{
    cv::FileStorage& fs = Config::file();
    cv::FileNode stereo_node = fs["front_stereo"];
    cv::FileNode right_node = stereo_node["right"];
    
    return loadCamera(right_node, "front_right");
}
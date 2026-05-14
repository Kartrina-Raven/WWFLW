#pragma once

#include "transform_utils/camera_model.h"
#include "transform_utils/config.h"

class CameraLoader
{
public:
    static Camera loadCamera(const cv::FileNode& camera_node, const std::string& name);

    static Camera loadFrontLeft();
    static Camera loadFrontRight();

private:
    static Eigen::Matrix3d readRotation(const cv::FileNode& rotation_node);
    static Eigen::Vector3d readTranslation(const cv::FileNode& translation_node);
};
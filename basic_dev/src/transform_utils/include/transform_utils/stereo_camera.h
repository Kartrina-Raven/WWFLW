#pragma once

#include "camera_model.h"

class StereoCamera
{
public:
    typedef std::shared_ptr<StereoCamera> Ptr;

    StereoCamera();

    Vector3d pixel2worldLeft(const Vector2d& pix_left, double disparity, const SE3& T_b_w);

    // 左右像素点已知 → 世界坐标
    Vector3d pixel2WorldFromDisparity(const Vector2d& pix_left, const Vector2d& pix_right, const SE3& T_b_w);

    // 右目匹配：给左目像素在右目图像中小范围搜索
    Vector2d matchRightPixel(const cv::Mat& left_img, const cv::Mat& right_img, const Vector2d& pix_left);

    Camera left_camera_;
    Camera right_camera_;
    double baseline_;
};


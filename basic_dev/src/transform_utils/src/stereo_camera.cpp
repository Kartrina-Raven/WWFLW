#include "transform_utils/stereo_camera.h"
#include "transform_utils/camera_load.h"
#include "transform_utils/config.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

StereoCamera::StereoCamera()
{
    // 初始化左右相机，从配置文件读取
    left_camera_  = CameraLoader::loadFrontLeft();
    right_camera_ = CameraLoader::loadFrontRight();

    //直接给了，测试完后我放config里读取吧
    baseline_ = 0.300;
}



// 左目像素 + 视差 → 世界坐标
Vector3d StereoCamera::pixel2worldLeft(const Vector2d& pix_left,double disparity,const SE3& T_b_w)
{
    //防止视差过小
    if (disparity <= 1e-6)
    {
        std::cerr << "[StereoCamera] invalid disparity: " << disparity << std::endl;
        return Vector3d::Zero();
    }

    double depth = left_camera_.fx_ * baseline_ / disparity;
    return left_camera_.pixel2world(pix_left, T_b_w, depth);
}



// 左目像素 + 右目像素 → 世界坐标
Vector3d StereoCamera::pixel2WorldFromDisparity(const Vector2d& pix_left,const Vector2d& pix_right,const SE3& T_b_w)
{
    double disparity = pix_left.x() - pix_right.x();
    //同样防止视差过小
    if (disparity <= 1e-6)
    {
        std::cerr << "[StereoCamera] invalid disparity: left_x = "
                  << pix_left.x() << ", right_x = " << pix_right.x() << std::endl;
        return Vector3d::Zero();
    }

    return pixel2worldLeft(pix_left, disparity, T_b_w);
}



//急需改进的部分！精度太差
Vector2d StereoCamera::matchRightPixel(const cv::Mat& left_img,const cv::Mat& right_img,const Vector2d& pix_left)
{
    if (left_img.empty() || right_img.empty())
    {
        std::cerr << "[StereoCamera] input image is empty." << std::endl;
        return Vector2d(-1, -1);
    }

    const int window_size = 15;
    const int half_window = window_size / 2;

    int u_l = static_cast<int>(pix_left.x());
    int v_l = static_cast<int>(pix_left.y());

    // 左图 patch 边界检查
    if (u_l - half_window < 0 || u_l + half_window >= left_img.cols ||
        v_l - half_window < 0 || v_l + half_window >= left_img.rows)
    {
        std::cerr << "[StereoCamera] left patch out of image boundary." << std::endl;
        return Vector2d(-1, -1);
    }

    cv::Rect left_rect(u_l - half_window, v_l - half_window, window_size, window_size);
    cv::Mat patch_left = left_img(left_rect);

    // 在右图同一行附近搜索
    int col_start = std::max(half_window, u_l - 60);
    int col_end   = std::min(right_img.cols - half_window - 1, u_l + 10);

    // 右图行边界检查
    if (v_l - half_window < 0 || v_l + half_window >= right_img.rows)
    {
        std::cerr << "[StereoCamera] right patch row out of image boundary." << std::endl;
        return Vector2d(-1, -1);
    }

    double min_score = DBL_MAX;
    int best_u_r = -1;

    for (int u_r = col_start; u_r <= col_end; ++u_r)
    {
        cv::Rect right_rect(u_r - half_window, v_l - half_window, window_size, window_size);
        cv::Mat patch_right = right_img(right_rect);

        cv::Mat diff;
        cv::absdiff(patch_left, patch_right, diff);
        double score = cv::sum(diff)[0];

        if (score < min_score)
        {
            min_score = score;
            best_u_r = u_r;
        }
    }

    if (best_u_r < 0)
    {
        std::cerr << "[StereoCamera] failed to find right match." << std::endl;
        return Vector2d(-1, -1);
    }

    return Vector2d(best_u_r, v_l);
}
#include "transform_utils/camera_model.h"

Camera::Camera()
{
    fx_ = 0.0;
    fy_ = 0.0;
    cx_ = 0.0;
    cy_ = 0.0;
    name_ = "";

    R_b_c_ = Eigen::Matrix3d::Identity();
    t_b_c_ = Eigen::Vector3d::Zero();
}


Vector3d Camera::world2camera(const Vector3d& p_w, const SE3& T_b_w)
{
    Vector3d p_b = T_b_w * p_w;
    Vector3d p_c = R_b_c_.transpose() * (p_b - t_b_c_);
    return p_c;
}


Vector3d Camera::camera2world(const Vector3d& p_c, const SE3& T_b_w)
{
    Vector3d p_b = R_b_c_ * p_c + t_b_c_;
    Vector3d p_w = T_b_w.inverse() * p_b;
    return p_w;
}


Vector2d Camera::camera2pixel(const Vector3d& p_c)
{
    return Vector2d(
        fx_ * p_c[0] / p_c[2] + cx_,
        fy_ * p_c[1] / p_c[2] + cy_
    );
}


Vector3d Camera::pixel2camera(const Vector2d& p_p, double depth)
{
    return Vector3d(
        (p_p[0] - cx_) * depth / fx_,
        (p_p[1] - cy_) * depth / fy_,
        depth
    );
}


Vector3d Camera::pixel2world(const Vector2d& p_p, const SE3& T_b_w, double depth)
{
    Vector3d p_c = pixel2camera(p_p, depth);
    return camera2world(p_c, T_b_w);
}


Vector2d Camera::world2pixel(const Vector3d& p_w, const SE3& T_b_w)
{
    Vector3d p_c = world2camera(p_w, T_b_w);
    return camera2pixel(p_c);
}
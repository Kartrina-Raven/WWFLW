#pragma once

#include "transform_utils/common_included.h"


class Camera
{
public:
    double   fx_, fy_, cx_, cy_;  // Camera intrinsics 
    std::string name_;
    Eigen::Matrix3d R_b_c_;
    Eigen::Vector3d t_b_c_;

    Camera();
    Camera (std::string name,double fx, double fy, double cx, double cy,Eigen::Matrix3d R_b_c,Eigen::Vector3d t_b_c) :
        fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ),name_(name)
        ,R_b_c_(R_b_c), t_b_c_(t_b_c)
    {}

    // coordinate transform: world, camera, pixel
    Vector3d world2camera( const Vector3d& p_w, const SE3& T_b_w );
    Vector3d camera2world( const Vector3d& p_c, const SE3& T_b_w );
    Vector2d camera2pixel( const Vector3d& p_c );
    Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
    Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_b_w, double depth=1 );
    Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_b_w );

};
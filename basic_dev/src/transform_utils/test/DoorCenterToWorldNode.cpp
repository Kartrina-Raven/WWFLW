#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "transform_utils/config.h"
#include "transform_utils/stereo_camera.h"
#include "transform_utils/common_included.h"

#include "yolov11_ros_msgs/BoundingBoxes.h"
#include "yolov11_ros_msgs/BoundingBox.h"
#include "transform_utils/WayPoints.h"

class DoorCenterToWorldNode
{
public:
    explicit DoorCenterToWorldNode(ros::NodeHandle& nh)
        : nh_(nh),
          has_left_img_(false),
          has_right_img_(false),
          has_odom_(false),
          waypoint_sent_(false),
          stable_count_(0),
          latest_gate_world_(Eigen::Vector3d::Zero()),
          has_stable_gate_(false),
          stable_gate_world_(Eigen::Vector3d::Zero()),
          waypoint_publish_period_(2.0),
          coord_fix_x_(5.00984),
          coord_fix_y_(-9.39357),
          coord_fix_z_(6.08121),
          flip_y_(true),
          flip_z_(true)
    {
        // -----------------------------
        // 参数
        // -----------------------------
        std::string default_config =
            ros::package::getPath("transform_utils") + "/config/front_stereo.yaml";

        nh_.param<std::string>("config_path", config_path_, default_config);
        nh_.param<std::string>("target_class", target_class_, std::string("door"));
        nh_.param<double>("min_probability", min_probability_, 0.5);

        nh_.param<std::string>("bbox_topic", bbox_topic_,
                               std::string("/yolov11/front_left/BoundingBoxes"));
        nh_.param<std::string>("left_image_topic", left_image_topic_,
                               std::string("/airsim_node/drone_1/front_left/Scene"));
        nh_.param<std::string>("right_image_topic", right_image_topic_,
                               std::string("/airsim_node/drone_1/front_right/Scene"));
        nh_.param<std::string>("odom_topic", odom_topic_, std::string("/eskf_odom"));
        nh_.param<std::string>("gps_topic", gps_topic_,
                               std::string("/airsim_node/drone_1/gps"));

        nh_.param<std::string>("gate_world_topic", gate_world_topic_,
                               std::string("/gate_center_world"));
        nh_.param<std::string>("waypoint_topic", waypoint_topic_,
                               std::string("/waypoints"));
        nh_.param<std::string>("edited_gps_topic", edited_gps_topic_,
                               std::string("/airsim_node/drone_1/edited_gps"));

        // waypoint 生成相关参数
        nh_.param<bool>("publish_waypoint_once", publish_waypoint_once_, true);
        nh_.param<int>("stable_count_threshold", stable_count_threshold_, 5);
        nh_.param<double>("stable_dist_threshold", stable_dist_threshold_, 1.0);
        nh_.param<bool>("use_approach_point", use_approach_point_, true);
        nh_.param<double>("approach_offset_x", approach_offset_x_, -5.0);
        nh_.param<double>("approach_offset_y", approach_offset_y_, 0.0);
        nh_.param<double>("approach_offset_z", approach_offset_z_, 0.0);

        // 定时周期发布 waypoint
        nh_.param<double>("waypoint_publish_period", waypoint_publish_period_, 2.0);

        // 坐标修正参数（可在 launch 里调）
        nh_.param<bool>("flip_y", flip_y_, true);
        nh_.param<bool>("flip_z", flip_z_, true);
        nh_.param<double>("coord_fix_x", coord_fix_x_, 5.00984);
        nh_.param<double>("coord_fix_y", coord_fix_y_, -9.39357);
        nh_.param<double>("coord_fix_z", coord_fix_z_, 6.08121);

        ROS_INFO_STREAM("[door_center_to_world] config_path = " << config_path_);
        ROS_INFO_STREAM("[door_center_to_world] target_class = " << target_class_);
        ROS_INFO_STREAM("[door_center_to_world] bbox_topic = " << bbox_topic_);
        ROS_INFO_STREAM("[door_center_to_world] left_image_topic = " << left_image_topic_);
        ROS_INFO_STREAM("[door_center_to_world] right_image_topic = " << right_image_topic_);
        ROS_INFO_STREAM("[door_center_to_world] odom_topic = " << odom_topic_);
        ROS_INFO_STREAM("[door_center_to_world] gps_topic = " << gps_topic_);
        ROS_INFO_STREAM("[door_center_to_world] gate_world_topic = " << gate_world_topic_);
        ROS_INFO_STREAM("[door_center_to_world] waypoint_topic = " << waypoint_topic_);
        ROS_INFO_STREAM("[door_center_to_world] edited_gps_topic = " << edited_gps_topic_);
        ROS_INFO_STREAM("[door_center_to_world] waypoint_publish_period = " << waypoint_publish_period_);
        ROS_INFO_STREAM("[door_center_to_world] flip_y = " << (flip_y_ ? "true" : "false"));
        ROS_INFO_STREAM("[door_center_to_world] flip_z = " << (flip_z_ ? "true" : "false"));
        ROS_INFO_STREAM("[door_center_to_world] coord_fix = ("
                        << coord_fix_x_ << ", " << coord_fix_y_ << ", " << coord_fix_z_ << ")");

        // -----------------------------
        // 初始化配置和双目模型
        // -----------------------------
        Config::setParameterFile(config_path_);
        stereo_cam_ = std::make_shared<StereoCamera>();

        // -----------------------------
        // 订阅与发布
        // -----------------------------
        bbox_sub_ = nh_.subscribe(bbox_topic_, 10,
                                  &DoorCenterToWorldNode::bboxCallback, this);
        left_img_sub_ = nh_.subscribe(left_image_topic_, 10,
                                      &DoorCenterToWorldNode::leftImageCallback, this);
        right_img_sub_ = nh_.subscribe(right_image_topic_, 10,
                                       &DoorCenterToWorldNode::rightImageCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20,
                                  &DoorCenterToWorldNode::odomCallback, this);
        gps_sub_ = nh_.subscribe(gps_topic_, 20,
                                 &DoorCenterToWorldNode::gpsCallback, this);

        gate_world_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(gate_world_topic_, 10);

        // latched 保留，方便后启动节点拿到最近一次
        waypoint_pub_ = nh_.advertise<transform_utils::WayPoints>(waypoint_topic_, 1, true);

        edited_gps_pub_ =
            nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(edited_gps_topic_, 10);

        // Timer：每隔 waypoint_publish_period_ 秒发一次稳定后的 waypoint
        waypoint_timer_ = nh_.createTimer(
            ros::Duration(waypoint_publish_period_),
            &DoorCenterToWorldNode::waypointTimerCallback,
            this
        );

        ROS_INFO("[door_center_to_world] node initialized.");
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber bbox_sub_;
    ros::Subscriber left_img_sub_;
    ros::Subscriber right_img_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_sub_;

    ros::Publisher gate_world_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher edited_gps_pub_;

    ros::Timer waypoint_timer_;

    std::shared_ptr<StereoCamera> stereo_cam_;

    cv::Mat left_gray_;
    cv::Mat right_gray_;
    bool has_left_img_;
    bool has_right_img_;
    bool has_odom_;

    SE3 T_b_w_;  // world -> body
    ros::Time latest_odom_stamp_;

    std::string config_path_;
    std::string target_class_;
    double min_probability_;

    std::string bbox_topic_;
    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string odom_topic_;
    std::string gps_topic_;
    std::string gate_world_topic_;
    std::string waypoint_topic_;
    std::string edited_gps_topic_;

    bool waypoint_sent_;
    bool publish_waypoint_once_;
    int stable_count_;
    int stable_count_threshold_;
    double stable_dist_threshold_;

    bool use_approach_point_;
    double approach_offset_x_;
    double approach_offset_y_;
    double approach_offset_z_;

    Eigen::Vector3d latest_gate_world_;

    // 稳定目标 & 周期发布
    bool has_stable_gate_;
    Eigen::Vector3d stable_gate_world_;
    double waypoint_publish_period_;

    // 坐标修正参数
    double coord_fix_x_;
    double coord_fix_y_;
    double coord_fix_z_;
    bool flip_y_;
    bool flip_z_;

private:
    bool isReasonableGateCorners(const std::vector<Eigen::Vector3d>& pts) const
{
    if (pts.size() != 4) return false;

    const Eigen::Vector3d& lt = pts[0]; // 左上
    const Eigen::Vector3d& rt = pts[1]; // 右上
    const Eigen::Vector3d& lb = pts[2]; // 左下
    const Eigen::Vector3d& rb = pts[3]; // 右下

    double top_w    = (lt - rt).norm();
    double bottom_w = (lb - rb).norm();
    double left_h   = (lt - lb).norm();
    double right_h  = (rt - rb).norm();
    double diag1    = (lt - rb).norm();
    double diag2    = (rt - lb).norm();

    // 1. 边长不能太小，避免退化
    if (top_w < 0.3 || bottom_w < 0.3 || left_h < 0.3 || right_h < 0.3)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] gate geometry rejected: side too small.");
        return false;
    }

    // 2. 上下边长度应接近
    double width_ratio = std::max(top_w, bottom_w) / std::min(top_w, bottom_w);
    if (width_ratio > 1.8)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] gate geometry rejected: top/bottom width mismatch.");
        return false;
    }

    // 3. 左右边长度应接近
    double height_ratio = std::max(left_h, right_h) / std::min(left_h, right_h);
    if (height_ratio > 1.8)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] gate geometry rejected: left/right height mismatch.");
        return false;
    }

    // 4. 对角线应接近
    double diag_ratio = std::max(diag1, diag2) / std::min(diag1, diag2);
    if (diag_ratio > 1.5)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] gate geometry rejected: diagonal mismatch.");
        return false;
    }

    // 5. 四点深度不要差太离谱
    double z_min = std::min(std::min(lt.z(), rt.z()), std::min(lb.z(), rb.z()));
    double z_max = std::max(std::max(lt.z(), rt.z()), std::max(lb.z(), rb.z()));
    if ((z_max - z_min) > 3.0)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] gate geometry rejected: depth spread too large.");
        return false;
    }

    return true;
}
    Eigen::Vector3d applyCoordinateFix(const Eigen::Vector3d& p)
    {
        Eigen::Vector3d out = p;

        out.x() = out.x() + coord_fix_x_;
        out.y() = (flip_y_ ? -out.y() : out.y()) + coord_fix_y_;
        out.z() = (flip_z_ ? -out.z() : out.z()) + coord_fix_z_;

        return out;
    }

    void leftImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

            cv::cvtColor(cv_ptr->image, left_gray_, cv::COLOR_BGR2GRAY);
            has_left_img_ = true;
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("[door_center_to_world] left image cv_bridge exception: " << e.what());
        }
    }

    void rightImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

            cv::cvtColor(cv_ptr->image, right_gray_, cv::COLOR_BGR2GRAY);
            has_right_img_ = true;
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR_STREAM("[door_center_to_world] right image cv_bridge exception: " << e.what());
        }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        Eigen::Quaterniond q_w_b(q.w, q.x, q.y, q.z);
        Eigen::Vector3d t_w_b(p.x, p.y, p.z);

        // T_w_b: body -> world
        SE3 T_w_b(SO3(q_w_b.toRotationMatrix()), t_w_b);

        // StereoCamera 用的是 T_b_w: world -> body
        T_b_w_ = T_w_b.inverse();

        latest_odom_stamp_ = msg->header.stamp;
        has_odom_ = true;
    }

    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseWithCovarianceStamped edited_gps;
        edited_gps.header.stamp = ros::Time::now();
        edited_gps.header.frame_id = "map";
        edited_gps.pose.pose = msg->pose;

        edited_gps.pose.covariance[0]  = 1e-7; // x
        edited_gps.pose.covariance[7]  = 1e-7; // y
        edited_gps.pose.covariance[14] = 1e-7; // z
        edited_gps.pose.covariance[21] = 0.1;  // roll
        edited_gps.pose.covariance[28] = 0.1;  // pitch
        edited_gps.pose.covariance[35] = 0.1;  // yaw

        edited_gps_pub_.publish(edited_gps);
    }

    bool selectBestBox(const yolov11_ros_msgs::BoundingBoxesConstPtr& msg,
                       yolov11_ros_msgs::BoundingBox& best_box)
    {
        bool found = false;
        double best_score = -1.0;

        for (const auto& box : msg->bounding_boxes)
        {
            if (box.Class != target_class_) continue;
            if (box.probability < min_probability_) continue;

            double area = static_cast<double>(box.xmax - box.xmin) *
                          static_cast<double>(box.ymax - box.ymin);
            double score = box.probability * area;

            if (!found || score > best_score)
            {
                best_score = score;
                best_box = box;
                found = true;
            }
        }

        return found;
    }

    void publishGateWorldPose(const Eigen::Vector3d& gate_world)
    {
        geometry_msgs::PoseStamped out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "world";

        out_msg.pose.position.x = gate_world.x();
        out_msg.pose.position.y = gate_world.y();
        out_msg.pose.position.z = gate_world.z();

        out_msg.pose.orientation.w = 1.0;
        out_msg.pose.orientation.x = 0.0;
        out_msg.pose.orientation.y = 0.0;
        out_msg.pose.orientation.z = 0.0;

        gate_world_pub_.publish(out_msg);
    }

    void publishWaypoints(const Eigen::Vector3d& gate_world)
    {
        transform_utils::WayPoints msg;

        if (use_approach_point_)
        {
            geometry_msgs::Point p_approach;
            p_approach.x = gate_world.x() + approach_offset_x_;
            p_approach.y = gate_world.y() + approach_offset_y_;
            p_approach.z = gate_world.z() + approach_offset_z_;
            msg.points.push_back(p_approach);
        }

        geometry_msgs::Point p_gate;
        p_gate.x = gate_world.x();
        p_gate.y = gate_world.y();
        p_gate.z = gate_world.z();
        msg.points.push_back(p_gate);

        waypoint_pub_.publish(msg);
        waypoint_sent_ = true;

        ROS_INFO("[door_center_to_world] published /waypoints.");
        ROS_INFO("[door_center_to_world] waypoint size = %zu", msg.points.size());
        for (size_t i = 0; i < msg.points.size(); ++i)
        {
            ROS_INFO("[door_center_to_world] waypoint[%zu] = (%.3f, %.3f, %.3f)",
                     i,
                     msg.points[i].x,
                     msg.points[i].y,
                     msg.points[i].z);
        }
    }

    void waypointTimerCallback(const ros::TimerEvent&)
    {
        if (!has_stable_gate_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[door_center_to_world] no stable gate yet, timer publish skipped.");
            return;
        }

        // 周期发布模式下，不再受“只发一次”拦截
        publishWaypoints(stable_gate_world_);
    }

    void tryPublishStableWaypoint(const Eigen::Vector3d& gate_world)
    {
        if (stable_count_ == 0)
        {
            latest_gate_world_ = gate_world;
            stable_count_ = 1;
        }
        else
        {
            double dist = (gate_world - latest_gate_world_).norm();
            if (dist < stable_dist_threshold_)
            {
                stable_count_++;
                latest_gate_world_ = 0.5 * latest_gate_world_ + 0.5 * gate_world;
            }
            else
            {
                stable_count_ = 1;
                latest_gate_world_ = gate_world;
            }
        }

        ROS_INFO_THROTTLE(0.5,
                          "[door_center_to_world] stable_count=%d/%d, latest_gate=(%.3f, %.3f, %.3f)",
                          stable_count_,
                          stable_count_threshold_,
                          latest_gate_world_.x(),
                          latest_gate_world_.y(),
                          latest_gate_world_.z());

        if (stable_count_ >= stable_count_threshold_)
        {
            stable_gate_world_ = latest_gate_world_;
            has_stable_gate_ = true;

            ROS_INFO_THROTTLE(0.5,
                              "[door_center_to_world] stable gate updated: (%.3f, %.3f, %.3f)",
                              stable_gate_world_.x(),
                              stable_gate_world_.y(),
                              stable_gate_world_.z());
        }
    }

    
void bboxCallback(const yolov11_ros_msgs::BoundingBoxesConstPtr& msg)
{
    if (!has_left_img_ || !has_right_img_ || !has_odom_)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] waiting for left/right image and odom...");
        return;
    }

    yolov11_ros_msgs::BoundingBox best_box;
    if (!selectBestBox(msg, best_box))
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] no valid target bbox found.");
        return;
    }

    // ---------------------------
    // 1) bbox 基本信息
    // ---------------------------
    double xmin = best_box.xmin;
    double xmax = best_box.xmax;
    double ymin = best_box.ymin;
    double ymax = best_box.ymax;

    double w = xmax - xmin;
    double h = ymax - ymin;

    if (w < 50.0 || h < 50.0)
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] bbox too small.");
        return;
    }

    // 向内收缩，避免直接取框边界和背景
    const double shrink_ratio = 0.18;
    double dx = w * shrink_ratio;
    double dy = h * shrink_ratio;

    double x1 = xmin + dx;
    double x2 = xmax - dx;
    double y1 = ymin + dy;
    double y2 = ymax - dy;

    // 四个缩进去的角点：左上、右上、左下、右下
    std::vector<Vector2d> pts_left;
    pts_left.reserve(4);
    pts_left.emplace_back(x1, y1); // 左上
    pts_left.emplace_back(x2, y1); // 右上
    pts_left.emplace_back(x1, y2); // 左下
    pts_left.emplace_back(x2, y2); // 右下

    const int img_w = left_gray_.cols;
    const int img_h = left_gray_.rows;
    const int border_margin = 20;

    std::vector<Vector3d> corners_world;
    std::vector<double> disparities;
    corners_world.reserve(4);
    disparities.reserve(4);

    // ---------------------------
    // 2) 四个角点必须全部匹配成功
    // ---------------------------
    for (size_t i = 0; i < pts_left.size(); ++i)
    {
        const Vector2d& pix_left = pts_left[i];

        // 左图边界检查
        if (pix_left.x() < border_margin || pix_left.x() >= img_w - border_margin ||
            pix_left.y() < border_margin || pix_left.y() >= img_h - border_margin)
        {
            ROS_WARN_THROTTLE(1.0, "[door_center_to_world] left pixel out of border, skip frame.");
            return;
        }

        Vector2d pix_right = stereo_cam_->matchRightPixel(left_gray_, right_gray_, pix_left);

        if (pix_right.x() < 0 || pix_right.y() < 0)
        {
            ROS_WARN_THROTTLE(1.0, "[door_center_to_world] right pixel match failed at corner %zu.", i);
            return;
        }

        // 右图边界检查
        if (pix_right.x() < border_margin || pix_right.x() >= img_w - border_margin ||
            pix_right.y() < border_margin || pix_right.y() >= img_h - border_margin)
        {
            ROS_WARN_THROTTLE(1.0, "[door_center_to_world] right pixel out of border, skip frame.");
            return;
        }

        double disparity = pix_left.x() - pix_right.x();

        if (disparity < 5.0 || disparity > 200.0)
        {
            ROS_WARN_THROTTLE(1.0,
                              "[door_center_to_world] disparity out of range at corner %zu: %.3f",
                              i, disparity);
            return;
        }

        Vector3d p_world_raw =
            stereo_cam_->pixel2WorldFromDisparity(pix_left, pix_right, T_b_w_);

        Vector3d p_world_fixed = applyCoordinateFix(p_world_raw);

        if (!std::isfinite(p_world_fixed.x()) ||
            !std::isfinite(p_world_fixed.y()) ||
            !std::isfinite(p_world_fixed.z()))
        {
            ROS_WARN_THROTTLE(1.0, "[door_center_to_world] invalid world point at corner %zu.", i);
            return;
        }

        corners_world.push_back(p_world_fixed);
        disparities.push_back(disparity);

        ROS_INFO_STREAM_THROTTLE(
            0.5,
            "[door_center_to_world] corner " << i
            << " left=(" << pix_left.x() << "," << pix_left.y() << ")"
            << " right=(" << pix_right.x() << "," << pix_right.y() << ")"
            << " disparity=" << disparity
            << " raw_world=(" << p_world_raw.x() << "," << p_world_raw.y() << "," << p_world_raw.z() << ")"
            << " fixed_world=(" << p_world_fixed.x() << "," << p_world_fixed.y() << "," << p_world_fixed.z() << ")"
        );
    }

    // ---------------------------
    // 3) 四点整体几何检查，不合理就不发布
    // ---------------------------
    if (!isReasonableGateCorners(corners_world))
    {
        return;
    }

    // ---------------------------
    // 4) 四点都合理时，再求门中心
    // ---------------------------
    Vector3d gate_world = Vector3d::Zero();
    for (const auto& p : corners_world)
    {
        gate_world += p;
    }
    gate_world /= 4.0;

    publishGateWorldPose(gate_world);
    tryPublishStableWaypoint(gate_world);

    double avg_disparity = 0.0;
    for (double d : disparities) avg_disparity += d;
    avg_disparity /= static_cast<double>(disparities.size());

    ROS_INFO_THROTTLE(
        0.5,
        "[door_center_to_world] 4-corner gate accepted, avg_disparity=%.3f, gate_world=(%.3f, %.3f, %.3f)",
        avg_disparity,
        gate_world.x(), gate_world.y(), gate_world.z()
    );
}};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "door_center_to_world_node");
    ros::NodeHandle nh("~");

    DoorCenterToWorldNode node(nh);

    ros::spin();
    return 0;
}
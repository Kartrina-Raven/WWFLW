#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "transform_utils/config.h"
#include "transform_utils/stereo_camera.h"
#include "transform_utils/common_included.h"

#include "yolov11_ros_msgs/BoundingBoxes.h"
#include "yolov11_ros_msgs/BoundingBox.h"

class DoorCenterToWorldNode
{
public:
    DoorCenterToWorldNode(ros::NodeHandle& nh)
        : nh_(nh)
    {
        // -----------------------------
        // 参数
        // -----------------------------
        std::string default_config =
            ros::package::getPath("transform_utils") + "/config/front_stereo.yaml";

        nh_.param<std::string>("config_path", config_path_, default_config);
        nh_.param<std::string>("target_class", target_class_, std::string("door"));
        nh_.param<double>("min_probability", min_probability_, 0.5);

        nh_.param<std::string>("bbox_topic", bbox_topic_, std::string("/yolov11/front_left/BoundingBoxes"));
        nh_.param<std::string>("left_image_topic", left_image_topic_,
                               std::string("/airsim_node/drone_1/front_left/Scene"));
        nh_.param<std::string>("right_image_topic", right_image_topic_,
                               std::string("/airsim_node/drone_1/front_right/Scene"));
        nh_.param<std::string>("odom_topic", odom_topic_, std::string("/eskf_odom"));
        nh_.param<std::string>("gate_world_topic", gate_world_topic_,
                               std::string("/gate_center_world"));

        // -----------------------------
        // 初始化配置和双目模型
        // -----------------------------
        ROS_INFO_STREAM("[door_center_to_world] config_path = " << config_path_);
        // 先初始化 Config
        Config::setParameterFile(config_path_);

    // 再创建 StereoCamera
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

        gate_world_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(gate_world_topic_, 10);

        has_left_img_ = false;
        has_right_img_ = false;
        has_odom_ = false;

        ROS_INFO("[door_center_to_world] node initialized.");
    }

private:
    ros::NodeHandle nh_;

    ros::Subscriber bbox_sub_;
    ros::Subscriber left_img_sub_;
    ros::Subscriber right_img_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher gate_world_pub_;

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
    std::string gate_world_topic_;

private:
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

        // 你的 Camera / StereoCamera 用的是 T_b_w: world -> body
        T_b_w_ = T_w_b.inverse();

        latest_odom_stamp_ = msg->header.stamp;
        has_odom_ = true;
    }

    bool selectBestBox(const yolov11_ros_msgs::BoundingBoxesConstPtr& msg,
                       yolov11_ros_msgs::BoundingBox& best_box)
    {
        bool found = false;
        double best_score = -1.0;

        for (const auto& box : msg->bounding_boxes)
        {
            // 这里假设消息字段名是 Class / probability / xmin / ymin / xmax / ymax
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
    // 构造四个角点
    // ---------------------------
    std::vector<Vector2d> pts_left;
    double xmin = best_box.xmin;
    double xmax = best_box.xmax;
    double ymin = best_box.ymin;
    double ymax = best_box.ymax;

    pts_left.emplace_back(xmin, ymin); // 左上
    pts_left.emplace_back(xmax, ymin); // 右上
    pts_left.emplace_back(xmin, ymax); // 左下
    pts_left.emplace_back(xmax, ymax); // 右下

    // ---------------------------
    // 匹配右目 + 转世界坐标
    // ---------------------------
    std::vector<Vector3d> valid_world_points;
    std::vector<double> valid_disparities;

    for (size_t i = 0; i < pts_left.size(); ++i)
    {
        const Vector2d& pix_left = pts_left[i];

        Vector2d pix_right = stereo_cam_->matchRightPixel(left_gray_, right_gray_, pix_left);

        if (pix_right.x() < 0 || pix_right.y() < 0)
        {
            continue;
        }

        double disparity = pix_left.x() - pix_right.x();
        if (disparity <= 1e-6 || disparity < 5.0 || disparity > 200.0)
        {
            continue;
        }

        Vector3d p_world =
            stereo_cam_->pixel2WorldFromDisparity(pix_left, pix_right, T_b_w_);

        valid_world_points.push_back(p_world);
        valid_disparities.push_back(disparity);

        ROS_INFO_STREAM_THROTTLE(
            0.5,
            "[door_center_to_world] corner " << i
            << " left=(" << pix_left.x() << "," << pix_left.y() << ")"
            << " right=(" << pix_right.x() << "," << pix_right.y() << ")"
            << " disparity=" << disparity
            << " world=(" << p_world.x() << "," << p_world.y() << "," << p_world.z() << ")"
        );
    }

    if (valid_world_points.empty())
    {
        ROS_WARN_THROTTLE(1.0, "[door_center_to_world] no valid matched points in current bbox.");
        return;
    }

    // ---------------------------
    // 求平均世界坐标
    // ---------------------------
    Vector3d gate_world = Vector3d::Zero();
    for (const auto& p : valid_world_points)
    {
        gate_world += p;
    }
    gate_world /= static_cast<double>(valid_world_points.size());

    // 临时修正
    gate_world.x() = gate_world.x();
    gate_world.y() = -gate_world.y();
    gate_world.z() = -gate_world.z();
    // 发布 PoseStamped
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

    // 日志
    double avg_disparity = 0.0;
    for (double d : valid_disparities) avg_disparity += d;
    avg_disparity /= valid_disparities.size();

    ROS_INFO_THROTTLE(
        0.5,
        "[door_center_to_world] valid_points=%zu, avg_disparity=%.3f, fused_world=(%.3f, %.3f, %.3f)",
        valid_world_points.size(),
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
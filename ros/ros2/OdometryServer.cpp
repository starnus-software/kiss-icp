// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <utility>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS 2 headers
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace kiss_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("odometry_node", options) {
    // clang-format off
    base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
    odom_frame_ = declare_parameter<std::string>("odom_frame", odom_frame_);
    enable_static_transformation_ = true;
    map_frame_ = declare_parameter<std::string>("map_frame", map_frame_);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", publish_odom_tf_);
    publish_debug_clouds_ = declare_parameter<bool>("visualize", publish_debug_clouds_);
    config_.max_range = 30.0;
    config_.min_range = 0.5;
    config_.deskew = declare_parameter<bool>("deskew", config_.deskew);
    config_.voxel_size = declare_parameter<double>("voxel_size", config_.max_range / 100.0);
    config_.max_points_per_voxel = declare_parameter<int>("max_points_per_voxel", config_.max_points_per_voxel);
    config_.initial_threshold = declare_parameter<double>("initial_threshold", config_.initial_threshold);
    config_.min_motion_th = declare_parameter<double>("min_motion_th", config_.min_motion_th);
    initial_position_x = declare_parameter<double>("initial_position_x", initial_position_x);
    initial_position_y = declare_parameter<double>("initial_position_y", initial_position_y);
    initial_position_z = declare_parameter<double>("initial_position_z", initial_position_z);
    initial_angle_x = declare_parameter<double>("initial_angle_x", initial_angle_x);
    initial_angle_y = declare_parameter<double>("initial_angle_y", initial_angle_y);
    initial_angle_z = declare_parameter<double>("initial_angle_z", initial_angle_z);
    initial_angle_w = declare_parameter<double>("initial_angle_w", initial_angle_w);

    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped transform_msg_odom_map;
    transform_msg_odom_map.header.frame_id = map_frame_;
    transform_msg_odom_map.child_frame_id = odom_frame_;
    transform_msg_odom_map.transform.translation.x = initial_position_x;
    transform_msg_odom_map.transform.translation.y = initial_position_y;
    transform_msg_odom_map.transform.translation.z = initial_position_z;
    transform_msg_odom_map.transform.rotation.x = initial_angle_x;
    transform_msg_odom_map.transform.rotation.y = initial_angle_y;
    transform_msg_odom_map.transform.rotation.z = initial_angle_z;
    transform_msg_odom_map.transform.rotation.w = initial_angle_w;

    static_broadcaster_->sendTransform(transform_msg_odom_map);

    if (config_.max_range < config_.min_range) {
        RCLCPP_WARN(get_logger(), "[WARNING] max_range is smaller than min_range, settng min_range to 0.0");
        config_.min_range = 0.0;
    }
    std::cout<<"Base Frame: "<<base_frame_<<std::endl;
    std::cout<<"odom_frame_: "<<odom_frame_<<std::endl;
    std::cout<<" initial_position_x: "<<initial_position_x<<std::endl;
    std::cout<<"publish_odom_tf_: "<<publish_odom_tf_<<std::endl;
    // clang-format on

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kiss_icp_merged_cloud", rclcpp::SensorDataQoS(),
        std::bind(&OdometryServer::RegisterFrame, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::Trigger>(
        "stop_node", std::bind(&OdometryServer::handleTriggerService, this,
                                     std::placeholders::_1, std::placeholders::_2));

    // Initialize publishers
    rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/kiss/odometry", qos);
    traj_publisher_ = create_publisher<nav_msgs::msg::Path>("/kiss/trajectory", qos);
    path_msg_.header.frame_id = odom_frame_;
    if (publish_debug_clouds_) {
        frame_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/frame", qos);
        kpoints_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/keypoints", qos);
        map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/kiss/local_map", qos);
    }

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_buffer_->setUsingDedicatedThread(true);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

    RCLCPP_INFO(this->get_logger(), "KISS-ICP ROS 2 odometry node initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_->_frameExists(source_frame) &&  //
        tf2_buffer_->_frameExists(target_frame) &&  //
        tf2_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, &err_msg)) {
        try {
            auto tf = tf2_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        }
    }
    RCLCPP_WARN(this->get_logger(), "Failed to find tf. Reason=%s", err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using KISS, ego-centric to the LiDAR
    const Sophus::SE3d kiss_pose = odometry_.poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint
    // frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return kiss_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * kiss_pose * cloud2base.inverse();
    }();

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);
    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(frame, keypoints, msg->header.stamp, cloud_frame_id);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const rclcpp::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Broadcast the tf ---
    if (publish_odom_tf_) {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_->sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_->publish(path_msg_);

    // publish odometry msg
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_->publish(std::move(odom_msg));
}

void OdometryServer::PublishClouds(const std::vector<Eigen::Vector3d> frame,
                                   const std::vector<Eigen::Vector3d> keypoints,
                                   const rclcpp::Time &stamp,
                                   const std::string &cloud_frame_id) {
    std_msgs::msg::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // Publish map
    const auto kiss_map = odometry_.LocalMap();

    if (!publish_odom_tf_) {
        // debugging happens in an egocentric world
        std_msgs::msg::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        frame_publisher_->publish(std::move(EigenToPointCloud2(frame, cloud_header)));
        kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, cloud_header)));
        map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, odom_header)));

        return;
    }

    // If transmitting to tf tree we know where the clouds are exactly
    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    frame_publisher_->publish(std::move(EigenToPointCloud2(frame, cloud2odom, odom_header)));
    kpoints_publisher_->publish(std::move(EigenToPointCloud2(keypoints, cloud2odom, odom_header)));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, cloud2base, odom_header)));
    } else {
        map_publisher_->publish(std::move(EigenToPointCloud2(kiss_map, odom_header)));
    }
}
 // namespace kiss_icp_ros
bool OdometryServer::handleTriggerService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Trigger service called.");

    // Customize the response message
    response->message = "Service triggered successfully.";
    rclcpp::shutdown();

    return true;
  }

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kiss_icp_ros::OdometryServer)

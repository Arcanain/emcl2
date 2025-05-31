// SPDX-FileCopyrightText: 2022 Ryuichi Ueda <ryuichiueda@gmail.com>
// SPDX-License-Identifier: LGPL-3.0-or-later
// CAUTION: Some lines came from amcl (LGPL).

#ifndef EMCL2__EMCL2_NODE_H_
#define EMCL2__EMCL2_NODE_H_

#include "emcl2/ExpResetMcl2.h"
#include "emcl2/LikelihoodFieldMap.h"
#include "emcl2/OdomModel.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace emcl2
{
class EMcl2Node : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit EMcl2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~EMcl2Node() override;

    // Main processing loop
    void loop();
    // Retrieve odometry update frequency (Hz)
    int getOdomFreq() const;

    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
    // Particle filter instance
    std::shared_ptr<ExpResetMcl2> pf_;

    // Lifecycle-aware publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr alpha_pub_;

    // Subscriptions & services
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;

    // Timer for loop
    rclcpp::TimerBase::SharedPtr loop_timer_;
	rclcpp::Time scan_time_stamp_;

    // Frame identifiers
    std::string footprint_frame_id_;
    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::string scan_frame_id_;
    std::string base_frame_id_;

    // TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    tf2::Transform latest_tf_;

    // Clock & timing
    rclcpp::Clock ros_clock_;
    int odom_freq_;
    double transform_tolerance_;

    // State flags and variables
    bool init_pf_;
    bool init_request_;
    bool initialpose_receive_;
    bool simple_reset_request_;
    bool scan_receive_;
    bool map_receive_;
    double init_x_, init_y_, init_t_;

    nav_msgs::msg::OccupancyGrid map_;

    // Initialization helpers
    void declareParameter();
    void initCommunication();
    void initTF();
    void initPF();
    std::shared_ptr<LikelihoodFieldMap> initMap();
    std::shared_ptr<OdomModel> initOdometry();

    // Callback handlers
    void receiveMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    bool cbSimpleReset(const std_srvs::srv::Empty::Request::ConstSharedPtr,
                       std_srvs::srv::Empty::Response::SharedPtr);
    void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

    // Publishing helpers
    void publishPose(double x, double y, double t,
                     double x_dev, double y_dev, double t_dev,
                     double xy_cov, double yt_cov, double tx_cov);
    void publishOdomFrame(double x, double y, double t);
    void publishParticles();

    // Utility functions
    bool getOdomPose(double & x, double & y, double & yaw);
    bool getLidarPose(double & x, double & y, double & yaw, bool & inv);
};

}  // namespace emcl2

#endif  // EMCL2__EMCL2_NODE_H_

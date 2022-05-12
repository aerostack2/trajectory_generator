/*!*******************************************************************************************
 *  \file       dynamic_trajectory_generator.hpp
 *  \brief      dynamic_trajectory_generator header file.
 *  \authors    Miguel Fernández Cortizas
 *              Pedro Arias Pérez
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef TRAJECTORY_GENERATOR_HPP_
#define TRAJECTORY_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>

#include "as2_core/node.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/utils.hpp"
#include "as2_core/tf_utils.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2/LinearMath/Quaternion.h>

#define WAYPOINTS_TOPIC "motion_reference/waypoints"
#define PATH_DEBUG_TOPIC "debug/traj_generated"
#define REF_TRAJ_TOPIC "debug/ref_traj_point"

class TrajectoryGenerator : public as2::Node
{
public:
  TrajectoryGenerator();
  ~TrajectoryGenerator(){};
  void setup();
  void run();

private:
  /** Services **/
  rclcpp::Service<as2_msgs::srv::SendTrajectoryWaypoints>::SharedPtr set_trajectory_waypoints_srv_;
  rclcpp::Service<as2_msgs::srv::SendTrajectoryWaypoints>::SharedPtr add_trajectory_waypoints_srv_;
  rclcpp::Service<as2_msgs::srv::SetSpeed>::SharedPtr set_speed_srv_;
  /** Subscriptions **/
  rclcpp::Subscription<as2_msgs::msg::PoseStampedWithID>::SharedPtr mod_waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr waypoints_sub_;
  /** Publishers **/
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_point_pub_;

  bool evaluate_trajectory_ = false;
  bool has_odom_ = false;

  dynamic_traj_generator::DynamicTrajectory trajectory_generator_;
  std::string frame_id_;
  int yaw_mode_ = 0;
  float begin_traj_yaw_ = 0.0f;
  std::vector<double> v_positions_;
  std::vector<double> v_velocities_;
  std::vector<double> v_accelerations_;

  dynamic_traj_generator::References references_;
  nav_msgs::msg::Odometry current_state_;

  std::thread plot_thread_;

  bool evaluateTrajectory(double _eval_time);
  void updateState();

  /** Publish **/
  void publishTrajectory();

  /** Services Callbacks **/
  void setTrajectoryWaypointsSrvCall(
      const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request> _request,
      std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response> _response);
  void addTrajectoryWaypointsSrvCall(
      const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request> _request,
      std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response> _response);
  void setSpeedSrvCall(const std::shared_ptr<as2_msgs::srv::SetSpeed::Request> _request,
                       std::shared_ptr<as2_msgs::srv::SetSpeed::Response> _response);
  /** Topic Callbacks **/
  void modifyWaypointCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg);
  void waypointsCallback(const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg);

  /** Debug functions **/
  void plotTrajectory();
  void plotTrajectoryThread();
  void plotRefTrajPoint();
};

/** Auxiliar Functions **/
double extractYawFromQuat(const geometry_msgs::msg::Quaternion &quat);

void generateDynamicPoint(const as2_msgs::msg::PoseStampedWithID &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(const geometry_msgs::msg::PoseStamped &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(const nav_msgs::msg::Odometry &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point);

#endif // TRAJECTORY_GENERATOR_HPP_

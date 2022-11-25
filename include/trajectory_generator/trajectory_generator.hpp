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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>

#include "as2_behavior/behavior_server.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_core/utils/tf_utils.hpp"
#include "as2_msgs/action/trajectory_generator.hpp"
#include "as2_msgs/msg/traj_gen_info.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"
#include "as2_msgs/msg/trajectory_waypoints_with_id.hpp"
#include "as2_msgs/srv/send_trajectory_waypoints.hpp"
#include "as2_msgs/srv/set_speed.hpp"
#include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
#include "dynamic_trajectory_generator/dynamic_waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_reference_handlers/trajectory_motion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define WAYPOINTS_TOPIC "motion_reference/waypoints"
#define PATH_DEBUG_TOPIC "debug/traj_generated"
#define REF_TRAJ_TOPIC "debug/ref_traj_point"

class TrajectoryGeneratorBehavior : public as2_behavior::BehaviorServer<
                                        as2_msgs::action::TrajectoryGenerator> {
 public:
  TrajectoryGeneratorBehavior();
  ~TrajectoryGeneratorBehavior(){};

 private:
  /** Subscriptions **/
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_sub_;

  /** Dynamic trajectory generator library */
  // dynamic_traj_generator::DynamicTrajectory trajectory_generator_;
  std::shared_ptr<dynamic_traj_generator::DynamicTrajectory>
      trajectory_generator_;

  /** Handlers **/
  as2::motionReferenceHandlers::TrajectoryMotion motion_handler;
  as2::tf::TfHandler tf_handler_;

  /** Parameters */
  // Parameters
  std::string base_link_frame_id_;
  std::string odom_frame_id_;

  // Behavior action parameters
  as2_msgs::action::TrajectoryGenerator::Goal goal_;
  as2_msgs::action::TrajectoryGenerator::Feedback feedback_;
  as2_msgs::action::TrajectoryGenerator::Result result_;

  // Yaw
  bool has_yaw_from_topic_ = false;
  float yaw_from_topic_ = 0.0f;

  // State
  geometry_msgs::msg::PoseStamped current_state_pose_;
  geometry_msgs::msg::TwistStamped current_state_twist_;

  // Debug
  bool enable_debug_;
  bool has_odom_ = false;

 private:
  /** As2 Behavior methods **/
  bool on_activate(
      std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> goal)
      override;

  bool on_modify(
      std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> goal)
      override {
    return false;
  };

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    return false;
  };

  bool on_pause(const std::shared_ptr<std::string> &message) override {
    return false;
  };

  bool on_resume(const std::shared_ptr<std::string> &message) override {
    return false;
  };

  as2_behavior::ExecutionStatus on_run(
      const std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal>
          &goal,
      std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Feedback>
          &feedback_msg,
      std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Result>
          &result_msg) override {
    return as2_behavior::ExecutionStatus::SUCCESS;
  };

  void on_excution_end(const as2_behavior::ExecutionStatus &state) override{};

  /** Topic Callbacks **/
  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void yawCallback(const std_msgs::msg::Float32::SharedPtr _msg);

  /** Trajectory generator functions */
  void setup();
  bool evaluateTrajectory(double _eval_time);

 private:
  /** For debuging **/

  /** Debug publishers **/
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_point_pub;
  rclcpp::Publisher<as2_msgs::msg::TrajGenInfo>::SharedPtr traj_gen_info_pub_;

  /** Debug functions **/
  // void plotTrajectory();
  // void plotTrajectoryThread();
  // void plotRefTrajPoint();
  // void publishTrajGenInfo();
  // void stop() {
  //   if (plot_thread_.joinable()) {
  //     plot_thread_.join();
  //     RCLCPP_INFO(this->get_logger(), "Plot thread joined");
  //   }
  //   trajectory_generator_.reset();
  //   traj_gen_info_msg_.active_status = as2_msgs::msg::TrajGenInfo::STOPPED;
  //   RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator stopped");
  // };
};

/** Auxiliar Functions **/
double extractYawFromQuat(const geometry_msgs::msg::Quaternion &quat);

void generateDynamicPoint(
    const as2_msgs::msg::PoseStampedWithID &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(
    const geometry_msgs::msg::PoseStamped &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);
void generateDynamicPoint(
    const nav_msgs::msg::Odometry &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point);

#endif  // TRAJECTORY_GENERATOR_HPP_

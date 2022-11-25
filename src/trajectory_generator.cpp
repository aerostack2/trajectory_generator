/*!*******************************************************************************************
 *  \file       trajectory_generator.cpp
 *  \brief      trajectory_generator implementation file.
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

#include "trajectory_generator.hpp"

TrajectoryGeneratorBehavior::TrajectoryGeneratorBehavior()
    : as2_behavior::BehaviorServer<as2_msgs::action::TrajectoryGenerator>(
          "TrajectoryGenerator"),
      motion_handler(this),
      tf_handler_(this) {
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist,
      as2_names::topics::self_localization::qos,
      std::bind(&TrajectoryGeneratorBehavior::state_callback, this,
                std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "traj_gen/yaw", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryGeneratorBehavior::yawCallback, this,
                std::placeholders::_1));

  /** Debug publishers **/
  ref_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      REF_TRAJ_TOPIC, 1);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(PATH_DEBUG_TOPIC, 1);

  traj_gen_info_pub_ = this->create_publisher<as2_msgs::msg::TrajGenInfo>(
      as2_names::topics::motion_reference::traj_gen_info, 1);
  return;
}

void TrajectoryGeneratorBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  if (!trajectory_generator_) {
    return;
  }

  try {
    auto [pose_msg, twist_msg] = tf_handler_.getState(
        *_twist_msg, odom_frame_id_, odom_frame_id_, base_link_frame_id_);

    if (!has_odom_) {
      RCLCPP_INFO(this->get_logger(), "State callback working");
      has_odom_ = true;
    }

    current_state_pose_ = pose_msg;
    current_state_twist_ = twist_msg;

    trajectory_generator_->updateVehiclePosition(
        Eigen::Vector3d(current_state_pose_.pose.position.x,
                        current_state_pose_.pose.position.y,
                        current_state_pose_.pose.position.z));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
  }
  return;
}

void TrajectoryGeneratorBehavior::yawCallback(
    const std_msgs::msg::Float32::SharedPtr _msg) {
  has_yaw_from_topic_ = true;
  yaw_from_topic_ = _msg->data;
}

bool TrajectoryGeneratorBehavior::on_activate(
    std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> goal) {
  setup();
  return true;
}

void TrajectoryGeneratorBehavior::setup() {
  trajectory_generator_ =
      std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  has_yaw_from_topic_ = false;
  has_odom_ = false;
  return;
}
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
          as2_names::actions::behaviours::trajectorygenerator),
      trajectory_motion_handler_(this),
      hover_motion_handler_(this),
      tf_handler_(this) {
  desired_frame_id_ = as2::tf::generateTfName(this, "odom");
  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");

  trajectory_generator_ =
      std::make_shared<dynamic_traj_generator::DynamicTrajectory>();

  state_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::self_localization::twist,
      as2_names::topics::self_localization::qos,
      std::bind(&TrajectoryGeneratorBehavior::stateCallback, this,
                std::placeholders::_1));

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "traj_gen/yaw", rclcpp::SensorDataQoS(),
      std::bind(&TrajectoryGeneratorBehavior::yawCallback, this,
                std::placeholders::_1));

  /** For faster waypoint modified */
  mod_waypoint_sub_ =
      this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
          as2_names::topics::motion_reference::modify_waypoint,
          as2_names::topics::motion_reference::qos_waypoint,
          std::bind(&TrajectoryGeneratorBehavior::modifyWaypointCallback, this,
                    std::placeholders::_1));

  /** Debug publishers **/
  ref_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      REF_TRAJ_TOPIC, 1);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(PATH_DEBUG_TOPIC, 1);
  return;
}

void TrajectoryGeneratorBehavior::stateCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
  try {
    geometry_msgs::msg::PoseStamped pose_msg =
        tf_handler_.getPoseStamped(desired_frame_id_, base_link_frame_id_,
                                   tf2_ros::fromMsg(_twist_msg->header.stamp));

    if (!has_odom_) {
      RCLCPP_INFO(this->get_logger(), "State callback working");
      has_odom_ = true;
    }

    // if (!trajectory_generator_) {
    //   return;
    // }

    current_yaw_ = as2::frame::getYawFromQuaternion(pose_msg.pose.orientation);
    trajectory_generator_->updateVehiclePosition(
        Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                        pose_msg.pose.position.z));
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
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal accepted");
  if (goal->trajectory_waypoints.header.frame_id != desired_frame_id_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Goal frame_id %s is not the same as desired frame_id %s",
                 goal->trajectory_waypoints.header.frame_id.c_str(),
                 desired_frame_id_.c_str());
    return false;
  }

  if (!has_odom_) {
    RCLCPP_ERROR(this->get_logger(), "No odometry information available");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal accepted");
  setup();

  // Generate vector of waypoints for trajectory generator, from
  // as2_msgs/PoseStampedWithId to
  // dynamic_traj_generator::DynamicWaypoint::Vector
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(goal->trajectory_waypoints.poses.size());
  for (auto &waypoint : goal->trajectory_waypoints.poses) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  // Set waypoints to trajectory generator
  trajectory_generator_->setWaypoints(waypoints_to_set);

  yaw_mode_ = goal->trajectory_waypoints.yaw;
  max_speed_ = goal->trajectory_waypoints.max_speed;
  return true;
}

void TrajectoryGeneratorBehavior::setup() {
  // trajectory_generator_ =
  //     std::make_shared<dynamic_traj_generator::DynamicTrajectory>();
  has_yaw_from_topic_ = false;
  has_odom_ = false;
  first_run_ = true;

  init_yaw_angle_ = current_yaw_;
  traj_command_.position = Eigen::Vector3d::Zero();
  traj_command_.velocity = Eigen::Vector3d::Zero();
  traj_command_.acceleration = Eigen::Vector3d::Zero();
}

bool TrajectoryGeneratorBehavior::on_modify(
    std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator goal modified");
  // Generate vector of waypoints for trajectory generator, from
  // as2_msgs/PoseStampedWithId to
  // dynamic_traj_generator::DynamicWaypoint::Vector
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(goal->trajectory_waypoints.poses.size());
  for (auto &waypoint : goal->trajectory_waypoints.poses) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    trajectory_generator_->modifyWaypoint(
        dynamic_waypoint.getName(), dynamic_waypoint.getCurrentPosition());

    RCLCPP_INFO(this->get_logger(), "waypoint[%s] added: (%.2f, %.2f, %.2f)",
                dynamic_waypoint.getName().c_str(),
                dynamic_waypoint.getOriginalPosition().x(),
                dynamic_waypoint.getOriginalPosition().y(),
                dynamic_waypoint.getOriginalPosition().z());  // DEBUG
  }

  return true;
}

/** For faster waypoint modified */
void TrajectoryGeneratorBehavior::modifyWaypointCallback(
    const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg) {
  RCLCPP_DEBUG(this->get_logger(),
               "Callback Waypoint[%s] to modify has been received",
               _msg->id.c_str());

  // if (!trajectory_generator_) {
  //   RCLCPP_WARN(
  //       this->get_logger(),
  //       "No trajectory generator available start trajectory generator first");
  // }
  dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
  Eigen::Vector3d position;
  position.x() = _msg->pose.position.x;
  position.y() = _msg->pose.position.y;
  position.z() = _msg->pose.position.z;
  trajectory_generator_->modifyWaypoint(_msg->id, position);
}

bool TrajectoryGeneratorBehavior::on_deactivate(
    const std::shared_ptr<std::string> &message) {
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator cancelled");
  return true;
}

bool TrajectoryGeneratorBehavior::on_pause(
    const std::shared_ptr<std::string> &message) {
  RCLCPP_WARN(this->get_logger(),
              "TrajectoryGenerator can not be paused, try "
              "to cancel it and start a new one");

  *message.get() = std::string(
      "TrajectoryGenerator can not be paused, try to cancel it and "
      "start a new one");

  // TODO: if we pause the trajectory generator, we should be able to resume it
  // later. But the future vehicle state should be taken into account, because
  // between pause and resume the vehicle could have moved
  return false;
}

bool TrajectoryGeneratorBehavior::on_resume(
    const std::shared_ptr<std::string> &message) {
  return false;
}

void TrajectoryGeneratorBehavior::on_excution_end(
    const as2_behavior::ExecutionStatus &state) {
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator end");
  // trajectory_generator_.reset();
  hover_motion_handler_.sendHover();
  return;
};

as2_behavior::ExecutionStatus TrajectoryGeneratorBehavior::on_run(
    const std::shared_ptr<const as2_msgs::action::TrajectoryGenerator::Goal>
        &goal,
    std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Feedback>
        &feedback_msg,
    std::shared_ptr<as2_msgs::action::TrajectoryGenerator::Result>
        &result_msg) {
  RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator running");
  bool publish_trajectory = false;
  if (first_run_) time_zero_ = this->now();
  rclcpp::Duration eval_time = this->now() - time_zero_;

  RCLCPP_INFO(this->get_logger(), "Time zero: %f", time_zero_.seconds());
  RCLCPP_INFO(this->get_logger(), "Current time: %f", this->now().seconds());
  RCLCPP_INFO(this->get_logger(), "Eval time: %f", eval_time.seconds());

  if (trajectory_generator_->getMaxTime() + 0.2 < eval_time.seconds() &&
      !first_run_) {
    RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator finished");
    return as2_behavior::ExecutionStatus::SUCCESS;
  }

  if (first_run_) {
    publish_trajectory = evaluateTrajectory(0);
    if (publish_trajectory) first_run_ = false;
  } else {
    publish_trajectory = evaluateTrajectory(eval_time.seconds());
  }

  if (!publish_trajectory) {
    // TODO: When trajectory_generator_->evaluateTrajectory == False?
    return as2_behavior::ExecutionStatus::FAILURE;
  }

  if (enable_debug_) {
    plotRefTrajPoint();
    if (trajectory_generator_->getWasTrajectoryRegenerated()) {
      RCLCPP_DEBUG(this->get_logger(), "Plot trajectory");
      plotTrajectory();
    }
  }

  trajectory_motion_handler_.sendTrajectoryCommandWithYawAngle(
      desired_frame_id_, yaw_angle_, traj_command_.position,
      traj_command_.velocity, traj_command_.acceleration);
  return as2_behavior::ExecutionStatus::RUNNING;
}

bool TrajectoryGeneratorBehavior::evaluateTrajectory(double _eval_time) {
  bool publish_trajectory = false;

  publish_trajectory =
      trajectory_generator_->evaluateTrajectory(_eval_time, traj_command_);

  switch (yaw_mode_.mode) {
    case as2_msgs::msg::YawMode::KEEP_YAW:
      yaw_angle_ = init_yaw_angle_;
      break;
    case as2_msgs::msg::YawMode::PATH_FACING:
      yaw_angle_ = computeYawAnglePathFacing();
      break;
    case as2_msgs::msg::YawMode::FIXED_YAW:
      yaw_angle_ = yaw_mode_.angle;
      break;
    case as2_msgs::msg::YawMode::YAW_FROM_TOPIC:
      if (has_yaw_from_topic_) {
        yaw_angle_ = yaw_from_topic_;
      } else {
        auto &clk = *this->get_clock();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), clk, 1000,
            "Yaw from topic not received yet, using last yaw angle");
        yaw_angle_ = init_yaw_angle_;
      }
      break;
    default:
      auto &clk = *this->get_clock();
      RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 5000,
                           "Unknown yaw mode, using keep yaw");
      yaw_angle_ = current_yaw_;
      break;
  }

  return publish_trajectory;
}

double TrajectoryGeneratorBehavior::computeYawAnglePathFacing() {
  if (fabs(traj_command_.velocity.x()) > 0.1 ||
      (traj_command_.velocity.y()) > 0.1) {
    return as2::frame::getVector2DAngle(traj_command_.velocity.x(),
                                        traj_command_.velocity.y());
  }
  return current_yaw_;
}

/** Debug functions **/

void TrajectoryGeneratorBehavior::plotTrajectory() {
  // launch async plot
  if (plot_thread_.joinable()) {
    plot_thread_.join();
  }
  plot_thread_ =
      std::thread(&TrajectoryGeneratorBehavior::plotTrajectoryThread, this);
}

void TrajectoryGeneratorBehavior::plotTrajectoryThread() {
  nav_msgs::msg::Path path_msg;
  const float step = 0.2;
  const float max_time = trajectory_generator_->getMaxTime();
  const float min_time = trajectory_generator_->getMinTime();
  dynamic_traj_generator::References refs;
  const int n_measures = (max_time - min_time) / step;
  auto time_stamp = this->now();
  path_msg.poses.reserve(n_measures);
  for (float time = min_time; time <= max_time; time += step) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = desired_frame_id_;
    pose_msg.header.stamp = time_stamp;
    trajectory_generator_->evaluateTrajectory(time, refs, true, true);
    pose_msg.pose.position.x = refs.position.x();
    pose_msg.pose.position.y = refs.position.y();
    pose_msg.pose.position.z = refs.position.z();
    path_msg.poses.emplace_back(pose_msg);
  }
  path_msg.header.frame_id = desired_frame_id_;
  path_msg.header.stamp = time_stamp;

  RCLCPP_INFO(this->get_logger(), "DEBUG: Plotting trajectory");
  path_pub_->publish(path_msg);
}

void TrajectoryGeneratorBehavior::plotRefTrajPoint() {
  visualization_msgs::msg::Marker point_msg;

  point_msg.header.frame_id = desired_frame_id_;
  point_msg.header.stamp = this->now();
  point_msg.type = visualization_msgs::msg::Marker::SPHERE;
  point_msg.action = visualization_msgs::msg::Marker::ADD;

  point_msg.color.r = 0.0f;
  point_msg.color.g = 0.0f;
  point_msg.color.b = 1.0f;
  point_msg.color.a = 1.0f;

  point_msg.scale.x = 0.2;
  point_msg.scale.y = 0.2;
  point_msg.scale.z = 0.2;

  point_msg.pose.position.x = traj_command_.position.x();
  point_msg.pose.position.y = traj_command_.position.y();
  point_msg.pose.position.z = traj_command_.position.z();

  ref_point_pub->publish(point_msg);
}

/** Auxiliar Functions **/
void generateDynamicPoint(
    const as2_msgs::msg::PoseStampedWithID &msg,
    dynamic_traj_generator::DynamicWaypoint &dynamic_point) {
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}
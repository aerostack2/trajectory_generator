#include "as2_trajectory_generator.hpp"

#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/clock.hpp>

As2TrajectoryGenerator::As2TrajectoryGenerator()
    : as2::Node("as2_trajectory_generator"),
      v_positions_(4),
      v_velocities_(4),
      v_accelerations_(4)
{
  set_trajectory_waypoints_srv_ = this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
      SET_WAYPOINTS_TOPIC, std::bind(&As2TrajectoryGenerator::setTrajectoryWaypointsSrvCall, this,
                                     std::placeholders::_1, // Corresponds to the 'request'  input
                                     std::placeholders::_2  // Corresponds to the 'response' input
                                     ));

  add_trajectory_waypoints_srv_ = this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
      ADD_WAYPOINTS_TOPIC, std::bind(&As2TrajectoryGenerator::addTrajectoryWaypointsSrvCall, this,
                                     std::placeholders::_1, // Corresponds to the 'request'  input
                                     std::placeholders::_2  // Corresponds to the 'response' input
                                     ));

  set_speed_srv_ = this->create_service<as2_msgs::srv::SetSpeed>(
      SET_SPEED_TOPIC, std::bind(&As2TrajectoryGenerator::setSpeedSrvCall, this,
                                 std::placeholders::_1, // Corresponds to the 'request'  input
                                 std::placeholders::_2  // Corresponds to the 'response' input
                                 ));

  mod_waypoint_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      MOD_WAYPOINTS_TOPIC, 1,
      std::bind(&As2TrajectoryGenerator::modifyWaypointCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ODOM_TOPIC, 1, std::bind(&As2TrajectoryGenerator::odomCallback, this, std::placeholders::_1));

  waypoints_sub_ = this->create_subscription<as2_msgs::msg::TrajectoryWaypoints>(
      WAYPOINTS_TOPIC, 1, std::bind(&As2TrajectoryGenerator::waypointsCallback, this, std::placeholders::_1));

  // trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
  //     this->generate_global_name(TRAJECTORY_TOPIC), 10);
  trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      TRAJECTORY_TOPIC, 10);

  // path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
  //     this->generate_global_name(PATH_DEBUG_TOPIC), 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      PATH_DEBUG_TOPIC, 1);
}

void As2TrajectoryGenerator::setup() {}

void As2TrajectoryGenerator::run()
{
  static bool first_time = true;
  static rclcpp::Time time_zero = rclcpp::Clock().now();
  static auto eval_time = time_zero - time_zero;
  static bool publish_trajectory = false;

  if (evaluate_trajectory_ && has_odom_)
  {
    if (first_time)
    {
      publish_trajectory = evaluateTrajectory(0);
      if (publish_trajectory)
        first_time = false;
      time_zero = rclcpp::Clock().now();
    }
    else
    {
      eval_time = rclcpp::Clock().now() - time_zero;
      publish_trajectory = evaluateTrajectory(eval_time.seconds());
    }

    if (publish_trajectory)
    {
      publishTrajectory();
      if (trajectory_generator_.getWasTrajectoryRegenerated())
      {
        RCLCPP_INFO(this->get_logger(), "Plot trajectory");
        plotTrajectory();
      }
    }
  }
}

bool As2TrajectoryGenerator::evaluateTrajectory(double _eval_time)
{
  bool publish_trajectory = false;
  publish_trajectory = trajectory_generator_.evaluateTrajectory(_eval_time, references_);

  for (int i = 0; i < 3; i++)
  {
    v_positions_[i] = references_.position[i];
    v_velocities_[i] = references_.velocity[i];
    v_accelerations_[i] = references_.acceleration[i];
  }

  switch (yaw_mode_)
  {
  case as2_msgs::msg::TrajectoryWaypoints::KEEP_YAW:
  {
    v_positions_[3] = begin_traj_yaw_ + M_PI / 2.0f; // FIXME: This is a hack to make the robot start
                                                     //        facing the correct direction
  }
  break;
  case as2_msgs::msg::TrajectoryWaypoints::PATH_FACING:
  {
    // RCLCPP_INFO(this->get_logger(), "PATH_FACING");
    static float prev_vx = references_.velocity.x();
    static float prev_vy = references_.velocity.y();
    if (fabs(references_.velocity.x()) > 0.01 || (references_.velocity.y()) > 0.01)
    {
      v_positions_[3] =
          -atan2f((double)references_.velocity.x(), (double)references_.velocity.y()) +
          M_PI / 2.0f;
      prev_vx = references_.velocity.x();
      prev_vy = references_.velocity.y();
    }
    else
    {
      v_positions_[3] = -atan2f((double)prev_vx, (double)prev_vy) + M_PI / 2.0f;
    }
  }
  break;
  case as2_msgs::msg::TrajectoryWaypoints::GENERATE_YAW_TRAJ:
  {
    v_positions_[3] = 0.0f;
    std::cerr << "YAW MODE NOT IMPLEMENTED YET" << std::endl;
  }
  break;

  default:
  {
    std::cerr << "YAW MODE NOT DEFINED" << std::endl;
  }
  break;
  }

  // RCLCPP_INFO(this->get_logger(), "ref z: %.3f", references_.position[2]);
  return publish_trajectory;
}

void As2TrajectoryGenerator::updateState()
{
  Eigen::Vector3d current_position(current_state_.pose.pose.position.x, current_state_.pose.pose.position.y, current_state_.pose.pose.position.z);

  // current_position.x() = current_state_.pose.pose.position.x;
  // current_position.y() = current_state_.pose.pose.position.y;
  // current_position.z() = current_state_.pose.pose.position.z;

  trajectory_generator_.updateVehiclePosition(current_position);
}

/*******************/
/** Topic Publish **/
/*******************/

void As2TrajectoryGenerator::publishTrajectory()
{
  trajectory_msgs::msg::JointTrajectoryPoint trajectoy_msg;

  trajectoy_msg.positions = v_positions_;
  trajectoy_msg.velocities = v_velocities_;
  trajectoy_msg.accelerations = v_accelerations_;

  // RCLCPP_INFO(this->get_logger(), "Publishing trajectory generated");
  trajectory_pub_->publish(trajectoy_msg);
  // RCLCPP_INFO(this->get_logger(), "Trajectory published successfully");
}

/************************/
/** Services Callbacks **/
/************************/

void As2TrajectoryGenerator::setTrajectoryWaypointsSrvCall(
    const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request> _request,
    std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response> _response)
{
  RCLCPP_INFO(this->get_logger(), "Waypoints to set has been received");
  _response->success = true; // TODO: Does it works?
  auto &waypoints_msg = *(_request.get());

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;

  yaw_mode_ = _request->waypoints.yaw_mode;

  waypoints_to_set.reserve(waypoints_msg.waypoints.poses.size());

  // waypoints_to_set.reserve(waypoints_msg.waypoints.poses.size() + 1); // TODO: Remove +1
  // dynamic_traj_generator::DynamicWaypoint current_state_point;
  // generateDynamicPoint(current_state_, current_state_point);
  // waypoints_to_set.emplace_back(current_state_point);

  for (auto waypoint : waypoints_msg.waypoints.poses)
  {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  begin_traj_yaw_ = extractYawFromQuat(current_state_.pose.pose.orientation);
  trajectory_generator_.setWaypoints(waypoints_to_set);
  evaluate_trajectory_ = true;
}

void As2TrajectoryGenerator::addTrajectoryWaypointsSrvCall(
    const std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Request> _request,
    std::shared_ptr<as2_msgs::srv::SendTrajectoryWaypoints::Response> _response)
{
  RCLCPP_INFO(this->get_logger(), "Waypoints to add has been received");
  _response->success = true;

  dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;

  for (auto waypoint : _request->waypoints.poses)
  {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    trajectory_generator_.appendWaypoint(dynamic_waypoint);
    RCLCPP_INFO(this->get_logger(), "waypoint[%s] added: (%.2f, %.2f, %.2f)", dynamic_waypoint.getName().c_str(), dynamic_waypoint.getOriginalPosition().x(), dynamic_waypoint.getOriginalPosition().y(), dynamic_waypoint.getOriginalPosition().z()); // DEBUG
  }
}

void As2TrajectoryGenerator::setSpeedSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetSpeed::Request> _request,
    std::shared_ptr<as2_msgs::srv::SetSpeed::Response> _response)
{
  float max_speed = _request->speed.speed;
  RCLCPP_INFO(this->get_logger(), "Speed (%.2f) to be set has been received", max_speed);
  if (max_speed <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Speed must be > 0.0 m/s");
    _response->success = false;
    return;
  }
  _response->success = true;

  trajectory_generator_.setSpeed(max_speed);
}

/*********************/
/** Topic Callbacks **/
/*********************/

void As2TrajectoryGenerator::modifyWaypointCallback(
    const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg)
{
  RCLCPP_INFO(this->get_logger(), "Waypoint[%s] to modify has been received", _msg->id.c_str());
  dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
  Eigen::Vector3d position;
  position.x() = _msg->pose.position.x;
  position.y() = _msg->pose.position.y;
  position.z() = _msg->pose.position.z;
  // generateDynamicPoint(*_msg, dynamic_waypoint);
  trajectory_generator_.modifyWaypoint(_msg->id, position);
}

void As2TrajectoryGenerator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg)
{
  // RCLCPP_INFO(this->get_logger(), "Odom callback working");
  if (!has_odom_)
  {
    RCLCPP_INFO(this->get_logger(), "Odom callback working");
    has_odom_ = true;
  }
  current_state_ = *(_msg.get());

  updateState();
}

void As2TrajectoryGenerator::waypointsCallback(
    const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg)
{
  RCLCPP_INFO(this->get_logger(), "Waypoints received");
  auto &waypoints_msg = *(_msg.get());

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(waypoints_msg.poses.size());
  // dynamic_traj_generator::DynamicWaypoint current_state_point;
  // generateDynamicPoint(current_state_, current_state_point);
  // waypoints_to_set.emplace_back(current_state_point);

  float max_speed = waypoints_msg.max_speed;
  yaw_mode_ = waypoints_msg.yaw_mode;
  // frame_id_ = waypoints_msg.header.frame_id;

  if (max_speed <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(), "Speed must be > 0.0 m/s");
    return;
  }

  trajectory_generator_.setSpeed(max_speed);

  for (auto waypoint : waypoints_msg.poses)
  {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  // DEBUG
  for (auto &dw : waypoints_to_set)
  {
    RCLCPP_INFO(this->get_logger(), "setting waypoint[%s]: (%.2f, %.2f, %.2f)", dw.getName().c_str(), dw.getOriginalPosition().x(), dw.getOriginalPosition().y(), dw.getOriginalPosition().z());
  }

  trajectory_generator_.setWaypoints(waypoints_to_set);
  evaluate_trajectory_ = true;
}

/********************/
/** Debug Funtions **/
/********************/

void As2TrajectoryGenerator::plotTrajectory()
{
  // launch async plot
  if (plot_thread_.joinable())
  {
    plot_thread_.join();
  }
  plot_thread_ = std::thread(&As2TrajectoryGenerator::plotTrajectoryThread, this);
}

void As2TrajectoryGenerator::plotTrajectoryThread()
{
  // while (!trajectory_generator_.getWasTrajectoryRegenerated())
  // {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  nav_msgs::msg::Path path_msg;
  const float step = 0.2;
  const float max_time = trajectory_generator_.getMaxTime();
  const float min_time = trajectory_generator_.getMinTime();
  dynamic_traj_generator::References refs;
  const int n_measures = (max_time - min_time) / step;
  auto time_stamp = rclcpp::Clock().now();
  path_msg.poses.reserve(n_measures);
  for (float time = min_time; time <= max_time; time += step)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id_;
    pose_msg.header.stamp = time_stamp;
    trajectory_generator_.evaluateTrajectory(time, refs, true, true);
    pose_msg.pose.position.x = refs.position.x();
    pose_msg.pose.position.y = refs.position.y();
    pose_msg.pose.position.z = refs.position.z();
    path_msg.poses.emplace_back(pose_msg);
  }
  path_msg.header.frame_id = frame_id_;
  path_msg.header.stamp = time_stamp;

  RCLCPP_INFO(this->get_logger(), "DEBUG: Plotting trajectory");
  path_pub_->publish(path_msg);
}

/*******************/
/** Aux Functions **/
/*******************/

double extractYawFromQuat(const geometry_msgs::msg::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 R(q);
  R.getRPY(roll, pitch, yaw);
  return yaw;
};

void generateDynamicPoint(const as2_msgs::msg::PoseStampedWithID &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

void generateDynamicPoint(const geometry_msgs::msg::PoseStamped &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  // dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

void generateDynamicPoint(const nav_msgs::msg::Odometry &msg,
                          dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  // dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.pose.position.x;
  position.y() = msg.pose.pose.position.y;
  position.z() = msg.pose.pose.position.z;
  dynamic_point.resetWaypoint(position);
}

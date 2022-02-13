#include "as2_trajectory_generator.hpp"

As2TrajectoryGenerator::As2TrajectoryGenerator()
    : as2::Node("as2_trajectory_generator"), v_positions_(4), v_velocities_(4), v_accelerations_(4)
{
  set_trajectory_waypoints_srv_ = this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
      SET_WAYPOINTS_TOPIC,
      std::bind(
          &As2TrajectoryGenerator::setTrajectoryWaypointsSrvCall, this,
          std::placeholders::_1, // Corresponds to the 'request'  input
          std::placeholders::_2  // Corresponds to the 'response' input
          ));

  add_trajectory_waypoints_srv_ = this->create_service<as2_msgs::srv::SendTrajectoryWaypoints>(
      ADD_WAYPOINTS_TOPIC,
      std::bind(
          &As2TrajectoryGenerator::addTrajectoryWaypointsSrvCall, this,
          std::placeholders::_1, // Corresponds to the 'request'  input
          std::placeholders::_2  // Corresponds to the 'response' input
          ));

  mod_waypoint_sub_ = this->create_subscription<as2_msgs::msg::PoseStampedWithID>(
      MOD_WAYPOINTS_TOPIC, 1, std::bind(&As2TrajectoryGenerator::modifyWaypointCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ODOM_TOPIC, 1, std::bind(&As2TrajectoryGenerator::odomCallback, this, std::placeholders::_1));
}

void As2TrajectoryGenerator::setup() {}

void As2TrajectoryGenerator::run()
{
  static bool first_time = true;
  static rclcpp::Time time_zero = rclcpp::Clock().now();
  static auto eval_time = time_zero - time_zero;
  static bool publish_trajectory;

  if (evaluate_trajectory_)
  {
    if (first_time)
    {
      time_zero = rclcpp::Clock().now();
      publish_trajectory = evaluateTrajectory(0);
      if (publish_trajectory)
        first_time = false;
    }
    else
    {
      eval_time = rclcpp::Clock().now() - time_zero;
      publish_trajectory = evaluateTrajectory(eval_time.seconds());
    }

    if (publish_trajectory)
      publishTrajectory();
  }
}

bool As2TrajectoryGenerator::evaluateTrajectory(double _eval_time)
{
  bool publish_trajectory = false;
  publish_trajectory = trajectory_generator_.evaluateTrajectory(_eval_time, references_);

  for (int i = 0; i < v_positions_.size(); i++)
  {
    v_positions_[i] = references_.position[i];
    v_velocities_[i] = references_.velocity[i];
    v_accelerations_[i] = references_.acceleration[i];
  }

  switch (yaw_mode_)
  {
  case as2_msgs::msg::TrajectoryWaypoints::KEEP_YAW:
  {
    v_positions_[3] = begin_traj_yaw_;
  }
  break;
  case as2_msgs::msg::TrajectoryWaypoints::PATH_FACING:
  {
    static float prev_vx = references_.velocity.x();
    static float prev_vy = references_.velocity.y();
    if (fabs(references_.velocity.x()) > 0.01 || (references_.velocity.y()) > 0.01)
    {
      v_positions_[3] = -atan2f((double)references_.velocity.x(), (double)references_.velocity.y()) + M_PI / 2.0f;
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

  return publish_trajectory;
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

  trajectory_pub_->publish(trajectoy_msg);
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
  waypoints_to_set.reserve(waypoints_msg.waypoints.poses.size() + 1); // TODO: Remove +1
  waypoints_to_set.emplace_back(drone_state_point_);

  for (auto waypoint : waypoints_msg.waypoints.poses)
  {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    generateDynamicPoint(waypoint, dynamic_waypoint);
    waypoints_to_set.emplace_back(dynamic_waypoint);
  }

  trajectory_generator_.setWaypoints(waypoints_to_set);
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
  }
}

void As2TrajectoryGenerator::setSpeedSrvCall(
    const std::shared_ptr<as2_msgs::srv::SetSpeed::Request> _request,
    std::shared_ptr<as2_msgs::srv::SetSpeed::Response> _response)
{

  float max_speed = _request->speed.speed;
  RCLCPP_INFO(this->get_logger(), "Speed (%n) to be set has been received", max_speed);
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

void As2TrajectoryGenerator::modifyWaypointCallback(const as2_msgs::msg::PoseStampedWithID::SharedPtr _msg)
{
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
  nav_msgs::msg::Odometry current_pose = *(_msg.get());
  generateDynamicPoint(current_pose, drone_state_point_);
}

void As2TrajectoryGenerator::waypointsCallback(const as2_msgs::msg::TrajectoryWaypoints::SharedPtr _msg)
{
  RCLCPP_INFO(this->get_logger(), "Trajectory received");
  auto &waypoints_msg = *(_msg.get());

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set;
  waypoints_to_set.reserve(waypoints_msg.poses.size() + 1); // TODO: Remove +1
  waypoints_to_set.emplace_back(drone_state_point_);

  float max_speed = waypoints_msg.max_speed;
  yaw_mode_ = waypoints_msg.yaw_mode;
  frame_id_ = waypoints_msg.header.frame_id;

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

  trajectory_generator_.setWaypoints(waypoints_to_set);
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

void generateDynamicPoint(const as2_msgs::msg::PoseStampedWithID &msg, dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.setCurrentPosition(position);
}

void generateDynamicPoint(const geometry_msgs::msg::PoseStamped &msg, dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  // dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.position.x;
  position.y() = msg.pose.position.y;
  position.z() = msg.pose.position.z;
  dynamic_point.setCurrentPosition(position);
}

void generateDynamicPoint(const nav_msgs::msg::Odometry &msg, dynamic_traj_generator::DynamicWaypoint &dynamic_point)
{
  // dynamic_point.setName(msg.id);
  Eigen::Vector3d position;
  position.x() = msg.pose.pose.position.x;
  position.y() = msg.pose.pose.position.y;
  position.z() = msg.pose.pose.position.z;
  dynamic_point.setCurrentPosition(position);
}
/********************/
/** Debug Funtions **/
/********************/

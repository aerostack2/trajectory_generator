#ifndef AS2_TRAJECTORY_GENERATOR_HPP_
#define AS2_TRAJECTORY_GENERATOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "as2_core/node.hpp"
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
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "as2_core/utils.hpp"
#include "as2_core/tf_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define SET_WAYPOINTS_TOPIC "set_trajectory_waypoints"
#define ADD_WAYPOINTS_TOPIC "add_trajectory_waypoints"
#define MOD_WAYPOINTS_TOPIC "mod_trajectory_waypoints"
#define SET_SPEED_TOPIC "set_speed"
#define ODOM_TOPIC "self_localization/odom"
#define WAYPOINTS_TOPIC "motion_reference/waypoints"
#define TRAJECTORY_TOPIC "motion_reference/trajectory"
#define PATH_DEBUG_TOPIC "debug/traj_generated"
#define REF_TRAJ_TOPIC "debug/ref_traj_point"

class As2TrajectoryGenerator : public as2::Node
{
public:
  As2TrajectoryGenerator();
  ~As2TrajectoryGenerator(){};
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
  // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ref_point_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ref_point_pub_;

  // trajectory_msgs::msg::JointTrajectoryPoint trajectory_msg_;

  bool evaluate_trajectory_ = false;
  bool has_odom_ = false;

  // nav_msgs::msg::Odometry current_pose_;
  // as2_msgs::msg::TrajectoryWaypointsWithID waypoints_to_set_;
  // dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_set_;
  // as2_msgs::msg::TrajectoryWaypoints waypoints_msgs_;

  dynamic_traj_generator::DynamicTrajectory trajectory_generator_;
  std::string frame_id_;
  int yaw_mode_ = 0;
  float begin_traj_yaw_ = 0.0f;
  std::vector<double> v_positions_;
  std::vector<double> v_velocities_;
  std::vector<double> v_accelerations_;

  dynamic_traj_generator::References references_;
  // dynamic_traj_generator::DynamicWaypoint drone_state_point_;
  nav_msgs::msg::Odometry current_state_;

  std::thread plot_thread_;

  bool evaluateTrajectory(double _eval_time);
  void updateState(); // updateVehiclePosition();

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

#endif // AS2_TRAJECTORY_GENERATOR_HPP_

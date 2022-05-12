// "Copyright [year] <Copyright Owner>"

#include "as2_core/core_functions.hpp"
#include "as2_trajectory_generator.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<As2TrajectoryGenerator>();
  node->preset_loop_frequency(50); // Node frequency for run and callbacks
  node->setup();
  // Node with only callbacks
  // as2::spinLoop(node);
  // Node with run
  as2::spinLoop(node, std::bind(&As2TrajectoryGenerator::run, node));

  rclcpp::shutdown();
  return 0;
}
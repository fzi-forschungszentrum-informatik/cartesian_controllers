// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    simulator_node.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/09/27
 *
 */
//-----------------------------------------------------------------------------



#include "rackki_learning/mujoco_simulator.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <chrono>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  auto simulator = std::thread(rackki_learning::MuJoCoSimulator::simulate);
  simulator.detach();

  auto node = rackki_learning::MuJoCoSimulator::getInstance().getNode();

  while (!node) // initialized
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

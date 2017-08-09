// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <cartesian_motion_controller/cartesian_motion_controller.h>

namespace position_controllers
{
  /**
   * @brief Cartesian motion controller that transforms end-effector target motion into commands for a chain of position interfaces.
   */
  typedef cartesian_motion_controller::CartesianMotionController<
    hardware_interface::PositionJointInterface> CartesianMotionController;
}

namespace velocity_controllers
{
  /**
   * @brief Cartesian motion controller that transforms end-effector target motion into commands for a chain of velocity interfaces.
   */
  typedef cartesian_motion_controller::CartesianMotionController<
    hardware_interface::VelocityJointInterface> CartesianMotionController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianMotionController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianMotionController, controller_interface::ControllerBase)

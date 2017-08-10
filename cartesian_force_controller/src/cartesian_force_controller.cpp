// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <cartesian_force_controller/cartesian_force_controller.h>

namespace position_controllers
{
  /**
   * @brief Cartesian compliance controller that implements Forward Dynamics Compliance Control (FDCC) [Scherzinger2017] on a position interface. 
   */
  typedef cartesian_force_controller::CartesianForceController<
    hardware_interface::PositionJointInterface> CartesianForceController;
}

namespace velocity_controllers
{
  /**
   * @brief Cartesian compliance controller that implements Forward Dynamics Compliance Control (FDCC) [Scherzinger2017] on a velocity interface. 
   */
  typedef cartesian_force_controller::CartesianForceController<
    hardware_interface::VelocityJointInterface> CartesianForceController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianForceController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianForceController, controller_interface::ControllerBase)

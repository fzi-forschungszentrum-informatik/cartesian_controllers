// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

namespace position_controllers
{
  /**
   * @brief Cartesian compliance controller that implements Forward Dynamics Compliance Control (FDCC) [Scherzinger2017] on a position interface. 
   */
  typedef cartesian_compliance_controller::CartesianComplianceController<
    hardware_interface::PositionJointInterface> CartesianComplianceController;
}

namespace velocity_controllers
{
  /**
   * @brief Cartesian compliance controller that implements Forward Dynamics Compliance Control (FDCC) [Scherzinger2017] on a velocity interface. 
   */
  typedef cartesian_compliance_controller::CartesianComplianceController<
    hardware_interface::VelocityJointInterface> CartesianComplianceController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::CartesianComplianceController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::CartesianComplianceController, controller_interface::ControllerBase)

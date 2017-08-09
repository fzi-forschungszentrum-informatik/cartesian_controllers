// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    SpatialPIDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <cartesian_motion_controller/SpatialPIDController.h>

// Other
#include <string>

namespace cartesian_motion_controller
{

SpatialPIDController::SpatialPIDController()
{
}

ctrl::Vector6D SpatialPIDController::operator()(const ctrl::Vector6D& error, const ros::Duration& period)
{
  // Perform pid control separately on each Cartesian dimension
  for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
  {
    m_cmd(i) = m_pid_controllers[i].computeCommand(error[i],period);
  }
  return m_cmd;
}

bool SpatialPIDController::init(ros::NodeHandle& nh)
{
  // Initialize pid controllers for each Cartesian dimension
  for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
  {
    m_pid_controllers.push_back(
        control_toolbox::Pid());
  }

  // Load default controller gains
  std::string solver_config = nh.getNamespace() + "/pid_gains";

  m_pid_controllers[0].initParam(solver_config + "/trans_x");
  m_pid_controllers[1].initParam(solver_config + "/trans_y");
  m_pid_controllers[2].initParam(solver_config + "/trans_z");
  m_pid_controllers[3].initParam(solver_config + "/rot_x");
  m_pid_controllers[4].initParam(solver_config + "/rot_y");
  m_pid_controllers[5].initParam(solver_config + "/rot_z");

  return true;
}

} // namespace

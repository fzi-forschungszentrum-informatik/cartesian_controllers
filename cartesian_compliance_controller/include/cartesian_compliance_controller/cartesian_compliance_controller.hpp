// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>

// Other
#include <boost/algorithm/clamp.hpp>

namespace cartesian_compliance_controller
{

template <class HardwareInterface>
CartesianComplianceController<HardwareInterface>::
CartesianComplianceController()
: Base::CartesianControllerBase()
{
}

template <class HardwareInterface>
bool CartesianComplianceController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  Base::general_init(hw,nh);

  if (!nh.getParam("compliance_ref_link",m_compliance_ref_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/compliance_ref_link" << " from parameter server");
  }
  if (!nh.getParam("ft_sensor_ref_link",m_ft_sensor_ref_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/ft_sensor_ref_link" << " from parameter server");
  }
  if (!nh.getParam("target_frame",m_target_frame))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/target_frame" << " from parameter server");
  }

  return true;
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
starting(const ros::Time& time)
{
  Base::general_starting(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
stopping(const ros::Time& time)
{
  Base::stopping(time);
}

template <class HardwareInterface>
void CartesianComplianceController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Control the robot motion in such a way that the resulting net force
  // vanishes. This internal control needs some simulation time steps.
  const int steps = 10;
  for (int i = 0; i < steps; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    ros::Duration internal_period(0.02);

    // Compute the net force
    ctrl::Vector6D error = computeComplianceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error,internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianComplianceController<HardwareInterface>::
computeComplianceError()
{
}

}

#endif

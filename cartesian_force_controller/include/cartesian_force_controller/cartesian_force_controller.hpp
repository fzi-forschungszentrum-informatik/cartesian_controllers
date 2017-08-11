// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_force_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_FORCE_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_FORCE_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_force_controller/cartesian_force_controller.h>

// Other
#include <boost/algorithm/clamp.hpp>

namespace cartesian_force_controller
{

template <class HardwareInterface>
CartesianForceController<HardwareInterface>::
CartesianForceController()
: Base::CartesianControllerBase()
{
}

template <class HardwareInterface>
bool CartesianForceController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  Base::init(hw,nh);

  if (!nh.getParam("ft_sensor_ref_link",m_ft_sensor_ref_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/ft_sensor_ref_link" << " from parameter server");
  }

  m_target_wrench_subscriber = nh.subscribe("target_wrench",2,&CartesianForceController<HardwareInterface>::targetWrenchCallback,this);
  m_ft_sensor_wrench_subscriber = nh.subscribe("ft_sensor_wrench",2,&CartesianForceController<HardwareInterface>::ftSensorWrenchCallback,this);

  return true;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
starting(const ros::Time& time)
{
  Base::starting(time);
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
stopping(const ros::Time& time)
{
  Base::stopping(time);
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
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
    ctrl::Vector6D error = computeForceError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error,internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
computeForceError()
{
  return m_target_wrench - m_ft_sensor_wrench;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  m_target_wrench = Base::displayInBaseLink(wrench,Base::m_end_effector_link);
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  m_ft_sensor_wrench = Base::displayInBaseLink(wrench,m_ft_sensor_ref_link);
}

}

#endif

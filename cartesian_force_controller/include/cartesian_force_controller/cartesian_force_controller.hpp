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
    return false;
  }

  m_signal_taring_server = nh.advertiseService("signal_taring",&CartesianForceController<HardwareInterface>::signalTaringCallback,this);
  m_target_wrench_subscriber = nh.subscribe("target_wrench",2,&CartesianForceController<HardwareInterface>::targetWrenchCallback,this);
  m_ft_sensor_wrench_subscriber = nh.subscribe("ft_sensor_wrench",2,&CartesianForceController<HardwareInterface>::ftSensorWrenchCallback,this);

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();
  m_weight_force.setZero();
  m_grav_comp_during_taring.setZero();
  m_center_of_gravity.setZero();

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

template <>
void CartesianForceController<hardware_interface::VelocityJointInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Simulate only one step forward.
  // The constant simulation time adds to solver stability.
  ros::Duration internal_period(0.02);

  ctrl::Vector6D error = computeForceError();

  Base::computeJointControlCmds(error,internal_period);

  Base::writeJointControlCmds();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
computeForceError()
{
  // Superimpose target wrench and sensor wrench in base frame
  return Base::displayInBaseLink(m_ft_sensor_wrench,m_ft_sensor_ref_link)
    + Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link)
    + compensateGravity();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
compensateGravity()
{
  ctrl::Vector6D compensating_force = ctrl::Vector6D::Zero();

  // Compute actual gravity effects in sensor frame
  ctrl::Vector6D tmp = Base::displayInTipLink(m_weight_force,m_ft_sensor_ref_link);
  tmp.tail<3>() = m_center_of_mass.cross(tmp.head<3>()); // M = r x F

  // Display in base link
  m_weight_force = Base::displayInBaseLink(tmp,m_ft_sensor_ref_link);

  // Add actual gravity compensation
  compensating_force -= m_weight_force;

  // Remove deprecated terms from moment of taring
  compensating_force -= m_grav_comp_during_taring;

  return compensating_force;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  m_target_wrench[0] = wrench.wrench.force.x;
  m_target_wrench[1] = wrench.wrench.force.y;
  m_target_wrench[2] = wrench.wrench.force.z;
  m_target_wrench[3] = wrench.wrench.torque.x;
  m_target_wrench[4] = wrench.wrench.torque.y;
  m_target_wrench[5] = wrench.wrench.torque.z;
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench)
{
  m_ft_sensor_wrench[0] = wrench.wrench.force.x;
  m_ft_sensor_wrench[1] = wrench.wrench.force.y;
  m_ft_sensor_wrench[2] = wrench.wrench.force.z;
  m_ft_sensor_wrench[3] = wrench.wrench.torque.x;
  m_ft_sensor_wrench[4] = wrench.wrench.torque.y;
  m_ft_sensor_wrench[5] = wrench.wrench.torque.z;
}

template <class HardwareInterface>
bool CartesianForceController<HardwareInterface>::
signalTaringCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  // Compute current gravity effects in sensor frame
  ctrl::Vector6D tmp = Base::displayInTipLink(m_weight_force,m_ft_sensor_ref_link);
  tmp.tail<3>() = m_center_of_mass.cross(tmp.head<3>()); // M = r x F

  // Taring the sensor is like adding a virtual force that exactly compensates
  // the weight force.
  tmp = -tmp;

  // Display in base link
  m_grav_comp_during_taring = Base::displayInBaseLink(tmp,m_ft_sensor_ref_link);

  res.message = "Got it.";
  res.success = true;
  return true;
}

}

#endif

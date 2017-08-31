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

// Debug
#include <debug_toolbox/RosDebugPublish.h>

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

  std::map<std::string, double> damping;
  if (!nh.getParam("damping",damping))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/damping" << " from parameter server");
    return false;
  }

  m_target_wrench_subscriber = nh.subscribe("target_wrench",2,&CartesianForceController<HardwareInterface>::targetWrenchCallback,this);
  m_ft_sensor_wrench_subscriber = nh.subscribe("ft_sensor_wrench",2,&CartesianForceController<HardwareInterface>::ftSensorWrenchCallback,this);

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  // Initialize damping
  ctrl::Vector6D tmp;
  tmp[0] = damping["trans"];
  tmp[1] = damping["trans"];
  tmp[2] = damping["trans"];
  tmp[3] = damping["rot"];
  tmp[4] = damping["rot"];
  tmp[5] = damping["rot"];
  m_damping = tmp.asDiagonal();

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

template <class HardwareInterface>
ctrl::Vector6D CartesianForceController<HardwareInterface>::
computeForceError()
{
  // Target - current in base orientation
  ctrl::Vector6D result= Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link)
         - Base::displayInBaseLink(m_ft_sensor_wrench,m_ft_sensor_ref_link);

         // Global damping always oposes motion
         ctrl::Vector6D damping_force = - m_damping * Base::m_forward_dynamics_solver.getEndEffectorVel();
         result += damping_force;

         // Debug
         ROS_DEBUG_PUBLISH(damping_force,"damping_force", Base::m_robot_base_link);

         return result;
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

}

#endif

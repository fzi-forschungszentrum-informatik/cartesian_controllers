////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

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

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_signal_taring_server = nh.advertiseService("signal_taring",&CartesianForceController<HardwareInterface>::signalTaringCallback,this);
  m_target_wrench_subscriber = nh.subscribe("target_wrench",2,&CartesianForceController<HardwareInterface>::targetWrenchCallback,this);
  m_ft_sensor_wrench_subscriber = nh.subscribe("ft_sensor_wrench",2,&CartesianForceController<HardwareInterface>::ftSensorWrenchCallback,this);

  // Initialize tool and gravity compensation
  std::map<std::string, double> gravity;
  if (!nh.getParam("gravity",gravity))
  {
    ROS_INFO_STREAM("Failed to load " << nh.getNamespace() + "/gravity" << " from parameter server");
    gravity["x"] = 0;
    gravity["y"] = 0;
    gravity["z"] = 0;
  }
  std::map<std::string, double> tool;
  if (!nh.getParam("tool",tool))
  {
    ROS_INFO_STREAM("Failed to load " << nh.getNamespace() + "/tool" << " from parameter server");
    tool["com_x"] = 0;
    tool["com_y"] = 0;
    tool["com_z"] = 0;
  }
  // In sensor frame
  m_center_of_mass = ctrl::Vector3D(tool["com_x"],tool["com_y"],tool["com_z"]);

  // In base frame
  m_weight_force.head<3>() = tool["mass"] * ctrl::Vector3D(gravity["x"],gravity["y"],gravity["z"]);
  m_weight_force.tail<3>() = ctrl::Vector3D::Zero();  // Update in control cycle
  m_grav_comp_during_taring = -m_weight_force;

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

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
  for (int i = 0; i < Base::m_iterations; ++i)
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
  return Base::displayInBaseLink(m_ft_sensor_wrench,m_new_ft_sensor_ref)
    + Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link)
    + compensateGravity();
}

template <class HardwareInterface>
void CartesianForceController<HardwareInterface>::
setFtSensorReferenceFrame(const std::string& new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_forward_dynamics_solver.getPositions());

  KDL::Frame sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      sensor_ref,
      m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      new_sensor_ref,
      m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
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
  KDL::Wrench tmp;
  tmp[0] = wrench.wrench.force.x;
  tmp[1] = wrench.wrench.force.y;
  tmp[2] = wrench.wrench.force.z;
  tmp[3] = wrench.wrench.torque.x;
  tmp[4] = wrench.wrench.torque.y;
  tmp[5] = wrench.wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;

  m_ft_sensor_wrench[0] = tmp[0];
  m_ft_sensor_wrench[1] = tmp[1];
  m_ft_sensor_wrench[2] = tmp[2];
  m_ft_sensor_wrench[3] = tmp[3];
  m_ft_sensor_wrench[4] = tmp[4];
  m_ft_sensor_wrench[5] = tmp[5];
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

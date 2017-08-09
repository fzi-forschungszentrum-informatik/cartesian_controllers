// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller_base.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_CONTROLLER_BASE_HPP_INCLUDED
#define CARTESIAN_CONTROLLER_BASE_HPP_INCLUDED

// Project
#include <cartesian_controller_base/cartesian_controller_base.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>

namespace cartesian_controller_base
{

template <class HardwareInterface>
CartesianControllerBase<HardwareInterface>::
CartesianControllerBase()
{
}

template <class HardwareInterface>
bool CartesianControllerBase<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;
  KDL::Chain  robot_chain;

  // Get controller specific configuration
  if (!nh.getParam("/robot_description",robot_description))
  {
    ROS_ERROR("Failed to load '/robot_description' from parameter server");
  }
  if (!nh.getParam("robot_base_link",m_robot_base_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
  }
  if (!nh.getParam("end_effector_link",m_end_effector_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    ROS_ERROR("Failed to parse urdf model from 'robot_description'");
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
  {
    const std::string error = ""
      "Failed to parse KDL tree from urdf model";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,robot_chain))
  {
    const std::string error = ""
      "Failed to parse robot chain from urdf model. "
      "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Get names of controllable joints from the parameter server
  if (!nh.getParam("joints",m_joint_names))
  {
    const std::string error = ""
    "Failed to load " + nh.getNamespace() + "/joints" + " from parameter server";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Get the joint handles to use in the control loop
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
  }

  // Initialize solver
  m_forward_dynamics_solver.init(robot_chain);

  // Initialize Cartesian pid controllers
  m_spatial_controller.init(nh);



  return true;
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
starting(const ros::Time& time)
{
  // Copy joint state to internal simulation
  m_forward_dynamics_solver.setStartState(m_joint_handles);
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
stopping(const ros::Time& time)
{
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
}

template <>
void CartesianControllerBase<hardware_interface::PositionJointInterface>::
writeJointControlCmds()
{
  // Take position commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(m_simulated_joint_motion.positions[i]);
  }
}

template <>
void CartesianControllerBase<hardware_interface::VelocityJointInterface>::
writeJointControlCmds()
{
  // Take velocity commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(m_simulated_joint_motion.velocities[i]);
  }
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period)
{
  // PID controlled system input
  m_cartesian_input = m_spatial_controller(error,period);

  m_simulated_joint_motion = m_forward_dynamics_solver.getJointControlCmds(
      period,
      m_cartesian_input);
}

} // namespace

#endif

// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    JointControllerAdapter.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/16
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <joint_to_cartesian_controller/JointControllerAdapter.h>

// ROS control
#include <joint_limits_interface/joint_limits_rosparam.h>

// Other
#include <stdexcept>

namespace joint_to_cartesian_controller
{

JointControllerAdapter::JointControllerAdapter()
{
}

bool JointControllerAdapter::init(const std::vector<std::string>& joint_names, ros::NodeHandle& nh)
{
  m_joint_names = joint_names;
  m_number_joints = m_joint_names.size();
  m_cmd.resize(m_number_joints);
  m_pos.resize(m_number_joints);
  m_vel.resize(m_number_joints);
  m_eff.resize(m_number_joints);

  // Initialize and register handles to the sensors
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_joint_state_handles.push_back(
        hardware_interface::JointStateHandle(
          m_joint_names[i],
          &m_pos[i],
          &m_vel[i],
          &m_eff[i]));

    m_state_interface.registerHandle(m_joint_state_handles[i]);
  }
  registerInterface(&m_state_interface);

  // Initialize and register handles to the actuators
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_joint_handles.push_back(
        hardware_interface::JointHandle(
          m_state_interface.getHandle(m_joint_names[i]),
          &m_cmd[i]));

    m_pos_interface.registerHandle(m_joint_handles[i]);
  }
  registerInterface(&m_pos_interface);

  // Read joint limits from param server
  joint_limits_interface::JointLimits limits;
  for (int i = 0; i < m_number_joints; ++i)
  {
    joint_limits_interface::getJointLimits(
        m_joint_names[i],
        nh,
        limits);
  }

  joint_limits_interface::SoftJointLimits soft_limits;

  // Initialize and register handles to the joint limits
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_limits_handles.push_back(
        joint_limits_interface::PositionJointSoftLimitsHandle(
          m_joint_handles[i],
          limits,
          soft_limits // deliberately empty
          ));
  }

  for (int i = 0; i < m_number_joints; ++i)
  {
    m_limits_interface.registerHandle(m_limits_handles[i]);
  }

  return true;
}

JointControllerAdapter::~JointControllerAdapter()
{
}

void JointControllerAdapter::read()
{
}

void JointControllerAdapter::write()
{
}

} // end namespace

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

bool JointControllerAdapter::init(const std::vector<hardware_interface::JointStateHandle>& state_handles, ros::NodeHandle& nh)
{
  for (size_t i = 0; i < state_handles.size(); ++i)
  {
    m_joint_names.push_back(state_handles[i].getName());
  }
  m_number_joints = m_joint_names.size();
  m_cmd.resize(m_number_joints);

  // Register external state_handles
  for (int i = 0; i < m_number_joints; ++i)
  {
    m_state_interface.registerHandle(state_handles[i]);
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

void JointControllerAdapter::write(KDL::JntArray& positions)
{
  if (positions.data.size() != m_cmd.size())
  {
    throw std::runtime_error("Joint number mismatch!");
  }

  // Fill positions for forward kinematics
  for (size_t i = 0; i < m_cmd.size(); ++i)
  {
    positions(i) = m_cmd[i];
  }
}

} // end namespace

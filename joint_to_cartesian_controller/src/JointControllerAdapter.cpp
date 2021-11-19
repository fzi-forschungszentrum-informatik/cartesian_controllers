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

  // Start where you are
  for (size_t i = 0; i < m_number_joints; ++i)
  {
    m_cmd[i] = state_handles[i].getPosition();
  }

  // Register external state_handles
  for (size_t i = 0; i < m_number_joints; ++i)
  {
    m_state_interface.registerHandle(state_handles[i]);
  }
  registerInterface(&m_state_interface);

  // Initialize and register handles to the actuators
  for (size_t i = 0; i < m_number_joints; ++i)
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
  for (size_t i = 0; i < m_number_joints; ++i)
  {
    joint_limits_interface::getJointLimits(
        m_joint_names[i],
        nh,
        limits);
  }

  joint_limits_interface::SoftJointLimits soft_limits;

  // Initialize and register handles to the joint limits
  for (size_t i = 0; i < m_number_joints; ++i)
  {
    m_limits_handles.push_back(
        joint_limits_interface::PositionJointSoftLimitsHandle(
          m_joint_handles[i],
          limits,
          soft_limits // deliberately empty
          ));
  }

  for (size_t i = 0; i < m_number_joints; ++i)
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
  if (static_cast<size_t>(positions.data.size()) != m_cmd.size())
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

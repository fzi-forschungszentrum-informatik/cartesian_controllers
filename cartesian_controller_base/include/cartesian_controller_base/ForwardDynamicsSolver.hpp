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
/*!\file    ForwardDynamicsSolver.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/09
 *
 */
//-----------------------------------------------------------------------------


// this package
#include <cartesian_controller_base/ForwardDynamicsSolver.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>

namespace cartesian_controller_base{

template <>
void ForwardDynamicsSolver::updateKinematics<hardware_interface::PositionJointInterface>(
    const std::vector<hardware_interface::JointHandle>&)
{
  // Keep feed forward simulation running
  m_last_positions = m_current_positions;

  // Pose w. r. t. base
  m_fk_pos_solver->JntToCart(m_current_positions,m_end_effector_pose);

  // Absolute velocity w. r. t. base
  KDL::FrameVel vel;
  m_fk_vel_solver->JntToCart(KDL::JntArrayVel(m_current_positions,m_current_velocities),vel);
  m_end_effector_vel[0] = vel.deriv().vel.x();
  m_end_effector_vel[1] = vel.deriv().vel.y();
  m_end_effector_vel[2] = vel.deriv().vel.z();
  m_end_effector_vel[3] = vel.deriv().rot.x();
  m_end_effector_vel[4] = vel.deriv().rot.y();
  m_end_effector_vel[5] = vel.deriv().rot.z();
}

template <>
void ForwardDynamicsSolver::updateKinematics<hardware_interface::VelocityJointInterface>(
    const std::vector<hardware_interface::JointHandle>& joint_handles)
{
  // Reset internal simulation with real robot state
  setStartState(joint_handles);

  updateKinematics<hardware_interface::PositionJointInterface>(joint_handles);
}

}

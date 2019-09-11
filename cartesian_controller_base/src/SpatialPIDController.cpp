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
/*!\file    SpatialPIDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <cartesian_controller_base/SpatialPIDController.h>

// Other
#include <string>

namespace cartesian_controller_base
{

SpatialPIDController::SpatialPIDController()
{
}

ctrl::Vector6D SpatialPIDController::operator()(const ctrl::Vector6D& error, const ros::Duration& period)
{
  // Perform pid control separately on each Cartesian dimension
  for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
  {
    m_cmd(i) = m_pid_controllers[i].computeCommand(error[i],period);
  }
  return m_cmd;
}

bool SpatialPIDController::init(ros::NodeHandle& nh)
{
  // Initialize pid controllers for each Cartesian dimension
  for (int i = 0; i < 6; ++i) // 3 transition, 3 rotation
  {
    m_pid_controllers.push_back(
        control_toolbox::Pid());
  }

  // Load default controller gains
  std::string solver_config = nh.getNamespace() + "/pid_gains";

  m_pid_controllers[0].initParam(solver_config + "/trans_x");
  m_pid_controllers[1].initParam(solver_config + "/trans_y");
  m_pid_controllers[2].initParam(solver_config + "/trans_z");
  m_pid_controllers[3].initParam(solver_config + "/rot_x");
  m_pid_controllers[4].initParam(solver_config + "/rot_y");
  m_pid_controllers[5].initParam(solver_config + "/rot_z");

  return true;
}

} // namespace

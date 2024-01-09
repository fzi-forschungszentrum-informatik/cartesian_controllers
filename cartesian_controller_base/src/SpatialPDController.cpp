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
/*!\file    SpatialPDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/SpatialPDController.h>

#include <string>

namespace cartesian_controller_base
{
SpatialPDController::SpatialPDController() {}

ctrl::Vector6D SpatialPDController::operator()(const ctrl::Vector6D & error,
                                               const rclcpp::Duration & period)
{
  // Perform pd control separately on each Cartesian dimension
  for (int i = 0; i < 6; ++i)  // 3 transition, 3 rotation
  {
    m_cmd(i) = m_pd_controllers[i](error[i], period);
  }
  return m_cmd;
}

#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
bool SpatialPDController::init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> handle)
#else
bool SpatialPDController::init(std::shared_ptr<rclcpp::Node> handle)
#endif
{
  // Create pd controllers for each Cartesian dimension
  for (int i = 0; i < 6; ++i)  // 3 transition, 3 rotation
  {
    m_pd_controllers.push_back(PDController());
  }

  // Load default controller gains
  std::string gains_config = "pd_gains";

  m_pd_controllers[0].init(gains_config + ".trans_x", handle);
  m_pd_controllers[1].init(gains_config + ".trans_y", handle);
  m_pd_controllers[2].init(gains_config + ".trans_z", handle);
  m_pd_controllers[3].init(gains_config + ".rot_x", handle);
  m_pd_controllers[4].init(gains_config + ".rot_y", handle);
  m_pd_controllers[5].init(gains_config + ".rot_z", handle);

  return true;
}

}  // namespace cartesian_controller_base

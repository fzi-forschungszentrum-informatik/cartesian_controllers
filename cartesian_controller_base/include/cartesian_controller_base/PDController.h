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
/*!\file    PDController.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/16
 *
 */
//-----------------------------------------------------------------------------

#ifndef PD_CONTROLLER_H_INCLUDED
#define PD_CONTROLLER_H_INCLUDED

#include "ROS2VersionConfig.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cartesian_controller_base
{
/**
 * @brief A proportional, derivative controller
 *
 * This one-dimensional controller exposes a local **p** gain and a **d** gain as parameters.
 *
 * We use a custom implementation to drop the integral gain of the `control_toolbox`.
 * The \ref _cartesian_controllers_ package builds upon a control plant that
 * already has an integrating part to eliminate steady state errors.
 * Exposing parameterization for integral gains would confuse users with unused complexity.
 */
class PDController
{
public:
  PDController();
  ~PDController();

#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
  void init(const std::string & params, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> handle);
#else
  void init(const std::string & params, std::shared_ptr<rclcpp::Node> handle);
#endif

  double operator()(const double & error, const rclcpp::Duration & period);

private:
#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> m_handle;
#else
  std::shared_ptr<rclcpp::Node> m_handle;  ///< handle for dynamic parameter interaction
#endif
  std::string m_params;  ///< namespace for parameter access

  // Gain parameters
  double m_p;  ///< proportional gain
  double m_d;  ///< derivative gain
  double m_last_p_error;
};

}  // namespace cartesian_controller_base

#endif

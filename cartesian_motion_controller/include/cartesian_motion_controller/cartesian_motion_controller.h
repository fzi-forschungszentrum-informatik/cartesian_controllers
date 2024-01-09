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
/*!\file    cartesian_motion_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_H_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace cartesian_motion_controller
{
/**
 * @brief A ROS2-control controller for Cartesian motion tracking
 *
 * This controller is meant for tracking Cartesian motion that is not known in
 * advance.  Common use cases are teleoperation or Cartesian end effector
 * teaching, in which the Cartesian motion is commanded with discrete target
 * poses.
 *
 * The controller receives the targets as \a geometry_msgs::msg::PoseStamped
 * and tries to reach those as best as possible.  Users can adjust the
 * controller's responsiveness to those targets with setting individual PD
 * gains for each Cartesian dimension.
 *
 * One benefit is that the controller automatically interpolates to obtain
 * smooth joint commands for distant, discretely sampled targets.
 * Users achieve this with setting qualitatively low P gains.
 *
 * For uses cases where a more precise tracking is needed, users may configure
 * this controller to a fast Inverse Kinematics solver, with setting
 * qualitatively high P gains and a higher number of internal solver iterations.
 *
 */
class CartesianMotionController : public virtual cartesian_controller_base::CartesianControllerBase
{
public:
  CartesianMotionController();
  virtual ~CartesianMotionController() = default;

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  virtual LifecycleNodeInterface::CallbackReturn on_init() override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  virtual controller_interface::return_type init(const std::string & controller_name) override;
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  controller_interface::return_type update() override;
#endif

  using Base = cartesian_controller_base::CartesianControllerBase;

protected:
  /**
     * @brief Compute the offset between a target pose and the current end effector pose
     *
     * The pose offset is formulated with a translational component and a rotational
     * component, using Rodrigues vector notation.
     *
     * The robot's current pose is computed with forward kinematics.
     *
     * @return The error as a 6-dim vector (linear, angular) w.r.t to the robot base link
     */
  ctrl::Vector6D computeMotionError();
  KDL::Frame m_target_frame;
  KDL::Frame m_current_frame;

  void targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_target_frame_subscr;
};

}  // namespace cartesian_motion_controller

#endif

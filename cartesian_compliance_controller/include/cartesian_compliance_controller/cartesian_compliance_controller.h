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
/*!\file    cartesian_compliance_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>
#include <cartesian_force_controller/cartesian_force_controller.h>
#include <cartesian_motion_controller/cartesian_motion_controller.h>

#include <controller_interface/controller_interface.hpp>

namespace cartesian_compliance_controller
{
/**
 * @brief A ROS2-control controller for Cartesian compliance control
 *
 * This controller is the combination of the \ref CartesianMotionController and
 * the \ref CartesianForceController.  Users can use this controller to track
 * Cartesian end effector motion that involves contact with the environment.
 * During operation, both interfaces can be used to command target poses
 * and target wrenches in parallel.
 * While the PD gains determine the controllers responsiveness, users can
 * additionally set a 6-dimensional stiffness for this controller, relating
 * the target pose offset to reaction forces with the environment.
 *
 * Note that the target wrench is superimposed with this stiffness, meaning that
 * the target wrench is fully compensated at some point by the virtual stiffness.
 * A common application is the tracking of a moving target in close proximity
 * to a surface, and applying an additional force profile to that surface.
 * To compensate for bigger offsets, users can set a low stiffness for the axes
 * where the additional forces are applied.
 *
 */
class CartesianComplianceController : public cartesian_motion_controller::CartesianMotionController,
                                      public cartesian_force_controller::CartesianForceController
{
public:
  CartesianComplianceController();

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
  using MotionBase = cartesian_motion_controller::CartesianMotionController;
  using ForceBase = cartesian_force_controller::CartesianForceController;

private:
  /**
     * @brief Compute the net force of target wrench and stiffness-related pose offset
     *
     * @return The remaining error wrench, given in robot base frame
     */
  ctrl::Vector6D computeComplianceError();

  ctrl::Matrix6D m_stiffness;
  std::string m_compliance_ref_link;
};

}  // namespace cartesian_compliance_controller

#endif

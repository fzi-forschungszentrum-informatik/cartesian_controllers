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
/*!\file    cartesian_force_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_FORCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_FORCE_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace cartesian_force_controller
{
/**
 * @brief A ROS2-control controller for Cartesian force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.  The
 * controller additionally listens to the specified force-torque sensor signals
 * and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Cartesian axes.
 * In contact, the controller regulates the net force of the two wrenches to zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class CartesianForceController : public virtual cartesian_controller_base::CartesianControllerBase
{
public:
  CartesianForceController();

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
     * @brief Compute the net force of target wrench and measured sensor wrench
     *
     * @return The remaining error wrench, given in robot base frame
     */
  ctrl::Vector6D computeForceError();
  std::string m_new_ft_sensor_ref;
  void setFtSensorReferenceFrame(const std::string & new_ref);

private:
  void targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft_sensor_wrench_subscriber;
  ctrl::Vector6D m_target_wrench;
  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  /**
     * Allow users to choose whether to specify their target wrenches in the
     * end-effector frame (= True) or the base frame (= False). The first one
     * is easier for explicit task programming, while the second one is more
     * intuitive for tele-manipulation.
     */
  bool m_hand_frame_control;
};

}  // namespace cartesian_force_controller

#endif

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
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include <cartesian_force_controller/cartesian_force_controller.h>

namespace cartesian_force_controller
{

CartesianForceController::CartesianForceController()
: Base::CartesianControllerBase(), m_hand_frame_control(true)
{
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<bool>("hand_frame_control", true);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;;
}
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianForceController::init(const std::string & controller_name)
{
  const auto ret = Base::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  auto_declare<bool>("hand_frame_control", true);

  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if(!Base::robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link << " is not part of the kinematic chain from "
                                             << Base::m_robot_base_link << " to "
                                             << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "target_wrench", 10, std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  m_ft_sensor_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "ft_sensor_wrench", 10, std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  Base::on_activate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianForceController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Stop drifting by sending zero joint velocities
  Base::computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  Base::writeJointControlCmds();
  Base::on_deactivate(previous_state);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC
controller_interface::return_type CartesianForceController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianForceController::update()
#endif
{
  // Synchronize the internal model and the real robot
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  Base::computeJointControlCmds(error,internal_period);

  // Write final commands to the hardware interface
  Base::writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control) // Assume end-effector frame by convention
  {
    target_wrench = Base::displayInBaseLink(m_target_wrench,Base::m_end_effector_link);
  }
  else // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  return m_ft_sensor_wrench + target_wrench;
}

void CartesianForceController::computeFtSensorTransform(const std::string& sensor_ref,
  const std::string& new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(Base::m_ik_solver->getPositions());

  m_ft_sensor_ref_link = sensor_ref;
  KDL::Frame sensor_frame;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      sensor_frame,
      m_ft_sensor_ref_link);

  KDL::Frame new_sensor_frame;
  Base::m_forward_kinematics_solver->JntToCart(
      jnts,
      new_sensor_frame,
      m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_frame.Inverse() * sensor_frame;
}

void CartesianForceController::targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianForceController::ftSensorWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  // Check if frame exists in kinematic tree
  const KDL::SegmentMap & segment_map = m_robot_tree.getSegments();
  if (segment_map.find(wrench->header.frame_id) == segment_map.end())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "FT Sensor wrench frame '%s' does not exist in kinematic tree",
      wrench->header.frame_id.c_str());
    return;
  }

  // If needed, compute new transform from sensor frame to end effector frame
  if (wrench->header.frame_id != m_ft_sensor_ref_link) {
    computeFtSensorTransform(wrench->header.frame_id, Base::m_end_effector_link);
  }

  // Copy wrench values to temporary variable
  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;
  tmp[3] = wrench->wrench.torque.x;
  tmp[4] = wrench->wrench.torque.y;
  tmp[5] = wrench->wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest
  tmp = m_ft_sensor_transform * tmp;
  for (int i = 0; i < 6; ++i) {
    m_ft_sensor_wrench[i] = tmp[i];
  }

  // If needed, display the measured wrench in the base frame
  if (wrench->header.frame_id != Base::m_robot_base_link) {
    m_ft_sensor_wrench = Base::displayInBaseLink(m_ft_sensor_wrench, wrench->header.frame_id);
  }
}

}

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController, controller_interface::ControllerInterface)

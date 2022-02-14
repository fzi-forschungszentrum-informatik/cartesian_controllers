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
/*!\file    cartesian_controller_base.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

// Project
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <cartesian_controller_base/cartesian_controller_base.h>

// KDL
#include <cmath>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

// URDF
#include <urdf/model.h>
#include <urdf_model/joint.h>

namespace cartesian_controller_base
{

CartesianControllerBase::CartesianControllerBase()
: m_already_initialized(false)
{
}

controller_interface::InterfaceConfiguration CartesianControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * 1); // only position for now
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

controller_interface::InterfaceConfiguration CartesianControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * 1); // only position for now
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

controller_interface::return_type CartesianControllerBase::init(const std::string & controller_name)
{
  // Initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  if (m_already_initialized)
  {
    return controller_interface::return_type::OK;
  }

  auto_declare<std::string>("ik_solver", "forward_dynamics");
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  auto_declare<double>("solver.error_scale", 1.0);
  auto_declare<int>("solver.iterations", 1);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianControllerBase::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  // Load user specified inverse kinematics solver
  std::string ik_solver = get_node()->get_parameter("ik_solver").as_string();
  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "cartesian_controller_base", "cartesian_controller_base::IKSolver"));
  try
  {
    m_ik_solver = m_solver_loader->createSharedInstance(ik_solver);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree   robot_tree;
  KDL::Chain  robot_chain;

  m_robot_description = get_node()->get_parameter("robot_description").as_string();
  if (m_robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_robot_base_link = get_node()->get_parameter("robot_base_link").as_string();
  if (m_robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  m_end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  if (m_end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(m_robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,robot_chain))
  {
    const std::string error = ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), error);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get names of actuated joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Parse joint limits
  KDL::JntArray upper_pos_limits(m_joint_names.size());
  KDL::JntArray lower_pos_limits(m_joint_names.size());
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    if (!robot_model.getJoint(m_joint_names[i]))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Joint %s does not appear in robot_description", m_joint_names[i].c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    if (robot_model.getJoint(m_joint_names[i])->type == urdf::Joint::CONTINUOUS)
    {
      upper_pos_limits(i) = std::nan("0");
      lower_pos_limits(i) = std::nan("0");
    }
    else
    {
      // Non-existent urdf limits are zero initialized
      upper_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->upper;
      lower_pos_limits(i) = robot_model.getJoint(m_joint_names[i])->limits->lower;
    }
  }

  // Initialize solvers
  m_ik_solver->init(get_node(), robot_chain,upper_pos_limits,lower_pos_limits);
  KDL::Tree tmp("not_relevant");
  tmp.addChain(robot_chain,"not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));
  m_iterations = get_node()->get_parameter("solver.iterations").as_int();
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(get_node());

  m_already_initialized = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianControllerBase::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CartesianControllerBase::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  // Copy joint state to internal simulation
  if (!m_ik_solver->setStartState(state_interfaces_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  };
  m_ik_solver->updateKinematics(command_interfaces_, state_interfaces_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CartesianControllerBase::writeJointControlCmds()
{
  // Only position commands for now
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    command_interfaces_[i].set_value(m_simulated_joint_motion.positions[i]);
  }
}

void CartesianControllerBase::computeJointControlCmds(const ctrl::Vector6D& error, const rclcpp::Duration& period)
{
  // PD controlled system input
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  m_cartesian_input = m_error_scale * m_spatial_controller(error,period);

  // Simulate one step forward
  m_simulated_joint_motion = m_ik_solver->getJointControlCmds(
      period,
      m_cartesian_input);

  m_ik_solver->updateKinematics();
}

ctrl::Vector6D CartesianControllerBase::displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      transform_kdl,
      from);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

ctrl::Matrix6D CartesianControllerBase::displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from)
{
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      R_kdl,
      from);

  // Adjust format
  ctrl::Matrix3D R;
  R <<
      R_kdl.M.data[0],
      R_kdl.M.data[1],
      R_kdl.M.data[2],
      R_kdl.M.data[3],
      R_kdl.M.data[4],
      R_kdl.M.data[5],
      R_kdl.M.data[6],
      R_kdl.M.data[7],
      R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3,3>() = R * tensor.topLeftCorner<3,3>() * R.transpose();
  tmp.bottomRightCorner<3,3>() = R * tensor.bottomRightCorner<3,3>() * R.transpose();

  return tmp;
}

ctrl::Vector6D CartesianControllerBase::displayInTipLink(const ctrl::Vector6D& vector, const std::string& to)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(
      m_ik_solver->getPositions(),
      transform_kdl,
      to);

  // Rotate into new reference frame
  wrench_kdl = transform_kdl.M.Inverse() * wrench_kdl;

  // Reassign
  ctrl::Vector6D out;
  for (int i = 0; i < 6; ++i)
  {
    out[i] = wrench_kdl(i);
  }

  return out;
}

} // namespace

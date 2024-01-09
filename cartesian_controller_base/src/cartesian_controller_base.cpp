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

#include <cartesian_controller_base/cartesian_controller_base.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>

#include <cmath>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace cartesian_controller_base
{
CartesianControllerBase::CartesianControllerBase() {}

controller_interface::InterfaceConfiguration
CartesianControllerBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size() * m_cmd_interface_types.size());
  for (const auto & type : m_cmd_interface_types)
  {
    for (const auto & joint_name : m_joint_names)
    {
      conf.names.push_back(joint_name + std::string("/").append(type));
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
CartesianControllerBase::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(m_joint_names.size());  // Only position
  for (const auto & joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
  }
  return conf;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_init()
{
  if (!m_initialized)
  {
    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    auto_declare<bool>("solver.publish_state_feedback", false);
    m_initialized = true;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type CartesianControllerBase::init(const std::string & controller_name)
{
  if (!m_initialized)
  {
    // Initialize lifecycle node
    const auto ret = ControllerInterface::init(controller_name);
    if (ret != controller_interface::return_type::OK)
    {
      return ret;
    }

    auto_declare<std::string>("ik_solver", "forward_dynamics");
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("command_interfaces", std::vector<std::string>());
    auto_declare<double>("solver.error_scale", 1.0);
    auto_declare<int>("solver.iterations", 1);
    auto_declare<bool>("solver.publish_state_feedback", false);

    m_initialized = true;
  }
  return controller_interface::return_type::OK;
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  if (m_configured)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Load user specified inverse kinematics solver
  std::string ik_solver = get_node()->get_parameter("ik_solver").as_string();
  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "cartesian_controller_base", "cartesian_controller_base::IKSolver"));
  try
  {
    m_ik_solver = m_solver_loader->createSharedInstance(ik_solver);
  }
  catch (pluginlib::PluginlibException & ex)
  {
    RCLCPP_ERROR(get_node()->get_logger(), ex.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

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
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(m_robot_base_link, m_end_effector_link, m_robot_chain))
  {
    const std::string error =
      ""
      "Failed to parse robot chain from urdf model. "
      "Do robot_base_link and end_effector_link exist?";
    RCLCPP_ERROR(get_node()->get_logger(), error.c_str());
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
      RCLCPP_ERROR(get_node()->get_logger(), "Joint %s does not appear in robot_description",
                   m_joint_names[i].c_str());
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
  m_ik_solver->init(get_node(), m_robot_chain, upper_pos_limits, lower_pos_limits);
  KDL::Tree tmp("not_relevant");
  tmp.addChain(m_robot_chain, "not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));
  m_iterations = get_node()->get_parameter("solver.iterations").as_int();
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(get_node());

  // Check command interfaces.
  // We support position, velocity, or both.
  m_cmd_interface_types = get_node()->get_parameter("command_interfaces").as_string_array();
  if (m_cmd_interface_types.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  for (const auto & type : m_cmd_interface_types)
  {
    if (type != hardware_interface::HW_IF_POSITION && type != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Unsupported command interface: %s. Choose position or velocity", type.c_str());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
  }

  // Controller-internal state publishing
  m_feedback_pose_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string(get_node()->get_name()) + "/current_pose", 3));

  m_feedback_twist_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
      get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        std::string(get_node()->get_name()) + "/current_twist", 3));

  m_configured = true;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  if (m_active)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Get command handles.
  for (const auto & type : m_cmd_interface_types)
  {
    if (!controller_interface::get_ordered_interfaces(command_interfaces_, m_joint_names, type,
                                                      (type == hardware_interface::HW_IF_POSITION)
                                                        ? m_joint_cmd_pos_handles
                                                        : m_joint_cmd_vel_handles))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.",
                   m_joint_names.size(), type.c_str(),
                   (type == hardware_interface::HW_IF_POSITION) ? m_joint_cmd_pos_handles.size()
                                                                : m_joint_cmd_vel_handles.size());
      return CallbackReturn::ERROR;
    }
  }

  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }

  // Copy joint state to internal simulation
  if (!m_ik_solver->setStartState(m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set start state");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  };
  m_ik_solver->updateKinematics();

  // Provide safe command buffers with starting where we are
  computeJointControlCmds(ctrl::Vector6D::Zero(), rclcpp::Duration::from_seconds(0));
  writeJointControlCmds();

  m_active = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianControllerBase::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  stopCurrentMotion();

  if (m_active)
  {
    m_joint_cmd_pos_handles.clear();
    m_joint_cmd_vel_handles.clear();
    m_joint_state_pos_handles.clear();
    this->release_interfaces();
    m_active = false;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CartesianControllerBase::writeJointControlCmds()
{
  if (get_node()->get_parameter("solver.publish_state_feedback").as_bool())
  {
    publishStateFeedback();
  }

  auto nan_in = [](const auto & values) -> bool
  {
    for (const auto & value : values)
    {
      if (std::isnan(value))
      {
        return true;
      }
    }
    return false;
  };

  if (nan_in(m_simulated_joint_motion.positions) || nan_in(m_simulated_joint_motion.velocities))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "NaN detected in internal model. It's unlikely to recover from this. Shutting down.");

#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
    get_node()->shutdown();
#elif defined CARTESIAN_CONTROLLERS_FOXY || defined CARTESIAN_CONTROLLERS_GALACTIC
    this->shutdown();
#endif
    return;
  }

  // Write all available types.
  for (const auto & type : m_cmd_interface_types)
  {
    if (type == hardware_interface::HW_IF_POSITION)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        m_joint_cmd_pos_handles[i].get().set_value(m_simulated_joint_motion.positions[i]);
      }
    }
    if (type == hardware_interface::HW_IF_VELOCITY)
    {
      for (size_t i = 0; i < m_joint_names.size(); ++i)
      {
        m_joint_cmd_vel_handles[i].get().set_value(m_simulated_joint_motion.velocities[i]);
      }
    }
  }
}

void CartesianControllerBase::computeJointControlCmds(const ctrl::Vector6D & error,
                                                      const rclcpp::Duration & period)
{
  // PD controlled system input
  m_error_scale = get_node()->get_parameter("solver.error_scale").as_double();
  m_cartesian_input = m_error_scale * m_spatial_controller(error, period);

  // Simulate one step forward
  m_simulated_joint_motion = m_ik_solver->getJointControlCmds(period, m_cartesian_input);

  m_ik_solver->updateKinematics();
}

ctrl::Vector6D CartesianControllerBase::displayInBaseLink(const ctrl::Vector6D & vector,
                                                          const std::string & from)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, from);

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

ctrl::Matrix6D CartesianControllerBase::displayInBaseLink(const ctrl::Matrix6D & tensor,
                                                          const std::string & from)
{
  // Get rotation to base
  KDL::Frame R_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), R_kdl, from);

  // Adjust format
  ctrl::Matrix3D R;
  R << R_kdl.M.data[0], R_kdl.M.data[1], R_kdl.M.data[2], R_kdl.M.data[3], R_kdl.M.data[4],
    R_kdl.M.data[5], R_kdl.M.data[6], R_kdl.M.data[7], R_kdl.M.data[8];

  // Treat diagonal blocks as individual 2nd rank tensors.
  // Display in base frame.
  ctrl::Matrix6D tmp = ctrl::Matrix6D::Zero();
  tmp.topLeftCorner<3, 3>() = R * tensor.topLeftCorner<3, 3>() * R.transpose();
  tmp.bottomRightCorner<3, 3>() = R * tensor.bottomRightCorner<3, 3>() * R.transpose();

  return tmp;
}

ctrl::Vector6D CartesianControllerBase::displayInTipLink(const ctrl::Vector6D & vector,
                                                         const std::string & to)
{
  // Adjust format
  KDL::Wrench wrench_kdl;
  for (int i = 0; i < 6; ++i)
  {
    wrench_kdl(i) = vector[i];
  }

  KDL::Frame transform_kdl;
  m_forward_kinematics_solver->JntToCart(m_ik_solver->getPositions(), transform_kdl, to);

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

void CartesianControllerBase::publishStateFeedback()
{
  // End-effector pose
  auto pose = m_ik_solver->getEndEffectorPose();
  if (m_feedback_pose_publisher->trylock())
  {
    m_feedback_pose_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_pose_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_pose_publisher->msg_.pose.position.x = pose.p.x();
    m_feedback_pose_publisher->msg_.pose.position.y = pose.p.y();
    m_feedback_pose_publisher->msg_.pose.position.z = pose.p.z();

    pose.M.GetQuaternion(m_feedback_pose_publisher->msg_.pose.orientation.x,
                         m_feedback_pose_publisher->msg_.pose.orientation.y,
                         m_feedback_pose_publisher->msg_.pose.orientation.z,
                         m_feedback_pose_publisher->msg_.pose.orientation.w);

    m_feedback_pose_publisher->unlockAndPublish();
  }

  // End-effector twist
  auto twist = m_ik_solver->getEndEffectorVel();
  if (m_feedback_twist_publisher->trylock())
  {
    m_feedback_twist_publisher->msg_.header.stamp = get_node()->now();
    m_feedback_twist_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_twist_publisher->msg_.twist.linear.x = twist[0];
    m_feedback_twist_publisher->msg_.twist.linear.y = twist[1];
    m_feedback_twist_publisher->msg_.twist.linear.z = twist[2];
    m_feedback_twist_publisher->msg_.twist.angular.x = twist[3];
    m_feedback_twist_publisher->msg_.twist.angular.y = twist[4];
    m_feedback_twist_publisher->msg_.twist.angular.z = twist[5];

    m_feedback_twist_publisher->unlockAndPublish();
  }
}

}  // namespace cartesian_controller_base

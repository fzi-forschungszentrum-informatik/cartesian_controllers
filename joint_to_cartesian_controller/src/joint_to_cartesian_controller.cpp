// Copyright (c) 2023, PAL Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "joint_to_cartesian_controller/joint_to_cartesian_controller.h"

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace joint_to_cartesian_controller
{

controller_interface::CallbackReturn JointToCartesianController::on_init()
{
  try
  {
    auto_declare<std::string>("robot_description", "");
    auto_declare<std::string>("robot_base_link", "");
    auto_declare<std::string>("end_effector_link", "");
    auto_declare<std::vector<std::string>>("interfaces", std::vector<std::string>());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointToCartesianController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
JointToCartesianController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = command_interface_names_;

  return state_interfaces_config;
}

controller_interface::CallbackReturn JointToCartesianController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interface_names_ = get_node()->get_parameter("interfaces").as_string_array();

  joints_cmd_sub_ = this->get_node()->create_subscription<DataType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const DataType::SharedPtr msg)
    {
      // check if message is correct size, if not ignore
      if (msg->data.size() == command_interface_names_.size())
      {
        rt_buffer_ptr_.writeFromNonRT(msg);
      }
      else
      {
        RCLCPP_ERROR(this->get_node()->get_logger(),
                     "Invalid command received of %zu size, expected %zu size", msg->data.size(),
                     command_interface_names_.size());
      }
    });

  // pre-reserve command interfaces
  command_interfaces_.reserve(command_interface_names_.size());

  urdf::Model robot_model;
  KDL::Tree robot_tree;

#if defined CARTESIAN_CONTROLLERS_JAZZY
  std::string robot_description = this->get_robot_description();
#else
  std::string robot_description = get_node()->get_parameter("robot_description").as_string();
#endif
  if (robot_description.empty())
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

  // Publishers
  m_pose_publisher = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    get_node()->get_name() + std::string("/target_frame"), 10);

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
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
    RCLCPP_ERROR(get_node()->get_logger(), "%s", error.c_str());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));

  // The names should be in the same order as for command interfaces for easier matching
  reference_interface_names_ = command_interface_names_;
  exported_state_interface_names_ = command_interface_names_;
  // for any case make reference interfaces size of command interfaces
  reference_interfaces_.resize(reference_interface_names_.size(),
                               std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointToCartesianController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   ordered_interfaces;
  // if (
  //   !controller_interface::get_ordered_interfaces(
  //     command_interfaces_, command_interface_names_, std::string(""), ordered_interfaces) ||
  //   command_interface_names_.size() != ordered_interfaces.size())
  // {
  //   RCLCPP_ERROR(
  //     this->get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
  //     command_interface_names_.size(), ordered_interfaces.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces;
  if (!controller_interface::get_ordered_interfaces(state_interfaces_, command_interface_names_,
                                                    std::string(""), state_interfaces) ||
      command_interface_names_.size() != state_interfaces.size())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
                 command_interface_names_.size(), state_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

  RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

  std::fill(reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointToCartesianController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

bool JointToCartesianController::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::return_type JointToCartesianController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  KDL::JntArray positions(reference_interfaces_.size());

  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    if (!std::isnan(reference_interfaces_[i]))
    {
      // command_interfaces_[i].set_value(reference_interfaces_[i]);
      positions(i) = reference_interfaces_[i];
    }
    else
    {
      auto optional = state_interfaces_[i].get_optional();
      if (optional)
      {
        positions(i) = optional.value();
      }
    }
    bool success = ordered_exported_state_interfaces_[i]->set_value(positions(i));
    if (!success)
    {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to set value for state interface %s"
                            << ordered_exported_state_interfaces_[i]->get_name());
      return controller_interface::return_type::ERROR;
    }
  }

  KDL::Frame tmp;
  m_fk_solver->JntToCart(positions, tmp);

  m_current_pose.pose.position.x = tmp.p.x();
  m_current_pose.pose.position.y = tmp.p.y();
  m_current_pose.pose.position.z = tmp.p.z();
  tmp.M.GetQuaternion(m_current_pose.pose.orientation.x, m_current_pose.pose.orientation.y,
                      m_current_pose.pose.orientation.z, m_current_pose.pose.orientation.w);

  m_current_pose.header.stamp = get_node()->now();
  m_current_pose.header.frame_id = m_robot_base_link;
  m_pose_publisher->publish(m_current_pose);

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
JointToCartesianController::on_export_state_interfaces()
{
  state_interfaces_values_.resize(exported_state_interface_names_.size(), 0.0);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
JointToCartesianController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

controller_interface::return_type JointToCartesianController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_commands = rt_buffer_ptr_.readFromRT();
  // message is valid
  if (!(!joint_commands || !(*joint_commands)))
  {
    if (reference_interfaces_.size() != (*joint_commands)->data.size())
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "command size (%zu) does not match number of reference interfaces (%zu)",
        (*joint_commands)->data.size(), reference_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
    reference_interfaces_ = (*joint_commands)->data;
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_to_cartesian_controller

PLUGINLIB_EXPORT_CLASS(joint_to_cartesian_controller::JointToCartesianController,
                       controller_interface::ChainableControllerInterface)

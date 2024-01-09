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
/*!\file    motion_control_handle.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_handles/motion_control_handle.h>
#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "visualization_msgs/msg/detail/interactive_marker_feedback__struct.hpp"

namespace cartesian_controller_handles
{
MotionControlHandle::MotionControlHandle() {}

MotionControlHandle::~MotionControlHandle() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionControlHandle::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(
        state_interfaces_, m_joint_names, hardware_interface::HW_IF_POSITION, m_joint_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(), hardware_interface::HW_IF_POSITION, m_joint_handles.size());
    return CallbackReturn::ERROR;
  }

  m_current_pose = getEndEffectorPose();
  m_server->setPose(m_marker.name, m_current_pose.pose);
  m_server->applyChanges();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionControlHandle::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  m_joint_handles.clear();
  this->release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
controller_interface::return_type MotionControlHandle::update(const rclcpp::Time & time,
                                                              const rclcpp::Duration & period)
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type MotionControlHandle::update()
#endif
{
  // Publish marker pose
  m_current_pose.header.stamp = get_node()->now();
  m_current_pose.header.frame_id = m_robot_base_link;
  m_pose_publisher->publish(m_current_pose);
  m_server->applyChanges();

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration MotionControlHandle::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::NONE;
  return conf;
}

controller_interface::InterfaceConfiguration MotionControlHandle::state_interface_configuration()
  const
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
MotionControlHandle::on_init()
{
#elif defined CARTESIAN_CONTROLLERS_FOXY
controller_interface::return_type MotionControlHandle::init(const std::string & controller_name)
{
  // Initialize lifecycle node
  const auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }
#endif

  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string> >("joints", std::vector<std::string>());

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  return controller_interface::return_type::OK;
#endif
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionControlHandle::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // Get kinematics specific configuration
  urdf::Model robot_model;
  KDL::Tree robot_tree;

  std::string robot_description = get_node()->get_parameter("robot_description").as_string();
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

  // Get names of the joints
  m_joint_names = get_node()->get_parameter("joints").as_string_array();
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Publishers
  m_pose_publisher = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    get_node()->get_name() + std::string("/target_frame"), 10);

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));
  m_current_pose = getEndEffectorPose();

  // Configure the interactive marker for usage in RViz
  m_server.reset(
    new interactive_markers::InteractiveMarkerServer(get_node()->get_name(), get_node()));
  m_marker.header.frame_id = m_robot_base_link;
  m_marker.header.stamp = get_node()->now();
  m_marker.scale = 0.1;
  m_marker.name = "motion_control_handle";
  m_marker.pose = m_current_pose.pose;
  m_marker.description = "6D control of link: " + m_end_effector_link;

  prepareMarkerControls(m_marker);

  // Add the interactive marker to the server
  m_server->insert(m_marker);

  // Add callback for motion in RViz
  m_server->setCallback(
    m_marker.name,
    std::bind(&MotionControlHandle::updateMotionControlCallback, this, std::placeholders::_1),
    visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);

  // Add callback for menu interaction in RViz
  m_server->setCallback(
    m_marker.name,
    std::bind(&MotionControlHandle::updateMarkerMenuCallback, this, std::placeholders::_1),
    visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT);

  // Activate configuration
  m_server->applyChanges();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MotionControlHandle::updateMotionControlCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  // Move marker in RViz
  m_server->setPose(feedback->marker_name, feedback->pose);
  m_server->applyChanges();

  // Store for later broadcasting
  m_current_pose.pose = feedback->pose;
  m_current_pose.header.stamp = get_node()->now();
}

void MotionControlHandle::updateMarkerMenuCallback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
}

void MotionControlHandle::prepareMarkerControls(visualization_msgs::msg::InteractiveMarker & marker)
{
  // Add colored sphere as visualization
  constexpr double marker_scale = 0.05;
  addMarkerVisualization(marker, marker_scale);

  // Create move and rotate controls along all axis
  addAxisControl(marker, 1, 0, 0);
  addAxisControl(marker, 0, 1, 0);
  addAxisControl(marker, 0, 0, 1);
}

void MotionControlHandle::addMarkerVisualization(
  visualization_msgs::msg::InteractiveMarker & marker, double scale)
{
  // Create a sphere as a handle
  visualization_msgs::msg::Marker visual;
  visual.type = visualization_msgs::msg::Marker::SPHERE;
  visual.scale.x = scale;  // bounding box in meter
  visual.scale.y = scale;
  visual.scale.z = scale;
  visual.color.r = 1.0;
  visual.color.g = 0.5;
  visual.color.b = 0.0;
  visual.color.a = 1.0;

  // Create a non-interactive control for the appearance
  visualization_msgs::msg::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(visual);
  marker.controls.push_back(visual_control);
}

void MotionControlHandle::addAxisControl(visualization_msgs::msg::InteractiveMarker & marker,
                                         double x, double y, double z)
{
  if (x == 0 && y == 0 && z == 0)
  {
    return;
  }

  visualization_msgs::msg::InteractiveMarkerControl control;

  double norm = std::sqrt(1 + x * x + y * y + z * z);
  control.orientation.w = 1 / norm;
  control.orientation.x = x / norm;
  control.orientation.y = y / norm;
  control.orientation.z = z / norm;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
}

geometry_msgs::msg::PoseStamped MotionControlHandle::getEndEffectorPose()
{
  KDL::JntArray positions(m_joint_handles.size());
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    positions(i) = m_joint_handles[i].get().get_value();
  }

  KDL::Frame tmp;
  m_fk_solver->JntToCart(positions, tmp);

  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = tmp.p.x();
  current.pose.position.y = tmp.p.y();
  current.pose.position.z = tmp.p.z();
  tmp.M.GetQuaternion(current.pose.orientation.x, current.pose.orientation.y,
                      current.pose.orientation.z, current.pose.orientation.w);

  return current;
}

}  // namespace cartesian_controller_handles

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_controller_handles::MotionControlHandle,
                       controller_interface::ControllerInterface)

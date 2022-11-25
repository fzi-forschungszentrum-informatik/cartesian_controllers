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
/*!\file    cartesian_controller_base.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_CONTROLLER_BASE_HPP_INCLUDED
#define CARTESIAN_CONTROLLER_BASE_HPP_INCLUDED

// Project
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

template <class HardwareInterface>
CartesianControllerBase<HardwareInterface>::
CartesianControllerBase()
: m_already_initialized(false)
{
}

template <class HardwareInterface>
bool CartesianControllerBase<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  if (m_already_initialized)
  {
    return true;
  }

  // Load user specified inverse kinematics solver
  std::string ik_solver = "forward_dynamics"; // Default
  nh.getParam("ik_solver", ik_solver);

  m_solver_loader.reset(new pluginlib::ClassLoader<IKSolver>(
    "cartesian_controller_base", "cartesian_controller_base::IKSolver"));
  try
  {
    m_ik_solver = m_solver_loader->createUniqueInstance(ik_solver);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;

  // Get controller specific configuration
  if (!ros::param::search("robot_description", robot_description))
  {
    ROS_ERROR_STREAM("Searched enclosing namespaces for robot_description but nothing found");
    return false;
  }
  if (!nh.getParam(robot_description, robot_description))
  {
    ROS_ERROR_STREAM("Failed to load " << robot_description << " from parameter server");
    return false;
  }
  if (!nh.getParam("robot_base_link",m_robot_base_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
    return false;
  }
  if (!nh.getParam("end_effector_link",m_end_effector_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
    return false;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    ROS_ERROR("Failed to parse urdf model from 'robot_description'");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
  {
    const std::string error = ""
      "Failed to parse KDL tree from urdf model";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,m_robot_chain))
  {
    const std::string error = ""
      "Failed to parse robot chain from urdf model. "
      "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Get names of controllable joints from the parameter server
  if (!nh.getParam("joints",m_joint_names))
  {
    const std::string error = ""
    "Failed to load " + nh.getNamespace() + "/joints" + " from parameter server";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Parse joint limits
  KDL::JntArray upper_pos_limits(m_joint_names.size());
  KDL::JntArray lower_pos_limits(m_joint_names.size());
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    if (!robot_model.getJoint(m_joint_names[i]))
    {
      const std::string error = ""
        "Joint " + m_joint_names[i] + " does not appear in /robot_description";
      ROS_ERROR_STREAM(error);
      throw std::runtime_error(error);
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

  // Get the joint handles to use in the control loop
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
  }

  // Initialize solvers
  m_ik_solver->init(nh, m_robot_chain,upper_pos_limits,lower_pos_limits);
  KDL::Tree tmp("not_relevant");
  tmp.addChain(m_robot_chain,"not_relevant");
  m_forward_kinematics_solver.reset(new KDL::TreeFkSolverPos_recursive(tmp));

  // Initialize Cartesian pd controllers
  m_spatial_controller.init(nh);

  // Controller-internal state publishing
  m_feedback_pose_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> >(
      nh, "current_pose", 3);

  m_feedback_twist_publisher =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> >(
      nh, "current_twist", 3);

  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_error_scale = 1.0;
  m_iterations = 1;
  m_callback_type = std::bind(
      &CartesianControllerBase<HardwareInterface>::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  m_dyn_conf_server.reset(
      new dynamic_reconfigure::Server<ControllerConfig>(
        ros::NodeHandle(nh.getNamespace() + "/solver")));
  m_dyn_conf_server->setCallback(m_callback_type);

  m_already_initialized = true;

  return true;
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
starting(const ros::Time& time)
{
  // Copy joint state to internal simulation
  m_ik_solver->setStartState(m_joint_handles);
  m_ik_solver->updateKinematics();

  // Provide safe command buffers with starting where we are
  computeJointControlCmds(ctrl::Vector6D::Zero(), ros::Duration(0));
  writeJointControlCmds();
}

template <>
void CartesianControllerBase<hardware_interface::PositionJointInterface>::
writeJointControlCmds()
{
  if (m_publish_state_feedback)
  {
    publishStateFeedback();
  }

  // Take position commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(m_simulated_joint_motion.positions[i]);
  }
}

template <>
void CartesianControllerBase<hardware_interface::VelocityJointInterface>::
writeJointControlCmds()
{
  if (m_publish_state_feedback)
  {
    publishStateFeedback();
  }

  // Take velocity commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(m_simulated_joint_motion.velocities[i]);
  }
}

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period)
{
  // PD controlled system input
  m_cartesian_input = m_error_scale * m_spatial_controller(error,period);

  // Simulate one step forward
  m_simulated_joint_motion = m_ik_solver->getJointControlCmds(
      period,
      m_cartesian_input);

  m_ik_solver->updateKinematics();
}

template <class HardwareInterface>
ctrl::Vector6D CartesianControllerBase<HardwareInterface>::
displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from)
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

template <class HardwareInterface>
ctrl::Matrix6D CartesianControllerBase<HardwareInterface>::
displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from)
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

template <class HardwareInterface>
ctrl::Vector6D CartesianControllerBase<HardwareInterface>::
displayInTipLink(const ctrl::Vector6D& vector, const std::string& to)
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

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
publishStateFeedback()
{
  // End-effector pose
  auto pose = m_ik_solver->getEndEffectorPose();
  if (m_feedback_pose_publisher->trylock()){
    m_feedback_pose_publisher->msg_.header.stamp = ros::Time::now();
    m_feedback_pose_publisher->msg_.header.frame_id = m_robot_base_link;
    m_feedback_pose_publisher->msg_.pose.position.x = pose.p.x();
    m_feedback_pose_publisher->msg_.pose.position.y = pose.p.y();
    m_feedback_pose_publisher->msg_.pose.position.z = pose.p.z();

    pose.M.GetQuaternion(
        m_feedback_pose_publisher->msg_.pose.orientation.x,
        m_feedback_pose_publisher->msg_.pose.orientation.y,
        m_feedback_pose_publisher->msg_.pose.orientation.z,
        m_feedback_pose_publisher->msg_.pose.orientation.w
        );

    m_feedback_pose_publisher->unlockAndPublish();
  }

  // End-effector twist
  auto twist = m_ik_solver->getEndEffectorVel();
  if (m_feedback_twist_publisher->trylock()){
    m_feedback_twist_publisher->msg_.header.stamp = ros::Time::now();
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

template <class HardwareInterface>
void CartesianControllerBase<HardwareInterface>::
dynamicReconfigureCallback(ControllerConfig& config, uint32_t level)
{
  m_error_scale = config.error_scale;
  m_iterations = config.iterations;
  m_publish_state_feedback = config.publish_state_feedback;
}

} // namespace

#endif

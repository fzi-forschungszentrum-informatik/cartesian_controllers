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
/*!\file    joint_to_cartesian_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/15
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <joint_to_cartesian_controller/joint_to_cartesian_controller.h>
#include <cartesian_controller_base/Utility.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>

// Other
#include <map>

namespace cartesian_controllers
{
  /**
   * @brief Connect joint-based controllers and transform their commands to Cartesian target poses
   *
   * This controller handles an internal controller manager, which can load
   * standard ros_controllers. The control commands from these controllers are
   * turned into Cartesian poses with forward kinematics, and can be used by
   * the Cartesian_controllers. An application of this controller is to
   * provide an easy interface to the rqt_joint_trajectory_controller plugin and
   * MoveIt!.
   */
  typedef joint_to_cartesian_controller::JointToCartesianController JointControllerAdapter;
}

PLUGINLIB_EXPORT_CLASS(cartesian_controllers::JointControllerAdapter, controller_interface::ControllerBase)






namespace joint_to_cartesian_controller
{

JointToCartesianController::JointToCartesianController()
{
}

bool JointToCartesianController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;

  // Get controller specific configuration
  if (!nh.getParam("/robot_description",robot_description))
  {
    ROS_ERROR("Failed to load '/robot_description' from parameter server");
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
  if (!nh.getParam("target_frame_topic",m_target_frame_topic))
  {
    m_target_frame_topic = "target_frame";
    ROS_WARN_STREAM("Failed to load "
        << nh.getNamespace() + "/target_frame_topic"
        << " from parameter server. "
        << "Will default to: "
        << nh.getNamespace() + m_target_frame_topic);
  }

  // Publishers
  m_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(m_target_frame_topic,10);

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
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link, m_robot_chain))
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

  // Get the joint handles to use in the control loop
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
  }

  // Adjust joint buffers
  m_positions.data = ctrl::VectorND::Zero(m_joint_handles.size());
  m_velocities.data = ctrl::VectorND::Zero(m_joint_handles.size());

  // Initialize controller adapter and according manager
  m_controller_adapter.init(m_joint_handles,nh);
  m_controller_manager.reset(new controller_manager::ControllerManager(&m_controller_adapter, nh));

  // Initialize forward kinematics solver
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(m_robot_chain));

  return true;
}

void JointToCartesianController::starting(const ros::Time& time)
{
  // Get current joint positions from hardware
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_positions(i) = m_joint_handles[i].getPosition();
  }
}

void JointToCartesianController::stopping(const ros::Time& time)
{
}

void JointToCartesianController::update(const ros::Time& time, const ros::Duration& period)
{
  // Note: The connected joint-based controller gets the feedback directly from
  // the joint state handles of this joint_to_cartesian_controller. So,
  // there's no need for a read() function.

  // Update connected joint controller
  m_controller_manager->update(time,period);

  // Get commanded positions
  m_controller_adapter.write(m_positions);

  // Solve forward kinematics
  KDL::Frame frame;
  m_fk_solver->JntToCart(m_positions,frame);

  // Publish end-effector pose
  geometry_msgs::PoseStamped target_pose = geometry_msgs::PoseStamped();
  target_pose.header.stamp = ros::Time::now();
  target_pose.header.frame_id = m_robot_base_link;
  target_pose.pose.position.x = frame.p.x();
  target_pose.pose.position.y = frame.p.y();
  target_pose.pose.position.z = frame.p.z();
  frame.M.GetQuaternion(
      target_pose.pose.orientation.x,
      target_pose.pose.orientation.y,
      target_pose.pose.orientation.z,
      target_pose.pose.orientation.w);
  m_pose_publisher.publish(target_pose);
}

}

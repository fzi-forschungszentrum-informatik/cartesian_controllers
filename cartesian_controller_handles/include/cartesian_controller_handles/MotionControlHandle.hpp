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
/*!\file    MotionControlHandle.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

// Project
#include <cartesian_controller_handles/MotionControlHandle.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>


namespace cartesian_controller_handles
{

template <class HardwareInterface>
MotionControlHandle<HardwareInterface>::
MotionControlHandle()
{
}

template <class HardwareInterface>
MotionControlHandle<HardwareInterface>::
~MotionControlHandle()
{
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
starting(const ros::Time& time)
{
  m_current_pose = getEndEffectorPose();
  m_server->setPose(m_marker.name,m_current_pose.pose);
  m_server->applyChanges();
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
stopping(const ros::Time& time)
{
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Publish marker pose
  m_current_pose.header.stamp = time;
  m_current_pose.header.frame_id = m_robot_base_link;
  m_pose_publisher.publish(m_current_pose);
  m_server->applyChanges();
}


template <class HardwareInterface>
bool MotionControlHandle<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;
  KDL::Chain  robot_chain;

  // Get configuration from parameter server
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
    ROS_ERROR("Failed to parse KDL tree from urdf model");
    return false;
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,robot_chain))
  {
    ROS_ERROR_STREAM("Failed to parse robot chain from urdf model.");
    return false;
  }

  // Get names of controllable joints from the parameter server
  if (!nh.getParam("joints",m_joint_names))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() << "/joints from parameter server";);
    return false;
  }

  // Get the joint handles to use in the control loop
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
  }

  // Initialize kinematics
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(robot_chain));
  m_current_pose = getEndEffectorPose();

  // Configure the interactive marker for usage in RViz
  m_server.reset(new interactive_markers::InteractiveMarkerServer(
    nh.getNamespace(), "", false));
  m_marker.header.frame_id = m_robot_base_link;
  m_marker.header.stamp = ros::Time(0);   // makes frame_id const
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
      boost::bind(&MotionControlHandle::updateMotionControlCallback,this,_1),
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

  // Add callback for menu interaction in RViz
  m_server->setCallback(
      m_marker.name,
      boost::bind(&MotionControlHandle::updateMarkerMenuCallback,this,_1),
      visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT);

  // Activate configuration
  m_server->applyChanges();

  return true;
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
updateMotionControlCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Move marker in RViz
  m_server->setPose(feedback->marker_name,feedback->pose);
  m_server->applyChanges();

  // Store for later broadcasting
  m_current_pose.pose = feedback->pose;
  m_current_pose.header.stamp = ros::Time::now();

}


template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
updateMarkerMenuCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
prepareMarkerControls(visualization_msgs::InteractiveMarker& marker)
{
  // Add colored sphere as visualization
  addMarkerVisualization(marker, 0.05);

  // Create move and rotate controls along all axis
  addAxisControl(marker,1,0,0);
  addAxisControl(marker,0,1,0);
  addAxisControl(marker,0,0,1);
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
addMarkerVisualization(visualization_msgs::InteractiveMarker& marker, double scale)
{
  // Create a sphere as a handle
  visualization_msgs::Marker visual;
  visual.type = visualization_msgs::Marker::SPHERE;
  visual.scale.x = scale;  // bounding box in meter
  visual.scale.y = scale;
  visual.scale.z = scale;
  visual.color.r = 1.0;
  visual.color.g = 0.5;
  visual.color.b = 0.0;
  visual.color.a = 1.0;

  // Create a non-interactive control for the appearance
  visualization_msgs::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(visual);
  marker.controls.push_back(visual_control);
}

template <class HardwareInterface>
void MotionControlHandle<HardwareInterface>::
addAxisControl(
    visualization_msgs::InteractiveMarker& marker, double x, double y, double z)
{
  if (x == y == z == 0)
  {
    return;
  }

  visualization_msgs::InteractiveMarkerControl control;

  double norm = std::sqrt(1 + x*x + y*y + z*z);
  control.orientation.w = 1/norm;
  control.orientation.x = x/norm;
  control.orientation.y = y/norm;
  control.orientation.z = z/norm;

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);

}

template <class HardwareInterface>
geometry_msgs::PoseStamped MotionControlHandle<HardwareInterface>::
getEndEffectorPose()
{
  KDL::JntArray positions(m_joint_handles.size());
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    positions(i) = m_joint_handles[i].getPosition();
  }

  KDL::Frame tmp;
  m_fk_solver->JntToCart(positions, tmp);

  geometry_msgs::PoseStamped current;
  current.pose.position.x = tmp.p.x();
  current.pose.position.y = tmp.p.y();
  current.pose.position.z = tmp.p.z();
  tmp.M.GetQuaternion(
      current.pose.orientation.x,
      current.pose.orientation.y,
      current.pose.orientation.z,
      current.pose.orientation.w);

  return current;
}

}

// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    MotionControlHandle.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_handles/MotionControlHandle.h>

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
}


template <class HardwareInterface>
bool MotionControlHandle<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  // Publishers
  m_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("target_frame",10);

  // Get configuration from parameter server
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


  // TODO: Get current pose from robot
  // m_current_pose

  // Configure the interactive marker for usage in RViz
  m_server.reset(new interactive_markers::InteractiveMarkerServer("motion_control_handle","",false));

  m_marker.header.frame_id = m_robot_base_link;
  m_marker.header.stamp = ros::Time(0);   // makes frame_id const
  m_marker.scale = 0.1;
  m_marker.name = "motion_control_handle";
  m_marker.pose = m_current_pose.pose;
  m_marker.description = "6D control of link: " + m_end_effector_link;

  // Create a sphere as a handle
  visualization_msgs::Marker visual;
  visual.type = visualization_msgs::Marker::SPHERE;
  visual.scale.x = 0.05;  // bounding box in meter
  visual.scale.y = 0.05;
  visual.scale.z = 0.05;
  visual.color.r = 1.0;
  visual.color.g = 0.5;
  visual.color.b = 0.0;
  visual.color.a = 1.0;

  // Create a non-interactive control for the appearance
  visualization_msgs::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(visual);
  m_marker.controls.push_back(visual_control);

  // Create move and rotate controls along all axis
  addAxisControl(m_marker,1,0,0);
  addAxisControl(m_marker,0,1,0);
  addAxisControl(m_marker,0,0,1);

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

}

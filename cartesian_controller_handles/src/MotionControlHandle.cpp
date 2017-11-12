// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    MotionControlHandle.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/11/06
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_handles/MotionControlHandle.h>

namespace cartesian_controller_handles
{

MotionControlHandle::MotionControlHandle()
{
  if (!init())
  {
    throw std::logic_error("Failed to initialize motion control handle.");
  }
  ROS_INFO_STREAM("Initialized motion control handle for: "
      << m_end_effector_link);
}

MotionControlHandle::~MotionControlHandle()
{
}

void MotionControlHandle::update()
{
  // Broadcast marker pose to TF
  m_tf_broadcaster.sendTransform(
      tf::StampedTransform(
        m_current_pose,
        ros::Time::now(),
        m_robot_base_link,
        m_target_frame));
}


bool MotionControlHandle::init()
{
  // Get configuration from parameter server
  ros::NodeHandle nh;
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
  if (!nh.getParam("target_frame",m_target_frame))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/target_frame" << " from parameter server");
    return false;
  }


  // Check Martin Gunthers answer for why this loop is necessary.
  // https://answers.ros.org/question/38222/tf-extrapolation-exception-using-rostime0/
  bool success = false;
  while (!success)
  {
    try
    {
      m_tf_listener.waitForTransform(
          m_robot_base_link,
          m_end_effector_link,
          ros::Time(0),
          ros::Duration(1.0)  // Will be ignored when throwing exceptions
          );

      tf::StampedTransform tmp;
      m_tf_listener.lookupTransform(
          m_robot_base_link,  // I want my pose displayed in this frame
          m_end_effector_link,
          ros::Time(0),
          tmp);

      success = true;

      // Start with the marker at current robot end-effector
      m_current_pose.setOrigin(tmp.getOrigin());
      m_current_pose.setRotation(tmp.getRotation());
    }
    catch (tf::TransformException& e)
    {
    }
    ros::Duration(0.1).sleep();
  }


  // Configure the interactive marker for usage in RViz
  m_server.reset(new interactive_markers::InteractiveMarkerServer("motion_control_handle","",false));

  m_marker.header.frame_id = m_robot_base_link;
  m_marker.header.stamp = ros::Time(0);   // makes frame_id const
  m_marker.scale = 0.1;
  m_marker.name = "motion_control_handle";
  tf::poseTFToMsg(m_current_pose,m_marker.pose);
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

void MotionControlHandle::updateMotionControlCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Move marker in RViz
  m_server->setPose(feedback->marker_name,feedback->pose);
  m_server->applyChanges();

  // Store for later broadcasting
  tf::poseMsgToTF(feedback->pose,m_current_pose);

}


void MotionControlHandle::updateMarkerMenuCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
}

void MotionControlHandle::addAxisControl(
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

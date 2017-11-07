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
}

MotionControlHandle::~MotionControlHandle()
{
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

  // Start with the marker at robot end-effector on startup
  m_tf_listener.waitForTransform(
    m_robot_base_link,
    m_end_effector_link,
    ros::Time(0),
    ros::Duration(10)
    );

  tf::StampedTransform tmp;
  m_tf_listener.lookupTransform(
    m_robot_base_link,  // I want my pose displayed in this frame
    m_end_effector_link,
    ros::Time(0),
    tmp);

  geometry_msgs::Pose initial_pose;
  tf::poseTFToMsg(tmp,initial_pose);


  // Configure the interactive marker for usage in RViz
  m_server.reset(new interactive_markers::InteractiveMarkerServer("test_marker","",false));

  m_marker.header.frame_id = "base_link";
  m_marker.header.stamp = ros::Time(0);   // makes frame_id const
  m_marker.name = "my_marker";
  m_marker.pose = initial_pose;
  m_marker.description = "6D Cartesian control of the <end_effector_name> end-effector.";

  // Create a sphere as a handle
  visualization_msgs::Marker visual;
  visual.type = visualization_msgs::Marker::SPHERE;
  visual.scale.x = 0.01;  // bounding box in meter
  visual.scale.y = 0.01;
  visual.scale.z = 0.01;
  visual.color.r = 0.6;
  visual.color.g = 0.0;
  visual.color.b = 0.2;
  visual.color.a = 1.0;

  // Create a non-interactive control for the appearance
  visualization_msgs::InteractiveMarkerControl visual_control;
  visual_control.always_visible = true;
  visual_control.markers.push_back(visual);
  m_marker.controls.push_back(visual_control);

  // Create a control which will move the handle
  visualization_msgs::InteractiveMarkerControl cartesian_control;
  cartesian_control.name = "move 6D";
  cartesian_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  m_marker.controls.push_back(cartesian_control);

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

  // Publish pose to TF
  tf::Transform pose;
  tf::poseMsgToTF(feedback->pose,pose);

  m_tf_broadcaster.sendTransform(
      tf::StampedTransform(
        pose,
        ros::Time::now(),
        m_robot_base_link,
        m_target_frame));
}


void MotionControlHandle::updateMarkerMenuCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
}

}

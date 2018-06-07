// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    MotionControlHandle.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/11/06
 *
 */
//-----------------------------------------------------------------------------

#ifndef MOTION_CONTROL_HANDLE_H_INCLUDED
#define MOTION_CONTROL_HANDLE_H_INCLUDED

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>

// Other
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>

namespace cartesian_controller_handles
{

/**
 * @brief Implements a drag-and-drop control handle in RViz
 *
 * This motion control handle can be used to control the end-effector of a
 * robotic manipulator in Cartesian space with an interactive marker.  The
 * motion of the marker (both orientation and translation) is broadcasted as a
 * \a target_frame to TF, which can be followed by motion-based Cartesian controllers.
 */
class MotionControlHandle
{
  public:
    MotionControlHandle();
    ~MotionControlHandle();

    /**
     * @brief Publish pose of the control handle to TF
     *
     * This function must be called periodically from an outside node.
     * The TF can be used as \a target_frame for \a Cartesian
     * \a Controllers.
     */
    void update();

  private:
    bool init();

    /**
     * @brief Move visual marker in RViz according to user interaction
     *
     * This function also stores the marker pose internally.
     *
     * @param feedback The message containing the current pose of the marker
     */
    void updateMotionControlCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    /**
     * @brief React to changes in the interactive marker menu
     *
     * @param feedback The message containing the current menu configuration
     */
    void updateMarkerMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    /**
     * @brief Adds interactive controls (arrows) to a marker.
     *
     * Both move and rotate controls are added along the specified
     * axis. The axis is normalized.
     *
     * @param marker The marker to which the controls are added
     * @param x X-axis component
     * @param y Y-axis component
     * @param z Z-axis component
     */
    void addAxisControl(visualization_msgs::InteractiveMarker& marker, double x, double y, double z);

    // Kinematics
    std::string               m_robot_base_link;
    std::string               m_end_effector_link;
    std::string               m_target_frame_topic;

    geometry_msgs::PoseStamped  m_current_pose;
    ros::Publisher  m_pose_publisher;
    tf::TransformListener m_tf_listener;

    // Interactive marker
    boost::shared_ptr<
      interactive_markers::InteractiveMarkerServer> m_server;

    visualization_msgs::InteractiveMarker           m_marker; //!< Controller handle for RViz

};

} // cartesian_controller_handles

#endif

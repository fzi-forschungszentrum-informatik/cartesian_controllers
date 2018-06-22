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

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

// Other
#include <boost/shared_ptr.hpp>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace cartesian_controller_handles
{

/**
 * @brief Implements a drag-and-drop control handle in RViz
 *
 * This motion control handle can be used to control the end-effector of a
 * robotic manipulator in Cartesian space with an interactive marker.  The
 * motion of the marker (both orientation and translation) is published as a
 * geometry_msgs/PoseStamped, which can be followed by motion-based Cartesian controllers.
 */
template <class HardwareInterface>
class MotionControlHandle : public controller_interface::Controller<HardwareInterface>
{
  public:
    MotionControlHandle();
    ~MotionControlHandle();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    /**
     * @brief Publish pose of the control handle as PoseStamped
     *
     */
    void update(const ros::Time& time, const ros::Duration& period);

  private:
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

    /**
     * @brief Get the current pose of the specified end-effector
     *
     * @return The current end-effector pose with respect to the specified base link
     */
    geometry_msgs::PoseStamped getEndEffectorPose();

    // Handles to the joints
    std::vector<
      hardware_interface::JointStateHandle>   m_joint_handles;
    std::vector<std::string>  m_joint_names;

    // Kinematics
    std::string   m_robot_base_link;
    std::string   m_end_effector_link;
    std::string   m_target_frame_topic;
    boost::shared_ptr<
      KDL::ChainFkSolverPos_recursive>  m_fk_solver;

    geometry_msgs::PoseStamped  m_current_pose;
    ros::Publisher  m_pose_publisher;

    // Interactive marker
    boost::shared_ptr<
      interactive_markers::InteractiveMarkerServer> m_server;

    visualization_msgs::InteractiveMarker           m_marker; //!< Controller handle for RViz

};

} // cartesian_controller_handles

#include <cartesian_controller_handles/MotionControlHandle.hpp>

#endif

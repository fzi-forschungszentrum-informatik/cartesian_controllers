// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_H_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_H_INCLUDED

// ROS
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// Project
#include <cartesian_motion_controller/ForwardDynamicsSolver.h>
#include <cartesian_motion_controller/SpatialPIDController.h>
#include <cartesian_motion_controller/Utility.h>

// tf
#include <tf/transform_listener.h>

// Other
#include <vector>
#include <string>

namespace cartesian_motion_controller
{

template <class HardwareInterface>
class CartesianMotionController : public controller_interface::Controller<HardwareInterface>
{
  public:
    CartesianMotionController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time&);

    void update(const ros::Time& time, const ros::Duration& period);

  private:

    void writeJointControlCmds(const trajectory_msgs::JointTrajectoryPoint& motion);

    void computeJointControlCmds(
        const ctrl::Vector6D& cartesian_input,  // The orientation is wrt the robot_base_link
        const ros::Duration& period,
        trajectory_msgs::JointTrajectoryPoint& joint_output);

    ctrl::Vector6D        computeMotionError();
    SpatialPIDController  controlMotionError;

    std::vector<hardware_interface::JointHandle>  m_joint_handles;
    std::vector<std::string>                      m_joint_names;
    ForwardDynamicsSolver                         m_forward_dynamics_solver;
    tf::TransformListener                         m_tf_listener;
    trajectory_msgs::JointTrajectoryPoint         m_simulated_joint_motion;
    std::string                                   m_robot_base_link;
    std::string                                   m_end_effector_link;
    std::string                                   m_target_frame;
};

}

#include <cartesian_motion_controller/cartesian_motion_controller.hpp>

#endif

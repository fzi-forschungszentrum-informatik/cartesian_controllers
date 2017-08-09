// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller_base.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_CONTROLLER_BASE_H_INCLUDED
#define CARTESIAN_CONTROLLER_BASE_H_INCLUDED

// ROS
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// Project
#include <cartesian_controller_base/ForwardDynamicsSolver.h>
#include <cartesian_controller_base/SpatialPIDController.h>
#include <cartesian_controller_base/Utility.h>

// Other
#include <vector>
#include <string>

namespace cartesian_controller_base
{

template <class HardwareInterface>
class CartesianControllerBase : public controller_interface::Controller<HardwareInterface>
{
  public:
    CartesianControllerBase();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

  protected:
    void writeJointControlCmds();

    void computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period);

    std::vector<hardware_interface::JointHandle>  m_joint_handles;
    std::vector<std::string>                      m_joint_names;
    ForwardDynamicsSolver                         m_forward_dynamics_solver;
    trajectory_msgs::JointTrajectoryPoint         m_simulated_joint_motion;
    SpatialPIDController                          m_spatial_controller;
    ctrl::Vector6D                                m_cartesian_input;
    std::string                                   m_robot_base_link;
    std::string                                   m_end_effector_link;
};

}

#include <cartesian_controller_base/cartesian_controller_base.hpp>

#endif

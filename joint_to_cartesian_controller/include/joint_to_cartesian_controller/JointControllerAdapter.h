// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    JointControllerAdapter.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/16
 *
 */
//-----------------------------------------------------------------------------

#ifndef JOINT_CONTROLLER_ADAPTER_H_INCLUDED
#define JOINT_CONTROLLER_ADAPTER_H_INCLUDED

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ROS control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

// KDL
#include <kdl/jntarray.hpp>

// Other
#include <vector>

namespace joint_to_cartesian_controller
{

class JointControllerAdapter : public hardware_interface::RobotHW
{
  public:

    JointControllerAdapter();
    ~JointControllerAdapter();

    bool init(const std::vector<hardware_interface::JointStateHandle>& handles, ros::NodeHandle& nh);

    void write(KDL::JntArray& positions);

  private:
    //! Number of actuated joints
    int m_number_joints;

    //! Actuated joints in order from base to tip
    std::vector<std::string> m_joint_names;

    hardware_interface::JointStateInterface m_state_interface;
    hardware_interface::PositionJointInterface m_pos_interface;

    joint_limits_interface::PositionJointSoftLimitsInterface m_limits_interface;

    std::vector<hardware_interface::JointHandle>                        m_joint_handles;
    std::vector<joint_limits_interface::PositionJointSoftLimitsHandle>  m_limits_handles;

    std::vector<double> m_cmd;
};

} // end namespace

#endif

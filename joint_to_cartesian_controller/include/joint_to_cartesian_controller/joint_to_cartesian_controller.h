// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    joint_to_cartesian_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/15
 *
 */
//-----------------------------------------------------------------------------

#ifndef JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED
#define JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED

// ROS
#include <ros/ros.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>

namespace joint_to_cartesian_controller
{

class JointToCartesianController
  : public controller_interface::Controller<hardware_interface::JointStateInterface>
{
  public:
    JointToCartesianController();

    bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);
};

}

#endif

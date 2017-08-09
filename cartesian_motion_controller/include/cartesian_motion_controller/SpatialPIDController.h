// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    SpatialPIDController.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/28
 *
 */
//-----------------------------------------------------------------------------

#ifndef SPATIAL_PID_CONTROLLER_H_INCLUDED
#define SPATIAL_PID_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_motion_controller/Utility.h>

// ROS
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>

namespace cartesian_motion_controller
{

class SpatialPIDController
{
  public:
    SpatialPIDController();

    bool init(ros::NodeHandle& nh);

    ctrl::Vector6D operator()(const ctrl::Vector6D& error, const ros::Duration& period);

  private:
    ctrl::Vector6D                    m_cmd;
    std::vector<control_toolbox::Pid> m_pid_controllers;

};

} // namespace

#endif

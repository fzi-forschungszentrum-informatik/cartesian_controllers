////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

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

/**
 * @brief A controller adapter in form of a ROS-control hardware interface
 */
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

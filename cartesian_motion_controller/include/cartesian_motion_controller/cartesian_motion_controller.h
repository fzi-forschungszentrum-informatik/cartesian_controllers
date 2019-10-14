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
/*!\file    cartesian_motion_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_H_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_controller_base/cartesian_controller_base.h>

// ROS
#include <kdl/frames.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace cartesian_motion_controller
{

/**
 * @brief A ROS-control controller for Cartesian motion tracking
 *
 * This controller is meant for tracking Cartesian motion that is not known in
 * advance.  Common use cases are teleoperation or Cartesian end effector
 * teaching, in which the Cartesian motion is commanded with discrete target
 * poses.
 *
 * The controller receives the targets as \a geometry_msgs::PoseStamped
 * messages and tries to reach those as best as possible.  Users can adjust the
 * controller's responsiveness to those targets with setting individual PD
 * gains for each Cartesian dimension.
 *
 * One benefit is that the controller automatically interpolates to obtain
 * smooth joint commands for distant, discretely sampled targets.
 * Users achieve this with setting qualitatively low P gains.
 *
 * For uses cases where a more precise tracking is needed, users may configure
 * this controller to a fast Inverse Kinematics solver, with setting
 * qualitatively high P gains. Note, however, that this requires
 * high-frequently published targets to avoid jumps on joint level.
 *
 * @tparam HardwareInterface The interface to support. Either PositionJointInterface or VelocityJointInterface
 */
template <class HardwareInterface>
class CartesianMotionController : public virtual cartesian_controller_base::CartesianControllerBase<HardwareInterface>
{
  public:
    CartesianMotionController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;

  protected:
    /**
     * @brief Compute the offset between a target pose and the current end effector pose
     *
     * The pose offset is formulated with a translational component and a rotational
     * component, using Rodrigues vector notation.
     *
     * The robot's current pose is computed with forward kinematics, using either
     * virtually simulated joint positions (for PositionJointInterface), or
     * real joint positions (for VelocityJointInterface).
     *
     * @return The error as a 6-dim vector (linear, angular) w.r.t to the robot base link
     */
    ctrl::Vector6D        computeMotionError();

  private:
    void targetFrameCallback(const geometry_msgs::PoseStamped& pose);

    ros::Subscriber m_target_frame_subscr;
    std::string     m_target_frame_topic;
    KDL::Frame      m_target_frame;
    KDL::Frame      m_current_frame;
};

}

#include <cartesian_motion_controller/cartesian_motion_controller.hpp>

#endif

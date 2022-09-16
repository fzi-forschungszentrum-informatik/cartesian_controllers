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
/*!\file    joint_to_cartesian_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/15
 *
 */
//-----------------------------------------------------------------------------

#ifndef JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED
#define JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED

// Project
#include <joint_to_cartesian_controller/JointControllerAdapter.h>

// ROS
#include <geometry_msgs/PoseStamped.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace joint_to_cartesian_controller
{

/**
 * @brief A controller to turn joint trajectories into a moving Cartesian target pose
 *
 * Use this controller, if you have (for some reason) only joint-based
 * trajectories and want to use Cartesian controllers from the
 * cartesian_controller package.
 *
 * This controller handles an internal controller manager, which can load
 * standard ros_controllers. The control commands from these controllers are
 * turned into Cartesian poses with forward kinematics, and can be used by
 * the Cartesian_controllers. An application of this controller is to
 * provide an easy interface to the rqt_joint_trajectory_controller plugin and
 * MoveIt!.
 *
 * Note, however, that transforming joint motion into Cartesian motion for
 * target following loses explicit control over the joints and collision checking.
 */
class JointToCartesianController
  : public controller_interface::Controller<hardware_interface::JointStateInterface>
{
  public:
    JointToCartesianController();

    bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

  private:
    std::string                m_end_effector_link;
    std::string                m_robot_base_link;
    std::string                m_target_frame_topic;
    KDL::JntArray              m_positions;
    KDL::JntArray              m_velocities;
    std::vector<std::string>   m_joint_names;
    ros::Publisher             m_pose_publisher;

    JointControllerAdapter     m_controller_adapter;

    KDL::Chain m_robot_chain;
    std::vector<
      hardware_interface::JointStateHandle>   m_joint_handles;

    std::shared_ptr<
      KDL::ChainFkSolverPos_recursive>        m_fk_solver;

    std::shared_ptr<
      controller_manager::ControllerManager>  m_controller_manager;

};

}

#endif

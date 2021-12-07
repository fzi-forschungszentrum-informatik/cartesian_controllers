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
/*!\file    IKSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------


#ifndef IKSOLVER_H_INCLUDED
#define IKSOLVER_H_INCLUDED

// Project
#include <cartesian_controller_base/Utility.h>

// ros_controls
#include <hardware_interface/joint_command_interface.h>

// ros general
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// other
#include <vector>
#include <memory>

// KDL
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

namespace cartesian_controller_base{

/*! \brief Base class to compute manipulator joint motion from Cartesian force inputs.
 *
 *  This is a base class for solvers, whose child classes will implement
 *  different inverse kinematics algorithms.
 */
class IKSolver
{
  public:
    IKSolver();
    virtual ~IKSolver();

    /**
     * @brief Compute joint target commands, using specific IK algorithms
     *
     * The resulting motion will be forwarded as reference to the low-level
     * joint control.
     *
     * @param period The duration in sec for this simulation step
     * @param net_force The applied net force, expressed in the root frame
     *
     * @return A point holding positions, velocities and accelerations of each joint
     */
    virtual trajectory_msgs::JointTrajectoryPoint getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force) = 0;

    /**
     * @brief Get the current end effector pose of the simulated robot
     *
     * The last link in the chain from the init() function is taken as end
     * effector. If \ref setStartState() has been called immediately before,
     * then the returned pose represents the real robots end effector pose.
     *
     * @return The end effector pose with respect to the robot base link. This
     * link is the same as the one implicitly given in the init() function.
     */
    const KDL::Frame& getEndEffectorPose() const;

    /**
     * @brief Get the current end effector velocity of the simulated robot
     *
     * The last link in the chain from the init() function is taken as end
     * effector.
     *
     * @return The end effector vel with respect to the robot base link. The
     * order is first translation, then rotation.
     */
    const ctrl::Vector6D& getEndEffectorVel() const;

    /**
     * @brief Get the current joint positions of the simulated robot
     *
     * @return The current joint positions
     */
    const KDL::JntArray& getPositions() const;

    //! Set initial joint configuration
    bool setStartState(const std::vector<hardware_interface::JointHandle>& joint_handles);

    /**
     * @brief Synchronize joint positions with the real robot
     *
     * Call this periodically in the controller's update() function.
     * The internal model's joint velocity is not sychronized. Derived IK
     * solvers should implement how to keep or reset those values.
     *
     * @param joint_handles Read handles to the joints.
     */
    void synchronizeJointPositions(const std::vector<hardware_interface::JointHandle>& joint_handles);

    /**
     * @brief Initialize the solver
     *
     * @param nh A node handle for namespace-local parameter management
     * @param chain The kinematic chain of the robot
     * @param upper_pos_limits Tuple with max positive joint angles
     * @param lower_pos_limits Tuple with max negative joint angles
     *
     * @return True, if everything went well
     */
    virtual bool init(ros::NodeHandle& nh,
                      const KDL::Chain& chain,
                      const KDL::JntArray& upper_pos_limits,
                      const KDL::JntArray& lower_pos_limits);

    /**
     * @brief Update the robot kinematics of the solver
     *
     * Call this periodically to update the internal simulation's forward
     * kinematics.
     */
    void updateKinematics();

  protected:

    /**
     * @brief Make sure positions stay in allowed margins
     *
     * Limit internal joint buffers to the position limits provided by URDF.
     * A joint will be treated as continuous, if the continuous type is set for this joint.
     * If both upper and lower limits are zero, the joint appears fixed.  Note
     * that this is the default urdf initializer if limits are omitted.
     */
    void applyJointLimits();

    //! The underlying physical system
    KDL::Chain m_chain;

    //! Number of controllable joint
    int m_number_joints;

    // Internal buffers
    KDL::JntArray m_current_positions;
    KDL::JntArray m_current_velocities;
    KDL::JntArray m_current_accelerations;
    KDL::JntArray m_last_positions;
    KDL::JntArray m_last_velocities;

    // Joint limits
    KDL::JntArray m_upper_pos_limits;
    KDL::JntArray m_lower_pos_limits;

    // Forward kinematics
    std::shared_ptr<
      KDL::ChainFkSolverPos_recursive>  m_fk_pos_solver;
    std::shared_ptr<
      KDL::ChainFkSolverVel_recursive>  m_fk_vel_solver;
    KDL::Frame                          m_end_effector_pose;
    ctrl::Vector6D                      m_end_effector_vel;
};


} // namespace

#endif

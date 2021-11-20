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
/*!\file    JacobianTransposeSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/26
 *
 */
//-----------------------------------------------------------------------------

#ifndef JACOBIAN_TRANSPOSE_SOLVER_H_INCLUDED
#define JACOBIAN_TRANSPOSE_SOLVER_H_INCLUDED

// Project
#include <cartesian_controller_base/IKSolver.h>

// KDL
#include <kdl/jacobian.hpp>

namespace cartesian_controller_base{

  /**
   * \brief A Jacobian transpose IK solver for Cartesian controllers
   *
   *
 *  The resulting joint accelerations are computed according to
 *  \f$ \ddot{q} = J^T f \f$
 *  Where \f$ J \f$ denotes the manipulator's joint Jacobian and \f$ f \f$ is
 *  the applied force to the end effector.
 *  This implements the dynamical system from Wolovich and Elliot
 *  https://ieeexplore.ieee.org/abstract/document/4048118 with \f$ \alpha = 1 \f$ with
 *  the difference that this implementation does not accumulate velocity during time integration.
 *  The system always starts anew in each control cycle with instantaneous
 *  accelerations, having the benefit that no damping is required to avoid overshooting.
   */
class JacobianTransposeSolver : public IKSolver
{
  public:
    JacobianTransposeSolver();
    ~JacobianTransposeSolver();

    /**
     * \brief Compute joint target commands with the Jacobian transpose
     *
     * \param period The duration in sec for this simulation step
     * \param net_force The applied net force, expressed in the root frame
     *
     * \return A point holding positions, velocities and accelerations of each joint
     */
    trajectory_msgs::JointTrajectoryPoint getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force);

    /**
     * \brief Initialize the solver
     *
     * \param nh A node handle for namespace-local parameter management
     * \param chain The kinematic chain of the robot
     * \param upper_pos_limits Tuple with max positive joint angles
     * \param lower_pos_limits Tuple with max negative joint angles
     *
     * \return True, if everything went well
     */
    bool init(ros::NodeHandle& nh,
              const KDL::Chain& chain,
              const KDL::JntArray& upper_pos_limits,
              const KDL::JntArray& lower_pos_limits);

  private:
    std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
    KDL::Jacobian m_jnt_jacobian;
};

}

#endif

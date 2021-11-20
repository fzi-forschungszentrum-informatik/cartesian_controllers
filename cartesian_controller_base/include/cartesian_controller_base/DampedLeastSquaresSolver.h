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
/*!\file    DampedLeastSquaresSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef DAMPED_LEAST_SQUARES_SOLVER_H_INCLUDED
#define DAMPED_LEAST_SQUARES_SOLVER_H_INCLUDED

// Project
#include <cartesian_controller_base/IKSolver.h>

// KDL
#include <kdl/jacobian.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cartesian_controller_base/DampedLeastSquaresSolverConfig.h>

namespace cartesian_controller_base{

  /**
   * \brief A damped least squares IK solver for Cartesian controllers
   *
   *
   *  The resulting joint velocities are computed according to
   *  \f$ \dot{q} = ( J^T J + \alpha^2 I )^{-1} J^T f \f$
   *  Where \f$ J \f$ denotes the manipulator's joint Jacobian and \f$ f \f$ is
   *  the applied force to the end effector.
   *  \f$ \alpha \f$ is a damping term.
   *  For controlling end effector motion, e.g. in the
   *  \ref cartesian_motion_controller::CartesianMotionController \f$ f \f$ should be
   *  thought of as an error direction vector that is mapped to wrench space
   *  with a unit stiffness.
   *
   *  The damped least squares formulation is according to Wampler
   *  https://ieeexplore.ieee.org/abstract/document/4075580  
   */
class DampedLeastSquaresSolver : public IKSolver
{
  public:
    DampedLeastSquaresSolver();
    ~DampedLeastSquaresSolver();

    /**
     * \brief Compute joint target commands with damped least squares
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
    double m_alpha;

    // IK solver specific dynamic reconfigure
    typedef cartesian_controller_base::DampedLeastSquaresSolverConfig
      IKConfig;

    void dynamicReconfigureCallback(IKConfig& config, uint32_t level);

    std::shared_ptr<dynamic_reconfigure::Server<IKConfig> > m_dyn_conf_server;
    dynamic_reconfigure::Server<IKConfig>::CallbackType m_callback_type;
};

}

#endif

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

// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    SelectivelyDampedLeastSquaresSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/06/21
 *
 */
//-----------------------------------------------------------------------------

#ifndef SELECTIVELY_DAMPED_LEAST_SQUARES_SOLVER_H_INCLUDED
#define SELECTIVELY_DAMPED_LEAST_SQUARES_SOLVER_H_INCLUDED

// Project
#include <cartesian_controller_base/IKSolver.h>

// KDL
#include <kdl/jacobian.hpp>

namespace cartesian_controller_base{

  /**
   * \brief A selectively damped least squares (SDLS) IK solver for Cartesian controllers
   *
   *  This implements the SDLS method by Buss and Kim from 2005.
   *
   *  The implementation is according to their paper:
   *  https://www.tandfonline.com/doi/abs/10.1080/2151237X.2005.10129202
   *
   *  which is available here:
   *  https://www.researchgate.net/profile/Samuel_Buss/publication/220494116_Selectively_Damped_Least_Squares_for_Inverse_Kinematics/links/09e4150cc04794d9d0000000/Selectively-Damped-Least-Squares-for-Inverse-Kinematics.pdf
   *
   *  It has the advantage over the DLS method in that it converges faster and
   *  does not require ad-hoc damping terms, i.e. users do not need to specify
   *  task dependent damping values, which can otherwise require numerous
   *  trials and expertise.  It is, however, more computationally evolved.
   *
   */
class SelectivelyDampedLeastSquaresSolver : public IKSolver
{
  public:
    SelectivelyDampedLeastSquaresSolver();
    ~SelectivelyDampedLeastSquaresSolver();

    /**
     * \brief Compute joint target commands with selectively damped least squares
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
    /**
     * @brief Helper function to clamp a column vector
     *
     * This literally implements ClampMaxAbs() from Buss' and Kim's paper.
     *
     * @param w The vector to clamp
     * @param d The threshold for the max allowed value
     *
     * @return The clamped vector
     */
    Eigen::Matrix<double, Eigen::Dynamic, 1>
    clampMaxAbs(const Eigen::Matrix<double, Eigen::Dynamic, 1>& w, double d);

    std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
    KDL::Jacobian m_jnt_jacobian;

};

}

#endif

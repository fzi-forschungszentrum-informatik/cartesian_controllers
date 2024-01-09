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
/*!\file    ForwardDynamicsSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

#ifndef FORWARD_DYNAMICS_SOLVER_H_INCLUDED
#define FORWARD_DYNAMICS_SOLVER_H_INCLUDED

#include <cartesian_controller_base/IKSolver.h>
#include <cartesian_controller_base/Utility.h>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <memory>
#include <vector>

#include "rclcpp/node.hpp"

namespace cartesian_controller_base
{
/*! \brief The default IK solver for Cartesian controllers
 *
 *  This class computes manipulator joint motion from Cartesian force inputs.
 *  As inputs, both forces and torques are applied to the mechanical system,
 *  representing the manipulator. The system is modeled as a chain of rigid
 *  bodies. All bodies except for the last one are massless. This is to
 *  achieve a nearly linear behavior in all joint configurations.
 *  The resulting joint accelerations are computed according to
 *  \f$ \ddot{q} = H^{-1} ( J^T f) \f$
 *  Where \f$ H \f$ denotes the joint space inertia matrix of the virtually
 *  conditioned system, \f$ J \f$ denotes the joint Jacobian and \f$ f \f$ is
 *  the applied force to the end effector.  The joint accelerations are
 *  integrated twice to obtain joint velocities and joint positions
 *  respectively.
 *  Check more details behind the solver here: https://arxiv.org/pdf/1908.06252.pdf
 */
class ForwardDynamicsSolver : public IKSolver
{
public:
  ForwardDynamicsSolver();
  ~ForwardDynamicsSolver();

  /**
     * @brief Compute joint target commands with approximate forward dynamics
     *
     * The resulting motion is the output of a forward dynamics simulation. It
     * can be forwarded to a real controller to mimic the simulated behavior.
     *
     * @param period The duration in sec for this simulation step
     * @param net_force The applied net force, expressed in the root frame
     *
     * @return A point holding positions, velocities and accelerations of each joint
     */
  trajectory_msgs::msg::JointTrajectoryPoint getJointControlCmds(
    rclcpp::Duration period, const ctrl::Vector6D & net_force) override;

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
#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
  bool init(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> nh,
#else
  bool init(std::shared_ptr<rclcpp::Node> nh,
#endif
            const KDL::Chain & chain, const KDL::JntArray & upper_pos_limits,
            const KDL::JntArray & lower_pos_limits) override;

private:
  //! Build a generic robot model for control
  bool buildGenericModel();

  // Forward dynamics
  std::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
  std::shared_ptr<KDL::ChainDynParam> m_jnt_space_inertia_solver;
  KDL::Jacobian m_jnt_jacobian;
  KDL::JntSpaceInertiaMatrix m_jnt_space_inertia;

  // Dynamic parameters
  std::shared_ptr<rclcpp::Node> m_handle;  ///< handle for dynamic parameter interaction
  const std::string m_params = "solver/forward_dynamics";  ///< namespace for parameter access

  /**
     * Virtual link mass
     * Virtual mass of the manipulator's links. The smaller this value, the
     * more does the end-effector (which has a unit mass of 1.0) dominate dynamic
     * behavior. Near singularities, a bigger value leads to smoother motion.
     */
  std::atomic<double> m_min = 0.1;
};

}  // namespace cartesian_controller_base

#endif

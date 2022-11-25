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
/*!\file    DampedLeastSquaresSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/27
 *
 */
//-----------------------------------------------------------------------------

// this package
#include <cartesian_controller_base/DampedLeastSquaresSolver.h>

// Pluginlib
#include <pluginlib/class_list_macros.h>

/**
 * \class cartesian_controller_base::DampedLeastSquaresSolver 
 *
 * Users may explicitly specify this solver with \a "damped_least_squares" as \a
 * ik_solver in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *     type: "<type_of_your_controller>"
 *     ik_solver: "damped_least_squares"
 *     ...
 *
 *     solver:
 *         ...
 *         damped_least_squares:
 *             alpha: 0.5
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::DampedLeastSquaresSolver, cartesian_controller_base::IKSolver)





namespace cartesian_controller_base{

  DampedLeastSquaresSolver::DampedLeastSquaresSolver()
    : m_alpha(0.01)
  {
  }

  DampedLeastSquaresSolver::~DampedLeastSquaresSolver(){}

  trajectory_msgs::JointTrajectoryPoint DampedLeastSquaresSolver::getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force)
  {
    // Compute joint jacobian
    m_jnt_jacobian_solver->JntToJac(m_current_positions,m_jnt_jacobian);

    // Compute joint velocities according to:
    // \f$ \dot{q} = ( J^T J + \alpha^2 I )^{-1} J^T f \f$
    ctrl::MatrixND identity;
    identity.setIdentity(m_number_joints, m_number_joints);

    m_current_velocities.data =
      (m_jnt_jacobian.data.transpose() * m_jnt_jacobian.data
       + m_alpha * m_alpha * identity).inverse() * m_jnt_jacobian.data.transpose() * net_force;

    // Integrate once, starting with zero motion
    m_current_positions.data = m_last_positions.data + 0.5 * m_current_velocities.data * period.toSec();

    // Make sure positions stay in allowed margins
    applyJointLimits();

    // Apply results
    trajectory_msgs::JointTrajectoryPoint control_cmd;
    for (int i = 0; i < m_number_joints; ++i)
    {
      control_cmd.positions.push_back(m_current_positions(i));
      control_cmd.velocities.push_back(m_current_velocities(i));

      // Accelerations should be left empty. Those values will be interpreted
      // by most hardware joint drivers as max. tolerated values. As a
      // consequence, the robot will move very slowly.
    }
    control_cmd.time_from_start = period; // valid for this duration

    // Update for the next cycle
    m_last_positions = m_current_positions;

    return control_cmd;
  }

  bool DampedLeastSquaresSolver::init(ros::NodeHandle& nh,
                                      const KDL::Chain& chain,
                                      const KDL::JntArray& upper_pos_limits,
                                      const KDL::JntArray& lower_pos_limits)
  {
    IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

    m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
    m_jnt_jacobian.resize(m_number_joints);

    // Connect dynamic reconfigure and overwrite the default values with values
    // on the parameter server. This is done automatically if parameters with
    // the according names exist.
    m_callback_type = std::bind(&DampedLeastSquaresSolver::dynamicReconfigureCallback,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2);

    m_dyn_conf_server.reset(
        new dynamic_reconfigure::Server<IKConfig>(
          ros::NodeHandle(nh.getNamespace() + "/solver/damped_least_squares")));
    m_dyn_conf_server->setCallback(m_callback_type);
    return true;
  }

    void DampedLeastSquaresSolver::dynamicReconfigureCallback(IKConfig& config, uint32_t level)
    {
      m_alpha = config.alpha;
    }

} // namespace

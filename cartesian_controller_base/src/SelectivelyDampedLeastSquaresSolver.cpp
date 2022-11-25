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
/*!\file    SelectivelyDampedLeastSquaresSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2020/03/27
 *
 */
//-----------------------------------------------------------------------------

// this package
#include <cartesian_controller_base/SelectivelyDampedLeastSquaresSolver.h>

// Pluginlib
#include <pluginlib/class_list_macros.h>

/**
 * \class cartesian_controller_base::SelectivelyDampedLeastSquaresSolver 
 *
 * Users may explicitly specify this solver with \a "selectively_damped_least_squares" as \a
 * ik_solver in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *     type: "<type_of_your_controller>"
 *     ik_solver: "selectively_damped_least_squares"
 *     ...
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::SelectivelyDampedLeastSquaresSolver, cartesian_controller_base::IKSolver)





namespace cartesian_controller_base{

  SelectivelyDampedLeastSquaresSolver::SelectivelyDampedLeastSquaresSolver()
  {
  }

  SelectivelyDampedLeastSquaresSolver::~SelectivelyDampedLeastSquaresSolver(){}

  trajectory_msgs::JointTrajectoryPoint SelectivelyDampedLeastSquaresSolver::getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force)
  {
    // Compute joint Jacobian
    m_jnt_jacobian_solver->JntToJac(m_current_positions,m_jnt_jacobian);

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic> > JSVD(
      m_jnt_jacobian.data, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 6, 6> U = JSVD.matrixU();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V = JSVD.matrixV();
    Eigen::Matrix<double, Eigen::Dynamic, 1> s = JSVD.singularValues();

    // Default recommendation by Buss and Kim.
    const double gamma_max = 3.141592653 / 4;

    Eigen::Matrix<double, Eigen::Dynamic, 1> sum_phi =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(m_number_joints);

    // Compute each joint velocity with the SDLS method.  This implements the
    // algorithm as described in the paper (but for only one end-effector).
    // Also see Buss' own implementation:
    // https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/index.html

    for (int i = 0; i < m_number_joints; ++i)
    {
      double alpha = U.col(i).transpose() * net_force;

      double N = U.col(i).head(3).norm();
      double M = 0;
      for (int j = 0; j < m_number_joints; ++j)
      {
        double rho = m_jnt_jacobian.data.col(j).head(3).norm();
        M += std::abs(V.col(i)[j]) * rho;
      }
      M *= 1.0 / s[i];

      double gamma = std::min(1.0, N / M) * gamma_max;

      Eigen::Matrix<double, Eigen::Dynamic, 1> phi = clampMaxAbs(1.0 / s[i] * alpha * V.col(i), gamma);
      sum_phi += phi;
    }

    m_current_velocities.data = clampMaxAbs(sum_phi, gamma_max);

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

  bool SelectivelyDampedLeastSquaresSolver::init(ros::NodeHandle& nh,
                                      const KDL::Chain& chain,
                                      const KDL::JntArray& upper_pos_limits,
                                      const KDL::JntArray& lower_pos_limits)
  {
    IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

    m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
    m_jnt_jacobian.resize(m_number_joints);

    return true;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 1> SelectivelyDampedLeastSquaresSolver::clampMaxAbs(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& w, double d)
  {
    if (w.cwiseAbs().maxCoeff() <= d)
    {
      return w;
    }
    else
    {
      return d * w / w.cwiseAbs().maxCoeff();
    }
  }

} // namespace

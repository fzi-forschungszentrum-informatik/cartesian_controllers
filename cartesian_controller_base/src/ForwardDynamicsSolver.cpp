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
/*!\file    ForwardDynamicsSolver.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------

// this package
#include <cartesian_controller_base/ForwardDynamicsSolver.h>

// other
#include <map>
#include <sstream>
#include <boost/algorithm/clamp.hpp>
#include <eigen_conversions/eigen_kdl.h>

// KDL
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>

// DEBUG


namespace cartesian_controller_base{

  ForwardDynamicsSolver::ForwardDynamicsSolver()
  {
  }

  ForwardDynamicsSolver::~ForwardDynamicsSolver(){}

  trajectory_msgs::JointTrajectoryPoint ForwardDynamicsSolver::getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force)
  {

    // Compute joint space inertia matrix
    m_jnt_space_inertia_solver->JntToMass(m_current_positions,m_jnt_space_inertia);

    // Compute joint jacobian
    m_jnt_jacobian_solver->JntToJac(m_current_positions,m_jnt_jacobian);

    // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
    m_current_accelerations.data = m_jnt_space_inertia.data.inverse() * m_jnt_jacobian.data.transpose() * net_force;

    // Integrate once, starting with zero motion
    m_current_velocities.data = 0.5 * m_current_accelerations.data * period.toSec();

    // Integrate twice, starting with zero motion
    m_current_positions.data = m_last_positions.data + 0.5 * m_current_velocities.data * period.toSec();

    // Make sure positions stay in allowed margins
    for (int i = 0; i < m_number_joints; ++i)
    {
      m_current_positions(i) = boost::algorithm::clamp(
          m_current_positions(i),m_lower_pos_limits(i),m_upper_pos_limits(i));
    }

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

    return control_cmd;
  }


  const KDL::Frame& ForwardDynamicsSolver::getEndEffectorPose() const
  {
    return m_end_effector_pose;
  }

  const ctrl::Vector6D& ForwardDynamicsSolver::getEndEffectorVel() const
  {
    return m_end_effector_vel;
  }

  const KDL::JntArray& ForwardDynamicsSolver::getPositions() const
  {
    return m_current_positions;
  }


  bool ForwardDynamicsSolver::setStartState(
      const std::vector<hardware_interface::JointHandle>& joint_handles)
  {
    // Copy into internal buffers.
    for (int i = 0; i < joint_handles.size(); ++i)
    {
      m_current_positions(i)      = joint_handles[i].getPosition();
      m_current_velocities(i)     = joint_handles[i].getVelocity();
      m_current_accelerations(i)  = 0.0;
      m_last_positions(i)         = m_current_positions(i);
    }
    return true;
  }


  bool ForwardDynamicsSolver::init(
      const KDL::Chain& chain,
      const KDL::JntArray& upper_pos_limits,
      const KDL::JntArray& lower_pos_limits)
  {
    if (!buildGenericModel(chain))
    {
      ROS_ERROR("ForwardDynamicsSolver: Something went wrong in setting up the internal model.");
      return false;
    }

    // Initialize
    m_number_joints              = m_chain.getNrOfJoints();
    m_current_positions.data     = ctrl::VectorND::Zero(m_number_joints);
    m_current_velocities.data    = ctrl::VectorND::Zero(m_number_joints);
    m_current_accelerations.data = ctrl::VectorND::Zero(m_number_joints);
    m_last_positions.data        = ctrl::VectorND::Zero(m_number_joints);
    m_upper_pos_limits           = upper_pos_limits;
    m_lower_pos_limits           = lower_pos_limits;

    // Forward kinematics
    m_fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
    m_fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(m_chain));

    // Forward dynamics
    m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
    m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain,KDL::Vector::Zero()));
    m_jnt_jacobian.resize(m_number_joints);
    m_jnt_space_inertia.resize(m_number_joints);

    ROS_INFO("Forward dynamics solver initialized");
    ROS_INFO("Forward dynamics solver has control over %i joints", m_number_joints);

    return true;
  }

  bool ForwardDynamicsSolver::buildGenericModel(const KDL::Chain& input_chain)
  {
    m_chain = input_chain;

    // Set all masses and inertias to minimal (yet stable) values.
    double m_min = 0.001;
    double ip_min = 0.000001;
    for (size_t i = 0; i < m_chain.segments.size(); ++i)
    {
      // Fixed joint segment
      if (m_chain.segments[i].getJoint().getType() == KDL::Joint::None)
      {
        m_chain.segments[i].setInertia(
            KDL::RigidBodyInertia::Zero());
      }
      else  // relatively moving segment
      {
        m_chain.segments[i].setInertia(
            KDL::RigidBodyInertia(
              m_min,                // mass
              KDL::Vector::Zero(),  // center of gravity
              KDL::RotationalInertia(
                ip_min,             // ixx
                ip_min,             // iyy
                ip_min              // izz
                // ixy, ixy, iyz default to 0.0
                )));
      }
    }

    // Only give the last segment a generic mass and inertia.
    // See https://arxiv.org/pdf/1908.06252.pdf for a motivation for this setting.
    double m = 1;
    double ip = 1;
    m_chain.segments[m_chain.segments.size()-1].setInertia(
        KDL::RigidBodyInertia(
          m,
          KDL::Vector::Zero(),
          KDL::RotationalInertia(ip, ip, ip)));

    return true;
  }

} // namespace

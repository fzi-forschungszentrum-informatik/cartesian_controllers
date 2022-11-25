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
 * \date    2020/03/24
 *
 */
//-----------------------------------------------------------------------------

// this package
#include <cartesian_controller_base/ForwardDynamicsSolver.h>

// other
#include <map>
#include <sstream>
#include <eigen_conversions/eigen_kdl.h>

// KDL
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>

// Pluginlib
#include <pluginlib/class_list_macros.h>


/**
 * \class cartesian_controller_base::ForwardDynamicsSolver 
 *
 * Users may explicitly specify it with \a "forward_dynamics" as \a ik_solver
 * in their controllers.yaml configuration file for each controller:
 *
 * \code{.yaml}
 * <name_of_your_controller>:
 *     type: "<type_of_your_controller>"
 *     ik_solver: "forward_dynamics"
 *     ...
 *
 *     solver:
 *         ...
 *         forward_dynamics:
 *             link_mass: 0.5
 * \endcode
 *
 */
PLUGINLIB_EXPORT_CLASS(cartesian_controller_base::ForwardDynamicsSolver, cartesian_controller_base::IKSolver)





namespace cartesian_controller_base{

  ForwardDynamicsSolver::ForwardDynamicsSolver()
  {
  }

  ForwardDynamicsSolver::~ForwardDynamicsSolver(){}

  trajectory_msgs::JointTrajectoryPoint ForwardDynamicsSolver::getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force)
  {

    // Compute joint space inertia matrix with actualized link masses
    buildGenericModel();
    m_jnt_space_inertia_solver->JntToMass(m_current_positions,m_jnt_space_inertia);

    // Compute joint jacobian
    m_jnt_jacobian_solver->JntToJac(m_current_positions,m_jnt_jacobian);

    // Compute joint accelerations according to: \f$ \ddot{q} = H^{-1} ( J^T f) \f$
    m_current_accelerations.data = m_jnt_space_inertia.data.inverse() * m_jnt_jacobian.data.transpose() * net_force;

    // Numerical time integration with the Euler forward method
    m_current_positions.data = m_last_positions.data + m_last_velocities.data * period.toSec();
    m_current_velocities.data = m_last_velocities.data + m_current_accelerations.data * period.toSec();
    m_current_velocities.data *= 0.9;  // 10 % global damping against unwanted null space motion.
                                       // Will cause exponential slow-down without input.

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
    m_last_velocities = m_current_velocities;

    return control_cmd;
  }


  bool ForwardDynamicsSolver::init(ros::NodeHandle& nh,
                                   const KDL::Chain& chain,
                                   const KDL::JntArray& upper_pos_limits,
                                   const KDL::JntArray& lower_pos_limits)
  {
    IKSolver::init(nh, chain, upper_pos_limits, lower_pos_limits);

    if (!buildGenericModel())
    {
      ROS_ERROR("ForwardDynamicsSolver: Something went wrong in setting up the internal model.");
      return false;
    }

    // Forward dynamics
    m_jnt_jacobian_solver.reset(new KDL::ChainJntToJacSolver(m_chain));
    m_jnt_space_inertia_solver.reset(new KDL::ChainDynParam(m_chain,KDL::Vector::Zero()));
    m_jnt_jacobian.resize(m_number_joints);
    m_jnt_space_inertia.resize(m_number_joints);

    // Connect dynamic reconfigure and overwrite the default values with values
    // on the parameter server. This is done automatically if parameters with
    // the according names exist.
    m_callback_type = std::bind(&ForwardDynamicsSolver::dynamicReconfigureCallback,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2);

    m_dyn_conf_server.reset(
        new dynamic_reconfigure::Server<IKConfig>(
          ros::NodeHandle(nh.getNamespace() + "/solver/forward_dynamics")));

    m_dyn_conf_server->setCallback(m_callback_type);

    ROS_INFO("Forward dynamics solver initialized");
    ROS_INFO("Forward dynamics solver has control over %i joints", m_number_joints);

    return true;
  }

  bool ForwardDynamicsSolver::buildGenericModel()
  {
    // Set all masses and inertias to minimal (yet stable) values.
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

  void ForwardDynamicsSolver::dynamicReconfigureCallback(IKConfig& config, uint32_t level)
  {
    m_min = config.link_mass;
  }


} // namespace

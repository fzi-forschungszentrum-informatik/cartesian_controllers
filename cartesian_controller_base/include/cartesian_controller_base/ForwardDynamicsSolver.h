// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    ForwardDynamicsSolver.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/14
 *
 */
//-----------------------------------------------------------------------------


#ifndef FORWARD_DYNAMICS_CONTROLLER_H_INCLUDED
#define FORWARD_DYNAMICS_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_controller_base/Utility.h>

// ros_controls
#include <hardware_interface/joint_command_interface.h>

// ros general
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// other
#include <vector>
#include <boost/shared_ptr.hpp>

// KDL
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace cartesian_controller_base{

/*! \brief This class computes manipulator joint motion from Cartesian force inputs.
 *
 *  As inputs, both forces and torques are applied to the mechanical system,
 *  representing the manipulator. The system is modeled as a chain of rigid
 *  bodies. All bodies except for the last one are massless. This is to
 *  achieve a nearly linear behavior in all joint configurations.
 *  The resulting joint accelerations are computed according to
 *  \f$ \ddot{q} = H^{-1} ( J^T f) \f$
 *  Where \f$ H \f$ denotes the joint space inertia matrix, \f$ J \f$ denotes
 *  the joint Jacobian and \f$ f \f$ is the applied force to the end effector.
 *  The joint accelerations are integrated twice to obtain joint velocities and
 *  joint positions respectively.
 */
class ForwardDynamicsSolver
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
    trajectory_msgs::JointTrajectoryPoint getJointControlCmds(
        ros::Duration period,
        const ctrl::Vector6D& net_force);

    /**
     * @brief Get the current end effector pose of the simulated robot
     *
     * The last link in the chain from the init() function is taken as end
     * effector.
     *
     * @return The end effector pose with respect to the robot base link. This
     * link is the same as the one implicitly given in the init() function.
     */
    const tf::StampedTransform& getEndEffectorPose() const;

    /**
     * @brief Get the current joint positions of the simulated robot
     *
     * @return The current joint positions
     */
    const KDL::JntArray& getPositions() const;

    //! Set initial joint configuration
    bool setStartState(const std::vector<hardware_interface::JointHandle>& joint_handles);

    //! Initialize
    bool init(const KDL::Chain& chain,
              const KDL::JntArray& upper_pos_limits,
              const KDL::JntArray& lower_pos_limits);

    //! Call this function during activation
    void updateKinematics();

  private:

    //! Build a generic robot model for control
    bool buildGenericModel(const KDL::Chain& input_chain);

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
    KDL::JntArray m_last_accelerations;

    // Joint limits
    KDL::JntArray m_upper_pos_limits;
    KDL::JntArray m_lower_pos_limits;

    // Forward kinematics
    boost::shared_ptr<
      KDL::ChainFkSolverPos_recursive>  m_fk_pos_solver;
    tf::StampedTransform                m_end_effector_pose;

    // Forward dynamics
    boost::shared_ptr<KDL::ChainJntToJacSolver> m_jnt_jacobian_solver;
    boost::shared_ptr<KDL::ChainDynParam>       m_jnt_space_inertia_solver;
    KDL::Jacobian                               m_jnt_jacobian;
    KDL::JntSpaceInertiaMatrix                  m_jnt_space_inertia;

};


} // namespace

#endif


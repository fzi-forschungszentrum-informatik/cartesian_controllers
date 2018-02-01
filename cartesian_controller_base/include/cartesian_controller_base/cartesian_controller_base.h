// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller_base.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_CONTROLLER_BASE_H_INCLUDED
#define CARTESIAN_CONTROLLER_BASE_H_INCLUDED

// ROS
#include <ros/node_handle.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/WrenchStamped.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

// KDL
#include <kdl/treefksolverpos_recursive.hpp>

// Project
#include <cartesian_controller_base/ForwardDynamicsSolver.h>
#include <cartesian_controller_base/SpatialPIDController.h>
#include <cartesian_controller_base/Utility.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cartesian_controller_base/CartesianControllerConfig.h>

// Other
#include <vector>
#include <string>

namespace cartesian_controller_base
{

template <class HardwareInterface>
class CartesianControllerBase : public controller_interface::Controller<HardwareInterface>
{
  public:
    CartesianControllerBase();
    virtual ~CartesianControllerBase<HardwareInterface>(){};

    virtual bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    virtual void starting(const ros::Time& time);

    void pause(const ros::Time& time);


  protected:
    void writeJointControlCmds();

    void computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period);

    ctrl::Vector6D displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from);
    ctrl::Matrix6D displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from);
    ctrl::Vector6D displayInTipLink(const ctrl::Vector6D& vector, const std::string& to);

    boost::shared_ptr<KDL::TreeFkSolverPos_recursive>
                            m_forward_kinematics_solver;
    ForwardDynamicsSolver   m_forward_dynamics_solver;
    std::string             m_end_effector_link;
    std::string             m_robot_base_link;

  private:
    std::vector<hardware_interface::JointHandle>      m_joint_handles;
    std::vector<std::string>                          m_joint_names;
    trajectory_msgs::JointTrajectoryPoint             m_simulated_joint_motion;
    SpatialPIDController                              m_spatial_controller;
    ctrl::Vector6D                                    m_cartesian_input;

    // Against multi initialization in multi inheritance scenarios
    bool m_already_initialized;;

    // Dynamic reconfigure
    typedef cartesian_controller_base::CartesianControllerConfig
      ControllerConfig;

    void dynamicReconfigureCallback(ControllerConfig& config, uint32_t level);

    boost::shared_ptr<dynamic_reconfigure::Server<ControllerConfig> > m_dyn_conf_server;
    dynamic_reconfigure::Server<ControllerConfig>::CallbackType m_callback_type;
};

}

#include <cartesian_controller_base/cartesian_controller_base.hpp>

#endif

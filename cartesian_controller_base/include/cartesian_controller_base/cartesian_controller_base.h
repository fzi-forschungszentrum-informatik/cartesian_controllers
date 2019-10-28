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
#include <cartesian_controller_base/SpatialPDController.h>
#include <cartesian_controller_base/Utility.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cartesian_controller_base/CartesianControllerConfig.h>

// Other
#include <vector>
#include <string>

namespace cartesian_controller_base
{

/**
 * @brief Base class for each cartesian controller
 *
 * This class implements a common forward dynamics based solver for Cartesian
 * end effector error control, using the ROS-control framework.  Different
 * child class controllers will define what this error represents and should
 * call \ref computeJointControlCmds with that error.  The control commands are
 * sent to the hardware with \ref writeJointControlCmds.
 *
 * @tparam HardwareInterface The interface to support. Either PositionJointInterface or VelocityJointInterface
 */
template <class HardwareInterface>
class CartesianControllerBase : public controller_interface::Controller<HardwareInterface>
{
  public:
    CartesianControllerBase();
    virtual ~CartesianControllerBase<HardwareInterface>(){};

    virtual bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    virtual void starting(const ros::Time& time);

    void pause(const ros::Time& time);

    bool resume(const ros::Time& time);


  protected:
    /**
     * @brief Write joint control commands to the real hardware
     *
     * Depending on the hardware interface used, this is either joint positions
     * or velocities.
     */
    void writeJointControlCmds();

    /**
     * @brief Compute one control step using forward dynamics simulation
     *
     * Check \ref ForwardDynamicsSolver for details.
     *
     * @param error The error to minimize
     * @param period The period for this control cycle
     */
    void computeJointControlCmds(const ctrl::Vector6D& error, const ros::Duration& period);

    /**
     * @brief Display the given vector in the given robot base link
     *
     * @param vector The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
    ctrl::Vector6D displayInBaseLink(const ctrl::Vector6D& vector, const std::string& from);

    /**
     * @brief Display the given tensor in the robot base frame
     *
     * @param tensor The quantity to transform
     * @param from The reference frame where the quantity was formulated
     *
     * @return The quantity in the robot base frame
     */
    ctrl::Matrix6D displayInBaseLink(const ctrl::Matrix6D& tensor, const std::string& from);

    /**
     * @brief Display a given vector in a new reference frame
     *
     * The vector is assumed to be given in the robot base frame.
     *
     * @param vector The quantity to transform
     * @param to The reference frame in which to formulate the quantity
     *
     * @return The quantity in the new frame
     */
    ctrl::Vector6D displayInTipLink(const ctrl::Vector6D& vector, const std::string& to);

    boost::shared_ptr<KDL::TreeFkSolverPos_recursive>
                            m_forward_kinematics_solver;
    ForwardDynamicsSolver   m_forward_dynamics_solver;
    std::string             m_end_effector_link;
    std::string             m_robot_base_link;

    bool m_paused;
    int m_iterations;

  private:
    std::vector<hardware_interface::JointHandle>      m_joint_handles;
    std::vector<std::string>                          m_joint_names;
    trajectory_msgs::JointTrajectoryPoint             m_simulated_joint_motion;
    SpatialPDController                              m_spatial_controller;
    ctrl::Vector6D                                    m_cartesian_input;
    double m_error_scale;

    // Against multi initialization in multi inheritance scenarios
    bool m_already_initialized;;

    // Dynamic reconfigure
    typedef cartesian_controller_base::CartesianControllerConfig ControllerConfig;

    void dynamicReconfigureCallback(ControllerConfig& config, uint32_t level);

    boost::shared_ptr<dynamic_reconfigure::Server<ControllerConfig> > m_dyn_conf_server;
    dynamic_reconfigure::Server<ControllerConfig>::CallbackType m_callback_type;
};

}

#include <cartesian_controller_base/cartesian_controller_base.hpp>

#endif

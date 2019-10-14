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
/*!\file    cartesian_force_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_FORCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_FORCE_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_controller_base/cartesian_controller_base.h>

// ROS
#include <std_srvs/Trigger.h>

namespace cartesian_force_controller
{

/**
 * @brief A ROS-control controller for Cartesian force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::WrenchStamped targets to steer the robot in task space.  The
 * controller additionally listens to the specified force-torque sensor signals
 * and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Cartesian axes.
 * In contact, the controller regulates the net force of the two wrenches to zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 * @tparam HardwareInterface The interface to support. Either PositionJointInterface or VelocityJointInterface
 */
template <class HardwareInterface>
class CartesianForceController : public virtual cartesian_controller_base::CartesianControllerBase<HardwareInterface>
{
  public:
    CartesianForceController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;

  protected:
    /**
     * @brief Compute the net force out of target wrench and measured sensor wrench
     *
     * @return The remaining error wrench, given in robot base frame
     */
    ctrl::Vector6D        computeForceError();
    std::string           m_new_ft_sensor_ref;
    void setFtSensorReferenceFrame(const std::string& new_ref);

  private:
    ctrl::Vector6D        compensateGravity();

    void targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench);
    void ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench);
    bool signalTaringCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    ros::ServiceServer    m_signal_taring_server;
    ros::Subscriber       m_target_wrench_subscriber;
    ros::Subscriber       m_ft_sensor_wrench_subscriber;
    ctrl::Vector6D        m_target_wrench;
    ctrl::Vector6D        m_ft_sensor_wrench;
    ctrl::Vector6D        m_weight_force;
    ctrl::Vector6D        m_grav_comp_during_taring;
    ctrl::Vector3D        m_center_of_mass;
    std::string           m_ft_sensor_ref_link;
    KDL::Frame            m_ft_sensor_transform;
};

}

#include <cartesian_force_controller/cartesian_force_controller.hpp>

#endif

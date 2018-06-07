// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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

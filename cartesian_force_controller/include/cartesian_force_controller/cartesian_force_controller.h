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

// tf
#include <tf/transform_listener.h>

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

  private:
    void targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench);
    void ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench);

    ros::Subscriber       m_target_wrench_subscriber;
    ros::Subscriber       m_ft_sensor_wrench_subscriber;
    ctrl::Vector6D        m_target_wrench;
    ctrl::Vector6D        m_ft_sensor_wrench;
    std::string           m_ft_sensor_ref_link;

    ctrl::Matrix6D        m_damping;
};

}

#include <cartesian_force_controller/cartesian_force_controller.hpp>

#endif

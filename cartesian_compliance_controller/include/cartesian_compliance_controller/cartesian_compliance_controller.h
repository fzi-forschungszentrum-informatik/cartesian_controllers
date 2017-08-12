// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_compliance_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_COMPLIANCE_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_controller_base/cartesian_controller_base.h>

// tf
#include <tf/transform_listener.h>

namespace cartesian_compliance_controller
{

template <class HardwareInterface>
class CartesianComplianceController : public cartesian_controller_base::CartesianControllerBase<HardwareInterface>
{
  public:
    CartesianComplianceController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;

  private:
    ctrl::Vector6D        computeComplianceError();
    void targetWrenchCallback(const geometry_msgs::WrenchStamped& wrench);
    void ftSensorWrenchCallback(const geometry_msgs::WrenchStamped& wrench);

    tf::TransformListener m_tf_listener;
    ros::Subscriber       m_target_wrench_subscriber;
    ros::Subscriber       m_ft_sensor_wrench_subscriber;
    ctrl::Vector6D        m_target_wrench;
    ctrl::Vector6D        m_ft_sensor_wrench;
    std::string           m_ft_sensor_ref_link;
    std::string           m_compliance_ref_link;
    std::string           m_target_frame;
};

}

#include <cartesian_compliance_controller/cartesian_compliance_controller.hpp>

#endif

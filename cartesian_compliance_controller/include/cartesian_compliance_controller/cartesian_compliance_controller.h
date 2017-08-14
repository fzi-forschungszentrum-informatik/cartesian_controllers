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
#include <cartesian_motion_controller/cartesian_motion_controller.h>
#include <cartesian_force_controller/cartesian_force_controller.h>

// tf
#include <tf/transform_listener.h>

namespace cartesian_compliance_controller
{

template <class HardwareInterface>
class CartesianComplianceController
: public cartesian_motion_controller::CartesianMotionController<HardwareInterface>
, public cartesian_force_controller::CartesianForceController<HardwareInterface>
{
  public:
    CartesianComplianceController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;
    typedef cartesian_motion_controller::CartesianMotionController<HardwareInterface> MotionBase;
    typedef cartesian_force_controller::CartesianForceController<HardwareInterface> ForceBase;

  private:
    ctrl::Vector6D        computeComplianceError();

    ctrl::Matrix6D        m_stiffness;
    std::string           m_compliance_ref_link;
};

}

#include <cartesian_compliance_controller/cartesian_compliance_controller.hpp>

#endif

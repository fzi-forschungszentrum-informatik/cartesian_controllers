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
class CartesianForceController : public cartesian_controller_base::CartesianControllerBase<HardwareInterface>
{
  public:
    CartesianForceController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;

  private:
    ctrl::Vector6D        computeForceError();

    std::string           m_ft_sensor_ref_link;
};

}

#include <cartesian_force_controller/cartesian_force_controller.hpp>

#endif

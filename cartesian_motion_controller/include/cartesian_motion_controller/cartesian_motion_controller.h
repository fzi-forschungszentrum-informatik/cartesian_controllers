// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_H_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_H_INCLUDED

// Project
#include <cartesian_controller_base/cartesian_controller_base.h>

// tf
#include <tf/transform_listener.h>

namespace cartesian_motion_controller
{

template <class HardwareInterface>
class CartesianMotionController : public virtual cartesian_controller_base::CartesianControllerBase<HardwareInterface>
{
  public:
    CartesianMotionController();

    bool init(HardwareInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

    typedef cartesian_controller_base::CartesianControllerBase<HardwareInterface> Base;

  private:
    ctrl::Vector6D        computeMotionError();

    tf::TransformListener m_tf_listener;
    std::string           m_target_frame;
};

}

#include <cartesian_motion_controller/cartesian_motion_controller.hpp>

#endif

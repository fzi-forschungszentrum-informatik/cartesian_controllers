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

// ROS
#include <kdl/frames.hpp>
#include <geometry_msgs/PoseStamped.h>

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

  protected:
    ctrl::Vector6D        computeMotionError();

  private:
    void targetFrameCallback(const geometry_msgs::PoseStamped& pose);

    ros::Subscriber m_target_frame_subscr;
    std::string     m_target_frame_topic;
    KDL::Frame      m_target_frame;
    KDL::Frame      m_current_frame;
};

}

#include <cartesian_motion_controller/cartesian_motion_controller.hpp>

#endif

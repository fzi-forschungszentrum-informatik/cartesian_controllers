// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_motion_controller/cartesian_motion_controller.h>

// Other
#include <boost/algorithm/clamp.hpp>

namespace cartesian_motion_controller
{

template <class HardwareInterface>
CartesianMotionController<HardwareInterface>::
CartesianMotionController()
: Base::CartesianControllerBase()
{
}

template <class HardwareInterface>
bool CartesianMotionController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  Base::init(hw,nh);

  if (!nh.getParam("target_frame",m_target_frame))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/target_frame" << " from parameter server");
    return false;
  }

  return true;
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
starting(const ros::Time& time)
{
  Base::starting(time);
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
stopping(const ros::Time& time)
{
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  const int steps = 10;
  for (int i = 0; i < steps; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    ros::Duration internal_period(0.02);

    // Compute the motion error = target - current.
    ctrl::Vector6D error = computeMotionError();

    // Turn Cartesian error into joint motion
    Base::computeJointControlCmds(error,internal_period);
  }

  // Write final commands to the hardware interface
  Base::writeJointControlCmds(period);
}

template <class HardwareInterface>
ctrl::Vector6D CartesianMotionController<HardwareInterface>::
computeMotionError()
{
  // Compute motion error wrt robot_base_link
  tf::StampedTransform current_pose = Base::m_forward_dynamics_solver.getEndEffectorPose();
  tf::StampedTransform target_pose;

  try
  {
    /* // This blocks eternally, but why?

    m_tf_listener.waitForTransform( // returns immediately if frames exist
        m_robot_base_link,
        m_target_frame,
        ros::Time(0),   // anyone will be ok
        ros::Duration(0.1)
        );
    */

    // This lookupTransform() version really uses the latest transform and
    // doesn't limit the lookup to the last common time.
    m_tf_listener.lookupTransform(
        Base::m_robot_base_link,  // I want my pose displayed in this frame
        ros::Time(0),
        m_target_frame,
        ros::Time(0),
        Base::m_robot_base_link,  // Const reference
        target_pose);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN_THROTTLE(3,"CartesianMotionController: %s",e.what());
    return ctrl::Vector6D::Zero();
  }

  // Use KDL math
  KDL::Frame current_pose_kdl;
  KDL::Frame target_pose_kdl;
  tf::transformTFToKDL(current_pose,current_pose_kdl);
  tf::transformTFToKDL(target_pose,target_pose_kdl);

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl = target_pose_kdl * current_pose_kdl.Inverse();

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle    = boost::algorithm::clamp(angle,-max_angle,max_angle);
  distance = boost::algorithm::clamp(distance,-max_distance,max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}

} // namespace

#endif

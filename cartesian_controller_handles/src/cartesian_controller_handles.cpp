// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller_handles.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2018/06/20
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <cartesian_controller_handles/MotionControlHandle.h>

namespace position_controllers
{
  /**
   * @brief Cartesian motion controller handle to expose an interactive marker for end-effector control in RViz.
   */
  typedef cartesian_controller_handles::MotionControlHandle<
    hardware_interface::PositionJointInterface> MotionControlHandle;
}


PLUGINLIB_EXPORT_CLASS(position_controllers::MotionControlHandle, position_controllers::MotionControlHandle)

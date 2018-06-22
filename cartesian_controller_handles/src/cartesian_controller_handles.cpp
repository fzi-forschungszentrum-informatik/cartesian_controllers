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
   *
   * Can be specified as a controller of type position_controllers/MotionControlHandle.
   */
  typedef cartesian_controller_handles::MotionControlHandle<
    hardware_interface::JointStateInterface> MotionControlHandle;
}


PLUGINLIB_EXPORT_CLASS(position_controllers::MotionControlHandle, controller_interface::ControllerBase)

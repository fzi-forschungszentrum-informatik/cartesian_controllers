////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
/*!\file    PoseParameterHandle.h
 *
 * \author  Captain Yoshi <captain.yoshisaur@gmail.com>
 * \date    2023/12/17
 *
 */
//-----------------------------------------------------------------------------

#ifndef POSE_PARAMETER_HANDLE_H_INCLUDED
#define POSE_PARAMETER_HANDLE_H_INCLUDED

// STD
#include <atomic>

// ROS
#include <ros/ros.h>

// ros_control
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_box.h>

// KDL
#include <kdl/frames.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cartesian_controller_base/SpatialPoseConfig.h>

namespace cartesian_controller_base
{

/**
 * @brief A pose parameter getter utility
 *
 * Motivation for this custom implementation:
 *
 *  - Retrieve the pose parameter from dynamic reconfigure
 *
 *  - Check if the parameter has been updated atomically (atomically)
 */
class PoseParameterHandle
{
  public:
    PoseParameterHandle();
    PoseParameterHandle(const PoseParameterHandle& other); ///< RealtimeBuffer needs special treatment
    ~PoseParameterHandle();

    void init(const std::string& name_space);

    bool has_new_pose();

    KDL::Frame get_pose();

  private:
    std::atomic<bool> m_pose_updated{false};

    realtime_tools::RealtimeBuffer<KDL::Frame> m_transform_kdl;

    // Dynamic reconfigure
    typedef cartesian_controller_base::SpatialPoseConfig Config;
    void dynamicReconfigureCallback(Config& config, uint32_t level);

    std::shared_ptr<dynamic_reconfigure::Server<Config> > m_dyn_conf_server;
    dynamic_reconfigure::Server<Config>::CallbackType m_callback_type;

};

}

#endif

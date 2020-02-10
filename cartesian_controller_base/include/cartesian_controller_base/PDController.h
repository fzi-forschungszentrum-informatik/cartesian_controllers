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
/*!\file    PDController.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/16
 *
 */
//-----------------------------------------------------------------------------

#ifndef PD_CONTROLLER_H_INCLUDED
#define PD_CONTROLLER_H_INCLUDED

// ROS
#include <ros/ros.h>

// ros_control
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_box.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cartesian_controller_base/PDGainsConfig.h>

namespace cartesian_controller_base
{

/**
 * @brief A proportional, derivative controller
 *
 * Motivation for this custom implementation:
 *
 *  - Restriction to meaningful parameter ranges.
 *  Users should be able to use the sliders in dynamic
 *  reconfigure to find suitable parameters for their setup.
 *
 *  - No integral gain.
 *  The \ref cartesian_controllers package builds upon a control plant that
 *  already has an integrating part to eliminate steady state errors.
 *  Additionally exposing I-related parameters in dynamic reconfigure distracts
 *  users with unused complexity.
 */
class PDController
{
  public:
    PDController();
    PDController(const PDController& other); ///< RealtimeBuffer needs special treatment
    ~PDController();

    void init(const std::string& name_space);

    double operator()(const double& error, const ros::Duration& period);

  private:
    struct PDGains
    {
      PDGains()
        : m_p(0), m_d(0)
      {};

      PDGains(double p, double d)
        : m_p(p), m_d(d)
      {};

      double m_p; ///< proportional gain
      double m_d; ///< derivative gain
    };

    realtime_tools::RealtimeBuffer<PDGains> m_gains;

    double m_last_p_error;

    // Dynamic reconfigure
    typedef cartesian_controller_base::PDGainsConfig PDGainsConfig;
    void dynamicReconfigureCallback(PDGainsConfig& config, uint32_t level);

    boost::shared_ptr<dynamic_reconfigure::Server<PDGainsConfig> > m_dyn_conf_server;
    dynamic_reconfigure::Server<PDGainsConfig>::CallbackType m_callback_type;

};

}

#endif

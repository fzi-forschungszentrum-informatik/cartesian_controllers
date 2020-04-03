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
/*!\file    PDController.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2019/10/16
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PDController.h>

namespace cartesian_controller_base
{

PDController::PDController()
  : m_last_p_error(0.0)
{
}

PDController::PDController(const PDController& other)
  : m_last_p_error(other.m_last_p_error)
  , m_dyn_conf_server(other.m_dyn_conf_server)
{
  // Copy constructor would bind non-const ref
  // to const, so use copy assignment operator.
  m_gains = other.m_gains;
}

PDController::~PDController()
{
}


void PDController::init(const std::string& name_space)
{
  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type = boost::bind(
      &PDController::dynamicReconfigureCallback, this, _1, _2);

  m_dyn_conf_server.reset(
      new dynamic_reconfigure::Server<PDGainsConfig>(
        ros::NodeHandle(name_space)));
  m_dyn_conf_server->setCallback(m_callback_type);

}


double PDController::operator()(const double& error, const ros::Duration& period)
{
  if (period == ros::Duration(0.0))
  {
    return 0.0;
  }

  PDGains gains(*m_gains.readFromRT());
  double result = gains.m_p * error + gains.m_d * (error - m_last_p_error) / period.toSec();

  m_last_p_error = error;
  return result;
}

void PDController::dynamicReconfigureCallback(PDGainsConfig& config, uint32_t level)
{
  m_gains.writeFromNonRT(PDGains(config.p, config.d));
}

}

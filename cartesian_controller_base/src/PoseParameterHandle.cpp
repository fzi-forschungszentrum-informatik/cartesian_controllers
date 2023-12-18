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
/*!\file    PoseParameterHandle.cpp
 *
 * \author  Captain Yoshi <captain.yoshisaur@gmail.com>
 * \date    2023/12/17
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_controller_base/PoseParameterHandle.h>

namespace cartesian_controller_base
{

PoseParameterHandle::PoseParameterHandle()
{
}

PoseParameterHandle::PoseParameterHandle(const PoseParameterHandle& other)
  : m_dyn_conf_server(other.m_dyn_conf_server)
{
  // Copy constructor would bind non-const ref
  // to const, so use copy assignment operator.
  m_transform_kdl = other.m_transform_kdl;
}

PoseParameterHandle::~PoseParameterHandle()
{
}


void PoseParameterHandle::init(const std::string& name_space)
{
  // Connect dynamic reconfigure and overwrite the default values with values
  // on the parameter server. This is done automatically if parameters with
  // the according names exist.
  m_callback_type = std::bind(
    &PoseParameterHandle::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

  m_dyn_conf_server.reset(
      new dynamic_reconfigure::Server<SpatialPoseConfig>(
        ros::NodeHandle(name_space)));
  m_dyn_conf_server->setCallback(m_callback_type);

}

bool PoseParameterHandle::has_new_pose()
{
  // Enables a compare exchange atomatically, e.g. we are sure to not miss
  // any new pose updates (not the pose msg but the availability of the pose)
  //
  // Per C++11 § 29.6.5:
  // A consequence of spurious failure is that nearly all uses of weak compare-and-exchange will be in a loop.
  //
  // We don't need a loop because on a spurious failure:
  //   1) Missing an update on a couple of cyclesWe will eventually succeed on the next call (this method must be used in the ros_control::update)
  //   2) The code does nothing harmfull on a failure
  //
  bool expected = true;
  if(std::atomic_compare_exchange_weak(&m_pose_updated,&expected,false))
  {
    return true;
  }
  else{
    // WARNING Might be a spurious failure. Be carefull what you add here...
    return false;
  }
}


KDL::Frame PoseParameterHandle::get_pose()
{
  return *m_transform_kdl.readFromRT();
}

void PoseParameterHandle::dynamicReconfigureCallback(Config& config, uint32_t level)
{
  m_transform_kdl.writeFromNonRT(KDL::Frame(KDL::Rotation::Quaternion(config.qx, config.qy, config.qz, config.qw),
                                 KDL::Vector(config.px, config.py, config.pz)));

  m_pose_updated = true;
}

}

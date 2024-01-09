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
/*!\file    Utility.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2016/02/16
 *
 */
//-----------------------------------------------------------------------------

#ifndef UTILITY_H_INCLUDED
#define UTILITY_H_INCLUDED

#include <Eigen/Dense>

/*! \brief Convenience typedefs
   *
   *  Note: For 6D vectors the order is first linear, then angular
   */
namespace ctrl
{
typedef Eigen::Matrix<double, 6, 1> Vector6D;

typedef Eigen::VectorXd VectorND;

typedef Eigen::Vector3d Vector3D;

typedef Eigen::MatrixXd MatrixND;

typedef Eigen::Matrix3d Matrix3D;

typedef Eigen::Matrix<double, 6, 6> Matrix6D;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixND;

}  // namespace ctrl

#endif

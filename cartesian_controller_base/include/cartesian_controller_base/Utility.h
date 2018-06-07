// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
  namespace ctrl{

    typedef Eigen::Matrix<double,6,1> Vector6D;

    typedef Eigen::VectorXd VectorND;

    typedef Eigen::Vector3d Vector3D;

    typedef Eigen::MatrixXd MatrixND;

    typedef Eigen::Matrix3d Matrix3D;

    typedef Eigen::Matrix<double,6,6> Matrix6D;

  }

#endif

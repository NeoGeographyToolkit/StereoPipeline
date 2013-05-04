// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file TiePointTransform.h
///

#ifndef __TIEPOINTTRANSFORM_H__
#define __TIEPOINTTRANSFORM_H__

#include <vw/Image/Transform.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/BBox.h>

#include <MBA.h>

/// class TiePointTransform
///
/// A transform functor that maps a set of input pixel locations onto a 
/// set of output pixel locations, using a combination of a homography 
/// and a multilevel B-spline approximation.  The homography is computed 
/// first and factored out prior to the B-spline computation, reducing   
/// biasing peculiarities of the B-spline warp.
/// 
/// **NOTE** 
/// The homography is also used to compute the forward transform.  This 
/// gives you a quick way to comptue the approximate an output bounding 
/// box using forward_bbox(), but it is only *approximate*!  Computing 
/// a true forward() mapping function would require tricky nonlinear 
/// optimization which we don't bother doing for now.

class TiePointTransform : public vw::TransformHelper<TiePointTransform,vw::ContinuousFunction,vw::ContinuousFunction> {
protected:
  vw::Matrix3x3 m_H, m_H_inverse;
  UCBspl::SplineSurface m_fx, m_fy;
public:

  TiePointTransform( std::vector<vw::Vector2> const& input, std::vector<vw::Vector2> const& output ) {
    using namespace vw;

    boost::shared_ptr<std::vector<double> > x_arr( new std::vector<double> );
    boost::shared_ptr<std::vector<double> > y_arr( new std::vector<double> );
    boost::shared_ptr<std::vector<double> > zx_arr( new std::vector<double> );
    boost::shared_ptr<std::vector<double> > zy_arr( new std::vector<double> );

    BBox2 bbox;
    for( unsigned i=0; i<input.size(); ++i ) {
      x_arr->push_back( output[i].x() );
      y_arr->push_back( output[i].y() );
      zx_arr->push_back( input[i].x() );
      zy_arr->push_back( input[i].y() );
      bbox.grow( output[i] );
    }

    Matrix<double> inM(3,x_arr->size());
    Matrix<double> outM(3,x_arr->size());
    for( unsigned i=0; i<x_arr->size(); ++i ) {
      inM(0,i) = (*zx_arr)[i];
      inM(1,i) = (*zy_arr)[i];
      inM(2,i) = 1;
      outM(0,i) = (*x_arr)[i];
      outM(1,i) = (*y_arr)[i];
      outM(2,i) = 1;
    }
    Matrix3x3 M = inM*pseudoinverse(outM);

    inM = M * outM;
    for( unsigned i=0; i<x_arr->size(); ++i ) {
      (*zx_arr)[i] -= inM(0,i) / inM(2,i);
      (*zy_arr)[i] -= inM(1,i) / inM(2,i);
    }

    MBA mba_x(x_arr,y_arr,zx_arr);
    MBA mba_y(x_arr,y_arr,zy_arr);

    int m0 = 1, n0 = 1, numIterations = 3;
    if (bbox.width() > bbox.height() )
      m0 = round( bbox.width() / bbox.height() );
    else 
      n0 = round( bbox.height() / bbox.width() );

    mba_x.MBAalg(m0,n0,numIterations);
    m_fx = mba_x.getSplineSurface();
    
    mba_y.MBAalg(m0,n0,numIterations);
    m_fy = mba_y.getSplineSurface();

    m_H_inverse = M;
    m_H = inverse( m_H_inverse );
  }

  inline vw::Vector2 reverse( vw::Vector2 const& p ) const {
    double w = m_H_inverse(2,0) * p(0) + m_H_inverse(2,1) * p(1) + m_H_inverse(2,2);
    return vw::Vector2( ( m_H_inverse(0,0) * p(0) + m_H_inverse(0,1) * p(1) + m_H_inverse(0,2) ) / w + m_fx.f(p.x(),p.y()),
                        ( m_H_inverse(1,0) * p(0) + m_H_inverse(1,1) * p(1) + m_H_inverse(1,2) ) / w + m_fy.f(p.x(),p.y()));
  }
		
  inline vw::Vector2 forward( vw::Vector2 const& p ) const {
    double w = m_H(2,0) * p(0) + m_H(2,1) * p(1) + m_H(2,2);
    return vw::Vector2( ( m_H(0,0) * p(0) + m_H(0,1) * p(1) + m_H(0,2) ) / w,
                        ( m_H(1,0) * p(0) + m_H(1,1) * p(1) + m_H(1,2) ) / w);
  }
};

#endif // __TIEPOINTTRANSFORM__

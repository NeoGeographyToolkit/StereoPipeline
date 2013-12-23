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


#include <asp/Core/AffineEpipolar.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/LinearAlgebra.h>
#include <vw/InterestPoint/InterestData.h>

#include <vector>

using namespace vw;

namespace asp {

  // Solves for Affine Fundamental Matrix as per instructions in
  // Multiple View Geometry.
  Matrix<double>
  linear_affine_fundamental_matrix( std::vector<ip::InterestPoint> const& ip1,
                                    std::vector<ip::InterestPoint> const& ip2 ) {

    // (i) Compute the centroid of X and delta X
    Matrix<double> delta_x(ip1.size(),4);
    Vector4 mean_x;
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      delta_x(i,0) = ip2[i].x;
      delta_x(i,1) = ip2[i].y;
      delta_x(i,2) = ip1[i].x;
      delta_x(i,3) = ip1[i].y;
      mean_x += select_row(delta_x,i) / double(ip1.size());
    }
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      select_row(delta_x,i) -= mean_x;
    }

    Matrix<double> U, VT;
    Vector<double> S;
    svd(transpose(delta_x),U,S,VT);
    Vector<double> N = select_col( U, 3 );
    double e = -transpose(N)*mean_x;
    Matrix<double> f(3,3);
    f(0,2) = N(0);
    f(1,2) = N(1);
    f(2,2) = e;
    f(2,0) = N(2);
    f(2,1) = N(3);

    return f;
  }

  void solve_y_scaling( std::vector<ip::InterestPoint> const& ip1,
                        std::vector<ip::InterestPoint> const& ip2,
                        Matrix<double>& affine_left,
                        Matrix<double>& affine_right ) {
    Matrix<double> a( ip1.size(), 2 );
    Vector<double> b( ip1.size() );
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      select_row(a,i) = subvector(affine_right*Vector3(ip2[i].x,ip2[i].y,1),1,2);
      b[i] = (affine_left*Vector3(ip1[i].x,ip1[i].y,1))(1);
    }
    Vector<double> scaling = least_squares( a, b );
    submatrix(affine_right,0,0,2,2) *= scaling[0];
    affine_right(1,2) = scaling[1];
  }

  void solve_x_shear( std::vector<ip::InterestPoint> const& ip1,
                      std::vector<ip::InterestPoint> const& ip2,
                      Matrix<double>& affine_left,
                      Matrix<double>& affine_right ) {
    Matrix<double> a( ip1.size(), 3 );
    Vector<double> b( ip1.size() );
    for ( size_t i = 0; i < ip1.size(); i++ ) {
      select_row(a,i) = affine_right * Vector3(ip2[i].x,ip2[i].y,1);
      b[i] = (affine_left*Vector3(ip1[i].x,ip1[i].y,1))(0);
    }
    Vector<double> shear = least_squares( a, b );
    Matrix<double> interm = math::identity_matrix<3>();
    interm(0,1) = -shear[1] / 2.0;
    affine_left = interm * affine_left;
    interm = math::identity_matrix<3>();
    interm(0,0) = shear[0];
    interm(0,1) = shear[1] / 2.0;
    interm(0,2) = shear[2];
    affine_right = interm * affine_right;
  }

  // Main function that other parts of ASP should use
  Vector2i
  affine_epipolar_rectification( Vector2i const& left_size,
                                 Vector2i const& right_size,
                                 std::vector<ip::InterestPoint> const& ip1,
                                 std::vector<ip::InterestPoint> const& ip2,
                                 Matrix<double>& left_matrix,
                                 Matrix<double>& right_matrix ) {
    // Create affine fundamental matrix
    Matrix<double> fund = linear_affine_fundamental_matrix( ip1, ip2 );

    // Solve for rotation matrices
    double Hl = sqrt( fund(2,0)*fund(2,0) + fund(2,1)*fund(2,1) );
    double Hr = sqrt( fund(0,2)*fund(0,2) + fund(1,2)*fund(1,2) );
    Vector2 epipole(-fund(2,1),fund(2,0)), epipole_prime(-fund(1,2),fund(0,2));
    if ( epipole.x() < 0 )
      epipole = -epipole;
    if ( epipole_prime.x() < 0 )
      epipole_prime = -epipole_prime;
    epipole.y() = -epipole.y();
    epipole_prime.y() = -epipole_prime.y();

    left_matrix = math::identity_matrix<3>();
    right_matrix = math::identity_matrix<3>();
    left_matrix(0,0) = epipole[0]/Hl;
    left_matrix(0,1) = -epipole[1]/Hl;
    left_matrix(1,0) = epipole[1]/Hl;
    left_matrix(1,1) = epipole[0]/Hl;
    right_matrix(0,0) = epipole_prime[0]/Hr;
    right_matrix(0,1) = -epipole_prime[1]/Hr;
    right_matrix(1,0) = epipole_prime[1]/Hr;
    right_matrix(1,1) = epipole_prime[0]/Hr;

    // Solve for ideal scaling and translation
    solve_y_scaling( ip1, ip2, left_matrix, right_matrix );

    // Solve for ideal shear, scale, and translation of X axis
    solve_x_shear( ip1, ip2, left_matrix, right_matrix );

    // Work out the ideal render size.
    BBox2i output_bbox, right_bbox;
    output_bbox.grow( subvector(left_matrix*Vector3(0,0,1),0,2) );
    output_bbox.grow( subvector(left_matrix*Vector3(left_size.x(),0,1),0,2) );
    output_bbox.grow( subvector(left_matrix*Vector3(left_size.x(),left_size.y(),1),0,2) );
    output_bbox.grow( subvector(left_matrix*Vector3(0,left_size.y(),1),0,2) );
    right_bbox.grow( subvector(right_matrix*Vector3(0,0,1),0,2) );
    right_bbox.grow( subvector(right_matrix*Vector3(right_size.x(),0,1),0,2) );
    right_bbox.grow( subvector(right_matrix*Vector3(right_size.x(),right_size.y(),1),0,2) );
    right_bbox.grow( subvector(right_matrix*Vector3(0,right_size.y(),1),0,2) );
    output_bbox.crop( right_bbox );

    left_matrix(0,2) -= output_bbox.min().x();
    right_matrix(0,2) -= output_bbox.min().x();
    left_matrix(1,2) -= output_bbox.min().y();
    right_matrix(1,2) -= output_bbox.min().y();

    return Vector2i( output_bbox.width(), output_bbox.height() );
  }

}

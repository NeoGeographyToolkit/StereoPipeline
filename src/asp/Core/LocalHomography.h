// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file LocalDisparity.h
///

#ifndef __LOCAL_DISPARITY_H__
#define __LOCAL_DISPARITY_H__

#include <vw/Image/ImageView.h>
#include <vw/InterestPoint/InterestData.h>
#include <vector>

// Forward declaration
namespace asp {
  class Options;
}

namespace asp {

  void split_n_into_k(int n, int k, std::vector<int> & partition);

  void create_local_homographies(Options const& opt);

  void write_local_homographies(std::string const& local_hom_file,
                                vw::ImageView<vw::Matrix3x3> const& local_hom);
  void read_local_homographies(std::string const& local_hom_file,
                               vw::ImageView<vw::Matrix3x3> & local_hom);

  // Given a disparity map restricted to a subregion, find the homography
  // transform which aligns best the two images based on this disparity.
  template<class SeedDispT>
  vw::math::Matrix<double> homography_for_disparity(vw::BBox2i subregion,
                                                    SeedDispT const& disparity){

    // To do: I've never seen this, but it is possible that this
    // homography computation may fail if the disparity in subregion
    // has too few points or not scattered well. This function may
    // need to be made more robust by taking in the entire disparity,
    // not just cropped to subregion, as done now. And, we would crop
    // the disparity to increasingly larger boxes containing
    // subregion until the computation would succeed.

    VW_ASSERT(subregion.width() == disparity.cols() &&
              subregion.height() == disparity.rows(),
              vw::ArgumentErr() << "homography_for_disparity: "
              << "The sizes of subregion and disparity don't match.\n");

    // We will split the subregion into N x N boxes, and average the
    // disparity in each box, to reduce the run-time.
    int N = 10;

    std::vector<int> partitionx, partitiony;
    split_n_into_k(disparity.cols(), std::min(disparity.cols(), N), partitionx);
    split_n_into_k(disparity.rows(), std::min(disparity.rows(), N), partitiony);

    std::vector<vw::ip::InterestPoint> left_ip, right_ip;
    for (int ix = 0; ix < (int)partitionx.size()-1; ix++){
      for (int iy = 0; iy < (int)partitiony.size()-1; iy++){

        // First sum up the disparities in each subbox.
        double lx = 0, ly = 0, rx = 0, ry = 0, count = 0; // int may cause overflow
        for (int x = partitionx[ix]; x < partitionx[ix+1]; x++){
          for (int y = partitiony[iy]; y < partitiony[iy+1]; y++){

            typename SeedDispT::pixel_type disp = disparity(x, y);
            if (!is_valid(disp)) continue;
            lx += x; rx += (x + disp.child().x());
            ly += y; ry += (y + disp.child().y());
            count++;
          }
        }
        if (count == 0) continue; // no valid points

        // Do the averaging. We must add the box corner to the left and
        // right interest points.
        vw::ip::InterestPoint l, r;
        l.x = subregion.min().x() + lx/count; r.x = subregion.min().x() + rx/count;
        l.y = subregion.min().y() + ly/count; r.y = subregion.min().y() + ry/count;
        left_ip.push_back(l);
        right_ip.push_back(r);
      }
    }

    try {
      return homography_fit(right_ip, left_ip, bounding_box(disparity));
    }catch ( const vw::ArgumentErr& e ){
      // Will return the identity matrix.
    }
    return vw::math::identity_matrix<3>();

  }

} // namespace asp

#endif

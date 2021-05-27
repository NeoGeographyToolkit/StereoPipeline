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


/// \file LocalAlignment.cc
///

#include <asp/Core/LocalAlignment.h>
#include <vw/Math/Transform.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/Interpolation.h>

using namespace vw;

namespace asp {

  // Go from 1D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_disparity(// Inputs
                         vw::ImageViewRef<float> disp_1d, 
                         vw::BBox2i const& left_crop_win, 
                         vw::BBox2i const& right_crop_win,
                         vw::math::Matrix<double> const& left_align_mat,
                         vw::math::Matrix<double> const& right_align_mat,
                         // Output
                         vw::ImageView<vw::PixelMask<vw::Vector2f>> & disp_2d) {

    vw::HomographyTransform left_align_trans (left_align_mat);
    vw::HomographyTransform right_align_trans(right_align_mat);

    float nan_nodata = std::numeric_limits<float>::quiet_NaN(); // NaN value
    PixelMask<float> nodata_mask = PixelMask<float>(); // invalid value for a PixelMask

    ImageViewRef<PixelMask<float>> masked_disp_1d = create_mask(disp_1d, nan_nodata);

    // TODO(oalexan1): Here bilinear interpolation is used. This will
    // make the holes a little bigger where there is no data. Need
    // to figure out if it is desired to fill holes.
    ImageViewRef<PixelMask<float>> interp_disp_1d
      = interpolate(masked_disp_1d, BilinearInterpolation(),
                    ValueEdgeExtension<PixelMask<float>>(nodata_mask));
    
    disp_2d.set_size(left_crop_win.width(), left_crop_win.height());

    for (int col = 0; col < disp_2d.cols(); col++) {
      for (int row = 0; row < disp_2d.rows(); row++) {
        Vector2 left_pix(col, row);
        Vector2 left_trans_pix = left_align_trans.forward(left_pix);
        PixelMask<float> interp_disp = interp_disp_1d(left_trans_pix.x(), left_trans_pix.y());

        if (!is_valid(interp_disp)) {
          disp_2d(col, row) = PixelMask<Vector2f>();
          disp_2d(col, row).invalidate();
          continue;
        }

        // Since the disparity is 1D, the y value (row) is the same
        // as for the input.
        Vector2 right_trans_pix(left_trans_pix.x() + interp_disp.child(), left_trans_pix.y());

        // Undo the transform
        Vector2 right_pix = right_align_trans.reverse(right_trans_pix);

        // Un-transformed disparity
        Vector2 disp_pix = right_pix - left_pix;

        // Adjust for the fact that the two tiles before alignment
        // were crops from larger images
        disp_pix += (right_crop_win.min() - left_crop_win.min());
      
        disp_2d(col, row).child() = Vector2f(disp_pix.x(), disp_pix.y());
        disp_2d(col, row).validate();
      }   
    }
  
  }

} // namespace asp

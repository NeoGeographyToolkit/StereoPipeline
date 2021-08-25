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


#include <asp/Core/StereoSettings.h>
#include <asp/Core/Common.h>
#include <asp/Core/DisparityFilter.h>
#include <vw/Math/Transform.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/InterestPoint/MatrixIO.h>

using namespace vw;

namespace asp {

  // Filter D_sub. Must be called only for alignment method affineepipolar, homography,
  // and local_epipolar. For now this is not in use as a dataset where this would help
  // was not found. It was tested though.
  void filter_D_sub(ASPGlobalOptions & opt,
                    boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                    boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                    vw::cartography::Datum const& datum,
                    std::string const& d_sub_file,
                    Vector2 const& outlier_removal_params) {

    if (outlier_removal_params[0] >= 100.0)
      return; // The user chose to skip outlier filtering
  
    vw_out() << "Filtering outliers in D_sub.\n";
  
    vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> sub_disp;
    vw::Vector2 upsample_scale;
    asp::load_D_sub_and_scale(opt, d_sub_file, sub_disp, upsample_scale);

    Matrix<double> left_global_mat  = math::identity_matrix<3>();
    Matrix<double> right_global_mat = math::identity_matrix<3>();
    read_matrix(left_global_mat, opt.out_prefix + "-align-L.exr");
    read_matrix(right_global_mat, opt.out_prefix + "-align-R.exr");
    vw::HomographyTransform left_trans(left_global_mat);
    vw::HomographyTransform right_trans(right_global_mat);

    // Set up the stereo model for doing triangulation
    double angle_tol = vw::stereo::StereoModel
      ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
    stereo::StereoModel model(left_camera_model.get(), right_camera_model.get(),
                              stereo_settings().use_least_squares, angle_tol);

    float HIGH_ERROR = std::numeric_limits<float>::max();
    ImageView<float> tri_err(sub_disp.cols(), sub_disp.rows());
    ImageView<float> height(sub_disp.cols(), sub_disp.rows());

    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);
      
        if (!is_valid(disp)) {
          tri_err(col, row) = HIGH_ERROR;
          height(col, row) = HIGH_ERROR;
          continue;
        }
      
        Vector2 left_pix(col, row);
        Vector2 right_pix = left_pix + disp.child();

        // Scale to full resolution
        left_pix  = elem_prod(left_pix, upsample_scale);
        right_pix = elem_prod(right_pix, upsample_scale);

        // Undo the alignment transform
        left_pix = left_trans.reverse(left_pix);
        right_pix = right_trans.reverse(right_pix);

        double err;
        Vector3 xyz;
        try {
          xyz = model(left_pix, right_pix, err);
        } catch(...) {
          xyz = Vector3();
        }
      
        // The call returns the zero error and zero xyz to indicate a
        // failed ray intersection so replace it in those cases with a
        // very high error.
        if (err == 0 || xyz == Vector3()) {
          tri_err(col, row) = HIGH_ERROR;
          height(col, row) = HIGH_ERROR;
          continue;
        }
        
        tri_err(col, row) = err;

        Vector3 llh = datum.cartesian_to_geodetic(xyz);
        height(col, row) = llh[2];
      }
    }

    // Put the valid heights in a vector
    std::vector<double> vals;
    vals.reserve(sub_disp.cols() * sub_disp.rows());
    vals.clear();
    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        if (height(col, row) == HIGH_ERROR) continue;
        vals.push_back(height(col, row));
      }
    }

    // Find the outlier brackets
    double pct = outlier_removal_params[0];
    double factor = outlier_removal_params[1];
    double pct_fraction = 1.0 - pct/100.0;
    double b = -1.0, e = -1.0;
    vw::math::find_outlier_brackets(vals, pct_fraction, factor, b, e);

    // Apply the outlier threshold
    int count = 0;
    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        if (height(col, row) == HIGH_ERROR) continue; // already invalid
        if (height(col, row) < b || height(col, row) > e) {
          height(col, row)  = HIGH_ERROR;
          tri_err(col, row) = HIGH_ERROR;
          count++;
        }
      }
    }
    std::cout << "Number (and fraction) of removed outliers by the height check: "
              << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";

    // Put the tri errors in a vector
    vals.clear();
    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        if (tri_err(col, row) == HIGH_ERROR) continue; // already invalid
        vals.push_back(tri_err(col, row));
      }
    }

    // Find the outlier brackets. We will ignore b, as the triangulation
    // errors are non-negative. Since the triangulation errors, unlike
    // the heights, are usually rather uniform, adjust pct from 95 to
    // 75.
    double pct2 = (75.0/95.0) * outlier_removal_params[0];
    double pct_fraction2 = 1.0 - pct2/100.0;
    b = -1.0;
    e = -1.0;
    vw::math::find_outlier_brackets(vals, pct_fraction2, factor, b, e);

    // Apply the outlier threshold
    count = 0;
    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        if (tri_err(col, row) == HIGH_ERROR) continue;
        if (tri_err(col, row) > e) {
          height(col, row) = HIGH_ERROR;
          tri_err(col, row) = HIGH_ERROR;
          count++;
        }
      }
    }
    std::cout << "Number (and fraction) of removed outliers by the triangulation error check: "
              << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";

    // Invalidate the D_sub entries that are outliers
    for (int col = 0; col < sub_disp.cols(); col++) {
      for (int row = 0; row < sub_disp.rows(); row++) {
        if (tri_err(col, row) == HIGH_ERROR)
          sub_disp(col, row).invalidate();
      }
    }

    vw_out() << "Writing filtered D_sub: " << d_sub_file << std::endl;
    block_write_gdal_image(d_sub_file, sub_disp, opt,
                           TerminalProgressCallback("asp","\t D_sub: "));
  } 

} // end namespace asp

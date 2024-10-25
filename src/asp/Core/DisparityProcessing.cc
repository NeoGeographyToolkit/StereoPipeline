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
#include <asp/Core/DisparityProcessing.h>
#include <vw/Math/Transform.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/FileIO/MatrixIO.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/InterestPoint/InterestData.h>

using namespace vw;
using namespace vw::cartography;

namespace asp {
  
/// Load the D_sub file fully in memory
bool load_D_sub(std::string const& d_sub_file,
                ImageView<PixelMask<Vector2f>> & sub_disp) {

  if (!boost::filesystem::exists(d_sub_file))
    return false;
  
  // Check the data type of the file.
  boost::shared_ptr<DiskImageResource> rsrc(DiskImageResourcePtr(d_sub_file));
  ChannelTypeEnum disp_data_type = rsrc->channel_type();
  
  if (disp_data_type == VW_CHANNEL_INT32) // Cast the integer file to float
    sub_disp = copy(pixel_cast<PixelMask<Vector2f>>(DiskImageView<PixelMask<Vector2i>>(d_sub_file)));
  else // File on disk is float
    sub_disp = copy(DiskImageView<PixelMask<Vector2f>>(d_sub_file));
  return true;
}

// Load the low-res disparity and the scale needed to convert it to full-res
void load_D_sub_and_scale(std::string                           const & out_prefix, 
                          std::string                           const & d_sub_file, 
                          vw::ImageView<vw::PixelMask<vw::Vector2f>>  & sub_disp,
                          vw::Vector2                                 & upsample_scale) {
  
  DiskImageView<float> L(out_prefix + "-L.tif");
  
  if (!load_D_sub(d_sub_file, sub_disp)) {
    std::string msg = "Could not read low-res disparity: " + d_sub_file + ". " 
     + "Check your run. See if --skip-low-res-disparity-comp is on.";
    vw_throw(ArgumentErr() << msg << "\n");
  }

  upsample_scale = Vector2(double(L.cols()) / double(sub_disp.cols()),
                           double(L.rows()) / double(sub_disp.rows()));
}

// Filter D_sub. All alignment methods are supported.
void filter_D_sub(ASPGlobalOptions const& opt,
                  vw::TransformPtr tx_left, vw::TransformPtr tx_right,
                  boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                  boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                  vw::cartography::Datum const& datum,
                  std::string const& d_sub_file,
                  Vector2 const& outlier_removal_params) {
  
  if (outlier_removal_params[0] >= 100.0)
    return; // The user chose to skip outlier filtering
  
  vw_out() << "Filtering outliers in D_sub based on --outlier-removal-params.\n";

  double pct = outlier_removal_params[0];
  double factor = outlier_removal_params[1];
  double pct_fraction = 1.0 - pct/100.0;

  vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp;
  vw::Vector2 upsample_scale;
  asp::load_D_sub_and_scale(opt.out_prefix, d_sub_file, sub_disp, upsample_scale);

  // Will save the sub PC file, since we compute these anyway
  // for the purpose of filtering.
  vw::ImageView<vw::Vector<double, 4>> sub_pc(sub_disp.cols(), sub_disp.rows());
  for (int col = 0; col < sub_pc.cols(); col++) {
    for (int row = 0; row < sub_pc.rows(); row++) {
      for (int coord = 0; coord < 4; coord++) {
        sub_pc(col, row)[coord] = 0;
      }
    }
  }
  
  // Find the disparity values in x and y
  std::vector<double> dx, dy;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);
      
      if (!is_valid(disp)) 
        continue;
      
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      double diffX = right_pix.x() - left_pix.x();
      double diffY = right_pix.y() - left_pix.y();
      dx.push_back(diffX);
      dy.push_back(diffY);
    }
  }
  
  if (dx.empty())
    vw_throw(ArgumentErr() << "Empty disparity.");

  // Find the outlier brackets based on values in x and y
  double bx = -1.0, ex = -1.0;
  vw::math::find_outlier_brackets(dx, pct_fraction, factor, bx, ex);
  vw_out() <<"Inlier range based on x coordinate of disparity: " << bx << ' ' << ex <<".\n";

  double by = -1.0, ey = -1.0;
  vw::math::find_outlier_brackets(dy, pct_fraction, factor, by, ey);
  vw_out() <<"Inlier range based on y coordinate of disparity: " << by << ' ' << ey <<".\n";

  int count = 0;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);
      
      if (!is_valid(disp)) 
        continue;
      
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      double diffX = right_pix.x() - left_pix.x();
      double diffY = right_pix.y() - left_pix.y();

      if (diffX < bx || diffX > ex || diffY < by || diffY > ey) {
        sub_disp(col, row).invalidate();
        count++;
      }
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by disparity values in x and y: "
           << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";
  
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
      left_pix = tx_left->reverse(left_pix);
      right_pix = tx_right->reverse(right_pix);

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

      // Save the triangulated point
      Vector<double, 4> P;
      subvector(P, 0, 3) = xyz;
      P[3] = err;
      sub_pc(col, row) = P;
      
      Vector3 llh = datum.cartesian_to_geodetic(xyz);
      height(col, row) = llh[2];
    }
  }

  // Put the valid heights in a vector
  std::vector<double> vals;
  // Careful below to avoid integer overflow
  vals.reserve(std::int64_t(sub_disp.cols()) * std::int64_t(sub_disp.rows()));
  vals.clear();
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (height(col, row) >= HIGH_ERROR) continue; // already invalid
      vals.push_back(height(col, row));
    }
  }

  // Print a warning if no valid points were found. Maybe the cameras
  // are too similar.
  if (vals.empty()) {
    vw::vw_out(vw::WarningMessage) << "No valid points found during triangulation. "
     << "Skipping any further outlier removal. Check the stereo convergence "
     << "angle or consider reducing the value of --min-triangulation-angle.\n";
     return;
  }
  
  // Find the outlier brackets
  double b = -1.0, e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction, factor, b, e);
  vw_out() <<"Height above datum inlier range: " << b << ' ' << e <<".\n";

  // Apply the outlier threshold
  count = 0;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (height(col, row) >= HIGH_ERROR) continue; // already invalid
      if (height(col, row) < b || height(col, row) > e) {
        height(col, row)  = HIGH_ERROR;
        tri_err(col, row) = HIGH_ERROR;
        count++;
      }
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by the height check: "
           << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";
    
  // Put the tri errors in a vector
  vals.clear();
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (tri_err(col, row) >= HIGH_ERROR) continue; // already invalid
      vals.push_back(tri_err(col, row));
    }
  }

  // Find the outlier brackets. Since the triangulation errors, unlike
  // the heights, are usually rather uniform, adjust pct from 95 to
  // 90.
  double pct2 = std::max((90.0/95.0) * outlier_removal_params[0], 0.5);
  double pct_fraction2 = 1.0 - pct2/100.0;
  // Show some lenience below as due to jitter some errors could be somewhat bigger
  double factor2 = 2.0 * factor;
  b = -1.0;
  e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction2, factor2, b, e);
  vw_out() <<"Triangulation error inlier range: " << b << ' ' << e <<".\n";
    
  // Apply the outlier threshold
  count = 0;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (tri_err(col, row) >= HIGH_ERROR) continue; // already invalid
      // We will ignore b, as the triangulation errors are non-negative.
      if (tri_err(col, row) > e) {
        height(col, row) = HIGH_ERROR;
        tri_err(col, row) = HIGH_ERROR;
        count++;
      }
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by the triangulation error check: "
           << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";

  // TODO(oalexan1): Filter by user-given height range and max tri error.
    
  // Invalidate the D_sub entries that are outliers
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (tri_err(col, row) >= HIGH_ERROR || height(col, row) >= HIGH_ERROR) {
        sub_disp(col, row).invalidate();

        // Invalidate the point in the cloud
        subvector(sub_pc(col, row), 0, 3) = Vector3(0.0, 0.0, 0.0);
        sub_pc(col, row)[3] = 0.0;
      }
    }
  }

  vw_out() << "Writing filtered D_sub: " << d_sub_file << std::endl;
  block_write_gdal_image(d_sub_file, sub_disp, opt,
                         TerminalProgressCallback("asp","\t D_sub: "));

  std::string pc_sub_file = opt.out_prefix + "-PC_sub.tif";
  vw_out() << "Writing triangulated point cloud based on D_sub: " << pc_sub_file << std::endl;
  block_write_gdal_image(pc_sub_file, sub_pc, opt,
                         TerminalProgressCallback("asp","\t PC_sub: "));
  
} 

// Filter D_sub by reducing its spread around the median
void filter_D_sub_using_spread(ASPGlobalOptions const& opt, std::string const& d_sub_file,
                               double max_disp_spread) {
  
  if (max_disp_spread <= 0.0)
    return;

  vw_out() << "Filtering outliers in D_sub based on --max-disp-spread.\n";
  
  vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp;
  vw::Vector2 upsample_scale;
  asp::load_D_sub_and_scale(opt.out_prefix, d_sub_file, sub_disp, upsample_scale);

  std::vector<double> dx, dy;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);
      
      if (!is_valid(disp)) 
        continue;
      
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      // Scale to full resolution
      left_pix  = elem_prod(left_pix, upsample_scale);
      right_pix = elem_prod(right_pix, upsample_scale);

      double diffX = right_pix.x() - left_pix.x();
      double diffY = right_pix.y() - left_pix.y();
      dx.push_back(diffX);
      dy.push_back(diffY);
    }
  }
  
  // Do not throw an error, as sometimes the disparity is empty because
  // some other filtering may have removed all points in degenerate cases.
  // Just continue with the run and hope for the best.
  if (dx.empty())
    vw::vw_out(vw::WarningMessage) << "Empty disparity. Will not continue with filtering.\n";
  
  std::sort(dx.begin(), dx.end());
  std::sort(dy.begin(), dy.end());
  double mid_x = dx[dx.size()/2]; // median
  double mid_y = dy[dy.size()/2];
  double half = max_disp_spread / 2.0;
  
  BBox2 spread_box(mid_x - half, mid_y - half, max_disp_spread, max_disp_spread);
  
  // Wipe offending disparities
  int count = 0;
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      vw::PixelMask<vw::Vector2f> disp = sub_disp(col, row);
      
      if (!is_valid(disp)) 
        continue;
      
      Vector2 left_pix(col, row);
      Vector2 right_pix = left_pix + disp.child();

      // Scale to full resolution
      left_pix  = elem_prod(left_pix, upsample_scale);
      right_pix = elem_prod(right_pix, upsample_scale);

      double diffX = right_pix.x() - left_pix.x();
      double diffY = right_pix.y() - left_pix.y();

      Vector2 d(diffX, diffY);
      if (!spread_box.contains(d)) {
        count++;
        sub_disp(col, row).invalidate();
      }
    }
  }

  vw_out() << "Number (and fraction) of removed outliers by the disp spread check: "
           << count << " (" << double(count)/(sub_disp.cols() * sub_disp.rows()) << ").\n";
    
  vw_out() << "Writing filtered D_sub: " << d_sub_file << std::endl;
  block_write_gdal_image(d_sub_file, sub_disp, opt,
                         TerminalProgressCallback("asp","\t D_sub: "));
} 

// Compute an unaligned disparity image from the input disparity image
// and the image transforms.
// Note that the output image size is not the same as the input disparity image.
class UnalignDisparityView: public ImageViewBase<UnalignDisparityView>{
  
  DispImageType    const& m_disparity;
  vw::TransformPtr const& m_left_transform;
  vw::TransformPtr const& m_right_transform;

  ASPGlobalOptions const& m_opt;
  int m_num_cols, m_num_rows;
  bool m_is_map_projected;
  std::map <std::pair<int, int>, Vector2> m_unaligned_trans;
public:
  UnalignDisparityView(bool is_map_projected,
                       DispImageType    const& disparity,
                       vw::TransformPtr const& left_transform,
                       vw::TransformPtr const& right_transform,
                       ASPGlobalOptions const& opt):
    m_is_map_projected(is_map_projected), 
    m_disparity(disparity), m_left_transform(left_transform), 
    m_right_transform(right_transform), m_opt(opt),
    m_num_cols(0), m_num_rows(0) {

    // Compute the output image size
    
    if (!m_is_map_projected) {
      // The left image passed as input is the original
      // unprojected/unaligned one, hence use its size.
      std::string left_file  = m_opt.in_file1;
      DiskImageView<float> left_img(left_file);
      m_num_cols = left_img.cols();
      m_num_rows = left_img.rows();
    } else {
      // Map projected, need to check all the pixel coordinates.
      // This is going to be slow for large images!
      BBox2i img_box;

      // Use sampling as this operation is very slow.
      int sample_len = 10;
      int num_min_samples = 100;
      int col_sample = std::max(1, std::min(sample_len, m_disparity.cols()/num_min_samples));
      int row_sample = std::max(1, std::min(sample_len, m_disparity.rows()/num_min_samples));

      vw_out() << "\nEstimating the unaligned disparity dimensions.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = col_sample / std::max(double(m_disparity.cols()), 1.0);
      tpc.report_progress(0);

      for (int col = 0; col < m_disparity.cols(); col++) {

        // Ensure that the last column is picked
        if (col % col_sample != 0 && col != m_disparity.cols() - 1) 
          continue;

            for (int row = 0; row < m_disparity.rows(); row++) {
          
          // Ensure that the last row is picked
          if (row % row_sample != 0 && row != m_disparity.rows() - 1) 
            continue;

          // This is quite important to avoid an incorrectly computed img_box.
          typename DispImageType::pixel_type dpix = m_disparity(col, row);
          if (!is_valid(dpix))
            continue;

          // Unalign the left pixel
          Vector2 left_pix;
          try{
            left_pix  = m_left_transform->reverse(Vector2(col, row));
          }catch(...){
            continue;
          }
          img_box.grow(left_pix);

          // Save this lookup map for the future
          m_unaligned_trans[std::make_pair(col, row)] = left_pix;
        }

        tpc.report_incremental_progress(inc_amount);
      }
      tpc.report_finished();

      // Grow the box to account for the fact that we did a sub-sampling
      // and may have missed some points.
      Vector2 diff = img_box.max() - img_box.min();
      if (!img_box.empty()) {
        img_box.grow(img_box.min() - 0.1*diff);
        img_box.grow(img_box.max() + 0.1*diff);
      }
        
      m_num_cols = img_box.max().x();
      m_num_rows = img_box.max().y();

      vw_out() << "Dimensions are: " << m_num_cols << ' ' << m_num_rows << ".\n";
    } // Done computing the input image size
    
  } // End constructor

  // ImageView interface
  typedef PixelMask<Vector2f>                          pixel_type;
  typedef pixel_type                                   result_type;
  typedef ProceduralPixelAccessor<UnalignDisparityView> pixel_accessor;

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double /*i*/, double /*j*/, int32 /*p*/ = 0) const {
    vw_throw(vw::NoImplErr() << "UnalignDisparityView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    vw::TransformPtr local_left_transform;
    vw::TransformPtr local_right_transform;

    // For map-projected images the transforms are not thread-safe,
    // hence need to make a copy of them.
    if (!m_is_map_projected) {
      local_left_transform = m_left_transform;
      local_right_transform = m_right_transform;
    }else{
      local_left_transform = vw::cartography::mapproj_trans_copy(m_left_transform);
      local_right_transform = vw::cartography::mapproj_trans_copy(m_right_transform);
    }

    // We will do some averaging
    int KERNEL_SIZE = 1;
    
    BBox2i curr_bbox = bbox;
    curr_bbox.expand(2*KERNEL_SIZE);
    curr_bbox.crop(BBox2i(0, 0, cols(), rows()));
    
    // Initialize the unaligned disparity values for this tile.
    ImageView<pixel_type> unaligned_disp(curr_bbox.width(), curr_bbox.height());
    ImageView<int> count(curr_bbox.width(), curr_bbox.height());
    for (int col = 0; col < curr_bbox.width(); col++) {
      for (int row = 0; row < curr_bbox.height(); row++) {
        unaligned_disp(col, row) = pixel_type();
        unaligned_disp(col, row).invalidate();
        count(col, row) = 0;
      }
    }
    
    // Find the bounding box of pixels we will need from the disparity image.
    // For mapprojected images the forward() function is not always accurate,
    // and it is also very slow, hence avoid it.
    BBox2i disp_bbox;
    if (!m_is_map_projected) {
      BBox2i full_disp_bbox = bounding_box(m_disparity);
      for (int col = 0; col < unaligned_disp.cols(); col++) {
        for (int row = 0; row < unaligned_disp.rows(); row++) {
      
          // Get the pixel coordinate in the output image (left unaligned pixel),
          // Then get the pixel coordinate in the left input image.
          Vector2 output_pixel(col + curr_bbox.min()[0], row + curr_bbox.min()[1]);
          Vector2 left_aligned_pixel;
          try {
            left_aligned_pixel = local_left_transform->forward(output_pixel);
          }catch(...){
            // This can fail since we may apply it to pixels outside of range
            continue;
          }
          if (!full_disp_bbox.contains(left_aligned_pixel)) 
            continue;
          disp_bbox.grow(left_aligned_pixel);
        }
      }
    }else{
      for (int col = 0; col < m_disparity.cols(); col++) {
        for (int row = 0; row < m_disparity.rows(); row++) {
      
          std::pair<int, int> pix = std::make_pair(col, row);
          auto it = m_unaligned_trans.find(pix);
          if (it == m_unaligned_trans.end())
            continue;

          Vector2 rev = it->second;
          if (curr_bbox.contains(rev)) {
            disp_bbox.grow(Vector2(col, row));
          }
        }
      }

      // Grow the box to account for the fact that we did a sub-sampling
      // and may have missed some points.
      Vector2 diff = disp_bbox.max() - disp_bbox.min();
      if (!disp_bbox.empty()) {
        disp_bbox.grow(disp_bbox.min() - 0.1*diff);
        disp_bbox.grow(disp_bbox.max() + 0.1*diff);
      }
      
    }
    
    // Expand to take into account the sampling to be used below
    disp_bbox.expand(2*KERNEL_SIZE);

    // Crop to its maximum extent
    disp_bbox.crop(bounding_box(m_disparity));

    // Rasterize the section of the disparity image that we need for this tile
    typedef typename DispImageType::pixel_type DispPixelT;
    ImageView<DispPixelT> disp = crop(m_disparity, disp_bbox);

    for (int col = 0; col < disp.cols(); col++) {
      for (int row = 0; row < disp.rows(); row++) {
    
    DispPixelT dpix = disp(col, row);
    if (!is_valid(dpix))
      continue;

    // Go from position in the cropped disparity to the
    // position in the full disparity.
    int ucol = col + disp_bbox.min().x();
    int urow = row + disp_bbox.min().y();
    
    // De-warp left and right pixels to be in the camera coordinate system
    Vector2 left_pix, right_pix;
    try{
      left_pix  = local_left_transform->reverse (Vector2(ucol, urow));
      right_pix = local_right_transform->reverse(Vector2(ucol, urow)
                                                     + stereo::DispHelper(dpix));
    }catch(...){
      continue;
    }
    Vector2 dir = right_pix - left_pix; // disparity value
    
    // This averaging is useful in filling tiny holes and avoiding staircasing.
    // TODO: Use some weights. The closer contribution should have more weight.
    for (int icol = -KERNEL_SIZE; icol <= KERNEL_SIZE; icol++) {
      for (int irow = -KERNEL_SIZE; irow <= KERNEL_SIZE; irow++) {
        int lcol = round(left_pix[0]) + icol;
        int lrow = round(left_pix[1]) + irow;
        
        // shift to be in the domain of the cropped image
        lcol -= curr_bbox.min()[0];
        lrow -= curr_bbox.min()[1];
        if (lcol < 0 || lcol >= curr_bbox.width())  continue;
        if (lrow < 0 || lrow >= curr_bbox.height()) continue;
        if (!is_valid(unaligned_disp(lcol, lrow)))
          unaligned_disp(lcol, lrow).validate();
        unaligned_disp(lcol, lrow).child() += dir;
        count(lcol, lrow)++;
      }
    }
    
      }
    }
    
    for (int col = 0; col < unaligned_disp.cols(); col++) {
      for (int row = 0; row < unaligned_disp.rows(); row++) {
    if (count(col, row) == 0)
      unaligned_disp(col, row).invalidate();
    else
      unaligned_disp(col, row) /= double(count(col, row));
      }
    }
    
    // Use the crop trick to fake that the support region is the same size as
    // the entire image.
    return prerasterize_type(unaligned_disp, -curr_bbox.min().x(), 
                             -curr_bbox.min().y(), cols(), rows());
  }
  
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class UnalignDisparityView

// Take a given disparity and make it between the original unaligned images
void unalign_disparity(bool is_map_projected,
                       DispImageType    const& disparity,
                       vw::TransformPtr const& left_trans,
                       vw::TransformPtr const& right_trans,
                       ASPGlobalOptions const& opt,
                       std::string      const& disp_file) {
  Stopwatch sw;
  sw.start();

  cartography::GeoReference left_georef;
  bool   has_left_georef = false;
  bool   has_nodata      = false;
  double nodata          = -32768.0;
  vw_out() << "Unaligning the disparity.\n";
  vw_out() << "Writing: " << disp_file << "\n";
  vw::cartography::block_write_gdal_image
    (disp_file, 
     UnalignDisparityView(is_map_projected, disparity, left_trans, right_trans, opt),
     has_left_georef, left_georef,
     has_nodata, nodata, opt,
     TerminalProgressCallback("asp", "\t--> Undist disp:"));
  
  sw.stop();
  vw_out() << "Unaligning disparity elapsed time: " << sw.elapsed_seconds() << " seconds.\n";
  
}

// Auxiliary function to compute matches from disparity. A multiplier
// bigger than 1 will be passed in if too few matches are found.
void compute_matches_from_disp_aux(ASPGlobalOptions const& opt,
                                   DispImageType    const& disp,
                                   vw::TransformPtr const& left_trans,
                                   vw::TransformPtr const& right_trans,
                                   int max_num_matches,
                                   bool gen_triplets, bool is_map_projected,
                                   double multiplier, 
                                   // Output
                                   std::vector<vw::ip::InterestPoint> & left_ip, 
                                   std::vector<vw::ip::InterestPoint> & right_ip) {

  // Clear the outputs
  left_ip.clear();
  right_ip.clear();

  if (!gen_triplets) {

    // Use doubles to avoid integer overflow
    double num_pixels = double(disp.cols()) * double(disp.rows());
    double den = std::min(double(max_num_matches), num_pixels) * multiplier;
    double bin_len = sqrt(num_pixels/den);
    bin_len = std::max(1.0, bin_len);

    int lenx = round(disp.cols()/bin_len); lenx = std::max(1, lenx);
    int leny = round(disp.rows()/bin_len); leny = std::max(1, leny);

    // Iterate over bins.

    vw_out() << "Computing interest point matches based on disparity.\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(lenx);
    tpc.report_progress(0);

    for (int binx = 0; binx < lenx; binx++) {

      // Pick the disparity at the center of the bin
      int posx = round((binx+0.5)*bin_len);

      for (int biny = 0; biny < leny; biny++) {

        int posy = round((biny+0.5)*bin_len);

        if (posx >= disp.cols() || posy >= disp.rows()) 
          continue;
        typedef typename DispImageType::pixel_type DispPixelT;
        DispPixelT dpix = disp(posx, posy);
        if (!is_valid(dpix))
          continue;

        // De-warp left and right pixels to be in the camera coordinate system
        Vector2 left_pix  = left_trans->reverse (Vector2(posx, posy));
        Vector2 right_pix = right_trans->reverse(Vector2(posx, posy) + stereo::DispHelper(dpix));

        left_ip.push_back(ip::InterestPoint(left_pix.x(), left_pix.y()));
        right_ip.push_back(ip::InterestPoint(right_pix.x(), right_pix.y()));
      }

      tpc.report_incremental_progress(inc_amount);
    }
    tpc.report_finished();

  } else {

    // First create ip with left_ip being at integer multiple of bin size.
    // Then do the same for right_ip. This way there is a symmetry
    // and predictable location for ip. So if three images overlap,
    // a feature can often be seen in many of them whether a given
    // image is left in some pairs or right in some others.

    // Note that the code above is modified in subtle ways.

    // Interpolate the disparity. When interpolating out-of-range
    // pixels, return invalid values.
    typedef typename DispImageType::pixel_type DispPixelT;
    DispPixelT invalid_disp; invalid_disp.invalidate();
    vw::ValueEdgeExtension<DispPixelT> invalid_ext(invalid_disp);
    auto interp_disp = interpolate(disp, vw::BilinearInterpolation(), invalid_ext);

    // Need these to not insert an ip twice, as then bundle_adjust
    // will wipe both copies
    std::map<double, double> left_done, right_done;
    
    // Start with the left
    {
      DiskImageView<float> left_img(opt.in_file1);

      vw::BBox2 box = vw::bounding_box(left_img);
      // If images are mapprojected, need to find the bounding box of
      // unaligned pixels.
      if (is_map_projected) {
        
        // This is very slow. For now, just turn off this logic until it is studied more.
        vw::vw_throw(vw::ArgumentErr() << "Option --num-matches-from-disp-triplets does not work with mapprojected images. Use instead --num-matches-from-disparity.\n");
        box = left_trans->reverse_bbox(box); 
      }
      
      // Use doubles to avoid integer overflow
      double num_pixels = double(left_img.cols()) * double(left_img.rows());
      double den = std::min(double(max_num_matches), num_pixels) * multiplier;
      double bin_len = sqrt(num_pixels/den);
      bin_len = std::max(1.0, bin_len);

      int lenx = round(left_img.cols()/bin_len); lenx = std::max(1, lenx);
      int leny = round(left_img.rows()/bin_len); leny = std::max(1, leny);

      // Iterate over bins.

      vw_out() << "Computing interest point matches based on disparity.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(lenx);
      tpc.report_progress(0);

      for (int binx = 0; binx <= lenx; binx++) {

        int posx = binx*bin_len; // integer multiple of bin length

        for (int biny = 0; biny <= leny; biny++) {

          int posy = biny*bin_len; // integer multiple of bin length

          if (posx >= left_img.cols() || posy >= left_img.rows()) 
            continue;

          Vector2 left_pix(posx, posy);
          Vector2 trans_left_pix, trans_right_pix, right_pix;
          
          // Make the left pixel go to the disparity domain. Find the corresponding
          // right pixel. And make that one go to the right image domain.
          // left_pix is unaligned left pixel, while trans_left_pix is the aligned one.
          // This one is no longer integer, so need to interpolate to find its
          // disparity.
          trans_left_pix = left_trans->forward(left_pix);
          if (trans_left_pix[0] < 0 || trans_left_pix[0] >= disp.cols()) continue;
          if (trans_left_pix[1] < 0 || trans_left_pix[1] >= disp.rows()) continue;
          DispPixelT dpix = interp_disp(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix = right_trans->reverse(trans_right_pix);

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress(inc_amount);
      }
      tpc.report_finished();
    }
    
    // Now create ip in predictable locations for the right image. This is hard,
    // as the disparity goes from left to right, so we need to examine every disparity.
    typedef typename DispImageType::pixel_type DispPixelT;
    {
      DiskImageView<float> right_img(opt.in_file2);
    
      double num_pixels = double(right_img.cols()) * double(right_img.rows());
      double den = std::min(double(max_num_matches), num_pixels) * multiplier;
      // The bin len must be integer here
      int bin_len = round(sqrt(num_pixels/den));
      bin_len = std::max(1, bin_len);

      // Iterate over disparity.

      vw_out() << "Doing a second pass. This will be very slow.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(disp.cols());
      tpc.report_progress(0);

      for (int col = 0; col < disp.cols(); col++) {
        for (int row = 0; row < disp.rows(); row++) {

          Vector2 trans_left_pix(col, row);
          Vector2 left_pix, trans_right_pix, right_pix;

          DispPixelT dpix = disp(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;

          // Compute the left and right pixels. 
          left_pix        = left_trans->reverse(trans_left_pix);
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix       = right_trans->reverse(trans_right_pix);

          // If the right pixel is a multiple of the bin size, keep
          // it.
          right_pix = round(right_pix); // very important
          if (int(right_pix[0]) % bin_len != 0) continue;
          if (int(right_pix[1]) % bin_len != 0) continue;

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress(inc_amount);
      }
      tpc.report_finished();
    }
    
  } // end considering multi-image friendly ip
  
  vw_out() << "Determined " << left_ip.size()
           << " interest point matches from disparity.\n";
}

/// Bin the disparities, and from each bin get a disparity value.
/// This will create a correspondence from the left to right image,
/// which we save in the match format.
/// When gen_triplets is true, and there are many overlapping images,
/// try hard to have many IP with the property that each such IP is seen
/// in more than two images. This helps with bundle adjustment.
void compute_matches_from_disp(ASPGlobalOptions const& opt,
                               DispImageType    const& disp,
                               vw::TransformPtr const& left_trans,
                               vw::TransformPtr const& right_trans,
                               std::string      const& match_file,
                               int max_num_matches,
                               bool gen_triplets, bool is_map_projected) {

  std::vector<vw::ip::InterestPoint> left_ip, right_ip;

  // Call the aux function
  double multiplier = 1.0;
  compute_matches_from_disp_aux(opt, disp, left_trans, right_trans,
                                max_num_matches, gen_triplets, is_map_projected,
                                multiplier, left_ip, right_ip);
  double ratio = double(left_ip.size())/double(max_num_matches);
  ratio = std::max(1e-8, ratio);
  if (ratio < 1.0) {
    vw_out() << "The number of matches is way less than the requested value. "
             << "Trying again.\n";
    multiplier = 1.0/ratio;
    compute_matches_from_disp_aux(opt, disp, left_trans, right_trans,
                                  max_num_matches, gen_triplets, is_map_projected,
                                  multiplier, left_ip, right_ip);
  }
  
  vw_out() << "Writing: " << match_file << std::endl;
  ip::write_binary_match_file(match_file, left_ip, right_ip);
}
  
} // end namespace asp

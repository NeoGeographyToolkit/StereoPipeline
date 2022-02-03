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
#include <vw/InterestPoint/MatrixIO.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/Map2CamTrans.h>

using namespace vw;

namespace asp {

// Filter D_sub. Must be called only for alignment method affineepipolar, homography,
// and local_epipolar. For now this is not in use as a dataset where this would help
// was not found. It was tested though.
// TODO(oalexan1): Also do filtering by the range of disparity values, like
// asp::filter_ip_by_disparity().
void filter_D_sub(ASPGlobalOptions const& opt,
                  boost::shared_ptr<vw::camera::CameraModel> left_camera_model, 
                  boost::shared_ptr<vw::camera::CameraModel> right_camera_model,
                  vw::cartography::Datum const& datum,
                  std::string const& d_sub_file,
                  Vector2 const& outlier_removal_params) {
  
  // TODO(oalexan1): Add here the epipolar alignment.
  // TODO(oalexan1): Save the ip obtained from disparity, so that
  // they can be checked.
  // TODO(oalexan1): Print all triangulation errors, as it appears
  // that this code is too generous.
    
  if (outlier_removal_params[0] >= 100.0)
    return; // The user chose to skip outlier filtering
  
  vw_out() << "Filtering outliers in D_sub based on --outlier-removal-params.\n";

  vw::ImageViewRef<vw::PixelMask<vw::Vector2f>> sub_disp_ref;
  vw::Vector2 upsample_scale;
  asp::load_D_sub_and_scale(opt, d_sub_file, sub_disp_ref, upsample_scale);

  // Use ImageView to read D_sub fully in memory so it can be modified
  vw::ImageView<vw::PixelMask<vw::Vector2f>> sub_disp = sub_disp_ref;
    
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
      if (height(col, row) >= HIGH_ERROR) continue; // already invalid
      vals.push_back(height(col, row));
    }
  }

  // Find the outlier brackets
  double pct = outlier_removal_params[0];
  double factor = outlier_removal_params[1];
  double pct_fraction = 1.0 - pct/100.0;
  double b = -1.0, e = -1.0;
  vw::math::find_outlier_brackets(vals, pct_fraction, factor, b, e);
  vw_out() <<"Height above datum inlier range: " << b << ' ' << e <<".\n";

  // Apply the outlier threshold
  int count = 0;
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
  double pct2 = (90.0/95.0) * outlier_removal_params[0];
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

  // TODO(oalexan1): Print the tri errors here. The estimated
  // range is too large.
    
  // Filter here also by distribution of disparities
  // and by user-given height range and max tri error.
    
  // Invalidate the D_sub entries that are outliers
  for (int col = 0; col < sub_disp.cols(); col++) {
    for (int row = 0; row < sub_disp.rows(); row++) {
      if (tri_err(col, row) >= HIGH_ERROR) {
        sub_disp(col, row).invalidate();
      }
    }
  }

  vw_out() << "Writing filtered D_sub: " << d_sub_file << std::endl;
  block_write_gdal_image(d_sub_file, sub_disp, opt,
                         TerminalProgressCallback("asp","\t D_sub: "));
} 

// A small function used in making a copy of the transform for map-projected images with
// a sanity check.
TransPtr make_transform_copy(TransPtr trans){
  vw::cartography::Map2CamTrans* t_ptr = dynamic_cast<vw::cartography::Map2CamTrans*>(trans.get());
  if (!t_ptr)
    vw_throw( vw::NoImplErr() << "Need to support new map projection transform in stereo_tri.");
  return TransPtr(new vw::cartography::Map2CamTrans(*t_ptr));
}
  
// Compute an unaligned disparity image from the input disparity image
// and the image transforms.
// Note that the output image size is not the same as the input disparity image.
class UnalignDisparityView: public ImageViewBase<UnalignDisparityView>{
  
  DispImageType const& m_disparity;
  TransPtr        const& m_left_transform;
  TransPtr        const& m_right_transform;

  ASPGlobalOptions const& m_opt;
  int m_num_cols, m_num_rows;
  bool m_is_map_projected;
  std::map <std::pair<int, int>, Vector2> m_unaligned_trans;
public:
  UnalignDisparityView(bool is_map_projected,
                       DispImageType const& disparity,
                       TransPtr        const& left_transform,
                       TransPtr        const& right_transform,
                       ASPGlobalOptions const& opt):
    m_is_map_projected(is_map_projected), 
    m_disparity(disparity), m_left_transform(left_transform), 
    m_right_transform(right_transform), m_opt(opt),
    m_num_cols(0), m_num_rows(0){

    // Compute the output image size
    
    if (!m_is_map_projected) {
      // The left image passed as input is the original
      // unprojected/unaligned one, hence use its size.
      std::string left_file  = m_opt.in_file1;
      DiskImageView<float> left_img(left_file);
      m_num_cols = left_img.cols();
      m_num_rows = left_img.rows();
    }else{
      // Map projected, need to check all the pixel coordinates.
      // This is going to be slow for large images!
      BBox2i img_box;

      // Use sampling as this operation is very slow.
      int sample_len = 10;
      int num_min_samples = 100;
      int col_sample = std::max(1, std::min(sample_len, m_disparity.cols()/num_min_samples));
      int row_sample = std::max(1, std::min(sample_len, m_disparity.rows()/num_min_samples));

      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = col_sample / std::max(double(m_disparity.cols()), 1.0);
      tpc.report_progress(0);
      vw_out() << "\nEstimating the unaligned disparity dimensions.\n";

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
	img_box.grow( img_box.min() - 0.1*diff);
	img_box.grow( img_box.max() + 0.1*diff);
      }
      
      m_num_cols = img_box.max().x();
      m_num_rows = img_box.max().y();

      vw_out() << "Dimensions are: " << m_num_cols << ' ' << m_num_rows << ".\n";
    }
    // Done computing the input image size.
  }

  // ImageView interface
  typedef PixelMask<Vector2f>                          pixel_type;
  typedef pixel_type                                   result_type;
  typedef ProceduralPixelAccessor<UnalignDisparityView> pixel_accessor;

  inline int32 cols  () const { return m_num_cols; }
  inline int32 rows  () const { return m_num_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(vw::NoImplErr() << "UnalignDisparityView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    TransPtr local_left_transform;
    TransPtr local_right_transform;

    // For map-projected images the transforms are not thread-safe,
    // hence need to make a copy of them.
    if (!m_is_map_projected) {
      local_left_transform = m_left_transform;
      local_right_transform = m_right_transform;
    }else{
      local_left_transform = make_transform_copy(m_left_transform);
      local_right_transform = make_transform_copy(m_right_transform);
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
	  std::map <std::pair<int, int>, Vector2>::const_iterator it = m_unaligned_trans.find(pix);
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
	disp_bbox.grow( disp_bbox.min() - 0.1*diff);
	disp_bbox.grow( disp_bbox.max() + 0.1*diff);
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
    
    // Use the crop trick to fake that the support region is the same size as the entire image.
    return prerasterize_type(unaligned_disp, -curr_bbox.min().x(), -curr_bbox.min().y(),
			     cols(), rows());
  }
  
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class UnalignDisparityView

// Take a given disparity and make it between the original unaligned images
void unalign_disparity(bool is_map_projected,
                       DispImageType    const& disparity,
                       TransPtr         const& left_trans,
                       TransPtr         const& right_trans,
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
     TerminalProgressCallback("asp", "\t--> Undist disp:") );
  
  sw.stop();
  vw_out() << "Unaligning disparity elapsed time: " << sw.elapsed_seconds() << " seconds.\n";
  
}
  
} // end namespace asp

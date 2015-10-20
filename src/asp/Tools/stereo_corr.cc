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


/// \file stereo_corr.cc
///

#include <vw/InterestPoint.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Tools/stereo.h>
#include <asp/Core/DemDisparity.h>
#include <asp/Core/LocalHomography.h>
#include <asp/Sessions/StereoSession.h>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
using namespace std;


// Read the search range from D_sub, and scale it to the full image
void read_search_range(Options & opt){

  // No D_sub is generated or should be used for seed mode 0.
  if (stereo_settings().seed_mode == 0) return;

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
			   Rmask(opt.out_prefix + "-rMask.tif");

  DiskImageView<PixelGray<float> > left_sub( opt.out_prefix+"-L_sub.tif" ),
				   right_sub( opt.out_prefix+"-R_sub.tif" );

  Vector2 downsample_scale( double(left_sub.cols()) / double(Lmask.cols()),
			    double(left_sub.rows()) / double(Lmask.rows()) );

  std::string d_sub_file = opt.out_prefix + "-D_sub.tif";
  if (!fs::exists(d_sub_file)) return;

  ImageView<PixelMask<Vector2i> > sub_disp;
  read_image(sub_disp, d_sub_file);
  BBox2i search_range = stereo::get_disparity_range( sub_disp );
  search_range.min() = floor(elem_quot(search_range.min(),downsample_scale));
  search_range.max() = ceil(elem_quot(search_range.max(),downsample_scale));
  stereo_settings().search_range = search_range;
}

void produce_lowres_disparity( Options & opt ) {

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
			   Rmask(opt.out_prefix + "-rMask.tif");

  DiskImageView<PixelGray<float> > left_sub( opt.out_prefix+"-L_sub.tif" ),
				   right_sub( opt.out_prefix+"-R_sub.tif" );

  Vector2 downsample_scale( double(left_sub.cols()) / double(Lmask.cols()),
			    double(left_sub.rows()) / double(Lmask.rows()) );

  DiskImageView<uint8> left_mask_sub ( opt.out_prefix+"-lMask_sub.tif" ),
		       right_mask_sub( opt.out_prefix+"-rMask_sub.tif" );

  BBox2i search_range( floor(elem_prod(downsample_scale,stereo_settings().search_range.min())),
		       ceil (elem_prod(downsample_scale,stereo_settings().search_range.max())) );

  if ( stereo_settings().seed_mode == 1 ) {

    // Use low-res correlation to get the low-res disparity
    Vector2i expansion( search_range.width(),
			search_range.height() );
    expansion *= stereo_settings().seed_percent_pad / 2.0f;
    // Expand by the user selected amount. Default is 25%.
    search_range.min() -= expansion;
    search_range.max() += expansion;
    VW_OUT(DebugMessage,"asp") << "D_sub search range: "
			       << search_range << " px\n";
    // Below we use on purpose stereo::CROSS_CORRELATION instead of
    // user's choice of correlation method, since this is the most
    // accurate, as well as reasonably fast for sub-sampled images.
    stereo::CostFunctionType cost_mode = stereo::CROSS_CORRELATION; // hard-coded
    Vector2i kernel_size  = stereo_settings().corr_kernel;
    int corr_timeout      = 5*stereo_settings().corr_timeout; // 5x, so try hard
    double seconds_per_op = 0.0;
    if (corr_timeout > 0)
      seconds_per_op = calc_seconds_per_op(cost_mode, left_sub,
					   right_sub, kernel_size);

    asp::block_write_gdal_image
      (opt.out_prefix + "-D_sub.tif",
       rm_outliers_using_thresh
       (stereo::pyramid_correlate
	(left_sub, right_sub,
	 left_mask_sub, right_mask_sub,
	 stereo::LaplacianOfGaussian(stereo_settings().slogW),
	 search_range, kernel_size, cost_mode,
	 corr_timeout, seconds_per_op,
	 stereo_settings().xcorr_threshold, stereo_settings().corr_max_levels),
	// To do: all these hard-coded values must be replaced with
	// appropriate params from user's stereo.default, for
	// consistency with how disparity is filtered in stereo_fltr,
	// when invoking disparity_cleanup_using_thresh.
	1, 1, // in stereo.default we have 5 5
	// Changing below the hard-coded value from 2.0 to using a
	// param.  The default value will still be 2.0 but is now
	// modifiable. Need to get rid of the 2.0/3.0 factor and
	// study how it affects the result.
	stereo_settings().rm_threshold*2.0/3.0,
	// Another change of hard-coded value to param. Get rid of 0.5/0.6
	// and study the effect.
	(stereo_settings().rm_min_matches/100.0)*0.5/0.6
	), opt,
       TerminalProgressCallback("asp", "\t--> Low-resolution disparity:")
       );

  }else if ( stereo_settings().seed_mode == 2 ) {
    // Use a DEM to get the low-res disparity
    boost::shared_ptr<camera::CameraModel> left_camera_model, right_camera_model;
    opt.session->camera_models(left_camera_model, right_camera_model);
    produce_dem_disparity(opt, left_camera_model, right_camera_model,
			  opt.session->name());
  }else if ( stereo_settings().seed_mode == 3 ) {
    // D_sub is already generated by now by sparse_disp
  }

  read_search_range(opt);
}

void lowres_correlation( Options & opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
	   << " ] : Stage 1 --> LOW-RESOLUTION CORRELATION \n";

  // Working out search range if need be
  if (stereo_settings().is_search_defined()) {
    vw_out() << "\t--> Using user-defined search range.\n";
  }else if (stereo_settings().seed_mode == 2){
    // Do nothing as we will compute the search range based on D_sub
  }else if (stereo_settings().seed_mode == 3){
    // Do nothing as low-res disparity is already done by sparse_disp
  } else {

    // Define the file name containing IP match information.
    string match_filename
      = ip::match_filename(opt.out_prefix, opt.in_file1, opt.in_file2);

    if (!fs::exists(match_filename)) {
      // If there is no match file for the input images, gather some IP from the
      // low resolution images. This routine should only run for:
      //   Pinhole + Epipolar
      //   Alignment method none
      // Everything else should gather IP's all the time.
      double sub_scale =
	sum(elem_quot( Vector2(file_image_size( opt.out_prefix+"-L_sub.tif" )),
		       Vector2(file_image_size( opt.out_prefix+"-L.tif" ) ) )) +
	sum(elem_quot( Vector2(file_image_size( opt.out_prefix+"-R_sub.tif" )),
		       Vector2(file_image_size( opt.out_prefix+"-R.tif" ) ) ));
      sub_scale /= 4.0f;

      stereo_settings().search_range =
	approximate_search_range(opt.out_prefix,
				 opt.out_prefix+"-L_sub.tif",
				 opt.out_prefix+"-R_sub.tif",
				 sub_scale );
    } else {
      // There exists a matchfile out there.
      vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file( match_filename, ip1, ip2 );

      Matrix<double> align_left_matrix  = math::identity_matrix<3>();
      Matrix<double> align_right_matrix = math::identity_matrix<3>();
      if ( fs::exists(opt.out_prefix+"-align-L.exr") )
	read_matrix(align_left_matrix, opt.out_prefix + "-align-L.exr");
      if ( fs::exists(opt.out_prefix+"-align-R.exr") )
	read_matrix(align_right_matrix, opt.out_prefix + "-align-R.exr");

      Vector2 left_size  = file_image_size( opt.out_prefix+"-L.tif" );
      Vector2 right_size = file_image_size( opt.out_prefix+"-R.tif" );

      BBox2 search_range;
      for ( size_t i = 0; i < ip1.size(); i++ ) {
	Vector3 r = align_right_matrix * Vector3(ip2[i].x, ip2[i].y, 1);
	Vector3 l = align_left_matrix  * Vector3(ip1[i].x, ip1[i].y, 1);

	// Don't divide by 0
	if (l[2] == 0 || r[2] == 0) continue;

	r /= r[2];
	l /= l[2];

	// Bugfix: skip obvious outliers, points way out there
	if (std::abs(l[0]) > 2*left_size[0]   || std::abs(l[1]) > 2*left_size[1]  ||
	    std::abs(r[0]) > 2*right_size[0]  || std::abs(r[1]) > 2*right_size[1] ) continue;

	search_range.grow( subvector(r,0,2) - subvector(l,0,2) );
      }
      stereo_settings().search_range = grow_bbox_to_int( search_range );
    }
    vw_out() << "\t--> Detected search range: "
	     << stereo_settings().search_range << "\n";
  }

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
    Rmask(opt.out_prefix + "-rMask.tif");

  // Performing disparity on sub images
  if ( stereo_settings().seed_mode > 0 ) {

    // Reuse prior existing D_sub if it exists, unless we
    // are cropping the images each time, when D_sub must
    // be computed anew each time.
    bool crop_left_and_right =
      ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
      ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) );
    bool rebuild = crop_left_and_right;

    string sub_disp_file = opt.out_prefix+"-D_sub.tif";
    try {
      vw_log().console_log().rule_set().add_rule(-1,"fileio");
      DiskImageView<PixelMask<Vector2i> > test(sub_disp_file);
      vw_settings().reload_config();
    } catch (vw::IOErr const& e) {
      vw_settings().reload_config();
      rebuild = true;
    } catch (vw::ArgumentErr const& e ) {
      // Throws on a corrupted file.
      vw_settings().reload_config();
      rebuild = true;
    }

    if ( rebuild )
      produce_lowres_disparity( opt );
    else
      vw_out() << "\t--> Using cached low-resolution disparity: "
	       << sub_disp_file << "\n";
  }

  // Create the local homographies based on D_sub
  if (stereo_settings().seed_mode > 0 && stereo_settings().use_local_homography){
    string local_hom_file = opt.out_prefix + "-local_hom.txt";
    try {
      ImageView<Matrix3x3> local_hom;
      read_local_homographies(local_hom_file, local_hom);
    } catch (vw::IOErr const& e) {
      create_local_homographies(opt);
    }
  }

  vw_out() << "\n[ " << current_posix_time_string()
	   << " ] : LOW-RESOLUTION CORRELATION FINISHED \n";
}

// This correlator takes a low resolution disparity image as an input
// so that it may narrow its search range for each tile that is processed.
template <class Image1T, class Image2T, class Mask1T, class Mask2T, class SeedDispT, class PProcT>
class SeededCorrelatorView : public ImageViewBase<SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT > > {
  Image1T   m_left_image;
  Image2T   m_right_image;
  Mask1T    m_left_mask;
  Mask2T    m_right_mask;
  SeedDispT m_sub_disp;
  SeedDispT m_sub_disp_spread;
  ImageView<Matrix3x3> const& m_local_hom;
  PProcT    m_preproc_func;

  // Settings
  Vector2 m_upscale_factor;
  BBox2i m_seed_bbox;
  BBox2i m_trans_crop_win;
  Vector2i m_kernel_size;
  stereo::CostFunctionType m_cost_mode;
  int m_corr_timeout;
  double m_seconds_per_op;

public:
  SeededCorrelatorView( ImageViewBase<Image1T>   const& left_image,
			ImageViewBase<Image2T>   const& right_image,
			ImageViewBase<Mask1T>    const& left_mask,
			ImageViewBase<Mask2T>    const& right_mask,
			ImageViewBase<SeedDispT> const& sub_disp,
			ImageViewBase<SeedDispT> const& sub_disp_spread,
			ImageView<Matrix3x3>     const& local_hom,
			stereo::PreFilterBase<PProcT> const& filter,
			BBox2i trans_crop_win,
			Vector2i const& kernel_size,
			stereo::CostFunctionType cost_mode,
			int corr_timeout, double seconds_per_op) :
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_left_mask(left_mask.impl()), m_right_mask(right_mask.impl()),
    m_sub_disp(sub_disp.impl()), m_sub_disp_spread(sub_disp_spread.impl()),
    m_local_hom(local_hom), m_preproc_func( filter.impl() ),
    m_trans_crop_win(trans_crop_win),
    m_kernel_size(kernel_size),  m_cost_mode(cost_mode),
    m_corr_timeout(corr_timeout), m_seconds_per_op(seconds_per_op){
    m_upscale_factor[0] = double(m_left_image.cols()) / m_sub_disp.cols();
    m_upscale_factor[1] = double(m_left_image.rows()) / m_sub_disp.rows();
    m_seed_bbox = bounding_box( m_sub_disp );
  }

  // Image View interface
  typedef PixelMask<Vector2i> pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<SeededCorrelatorView> pixel_accessor;

  inline int32 cols  () const { return m_left_image.cols(); }
  inline int32 rows  () const { return m_left_image.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()(double /*i*/, double /*j*/, int32 /*p*/ = 0) const {
    vw_throw(NoImplErr()
	     << "SeededCorrelatorView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // We do stereo only in m_trans_crop_win. Skip the current tile if
    // it does not intersect this region.
    BBox2i intersection = bbox; intersection.crop(m_trans_crop_win);
    if (intersection.empty()){
      return prerasterize_type(ImageView<pixel_type>(bbox.width(),
						     bbox.height()),
			       -bbox.min().x(), -bbox.min().y(),
			       cols(), rows() );
    }

    CropView<ImageView<pixel_type> > disparity = prerasterize_helper(bbox);

    // Set to invalid the disparity outside m_trans_crop_win.
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
	if (!m_trans_crop_win.contains(Vector2(col, row))){
	  disparity(col, row) = pixel_type();
	}
      }
    }

    return disparity;
  }

  inline prerasterize_type prerasterize_helper(BBox2i const& bbox) const {

    bool use_local_homography = stereo_settings().use_local_homography;

    Matrix<double> lowres_hom  = math::identity_matrix<3>();
    Matrix<double> fullres_hom = math::identity_matrix<3>();
    ImageViewRef<typename Image2T::pixel_type> right_trans_img;
    ImageViewRef<typename Mask2T::pixel_type > right_trans_mask;

    bool do_round = true; // round integer disparities after transform

    // User strategies
    BBox2f local_search_range;
    if ( stereo_settings().seed_mode > 0 ) {

      // The low-res version of bbox
      BBox2i seed_bbox( elem_quot(bbox.min(), m_upscale_factor),
			elem_quot(bbox.max(), m_upscale_factor) );
      seed_bbox.expand(1);
      seed_bbox.crop( m_seed_bbox );
      VW_OUT(DebugMessage, "stereo") << "Getting disparity range for : "
				     << seed_bbox << "\n";
      SeedDispT disparity_in_box = crop( m_sub_disp, seed_bbox );

      if (!use_local_homography){
	local_search_range = stereo::get_disparity_range( disparity_in_box );
      }else{
	int ts = Options::corr_tile_size();
	lowres_hom = m_local_hom(bbox.min().x()/ts, bbox.min().y()/ts);
	local_search_range = stereo::get_disparity_range
	  (transform_disparities(do_round, seed_bbox,
				 lowres_hom, disparity_in_box));
      }

      bool has_sub_disp_spread = ( m_sub_disp_spread.cols() != 0 &&
				   m_sub_disp_spread.rows() != 0 );

      // Sanity check: If m_sub_disp_spread was provided, it better have
      // the same size as sub_disp.
      if ( has_sub_disp_spread &&
	   m_sub_disp_spread.cols() != m_sub_disp.cols() &&
	   m_sub_disp_spread.rows() != m_sub_disp.rows() ){
	vw_throw( ArgumentErr() << "stereo_corr: D_sub and D_sub_spread must have equal sizes.\n");
      }

      if (has_sub_disp_spread){

	// Expand the disparity range by m_sub_disp_spread.
	SeedDispT spread_in_box = crop( m_sub_disp_spread, seed_bbox );

	if (!use_local_homography){
	  BBox2f spread = stereo::get_disparity_range( spread_in_box );
	  local_search_range.min() -= spread.max();
	  local_search_range.max() += spread.max();
	}else{
	  SeedDispT upper_disp
	    = transform_disparities(do_round, seed_bbox, lowres_hom,
				    disparity_in_box + spread_in_box);
	  SeedDispT lower_disp
	    = transform_disparities(do_round, seed_bbox, lowres_hom,
				    disparity_in_box - spread_in_box);
	  BBox2f upper_range = stereo::get_disparity_range(upper_disp);
	  BBox2f lower_range = stereo::get_disparity_range(lower_disp);

	  local_search_range = upper_range;
	  local_search_range.grow(lower_range);
	}
      }

      if (use_local_homography){
	Vector3 upscale( m_upscale_factor[0], m_upscale_factor[1], 1 );
	Vector3 dnscale( 1.0/m_upscale_factor[0], 1.0/m_upscale_factor[1], 1 );
	fullres_hom = diagonal_matrix(upscale)*lowres_hom*diagonal_matrix(dnscale);

	ImageViewRef< PixelMask<typename Image2T::pixel_type> >
	  right_trans_masked_img
	  = transform (copy_mask( m_right_image.impl(),
				  create_mask(m_right_mask.impl()) ),
		       HomographyTransform(fullres_hom),
		       m_left_image.impl().cols(), m_left_image.impl().rows());
	right_trans_img = apply_mask(right_trans_masked_img);
	right_trans_mask
	  = channel_cast_rescale<uint8>(select_channel(right_trans_masked_img, 1));
      }

      local_search_range = grow_bbox_to_int(local_search_range);
      // Expand local_search_range by 1. This is necessary since
      // m_sub_disp is integer-valued, and perhaps the search
      // range was supposed to be a fraction of integer bigger.
      local_search_range.expand(1);
      // Scale the search range to full-resolution
      local_search_range.min() = floor(elem_prod(local_search_range.min(),
						 m_upscale_factor));
      local_search_range.max() = ceil(elem_prod(local_search_range.max(),
						m_upscale_factor));

      VW_OUT(DebugMessage, "stereo") << "SeededCorrelatorView("
				     << bbox << ") search range "
				     << local_search_range << " vs "
				     << stereo_settings().search_range << "\n";

    } else{
      local_search_range = stereo_settings().search_range;
      VW_OUT(DebugMessage,"stereo") << "Searching with "
				    << stereo_settings().search_range << "\n";
    }

    if (use_local_homography){
      typedef stereo::PyramidCorrelationView<Image1T, ImageViewRef<typename Image2T::pixel_type>, Mask1T,ImageViewRef<typename Mask2T::pixel_type>, PProcT> CorrView;
      CorrView corr_view( m_left_image,   right_trans_img,
			  m_left_mask,    right_trans_mask,
			  m_preproc_func, local_search_range,
			  m_kernel_size,  m_cost_mode,
			  m_corr_timeout, m_seconds_per_op,
			  stereo_settings().xcorr_threshold,
			  stereo_settings().corr_max_levels );
      return corr_view.prerasterize(bbox);
    }else{
      typedef stereo::PyramidCorrelationView<Image1T, Image2T, Mask1T, Mask2T, PProcT> CorrView;
      CorrView corr_view( m_left_image,   m_right_image,
			  m_left_mask,    m_right_mask,
			  m_preproc_func, local_search_range,
			  m_kernel_size,  m_cost_mode,
			  m_corr_timeout, m_seconds_per_op,
			  stereo_settings().xcorr_threshold,
			  stereo_settings().corr_max_levels );
      return corr_view.prerasterize(bbox);
    }
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class Image1T, class Image2T, class Mask1T, class Mask2T, class SeedDispT, class PProcT>
SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT>
seeded_correlation( ImageViewBase<Image1T>        const& left,
		    ImageViewBase<Image2T>        const& right,
		    ImageViewBase<Mask1T>         const& lmask,
		    ImageViewBase<Mask2T>         const& rmask,
		    ImageViewBase<SeedDispT>      const& sub_disp,
		    ImageViewBase<SeedDispT>      const& sub_disp_spread,
		    ImageView<Matrix3x3>          const& local_hom,
		    stereo::PreFilterBase<PProcT> const& filter,
		    BBox2i trans_crop_win,
		    Vector2i const& kernel_size,
		    stereo::CostFunctionType cost_type,
		    int corr_timeout, double seconds_per_op) {
  typedef SeededCorrelatorView<Image1T, Image2T, Mask1T, Mask2T, SeedDispT, PProcT> return_type;
  return return_type( left.impl(), right.impl(), lmask.impl(), rmask.impl(),
		      sub_disp.impl(), sub_disp_spread.impl(),
		      local_hom, filter.impl(), trans_crop_win, kernel_size,
		      cost_type, corr_timeout, seconds_per_op );
}

void stereo_correlation( Options& opt ) {

  // Note that even when we are told to skip low-resolution correlation,
  // we must still go through the motions when seed_mode is 0, to be
  // able to get a search range, even though we don't write D_sub then.
  if (!stereo_settings().skip_low_res_disparity_comp || stereo_settings().seed_mode == 0)
    lowres_correlation(opt);

  if (stereo_settings().compute_low_res_disparity_only) return;

  vw_out() << "\n[ " << current_posix_time_string()
	   << " ] : Stage 1 --> CORRELATION \n";

  read_search_range(opt);

  // Provide the user with some feedback of what we are actually going
  // to use.
  vw_out()   << "\t--------------------------------------------------\n";
  vw_out()   << "\t   Kernel Size:    " << stereo_settings().corr_kernel << endl;
  if ( stereo_settings().seed_mode > 0 )
    vw_out() << "\t   Refined Search: "
	     << stereo_settings().search_range << endl;
  else
    vw_out() << "\t   Search Range:   "
	     << stereo_settings().search_range << endl;
  vw_out()   << "\t   Cost Mode:      " << stereo_settings().cost_mode << endl;
  vw_out(DebugMessage) << "\t   XCorr Threshold: " << stereo_settings().xcorr_threshold << endl;
  vw_out(DebugMessage) << "\t   Prefilter:       " << stereo_settings().pre_filter_mode << endl;
  vw_out(DebugMessage) << "\t   Prefilter Size:  " << stereo_settings().slogW << endl;
  vw_out() << "\t--------------------------------------------------\n";

  // Load up for the actual native resolution processing
  DiskImageView<PixelGray<float> > left_disk_image(opt.out_prefix+"-L.tif"),
    right_disk_image(opt.out_prefix+"-R.tif");
  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
    Rmask(opt.out_prefix + "-rMask.tif");
  ImageViewRef<PixelMask<Vector2i> > sub_disp;
  std::string dsub_file = opt.out_prefix+"-D_sub.tif";
  std::string spread_file = opt.out_prefix+"-D_sub_spread.tif";
  if ( stereo_settings().seed_mode > 0 )
    sub_disp =
      DiskImageView<PixelMask<Vector2i> >(dsub_file);
  ImageViewRef<PixelMask<Vector2i> > sub_disp_spread;
  if ( stereo_settings().seed_mode == 2 ||  stereo_settings().seed_mode == 3 ){
    // D_sub_spread is mandatory for seed_mode 2 and 3.
    sub_disp_spread =
      DiskImageView<PixelMask<Vector2i> >(spread_file);
  }else if ( stereo_settings().seed_mode == 1 ){
    // D_sub_spread is optional for seed_mode 1, we use it only if
    // it is provided.
    if (fs::exists(spread_file)) {
      try {
	sub_disp_spread = DiskImageView<PixelMask<Vector2i> >(spread_file);
      }
      catch (...) {}
    }
  }

  ImageView<Matrix3x3> local_hom;
  if ( stereo_settings().seed_mode > 0 && stereo_settings().use_local_homography ){
    string local_hom_file = opt.out_prefix + "-local_hom.txt";
    read_local_homographies(local_hom_file, local_hom);
  }

  stereo::CostFunctionType cost_mode;
  if      (stereo_settings().cost_mode == 0) cost_mode = stereo::ABSOLUTE_DIFFERENCE;
  else if (stereo_settings().cost_mode == 1) cost_mode = stereo::SQUARED_DIFFERENCE;
  else if (stereo_settings().cost_mode == 2) cost_mode = stereo::CROSS_CORRELATION;
  else
    vw_throw( ArgumentErr() << "Unknown value " << stereo_settings().cost_mode
	      << " for cost-mode.\n" );

  ImageViewRef<PixelMask<Vector2i> > fullres_disparity;
  Vector2i kernel_size = stereo_settings().corr_kernel;
  BBox2i trans_crop_win = stereo_settings().trans_crop_win;
  int corr_timeout      = stereo_settings().corr_timeout;
  double seconds_per_op = 0.0;
  if (corr_timeout > 0)
    seconds_per_op = calc_seconds_per_op(cost_mode, left_disk_image, right_disk_image,
					 kernel_size);

  if ( stereo_settings().pre_filter_mode == 2 ) {
    vw_out() << "\t--> Using LOG pre-processing filter with "
	     << stereo_settings().slogW << " sigma blur.\n";
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask,
			  sub_disp, sub_disp_spread, local_hom,
			  stereo::LaplacianOfGaussian(stereo_settings().slogW),
			  trans_crop_win, kernel_size, cost_mode, corr_timeout,
			  seconds_per_op );
  } else if ( stereo_settings().pre_filter_mode == 1 ) {
    vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
	     << stereo_settings().slogW << " sigma blur.\n";
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask,
			  sub_disp, sub_disp_spread, local_hom,
			  stereo::SubtractedMean(stereo_settings().slogW),
			  trans_crop_win, kernel_size, cost_mode, corr_timeout,
			  seconds_per_op );
  } else {
    vw_out() << "\t--> Using NO pre-processing filter." << endl;
    fullres_disparity =
      seeded_correlation( left_disk_image, right_disk_image, Lmask, Rmask,
			  sub_disp, sub_disp_spread, local_hom,
			  stereo::NullOperation(),
			  trans_crop_win, kernel_size, cost_mode, corr_timeout,
			  seconds_per_op );
  }

  cartography::GeoReference left_georef;
  bool has_left_georef = read_georeference(left_georef,  opt.out_prefix + "-L.tif");
  bool has_nodata = false;
  double nodata = -32768.0;

  string d_file = opt.out_prefix + "-D.tif";
  vw_out() << "Writing: " << d_file << "\n";
  asp::block_write_gdal_image(d_file, fullres_disparity,
			      has_left_georef, left_georef,
			      has_nodata, nodata, opt,
			      TerminalProgressCallback("asp", "\t--> Correlation :") );

  vw_out() << "\n[ " << current_posix_time_string()
	   << " ] : CORRELATION FINISHED \n";

}

int main(int argc, char* argv[]) {

  try {

    stereo_register_sessions();

    bool verbose = false;
    vector<Options> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, CorrelationDescription(),
			 verbose, output_prefix, opt_vec);
    Options opt = opt_vec[0];

    // Integer correlator requires large tiles
    //---------------------------------------------------------
    int ts = Options::corr_tile_size();
    opt.raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------
    stereo_correlation( opt );

  } ASP_STANDARD_CATCHES;

  return 0;
}

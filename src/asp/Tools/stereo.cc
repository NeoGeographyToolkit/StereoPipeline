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


/// \file stereo.cc
///

#include <vw/Cartography.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CorrelationView.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Tools/stereo.h>
#include <asp/Sessions/RPC/RPCModel.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

using namespace vw;
using namespace vw::cartography;

namespace asp {

  // Parse input command line arguments
  void handle_arguments( int argc, char *argv[], Options& opt,
                         boost::program_options::options_description const&
                         additional_options ) {

    // Print the command being run in debug mode.
    std::string run_cmd = "";
    for (int s = 0; s < argc; s++) run_cmd += std::string(argv[s]) + " ";
    VW_OUT(DebugMessage, "stereo") << "\n\n" << run_cmd << "\n";

    po::options_description general_options_sub("");
    general_options_sub.add_options()
      ("session-type,t", po::value(&opt.stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis dg rpc]")
      ("stereo-file,s", po::value(&opt.stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
      ("left-image-crop-win", po::value(&opt.left_image_crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"), "Do stereo in a subregion of the left image [default: use the entire image].");

    // We distinguish between all_general_options, which is all the
    // options we must parse, even if we don't need some of them, and
    // general_options, which are the options specifically used by the
    // current tool, and for which we also print the help message.

    po::options_description general_options("");
    general_options.add ( general_options_sub );
    general_options.add( additional_options );
    general_options.add( asp::BaseOptionsDescription(opt) );

    po::options_description all_general_options("");
    all_general_options.add ( general_options_sub );
    all_general_options.add( generate_config_file_options( opt ) );

    po::options_description positional_options("");
    positional_options.add_options()
      ("left-input-image", po::value(&opt.in_file1), "Left Input Image")
      ("right-input-image", po::value(&opt.in_file2), "Right Input Image")
      ("left-camera-model", po::value(&opt.cam_file1), "Left Camera Model File")
      ("right-camera-model", po::value(&opt.cam_file2), "Right Camera Model File")
      ("output-prefix", po::value(&opt.out_prefix), "Prefix for output filenames")
      ("input_dem", po::value(&opt.input_dem), "Input DEM")
      ("extra_argument1", po::value(&opt.extra_argument1), "Extra Argument 1")
      ("extra_argument2", po::value(&opt.extra_argument2), "Extra Argument 2")
      ("extra_argument3", po::value(&opt.extra_argument3), "Extra Argument 3");

    po::positional_options_description positional_desc;
    positional_desc.add("left-input-image", 1);
    positional_desc.add("right-input-image", 1);
    positional_desc.add("left-camera-model", 1);
    positional_desc.add("right-camera-model", 1);
    positional_desc.add("output-prefix", 1);
    positional_desc.add("input_dem", 1);
    positional_desc.add("extra_argument1", 1);
    positional_desc.add("extra_argument2", 1);
    positional_desc.add("extra_argument3", 1);

    std::string usage("[options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix> [DEM]\n  Extensions are automaticaly added to the output files.\n  Camera model arguments may be optional for some stereo session types (e.g., isis).\n  Stereo parameters should be set in the stereo.default file.");
    po::variables_map vm =
      asp::check_command_line( argc, argv, opt, general_options, all_general_options,
                               positional_options, positional_desc, usage, false );

    if (!vm.count("left-input-image") || !vm.count("right-input-image") ||
        !vm.count("left-camera-model") )
      vw_throw( ArgumentErr() << "Missing all of the correct input files.\n\n"
                << usage << general_options );

    // Read the config file
    try {
      po::options_description cfg_options;
      cfg_options.add( positional_options ); // The user can specify the
                                             // positional input from the
                                             // stereo.default if they want
                                             // to.
      cfg_options.add( generate_config_file_options( opt ) );

      // Append the options from the config file. Do not overwrite the
      // options already set on the command line.
      po::store(parse_asp_config_file(opt.stereo_default_filename,
                                      cfg_options), vm);
      po::notify( vm );
    } catch ( po::error const& e ) {
      vw::vw_throw( vw::ArgumentErr() << "Error parsing configuration file:\n"
                    << e.what() << "\n" );
    }
    asp::stereo_settings().validate();

    // If the user hasn't specified a stereo session type, we take a
    // guess here based on the file suffixes.
    if (opt.stereo_session_string.empty())
      guess_session_type(opt);

    // Some specialization here so that the user doesn't need to list
    // camera models on the command line for certain stereo session
    // types.  (e.g. isis).
    //
    // TODO: This modification of arguments should probably happen in
    // initialization and not be dependent on Stereo knowing what
    // session it is in.
    bool check_for_camera_models = true;
    if ( opt.stereo_session_string == "isis" ||
         opt.stereo_session_string == "rpc" ) {
      // Fix the ordering of the arguments if the user only supplies 3
      if (opt.out_prefix.empty()) {
        opt.out_prefix = opt.cam_file1;
        opt.cam_file1.clear();
      }
      check_for_camera_models = false;
    }

    if ( check_for_camera_models &&
         ( opt.out_prefix.empty() || opt.cam_file2.empty() ) )
      vw_throw( ArgumentErr() << "\nMissing output-prefix or right camera model.\n" );

    // Interpret the the last two coordinates of left_image_crop_win as
    // width and height rather than max_x and max_y
    BBox2i b = opt.left_image_crop_win;
    opt.left_image_crop_win = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
    // By default, we do stereo in the entire image
    DiskImageView<PixelGray<float> > left_image(opt.in_file1);
    BBox2i full_box = BBox2i(0, 0, left_image.cols(), left_image.rows());
    if (opt.left_image_crop_win == BBox2i(0, 0, 0, 0)){
      opt.left_image_crop_win = full_box;
    }
    // Ensure that the region is inside the maximum theoretical region
    opt.left_image_crop_win.crop(full_box);
    // Sanity check
    if (opt.left_image_crop_win.width() <= 0 || opt.left_image_crop_win.height() <= 0 ){
      vw_throw( ArgumentErr() << "Invalid region for doing stereo.\n\n"
                << usage << general_options );
    }

    fs::path out_prefix_path(opt.out_prefix);
    if (out_prefix_path.has_parent_path()) {
      if (!fs::is_directory(out_prefix_path.parent_path())) {
        vw_out() << "\nCreating output directory: "
                 << out_prefix_path.parent_path() << std::endl;
        fs::create_directory(out_prefix_path.parent_path());
      }
    }

    opt.session.reset( asp::StereoSession::create(opt.stereo_session_string) );
    opt.session->initialize(opt, opt.in_file1, opt.in_file2,
                            opt.cam_file1, opt.cam_file2,
                            opt.out_prefix, opt.input_dem, opt.extra_argument1,
                            opt.extra_argument2, opt.extra_argument3);

    user_safety_check(opt);

    // The last thing we do before we get started is to copy the
    // stereo.default settings over into the results directory so that
    // we have a record of the most recent stereo.default that was used
    // with this data set.
    asp::stereo_settings().write_copy( argc, argv,
                                       opt.stereo_default_filename,
                                       opt.out_prefix + "-stereo.default" );
  }

  void guess_session_type(Options& opt) {
    if ( asp::has_cam_extension( opt.cam_file1 ) &&
         asp::has_cam_extension( opt.cam_file2 ) ) {
      vw_out() << "\t--> Detected pinhole camera files. "
               << "Executing pinhole stereo pipeline.\n";
      opt.stereo_session_string = "pinhole";
      return;
    }
    if (boost::iends_with(boost::to_lower_copy(opt.in_file1), ".cub") &&
        boost::iends_with(boost::to_lower_copy(opt.in_file2), ".cub")) {
      vw_out() << "\t--> Detected ISIS cube files. "
               << "Executing ISIS stereo pipeline.\n";
      opt.stereo_session_string = "isis";
      return;
    }
    // RPC can be in the main file or it can be in the camera file
    try {
      StereoSessionRPC session;
      boost::shared_ptr<camera::CameraModel>
        left_model = session.camera_model( opt.in_file1, opt.cam_file1 ),
        right_model = session.camera_model( opt.in_file2, opt.cam_file2 );
      vw_out() << "\t--> Detected RPC Model inside image files. "
               << "Executing RPC stereo pipeline.\n";
      opt.stereo_session_string = "rpc";
      return;
    } catch ( vw::NotFoundErr const& e ) {
      // If it throws, it wasn't RPC
    }
    if (boost::iends_with(boost::to_lower_copy(opt.cam_file1), ".xml") &&
        boost::iends_with(boost::to_lower_copy(opt.cam_file2), ".xml")) {
      vw_out() << "\t--> Detected likely Digital Globe XML files. "
               << "Executing DG stereo pipeline.\n";
      opt.stereo_session_string = "dg";
      return;
    }

    // If we get to this point. We couldn't guess the session type
    vw_throw( ArgumentErr() << "Could not determine stereo session type. "
              << "Please set it explicitly.\n"
              << "using the -t switch. Options include: [pinhole isis dg rpc].\n" );
  }

  // Register Session types
  void stereo_register_sessions() {

#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    // Register the Isis file handler with the Vision Workbench
    // DiskImageResource system.
    DiskImageResource::register_file_type(".cub",
                                          DiskImageResourceIsis::type_static(),
                                          &DiskImageResourceIsis::construct_open,
                                          &DiskImageResourceIsis::construct_create);
#endif
    asp::StereoSession::register_session_type( "rpc",  &asp::StereoSessionRPC::construct);
    asp::StereoSession::register_session_type( "rmax", &asp::StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    asp::StereoSession::register_session_type( "isis", &asp::StereoSessionIsis::construct);
#endif

  }

  void user_safety_check(Options const& opt){

    //---------------------------------------------------------
    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1, camera_model2);

      Vector3 cam1_ctr = camera_model1->camera_center(Vector2());
      Vector3 cam2_ctr = camera_model2->camera_center(Vector2());
      Vector3 cam1_vec = camera_model1->pixel_to_vector(Vector2());
      Vector3 cam2_vec = camera_model2->pixel_to_vector(Vector2());
      // Do the cameras appear to be in the same location?
      if ( norm_2(cam1_ctr - cam2_ctr) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";

      // Developer friendly help
      VW_OUT(DebugMessage,"asp") << "Camera 1 location: " << cam1_ctr << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << cam2_ctr << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(cam2_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 Pointing Dir: " << cam1_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam1_vec, cam1_ctr) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 Pointing Dir: " << cam2_vec << "\n"
                                 << "      dot against pos: " << dot_prod(cam2_vec, cam2_ctr) << "\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model( camera_model1.get(), camera_model2.get() );
      double error;
      Vector3 point = model( Vector2(), Vector2(), error );
      if ( point != Vector3() // triangulation succeeded
           && (
               dot_prod( cam1_vec, point - cam1_ctr ) < 0
               ||
               dot_prod( cam2_vec, point - cam2_ctr ) < 0
               )
           ){
        vw_out(WarningMessage,"console")
          << "Your cameras appear not to be pointing at the same location!\n"
          << "\tA test vector triangulated backwards through\n"
          << "\tthe camera models. You should double check\n"
          << "\tyour input models as most likely stereo won't\n"
          << "\tbe able to triangulate.\n";
      }
    } catch ( camera::PixelToRayErr const& e ) {
    } catch ( camera::PointToPixelErr const& e ) {
      // Silent. Top Left pixel might not be valid on a map
      // projected image.
    }

    // Checks for map-projected images below

    if ( opt.session->has_lut_images() && stereo_settings().seed_mode == 2 )
      vw_throw( NoImplErr() << "Computation of low-resolution disparity from DEM is not implemented for map-projected images.\n");

    GeoReference georef;
    bool has_georef1 = read_georeference( georef, opt.in_file1 );
    bool has_georef2 = read_georeference( georef, opt.in_file2 );
    if ( opt.session->has_lut_images() && (!has_georef1 || !has_georef2)){
      vw_throw( ArgumentErr() << "The images are not map-projected, cannot use the provided DEM: "
                << opt.input_dem << ".\n\n");
    }

    if (opt.stereo_session_string == "dg" && has_georef1 && has_georef2 && opt.input_dem == "") {
      vw_out(WarningMessage) << "It appears that the input images are map-projected. In that case a DEM needs to be provided for stereo to give correct results.\n";
    }

  }

  template <class ImageT, class DEMImageT>
  class DemDisparity : public ImageViewBase<DemDisparity<ImageT, DEMImageT> > {
    ImageT m_left_image;
    double m_dem_accuracy;
    GeoReference m_dem_georef;
    const DEMImageT & m_dem;
    Vector2f m_downsample_scale;
    boost::shared_ptr<camera::CameraModel> m_left_camera_model;
    boost::shared_ptr<camera::CameraModel> m_right_camera_model;
    bool m_homography_align;
    Matrix<double>  m_align_matrix;
    int m_pixel_sample;
    ImageView<PixelMask<Vector2i> > & m_disparity_spread;

  public:
    DemDisparity( ImageViewBase<ImageT> const& left_image,
                  double dem_accuracy, GeoReference dem_georef,
                  DEMImageT const& dem,
                  Vector2f const& downsample_scale,
                  boost::shared_ptr<camera::CameraModel> left_camera_model,
                  boost::shared_ptr<camera::CameraModel> right_camera_model,
                  bool homography_align, Matrix<double> const& align_matrix,
                  int pixel_sample, ImageView<PixelMask<Vector2i> > & disparity_spread)
      :m_left_image(left_image.impl()),
       m_dem_accuracy(dem_accuracy),
       m_dem_georef(dem_georef),
       m_dem(dem),
       m_downsample_scale(downsample_scale),
       m_left_camera_model(left_camera_model),
       m_right_camera_model(right_camera_model),
       m_homography_align(homography_align),
       m_align_matrix(align_matrix),
       m_pixel_sample(pixel_sample),
       m_disparity_spread(disparity_spread){}

    // Image View interface
    typedef PixelMask<Vector2i> pixel_type;
    typedef pixel_type result_type;
    typedef ProceduralPixelAccessor<DemDisparity> pixel_accessor;

    inline int32 cols() const { return m_left_image.cols(); }
    inline int32 rows() const { return m_left_image.rows(); }
    inline int32 planes() const { return 1; }

    inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

    inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
      vw_throw(NoImplErr() << "DemDisparity::operator()(...) is not implemented");
      return pixel_type();
    }

    typedef CropView<ImageView<pixel_type> > prerasterize_type;
    inline prerasterize_type prerasterize(BBox2i const& bbox) const {

      std::cout << "start box " << bbox << std::endl;

      prerasterize_type lowres_disparity = prerasterize_type(ImageView<pixel_type>(bbox.width(),
                                                                                   bbox.height()),
                                                             -bbox.min().x(), -bbox.min().y(),
                                                             cols(), rows() );

      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        for (int col = bbox.min().x(); col < bbox.max().x(); col++){
          lowres_disparity(col, row).invalidate();
          m_disparity_spread(col, row).invalidate();
        }
      }

      double height_error_tol = std::max(m_dem_accuracy/4.0, 1.0); // height error in meters
      double max_abs_tol = height_error_tol/4.0; // abs cost function change b/w iterations
      double max_rel_tol = 1e-14;                // rel cost function change b/w iterations
      int num_max_iter   = 50;
      bool treat_nodata_as_zero = false;

      // Debug code
      //DiskImageView<PixelMask<Vector2f> > disk_disp("run3/res-F.tif");

      Vector3 prev_xyz;

      // Estimate the DEM region we expect to use and crop it into an
      // ImageView.  This will make the algorithm much faster than
      // accessing individual DEM pixels from disk. To do that, find
      // the pixel values on a small set of points of the diagonals of
      // the current tile.

      std::vector<Vector2> diagonals;
      int wid = bbox.width() - 1, hgt = bbox.height() - 1, dim = std::max(1, std::max(wid, hgt)/10);
      for (int i = 0; i <= dim; i++)
        diagonals.push_back(bbox.min() + Vector2(double(i)*wid/dim, double(i)*hgt/dim));
      for (int i = 0; i <= dim; i++)
        diagonals.push_back(bbox.min() + Vector2(double(i)*wid/dim, hgt - double(i)*hgt/dim));

      BBox2i dem_box;
      for (unsigned k = 0; k < diagonals.size(); k++){
        Vector2 left_lowres_pix = diagonals[k];
        Vector2 left_fullres_pix = elem_quot(left_lowres_pix, m_downsample_scale);
        bool has_intersection;
        Vector3 left_camera_ctr, left_camera_vec;
        try {
          left_camera_ctr = m_left_camera_model->camera_center(left_fullres_pix);
          left_camera_vec = m_left_camera_model->pixel_to_vector(left_fullres_pix);
        } catch (const camera::PixelToRayErr& /*e*/) {
          continue;
        }
        Vector3 xyz = camera_pixel_to_dem_xyz(left_camera_ctr, left_camera_vec,
                                              m_dem, m_dem_georef,
                                              treat_nodata_as_zero,
                                              has_intersection,
                                              height_error_tol, max_abs_tol,
                                              max_rel_tol, num_max_iter,
                                              prev_xyz
                                              );
        if ( !has_intersection || xyz == Vector3() ) continue;
        prev_xyz = xyz;

        Vector3 llh = m_dem_georef.datum().cartesian_to_geodetic( xyz );
        Vector2 pix = round(m_dem_georef.lonlat_to_pixel(subvector(llh, 0, 2)));
        dem_box.grow(pix);
      }

      // Expand the DEM box just in case as the above calculation is
      // not fool-proof if the DEM has a lot of no-data regions.
      int expand = std::max(100, (int)(0.1*std::max(dem_box.width(), dem_box.height())));
      dem_box.expand(expand);
      dem_box.crop(bounding_box(m_dem));

      // Crop the georef, read the DEM region in memory
      GeoReference georef_crop = crop(m_dem_georef, dem_box);
      ImageView <PixelMask<float> > dem_crop = crop(m_dem, dem_box);

      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        if (row%m_pixel_sample != 0) continue;

        // Must wipe the previous guess since we are now too far from it
        prev_xyz = Vector3();

        for (int col = bbox.min().x(); col < bbox.max().x(); col++){
          if (col%m_pixel_sample != 0) continue;

          Vector2 left_lowres_pix = Vector2(col, row);
          Vector2 left_fullres_pix = elem_quot(left_lowres_pix, m_downsample_scale);
          bool has_intersection;
          Vector3 left_camera_ctr, left_camera_vec;
          try {
            left_camera_ctr = m_left_camera_model->camera_center(left_fullres_pix);
            left_camera_vec = m_left_camera_model->pixel_to_vector(left_fullres_pix);
          } catch (const camera::PixelToRayErr& /*e*/) {
            continue;
          }
          Vector3 xyz = camera_pixel_to_dem_xyz(left_camera_ctr, left_camera_vec,
                                                dem_crop, georef_crop,
                                                treat_nodata_as_zero,
                                                has_intersection,
                                                height_error_tol, max_abs_tol,
                                                max_rel_tol, num_max_iter,
                                                prev_xyz
                                                );
          if ( !has_intersection || xyz == Vector3() ) continue;
          prev_xyz = xyz;

          // Since our DEM is only known approximately, the true
          // intersection point of the ray coming from the left camera
          // with the DEM could be anywhere within m_dem_accuracy from
          // xyz. Use that to get an estimate of the disparity
          // accuracy.

          ImageView< PixelMask<Vector2> > curr_pixel_disp_range(3, 1);
          double bias[] = {-1.0, 1.0, 0.0};
          int success[] = {0, 0, 0};

          for (int k = 0; k < curr_pixel_disp_range.cols(); k++){

            Vector2 right_fullres_pix;
            try {
              right_fullres_pix = m_right_camera_model->point_to_pixel(xyz + bias[k]*m_dem_accuracy*left_camera_vec);
            } catch ( camera::PointToPixelErr const& e ) {
              curr_pixel_disp_range(k, 0).invalidate();
              continue;
            }
            if (m_homography_align){
              right_fullres_pix = HomographyTransform(m_align_matrix).forward(right_fullres_pix);
            }

            // Debug code
            //std::cout.precision(20); std::cout << "\n\ncol row is " << col << ' ' << row << ' ' << cols() << " " << right_fullres_pix << ' ' << rows() << std::endl << std::endl << std::endl;
            //int k0 = int(left_fullres_pix[0]);
            //int k1 = int(left_fullres_pix[1]);
            //Vector2 fdisp = right_fullres_pix - left_fullres_pix;
            //PixelMask<Vector2i> ddisp = disk_disp(k0, k1);
            //std::cout << "pixel curr_pixel_disp_range: " << k0 << ' ' << k1 << ' ' << fdisp << ' ' << ddisp << " " << norm_2(ddisp.child() - fdisp) << std::endl;

            Vector2 right_lowres_pix = elem_prod(right_fullres_pix, m_downsample_scale);
            curr_pixel_disp_range(k, 0) = right_lowres_pix - left_lowres_pix;
            success[k] = 1;

            //std::cout << "value: " << col << ' ' << row << ' ' << k << ' ' << curr_pixel_disp_range(k, 0) << std::endl;

            // If the disparities at the endpoints of the range were successful,
            // don't bother with the middle estimate.
            if (k == 1 && success[0] && success[1]) break;
          }

          BBox2f search_range = stereo::get_disparity_range(curr_pixel_disp_range);
          if (search_range ==  BBox2f(0,0,0,0)) continue;

          lowres_disparity(col, row) = round( (search_range.min() + search_range.max())/2.0 );
          m_disparity_spread(col, row) = ceil( (search_range.max() - search_range.min())/2.0 );

          //std::cout << "range is " << search_range << std::endl;
          //xxxstd::cout << "disp is " << lowres_disparity(col, row) << std::endl;
          //std::cout << "accuracy is " << m_disparity_spread(col, row) << std::endl;
          //std::cout << std::endl;

        }
      }

      return lowres_disparity;
    }

    template <class DestT>
    inline void rasterize(DestT const& dest, BBox2i bbox) const {
      vw::rasterize(prerasterize(bbox), dest, bbox);
    }
  };

  template <class ImageT, class DEMImageT>
  DemDisparity<ImageT, DEMImageT>
  dem_disparity( ImageViewBase<ImageT> const& left,
                 double dem_accuracy, GeoReference dem_georef,
                 DEMImageT const& dem,
                 Vector2f const& downsample_scale,
                 boost::shared_ptr<camera::CameraModel> left_camera_model,
                 boost::shared_ptr<camera::CameraModel> right_camera_model,
                 bool homography_align, Matrix<double> const& align_matrix,
                 int pixel_sample,
                 ImageView<PixelMask<Vector2i> > & disparity_spread
                 ) {
    typedef DemDisparity<ImageT, DEMImageT> return_type;
    return return_type( left.impl(),
                        dem_accuracy, dem_georef,
                        dem, downsample_scale,
                        left_camera_model, right_camera_model,
                        homography_align, align_matrix,
                        pixel_sample, disparity_spread
                        );
  }

  void pre_correlation( Options & opt ) {

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 1 --> LOW-RESOLUTION CORRELATION \n";

    // Working out search range if need be
    if (stereo_settings().is_search_defined()) {
      vw_out() << "\t--> Using user-defined search range.\n";
    } else {

      // Match file between the input files
      std::string match_filename
        = ip::match_filename(opt.out_prefix, opt.in_file1, opt.in_file2);

      std::cout << "Match file: " << match_filename << std::endl;

      if (!fs::exists(match_filename)) {
        // If there is not any match files for the input image. Let's
        // gather some IP quickly from the low resolution images. This
        // routine should only run for:
        //   Pinhole + Epipolar
        //   Pinhole + None
        //   DG + None
        // Everything else should gather IP's all the time.
        float sub_scale =
          sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-L_sub.tif" )),
                         Vector2f(file_image_size( opt.out_prefix+"-L.tif" ) ) )) +
          sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-R_sub.tif" )),
                         Vector2f(file_image_size( opt.out_prefix+"-R.tif" ) ) ));
        sub_scale /= 4.0f;

        stereo_settings().search_range =
          approximate_search_range( opt.out_prefix+"-L_sub.tif",
                                    opt.out_prefix+"-R_sub.tif",
                                    ip::match_filename( opt.out_prefix,
                                                        opt.out_prefix+"-L_sub.tif",
                                                        opt.out_prefix+"-R_sub.tif"),
                                    sub_scale );
      } else {
        // There exists a matchfile out there.
        std::vector<ip::InterestPoint> ip1, ip2;
        ip::read_binary_match_file( match_filename, ip1, ip2 );

        Matrix<double> align_matrix = math::identity_matrix<3>();
        if ( fs::exists(opt.out_prefix+"-align.exr") )
          read_matrix(align_matrix, opt.out_prefix + "-align.exr");

        BBox2 search_range;
        for ( size_t i = 0; i < ip1.size(); i++ ) {
          Vector3 r = align_matrix * Vector3(ip2[i].x,ip2[i].y,1);
          r /= r[2];
          search_range.grow( subvector(r,0,2) - Vector2(ip1[i].x,ip1[i].y) );
        }
        stereo_settings().search_range = grow_bbox_to_int( search_range );
      }
      vw_out() << "\t--> Detected search range: " << stereo_settings().search_range << "\n";
    }

    DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
      Rmask(opt.out_prefix + "-rMask.tif");

    // Performing disparity on sub images
    if ( stereo_settings().seed_mode > 0 ) {
      // Re use prior existing D_sub if it exists
      bool rebuild = false;

      try {
        vw_log().console_log().rule_set().add_rule(-1,"fileio");
        DiskImageView<PixelMask<Vector2i> > test(opt.out_prefix+"-D_sub.tif");
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
    }

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : LOW-RESOLUTION CORRELATION FINISHED \n";
  }

  void produce_lowres_disparity( Options & opt ) {

    DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif"),
      Rmask(opt.out_prefix + "-rMask.tif");

    DiskImageView<PixelGray<float> > left_sub( opt.out_prefix+"-L_sub.tif" ),
      right_sub( opt.out_prefix+"-R_sub.tif" );

    Vector2f downsample_scale( float(left_sub.cols()) / float(Lmask.cols()),
                               float(left_sub.rows()) / float(Lmask.rows()) );

    DiskImageView<uint8> left_mask_sub( opt.out_prefix+"-lMask_sub.tif" ),
      right_mask_sub( opt.out_prefix+"-rMask_sub.tif" );

    BBox2i search_range( floor(elem_prod(downsample_scale,stereo_settings().search_range.min())),
                         ceil(elem_prod(downsample_scale,stereo_settings().search_range.max())) );

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
      // accurate, as well as reasonably fast for subsapled images.
      asp::block_write_gdal_image
        (opt.out_prefix + "-D_sub.tif",
         remove_outliers(stereo::pyramid_correlate
                         (left_sub, right_sub,
                          left_mask_sub, right_mask_sub,
                          stereo::LaplacianOfGaussian(stereo_settings().slogW),
                          search_range,
                          stereo_settings().corr_kernel,
                          stereo::CROSS_CORRELATION, 2
                          ),
                         1, 1, 2.0, 0.5
                         ), opt,
         TerminalProgressCallback("asp", "\t--> Low-resolution disparity:")
         );

    }else if ( stereo_settings().seed_mode == 2 ) {

      // Use a DEM to get the low-res disparity

      if (stereo_settings().is_search_defined())
        vw_out(WarningMessage) << "Computing low-resolution disparity from DEM. "
                               << "Will ignore corr-search value: "
                               << stereo_settings().search_range << ".\n";


      // Skip pixels to speed things up, particularly for ISIS and DG.
      int pixel_sample = 2;

      // To do: We don't need again the images below here
      DiskImageView<PixelGray<float> > left_image(opt.out_prefix+"-L.tif");
      DiskImageView<PixelGray<float> > left_image_sub(opt.out_prefix+"-L_sub.tif");

      std::string dem_file = stereo_settings().disparity_estimation_dem;
      if (dem_file == ""){
        vw_throw( ArgumentErr() << "stereo_corr: No value was provided for: disparity-estimation-dem.\n" );
      }
      double dem_accuracy = stereo_settings().disparity_estimation_dem_accuracy;
      if (dem_accuracy <= 0.0){
        vw_throw( ArgumentErr() << "stereo_corr: Invalid value for disparity-estimation-dem-accuracy: " << dem_accuracy << ".\n" );
      }

      GeoReference dem_georef;
      bool has_georef = cartography::read_georeference(dem_georef, dem_file);
      if (!has_georef)
        vw_throw( ArgumentErr() << "There is no georeference information in: " << dem_file << ".\n" );

      DiskImageView<float> dem_disk_image(dem_file);
      ImageViewRef<PixelMask<float > > dem = pixel_cast<PixelMask<float> >(dem_disk_image);
      boost::scoped_ptr<SrcImageResource> rsrc( DiskImageResource::open(dem_file) );
      if ( rsrc->has_nodata_read() ){
        double nodata_value = rsrc->nodata_read();
        if ( !std::isnan(nodata_value) )
          dem = create_mask(dem_disk_image, nodata_value);
      }

      // To do: This scale is duplicated
      Vector2f downsample_scale( float(left_image_sub.cols()) / float(left_image.cols()),
                                 float(left_image_sub.rows()) / float(left_image.rows()) );

      boost::shared_ptr<camera::CameraModel> left_camera_model, right_camera_model;
      opt.session->camera_models(left_camera_model, right_camera_model);

      Matrix<double> align_matrix = identity_matrix<3>();
      bool homography_align = (stereo_settings().alignment_method == "homography");
      if ( homography_align ) {
        // We used a homography to line up the images, we may want
        // to generate pre-alignment disparities before passing this information
        // onto the camera model in the next stage of the stereo pipeline.
        read_matrix(align_matrix, opt.out_prefix + "-align.exr");
        vw_out(DebugMessage,"asp") << "Alignment Matrix: " << align_matrix << "\n";
      }

      time_t Start_t, End_t;
      int time_task;
      Start_t = time(NULL);

      // Smaller tiles is better
      //---------------------------------------------------------
      Vector2 orig_tile_size = opt.raster_tile_size;
      opt.raster_tile_size = Vector2i(64, 64);

      // This image is small enough that we can keep it in memory
      ImageView<PixelMask<Vector2i> > disparity_spread(left_image_sub.cols(), left_image_sub.rows());

      ImageViewRef<PixelMask<Vector2i> > lowres_disparity
        = dem_disparity(left_image_sub,
                        dem_accuracy, dem_georef,
                        dem, downsample_scale,
                        left_camera_model, right_camera_model,
                        homography_align, align_matrix, pixel_sample,
                        disparity_spread
                        );
      std::string disparity_file = opt.out_prefix + "-D_sub.tif";
      vw_out() << "Writing low-resolution disparity: " << disparity_file << "\n";
      if ( opt.stereo_session_string == "isis" ){
        // ISIS does not support multi-threading
        // To do: Make consistent with what is below
        boost::scoped_ptr<DiskImageResource> drsrc( asp::build_gdal_rsrc( disparity_file,
                                                                        lowres_disparity, opt ) );
        write_image(*drsrc, lowres_disparity,
                    TerminalProgressCallback("asp", "\t--> Low-resolution disparity: "));
      }else{
        std::cout << "--not isis" << std::endl;
        asp::block_write_gdal_image( disparity_file,
                                     lowres_disparity,
                                     opt,
                                     TerminalProgressCallback("asp", "\t--> Low-resolution disparity:") );
      }

      std::string disp_spread_file = opt.out_prefix + "-D_sub_spread.tif";
      vw_out() << "Writing low-resolution disparity spread: " << disp_spread_file << "\n";
      asp::block_write_gdal_image( disp_spread_file,
                                     disparity_spread,
                                     opt,
                                     TerminalProgressCallback("asp", "\t--> Low-resolution disparity spread:") );

      End_t = time(NULL);    //record time that task 1 ends
      time_task = difftime(End_t, Start_t);    //compute elapsed time of task 1
      std::cout << "elapsed time: " << time_task << std::endl;

      // Go back to the original tile size
      opt.raster_tile_size = orig_tile_size;

#if 1 // Debug code
      ImageView<PixelMask<Vector2i> > lowres_disparity_disk;
      read_image( lowres_disparity_disk, opt.out_prefix + "-D_sub.tif" );
      ImageView<PixelMask<Vector2i> > lowres_disparity2(lowres_disparity_disk.cols()/pixel_sample, lowres_disparity_disk.rows()/pixel_sample);
      for (int col = 0; col < lowres_disparity2.cols(); col++){
        for (int row = 0; row < lowres_disparity2.rows(); row++){
          lowres_disparity2(col, row) = lowres_disparity_disk(pixel_sample*col, pixel_sample*row);
        }
      }
      asp::block_write_gdal_image( opt.out_prefix + "-D_sub2.tif",
                                   lowres_disparity2,
                                   opt,
                                   TerminalProgressCallback("asp", "\t--> Low-resolution disparity:") );
#endif


    }

    ImageView<PixelMask<Vector2i> > lowres_disparity;
    read_image( lowres_disparity, opt.out_prefix + "-D_sub.tif" );
    search_range =
      stereo::get_disparity_range( lowres_disparity );
    VW_OUT(DebugMessage,"asp") << "D_sub resolved search range: "
                               << search_range << " px\n";
    search_range.min() = floor(elem_quot(search_range.min(),downsample_scale));
    search_range.max() = ceil(elem_quot(search_range.max(),downsample_scale));
    stereo_settings().search_range = search_range;
  }

  // approximate search range
  //  Find interest points and grow them into a search range
  BBox2i
  approximate_search_range( std::string const& left_image,
                            std::string const& right_image,
                            std::string const& match_filename,
                            float scale ) {

    typedef PixelGray<float32> PixelT;
    vw_out() << "\t--> Using interest points to determine search window.\n";
    std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
    float i_scale = 1.0/scale;

    // String names
    std::string
      left_ip_file  = fs::path( left_image ).replace_extension("vwip").string(),
      right_ip_file = fs::path( right_image ).replace_extension("vwip").string();

    // Building / Loading Interest point data
    if ( fs::exists(match_filename) ) {

      vw_out() << "\t    * Using cached match file.\n";
      ip::read_binary_match_file(match_filename, matched_ip1, matched_ip2);

    } else {

      std::vector<ip::InterestPoint> ip1_copy, ip2_copy;

      if ( !fs::exists(left_ip_file) ||
           !fs::exists(right_ip_file) ) {

        // Worst case, no interest point operations have been performed before
        vw_out() << "\t    * Locating Interest Points\n";
        DiskImageView<PixelT> left_sub_image(left_image);
        DiskImageView<PixelT> right_sub_image(right_image);

        // Interest Point module detector code.
        float ipgain = 0.07;
        std::list<ip::InterestPoint> ip1, ip2;
        vw_out() << "\t    * Processing for Interest Points.\n";
        while ( ip1.size() < 1500 || ip2.size() < 1500 ) {
          ip1.clear(); ip2.clear();

          ip::OBALoGInterestOperator interest_operator( ipgain );
          ip::IntegralInterestPointDetector<ip::OBALoGInterestOperator> detector( interest_operator, 0 );

          ip1 = detect_interest_points( left_sub_image, detector );
          ip2 = detect_interest_points( right_sub_image, detector );

          ipgain *= 0.75;
          if ( ipgain < 1e-2 ) {
            vw_out() << "\t    * Unable to find desirable amount of Interest Points.\n";
            break;
          }
        }

        if ( ip1.size() < 8 || ip2.size() < 8 )
          vw_throw( InputErr() << "Unable to extract interest points from input images ["
                    << left_image << "," << right_image << "]! Unable to continue." );

        // Making sure we don't exceed 3000 points
        if ( ip1.size() > 3000 ) {
          ip1.sort(); ip1.resize(3000);
        }
        if ( ip2.size() > 3000 ) {
          ip2.sort(); ip2.resize(3000);
        }

        // Stripping out orientation. This allows for a better
        // possibility of interest point matches.
        //
        // This is no loss as images at this point are already aligned
        // since the dense correlator is not rotation-invariant.
        BOOST_FOREACH( ip::InterestPoint& ip, ip1 ) ip.orientation = 0;
        BOOST_FOREACH( ip::InterestPoint& ip, ip2 ) ip.orientation = 0;

        vw_out() << "\t    * Generating descriptors..." << std::flush;
        ip::SGradDescriptorGenerator descriptor;
        describe_interest_points( left_sub_image, descriptor, ip1 );
        describe_interest_points( right_sub_image, descriptor, ip2 );
        vw_out() << "done.\n";

        // Writing out the results
        vw_out() << "\t    * Caching interest points: "
                 << left_ip_file << " & " << right_ip_file << std::endl;
        ip::write_binary_ip_file( left_ip_file, ip1 );
        ip::write_binary_ip_file( right_ip_file, ip2 );

      }

      vw_out() << "\t    * Using cached IPs.\n";
      ip1_copy = ip::read_binary_ip_file(left_ip_file);
      ip2_copy = ip::read_binary_ip_file(right_ip_file);

      vw_out() << "\t    * Matching interest points\n";
      ip::InterestPointMatcher<ip::L2NormMetric,ip::NullConstraint> matcher(0.6);

      matcher(ip1_copy, ip2_copy, matched_ip1, matched_ip2,
              TerminalProgressCallback( "asp", "\t    Matching: "));
      vw_out(InfoMessage) << "\t    " << matched_ip1.size() << " putative matches.\n";

      vw_out() << "\t    * Rejecting outliers using RANSAC.\n";
      ip::remove_duplicates(matched_ip1, matched_ip2);
      std::vector<Vector3>
        ransac_ip1 = ip::iplist_to_vectorlist(matched_ip1),
        ransac_ip2 = ip::iplist_to_vectorlist(matched_ip2);
      std::vector<size_t> indices;

      try {
        // Figure out the inlier threshold .. it should be about 3% of
        // the edge lengths. This is a bit of a magic number, but I'm
        // pulling from experience that an inlier threshold of 30
        // worked best for 1024^2 AMC imagery.
        float inlier_threshold =
          0.0075 * ( sum( file_image_size( left_image ) ) +
                     sum( file_image_size( right_image ) ) );

        math::RandomSampleConsensus<math::HomographyFittingFunctor,math::InterestPointErrorMetric>
          ransac( math::HomographyFittingFunctor(), math::InterestPointErrorMetric(), 100, inlier_threshold, ransac_ip1.size()/2, true );
        Matrix<double> trans = ransac( ransac_ip1, ransac_ip2 );
        vw_out(DebugMessage,"asp") << "\t    * Ransac Result: " << trans << std::endl;
        vw_out(DebugMessage,"asp") << "\t      inlier thresh: "
                                   << inlier_threshold << " px" << std::endl;
        indices = ransac.inlier_indices(trans, ransac_ip1, ransac_ip2 );
      } catch ( vw::math::RANSACErr const& e ) {
        vw_out() << "-------------------------------WARNING---------------------------------\n";
        vw_out() << "\t    RANSAC failed! Unable to auto detect search range.\n\n";
        vw_out() << "\t    Please proceed cautiously!\n";
        vw_out() << "-------------------------------WARNING---------------------------------\n";
        return BBox2i(-10,-10,20,20);
      }

      { // Keeping only inliers
        std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
        for ( size_t i = 0; i < indices.size(); i++ ) {
          inlier_ip1.push_back( matched_ip1[indices[i]] );
          inlier_ip2.push_back( matched_ip2[indices[i]] );
        }
        matched_ip1 = inlier_ip1;
        matched_ip2 = inlier_ip2;
      }

      vw_out() << "\t    * Caching matches: " << match_filename << "\n";
      write_binary_match_file( match_filename, matched_ip1, matched_ip2);
    }

    // Find search window based on interest point matches
    namespace ba = boost::accumulators;
    ba::accumulator_set<float, ba::stats<ba::tag::variance> > acc_x, acc_y;
    for (size_t i = 0; i < matched_ip1.size(); i++) {
      acc_x(i_scale * ( matched_ip2[i].x - matched_ip1[i].x ));
      acc_y(i_scale * ( matched_ip2[i].y - matched_ip1[i].y ));
    }
    Vector2f mean( ba::mean(acc_x), ba::mean(acc_y) );
    vw_out(DebugMessage,"asp") << "Mean search is : " << mean << std::endl;
    Vector2f stddev( sqrt(ba::variance(acc_x)), sqrt(ba::variance(acc_y)) );
    BBox2i search_range( mean - 2.5*stddev,
                         mean + 2.5*stddev );
    return search_range;
  }

} // end namespace asp

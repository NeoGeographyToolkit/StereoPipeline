// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file stereo.cc
///

/************************************************************************
 *     File: stereo.cc
 *     Date: April 2005
 *       By: Michael Broxton and Larry Edwards
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group 
 * Function: Main program for the stereo pipeline 
 ************************************************************************/
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include <vw/Math/EulerAngles.h>
using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::cartography;

#include "StereoSettings.h"
#include "stereo.h"
#include "StereoSession.h"
#include "SurfaceNURBS.h"
#include "BundleAdjustUtils.h"
#include "MRO/DiskImageResourceDDD.h"	   // support for Malin DDD image files

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "apollo/StereoSessionApolloMetric.h"
#include "MRO/StereoSessionCTX.h"
#include "RMAX/StereoSessionRmax.h"

using namespace std;

// The stereo pipeline has several stages, which are enumerated below.
enum { PREPROCESSING = 0, 
       CORRELATION, 
       FILTERING, 
       POINT_CLOUD, 
       WIRE_MESH, 
       NUM_STAGES};

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

//***********************************************************************
// MAIN
//***********************************************************************

int main(int argc, char* argv[]) {

  // The default file type are automatically registered the first time
  // a file is opened or created, however we want to override some of
  // the defaults, so we explicitly register them here before registering
  // our own FileIO driver code.
  DiskImageResource::register_default_file_types();

  // Register the DDD file handler with the Vision Workbench
  // DiskImageResource system.  DDD is the proprietary format used by
  // Malin Space Science Systems.
  DiskImageResource::register_file_type(".ddd",
                                        DiskImageResourceDDD::type_static(),
                                        &DiskImageResourceDDD::construct_open,
                                        &DiskImageResourceDDD::construct_create);
  
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  // Register the Isis file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".cub",
                                        DiskImageResourceIsis::type_static(),
                                        &DiskImageResourceIsis::construct_open,
                                        &DiskImageResourceIsis::construct_create);
#endif 
  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int entry_point;
  int debug_level;
  unsigned cache_size;
  std::string stereo_session_string;
  std::string stereo_default_filename;
  std::string in_file1, in_file2, cam_file1, cam_file2, extra_arg1, extra_arg2, extra_arg3, extra_arg4;
  std::string out_prefix;
  std::string corr_debug_prefix;
  std::vector<vw::int32> crop_bounds(4);

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1800), "Cache size, in megabytes")
    ("session-type,t", po::value<std::string>(&stereo_session_string)->default_value("pinhole"), "Select the stereo session type to use for processing. [default: pinhole]")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)")
    ("crop-min-x", po::value<int32>(&(crop_bounds[0])), "Crop the aligned input images to these bounds ( <min_x> <min_y> <width> <height> ) prior to running through the correlator.  Useful for tuning settings before processing the whole image.")
    ("crop-min-y", po::value<int32>(&(crop_bounds[1])), "")
    ("crop-width", po::value<int32>(&(crop_bounds[2])), "")
    ("crop-height", po::value<int32>(&(crop_bounds[3])), "")
    ("corr-debug-prefix", po::value<std::string>(&corr_debug_prefix)->default_value(""), "Cause the pyramid correlator to save out debug imagery named with this prefix.")
    ("optimized-correlator", "Use the optimized correlator instead of the pyramid correlator.");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("left-camera-model", po::value<std::string>(&cam_file1), "Left Camera Model File")
    ("right-camera-model", po::value<std::string>(&cam_file2), "Right Camera Model File")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames")
    ("extra_argument1", po::value<std::string>(&extra_arg1), "Extra Argument 1")
    ("extra_argument2", po::value<std::string>(&extra_arg2), "Extra Argument 2")
    ("extra_argument3", po::value<std::string>(&extra_arg3), "Extra Argument 3")
    ("extra_argument4", po::value<std::string>(&extra_arg4), "Extra Argument 4");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("left-camera-model", 1);
  positional_options_desc.add("right-camera-model", 1);
  positional_options_desc.add("output-prefix", 1);
  positional_options_desc.add("extra_argument1", 1);
  positional_options_desc.add("extra_argument2", 1);
  positional_options_desc.add("extra_argument3", 1);
  positional_options_desc.add("extra_argument4", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") ||
      !vm.count("left-input-image") || !vm.count("right-input-image") || 
      !vm.count("left-camera-model") || !vm.count("right-camera-model") || 
      !vm.count("output-prefix")) {
    std::cout << "\nUsage: stereo [options] <Left_input_image> <Right_input_image> <Left_camera_file> <Right_camera_file> <output_file_prefix>\n"
              << "  the extensions are automaticaly added to the output files\n"
              << "  the parameters should be in stereo.default\n\n";
    std::cout << visible_options << std::endl;
    return 1;
  }

  // Read the stereo.conf file
  stereo_settings().read(stereo_default_filename);

  // Set search range from stereo.default file
  BBox2i search_range(Vector<int,2>(stereo_settings().h_corr_min, stereo_settings().v_corr_min),
                      Vector<int,2>(stereo_settings().h_corr_max, stereo_settings().v_corr_max));

  // Set the Vision Workbench debug level
  set_debug_level(debug_level);
  Cache::system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb

  // Create a fresh stereo session and query it for the camera models.
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "metric", &StereoSessionApolloMetric::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(in_file1, in_file2, cam_file1, cam_file2, 
                      out_prefix, extra_arg1, extra_arg2, extra_arg3, extra_arg4);

  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {

    std::string pre_preprocess_file1, pre_preprocess_file2;
    session->pre_preprocessing_hook(in_file1, in_file2, pre_preprocess_file1, pre_preprocess_file2);
    DiskImageView<PixelGray<uint8> > left_rectified_image(pre_preprocess_file1);
    DiskImageView<PixelGray<uint8> > right_rectified_image(pre_preprocess_file2);
    
    cout << "\nGenerating image masks..." << std::flush;
    int mask_buffer = std::max(stereo_settings().h_kern, stereo_settings().v_kern);
    ImageViewRef<uint8> Lmask = channel_cast_rescale<uint8>(disparity::generate_mask(left_rectified_image, mask_buffer));
    ImageViewRef<uint8> Rmask = channel_cast_rescale<uint8>(disparity::generate_mask(right_rectified_image, mask_buffer));
    cout << "Done.\n";
    write_image(out_prefix + "-lMask.tif", Lmask);
    write_image(out_prefix + "-rMask.tif", Rmask);
  }

  /*********************************************************************************/
  /*                            correlation step                                   */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    if (entry_point == CORRELATION) 
        cout << "\nStarting at the CORRELATION stage.\n";

    DiskImageView<PixelGray<float> > left_disk_image(out_prefix+"-L.tif");
    DiskImageView<PixelGray<float> > right_disk_image(out_prefix+"-R.tif");
            
    // If the user has specified a crop at the command line, we go
    // with the cropped region instead.
    BBox2i crop_bbox(crop_bounds[0],crop_bounds[1],crop_bounds[2],crop_bounds[3]);
    if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
      std::cout << "Cropping to bounding box: " << crop_bbox << "\n";

      // Do a quick check to make sure the user specified a reasonable crop region.
      if ((crop_bounds[0] < 0) || (crop_bounds[1] < 0) || 
          (crop_bounds[0] + crop_bounds[2] > left_disk_image.cols()) || 
          (crop_bounds[1] + crop_bounds[3] > left_disk_image.rows()) ) {
        std::cout << "Error: the specified crop region exceeds the dimensions of the original image.  \n Exiting.\n\n"; 
        exit(0); 
      }

      // Save a cropped version of the left image for reference.
      write_image(out_prefix+"-L-crop.tif", crop(left_disk_image, crop_bbox), TerminalProgressCallback());
      write_image(out_prefix+"-R-crop.tif", crop(right_disk_image, crop_bbox), TerminalProgressCallback());
    }

    // Set up the CorrelatorView Object
    CorrelatorView<PixelGray<float>, stereo::SlogStereoPreprocessingFilter> corr_view(left_disk_image, right_disk_image, 
                                                                                      stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW));
    corr_view.set_search_range(search_range);
    corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern, stereo_settings().v_kern));
    corr_view.set_cross_corr_threshold(stereo_settings().xcorr_treshold);
    corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_treshold);
    corr_view.set_subpixel_options(stereo_settings().do_h_subpixel, stereo_settings().do_v_subpixel);
    if (vm.count("corr-debug-prefix"))
      corr_view.set_debug_mode(corr_debug_prefix);

    std::cout << corr_view;
    std::cout<< "Building Disparity map...\n";
    ImageViewRef<PixelDisparity<float> > disparity_map;
    if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
      disparity_map = crop(corr_view, crop_bbox);
    } else {
      disparity_map = corr_view;
    }

    if (vm.count("optimized-correlator")) {
      vw::stereo::OptimizedCorrelator correlator( search_range.min().x(), search_range.max().x(), 
                                                  search_range.min().y(), search_range.max().y(),
                                                  stereo_settings().h_kern, stereo_settings().v_kern, 
                                                  true,                             // verbose
                                                  stereo_settings().xcorr_treshold,
                                                  stereo_settings().corrscore_rejection_treshold, // correlation score rejection threshold (1.0 disables, good values are 1.5 - 2.0)
                                                  stereo_settings().do_h_subpixel, stereo_settings().do_v_subpixel);   // h and v subpixel
      if (stereo_settings().slog) {
        std::cout << "Applying SLOG filter.\n";
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW));
      } else if (stereo_settings().log) {
        std::cout << "Applying LOG filter.\n";
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::LogStereoPreprocessingFilter(stereo_settings().slogW));
      } else {
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::NullStereoPreprocessingFilter());
      }
    }

    // Do some basic outlier rejection
    ImageViewRef<PixelDisparity<float> > proc_disparity_map = disparity::clean_up(disparity_map,
                                                                                  stereo_settings().rm_h_half_kern, stereo_settings().rm_v_half_kern,
                                                                                  stereo_settings().rm_treshold, stereo_settings().rm_min_matches/100.0);
    
    // Create a disk image resource and prepare to write a tiled
    // OpenEXR.
    DiskImageResourceOpenEXR disparity_map_rsrc(out_prefix + "-D.exr", disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(2048,disparity_map.cols()),std::min(2048, disparity_map.rows()));
    block_write_image( disparity_map_rsrc, disparity_map, TerminalProgressCallback() );
  }

  /***************************************************************************/
  /*                      Disparity Map Filtering                            */
  /***************************************************************************/
  if(entry_point <= FILTERING) {
    if (entry_point == FILTERING)
        cout << "\nStarting at the FILTERING stage.\n";

    try {
      std::cout << "\nUsing image " << out_prefix + "-D.exr" << " as disparity map image.\n";
      DiskImageView<PixelDisparity<float> > disparity_disk_image(out_prefix + "-D.exr");
      ImageViewRef<PixelDisparity<float> > disparity_map = disparity_disk_image;


      // Apply the Mask to the disparity map 
      std::cout << "\tApplying mask.\n";
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.tif");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.tif");
      disparity_map = disparity::mask(disparity_disk_image, Lmask, Rmask);

      std::cout << "Cleaning up disparity map prior to filtering processes (" << stereo_settings().rm_cleanup_passes << " passes).\n";
      for (int i = 0; i < stereo_settings().rm_cleanup_passes; ++i) {
        disparity_map = disparity::clean_up(disparity_disk_image,
                                            stereo_settings().rm_h_half_kern, stereo_settings().rm_v_half_kern,
                                            stereo_settings().rm_treshold, stereo_settings().rm_min_matches/100.0);
      }

      // Rasterize the results so far to a temporary file on disk.
      // This file is deleted once we complete the second half of the
      // disparity map filtering process.
      std::cout << "\trasterizing filtered disparity map to disk. \n" << std::flush;
      DiskCacheImageView<PixelDisparity<float> > filtered_disparity_map(disparity_map, "exr");

      // Write out the extrapolation mask imaege
      write_image(out_prefix + "-GoodPixelMap.tif", disparity::missing_pixel_image(filtered_disparity_map), TerminalProgressCallback());

      // Call out to NURBS hole filling code.  The hole filling is
      // done with a subsampled (by 4) images and then the hole filled
      // values are upsampled to the full resolution of the image
      // using bicubic interpolation.

      ImageViewRef<PixelDisparity<float> > hole_filled_disp_map = filtered_disparity_map;

      if(stereo_settings().fill_holes_NURBS) {
        DiskImageView<uint8> Lmask(out_prefix + "-lMask.tif");
        DiskImageView<uint8> Rmask(out_prefix + "-rMask.tif");
        std::cout << "Filling holes with bicubicly interpolated B-SPLINE surface... \n";
        hole_filled_disp_map = disparity::mask(HoleFillView(filtered_disparity_map, 4),Lmask, Rmask);
      } 

      DiskImageResourceOpenEXR disparity_map_rsrc(out_prefix + "-F.exr", hole_filled_disp_map.format() );
      disparity_map_rsrc.set_tiled_write(std::min(2048,hole_filled_disp_map.cols()),std::min(2048, hole_filled_disp_map.rows()));
      write_image(disparity_map_rsrc, hole_filled_disp_map, TerminalProgressCallback() ); 
    } catch (IOErr &e) { 
      cout << "\n An file IO error occurred during the filtering stage.  " << e.what() << "Exiting.\n\n";
      exit(0);
    }
  }


  /******************************************************************************/
  /*                           disparity to dot-cloud                           */
  /******************************************************************************/
  if (entry_point <= POINT_CLOUD) {
    if (entry_point == POINT_CLOUD) 
      std::cout << "\nStarting code at POINT_CLOUD stage.\n";

    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      session->camera_models(camera_model1, camera_model2);

      // If the user has generated a set of position and pose
      // corrections using the bundle_adjsut program, we read them in
      // here and incorporate them into our camera model.
      Vector3 position_correction;
      Quaternion<double> pose_correction;
      if (fs::exists(prefix_from_filename(in_file1)+".adjust")) {
        std::cout << "Adjusting left camera model using parameters in " << (prefix_from_filename(in_file1)+".adjust") << "\n";
        read_adjustments(prefix_from_filename(in_file1)+".adjust", position_correction, pose_correction);
        camera_model1 = boost::shared_ptr<CameraModel>(new AdjustedCameraModel(camera_model1, 
                                                                               position_correction,
                                                                               pose_correction));
      }
      if (fs::exists(prefix_from_filename(in_file2)+".adjust")) {
        std::cout << "Adjusting right camera model using parameters in " << (prefix_from_filename(in_file2)+".adjust") << "\n";
        read_adjustments(prefix_from_filename(in_file2)+".adjust", position_correction, pose_correction);
        camera_model2 = boost::shared_ptr<CameraModel>(new AdjustedCameraModel(camera_model2, 
                                                                               position_correction,
                                                                               pose_correction));
      }        

      std::string prehook_filename;
      session->pre_pointcloud_hook(out_prefix+"-F.exr", prehook_filename);
     
      DiskImageView<PixelDisparity<float> > disparity_map(prehook_filename);

      // Apply the stereo model.  This yields a image of 3D points in
      // space.  We build this image and immediately write out the
      // results to disk.
      std::cout << "Generating a 3D point cloud.   \n";
      StereoView<ImageView<PixelDisparity<float> > > stereo_image(disparity_map, *camera_model1, *camera_model2);

//       std::cout << "Computing Error image\n";
//       ImageView<float> error_img (stereo_image.cols(), stereo_image.rows() );
//       for (unsigned j=0; j < error_img.rows(); ++j) {
//         for (unsigned i=0; i < error_img.cols(); ++i) {
//           error_img(i,j) = stereo_image.error(i,j);
//         }
//       }
//       std::cout << "Done.\n";
//       write_image("error-image.tif", error_img);

      // If the distance from the left camera center to a point is
      // greater than the universe radius, we remove that pixel and
      // replace it with a zero vector, which is the missing pixel value
      // in the point_image.
      //
      // We apply the universe radius here and then write the result
      // directly to a file on disk.      
      UniverseRadiusFunc universe_radius_func(camera_model1->camera_center(Vector2(0,0)), stereo_settings().near_universe_radius, stereo_settings().far_universe_radius);
      ImageViewRef<Vector3> point_cloud = per_pixel_filter(stereo_image, universe_radius_func);

      DiskImageResourceOpenEXR point_cloud_rsrc(out_prefix + "-PC.exr", point_cloud.format() );
      point_cloud_rsrc.set_tiled_write(std::min(2048,point_cloud.cols()),std::min(2048, point_cloud.rows()));
      write_image(point_cloud_rsrc, point_cloud, TerminalProgressCallback());
      std::cout << universe_radius_func;

    } catch (IOErr&) { 
      cout << "\n Unable to start at point cloud stage.\n\tCould not read input files. Exiting.\n\n";
      exit(0);
    }
    
  }

  return(EXIT_SUCCESS);
}

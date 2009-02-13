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
#include "MedianFilter.h"

// Support for Malin DDD image files
#include "MRO/DiskImageResourceDDD.h"	   

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
#include "HRSC/StereoSessionHRSC.h"
#include "MOC/StereoSessionMOC.h"
#include "MRO/StereoSessionCTX.h"
#endif
#include "RMAX/StereoSessionRmax.h"

// Boost headers
#include <boost/thread/xtime.hpp>
// Posix time is not fully supported in the version of Boost for RHEL
// Workstation 4
#ifdef __APPLE__
#include <boost/date_time/posix_time/posix_time.hpp>
#else
#include <ctime>
#endif


using namespace std;

// The stereo pipeline has several stages, which are enumerated below.
enum { PREPROCESSING = 0, 
       CORRELATION, 
       REFINEMENT,
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

// Posix time is not fully supported in the version of Boost for RHEL
// Workstation 4
#ifndef __APPLE__
inline std::string
current_posix_time_string() 
{
  char time_string[2048];
  time_t t = time(0);
  struct tm* time_struct = localtime(&t);
  strftime(time_string, 2048, "%F %T", time_struct);
  return std::string(time_string);
}
#else
inline std::string
current_posix_time_string()
{
  std::ostringstream time_string_stream;
  time_string_stream << boost::posix_time::second_clock::local_time();
  return time_string_stream.str();
}
#endif

void print_usage(po::options_description const& visible_options) {
  std::cout << "\nUsage: stereo [options] <Left_input_image> <Right_input_image> [Left_camera_file] [Right_camera_file] <output_file_prefix>\n"
            << "  Extensions are automaticaly added to the output files.\n"
            << "  Camera model arguments may be optional for some stereo session types (e.g. isis).\n"
            << "  Stereo parameters should be set in the stereo.default file.\n\n";
  std::cout << visible_options << std::endl;
}

//***********************************************************************
// MAIN
//***********************************************************************
int main(int argc, char* argv[]) {

  TerminateHandler th[] = {BacktraceTerminate, PrettyTerminate, DefaultTerminate};
  
  for (int i = 0; i < 3; ++i)
    if (set_terminate_handler(th[i]))
      break;

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
  unsigned num_threads;
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
    ("threads", po::value<unsigned>(&num_threads)->default_value(0), "Select the number of processors (threads) to use.")
    ("session-type,t", po::value<std::string>(&stereo_session_string), "Select the stereo session type to use for processing. [options: pinhole isis]")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)")
    ("debug-level,d", po::value<int>(&debug_level)->default_value(vw::DebugMessage-1), "Set the debugging output level. (0-50+)")
    ("crop-min-x", po::value<int32>(&(crop_bounds[0])), "Crop the aligned input images to these bounds ( <min_x> <min_y> <width> <height> ) prior to running through the correlator.  Useful for tuning settings before processing the whole image.")
    ("crop-min-y", po::value<int32>(&(crop_bounds[1])), "")
    ("crop-width", po::value<int32>(&(crop_bounds[2])), "")
    ("crop-height", po::value<int32>(&(crop_bounds[3])), "")
    ("draft-mode", po::value<std::string>(&corr_debug_prefix)->default_value(""), "Cause the pyramid correlator to save out debug imagery named with this prefix.")
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
  if( vm.count("help") ) {
    print_usage(visible_options);
    exit(0);
  }

  // Checking to see if the user has made a mistake in running the program
  if (!vm.count("left-input-image") || !vm.count("right-input-image") || 
      !vm.count("left-camera-model")) {
    std::cout << "\nMissing all of the correct input files.\n";
    print_usage(visible_options);
    exit(0);
  }
  
  // Read the stereo.conf file
  stereo_settings().read(stereo_default_filename);

  // Set search range from stereo.default file
  BBox2i search_range(Vector<int,2>(stereo_settings().h_corr_min, stereo_settings().v_corr_min),
                      Vector<int,2>(stereo_settings().h_corr_max, stereo_settings().v_corr_max));

  // Set the Vision Workbench debug level
  set_debug_level(debug_level);
  Cache::system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb
  if ( num_threads != 0 ) {
    std::cout << "\t--> Setting number of processing threads to: " << num_threads << "\n";
    vw_settings().set_default_num_threads(num_threads);
  } 

  // Create a fresh stereo session and query it for the camera models.
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
  StereoSession::register_session_type( "hrsc", &StereoSessionHRSC::construct);
  StereoSession::register_session_type( "moc", &StereoSessionMOC::construct);
  StereoSession::register_session_type( "ctx", &StereoSessionCTX::construct);
#endif
  StereoSession::register_session_type( "rmax", &StereoSessionRmax::construct);
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
  StereoSession::register_session_type( "isis", &StereoSessionIsis::construct);
#endif

  int MEDIAN_FILTER = 0;

  // If the user hasn't specified a stereo session type, we take a
  // guess here based on the file suffixes.
  if (stereo_session_string.size() == 0) {
    if ( (boost::iends_with(cam_file1, ".cahvor") && boost::iends_with(cam_file2, ".cahvor")) || 
         (boost::iends_with(cam_file1, ".cahv") && boost::iends_with(cam_file2, ".cahv")) ||
         (boost::iends_with(cam_file1, ".pin") && boost::iends_with(cam_file2, ".pin")) ||
         (boost::iends_with(cam_file1, ".tsai") && boost::iends_with(cam_file2, ".tsai")) ) {
         vw_out(0) << "\t--> Detected pinhole camera files.  Executing pinhole stereo pipeline.\n";
         stereo_session_string = "pinhole";
    } 

    else if (boost::iends_with(in_file1, ".cub") && boost::iends_with(in_file2, ".cub")) {
      vw_out(0) << "\t--> Detected ISIS cube files.  Executing ISIS stereo pipeline.\n";
      stereo_session_string = "isis";
    } 
   
    else {
      vw_out(0) << "\n\n******************************************************************\n";
      vw_out(0) << "Could not determine stereo session type.   Please set it explicitly\n";
      vw_out(0) << "using the -t switch.  Options include: [pinhole isis].\n";
      vw_out(0) << "******************************************************************\n\n";
      exit(0);
    }
  }

  // Some specialization here so that the user doesn't need to list
  // camera models on the command line for certain stereo session
  // types.  (e.g. isis).
  bool check_for_camera_models = true;
  if ( stereo_session_string == "isis" ) {
    // Fix the ordering of the arguments if the user only supplies 3
    if (out_prefix.size() == 0) 
      out_prefix = cam_file1; 
    check_for_camera_models = false;
  }
  
  if ( check_for_camera_models &&
       (!vm.count("output-prefix") || !vm.count("right-camera-model")) ) {
    std::cout << "\nMissing output-prefix or right camera model.\n";
    print_usage(visible_options);
    exit(0);
  }

  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(in_file1, in_file2, cam_file1, cam_file2, 
                      out_prefix, extra_arg1, extra_arg2, extra_arg3, extra_arg4);

  // The last thing we do before we get started is to copy the
  // stereo.default settings over into the results directory so that
  // we have a record of the most recent stereo.default that was used
  // with this data set.
  stereo_settings().copy_settings(stereo_default_filename, out_prefix + "-stereo.default");

  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {
    vw_out(0) << "\n[ " << current_posix_time_string() << " ] : Stage 0 --> PREPROCESSING \n";

    std::string pre_preprocess_file1, pre_preprocess_file2;
    session->pre_preprocessing_hook(in_file1, in_file2, pre_preprocess_file1, pre_preprocess_file2);

    DiskImageView<PixelGray<uint8> > left_rectified_image(pre_preprocess_file1);
    DiskImageView<PixelGray<uint8> > right_rectified_image(pre_preprocess_file2);
    ImageViewRef<PixelGray<uint8> > left_image = left_rectified_image;
    ImageViewRef<PixelGray<uint8> > right_image = right_rectified_image;
   
    if (MEDIAN_FILTER == 1){
      vw_out(0) << "\t--> Median filtering.\n" << std::flush; 
      left_image  = fast_median_filter(left_rectified_image, 7);
      right_image = fast_median_filter(right_rectified_image, 7);
      write_image(out_prefix+"-median-L.tif", left_image);
      write_image(out_prefix+"-median-R.tif", right_image);
    } 

    vw_out(0) << "\t--> Generating image masks... " << std::flush;
    int mask_buffer = std::max(stereo_settings().h_kern, stereo_settings().v_kern);
    ImageViewRef<uint8> Lmask = channel_cast_rescale<uint8>(disparity::generate_mask(left_image, mask_buffer));
    ImageViewRef<uint8> Rmask = channel_cast_rescale<uint8>(disparity::generate_mask(right_image, mask_buffer));
    write_image(out_prefix + "-lMask.tif", Lmask);
    write_image(out_prefix + "-rMask.tif", Rmask);
    vw_out(0) << "done.\n";
  }

  /*********************************************************************************/
  /*                       Discrete (integer) Correlation                          */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    if (entry_point == CORRELATION) 
        cout << "\nStarting at the CORRELATION stage.\n";
    vw_out(0) << "\n[ " << current_posix_time_string() << " ] : Stage 1 --> CORRELATION \n";
    
    DiskImageView<uint8> Lmask(out_prefix + "-lMask.tif");
    DiskImageView<uint8> Rmask(out_prefix + "-rMask.tif");

    // Median filter
    string filename_L, filename_R;
    if (MEDIAN_FILTER==1){
      filename_L = out_prefix+"-median-L.tif";
      filename_R = out_prefix+"-median-R.tif";
    } else {
      filename_L = out_prefix+"-L.tif";
      filename_R = out_prefix+"-R.tif";
    }
    DiskImageView<PixelGray<float> > left_disk_image(filename_L);
    DiskImageView<PixelGray<float> > right_disk_image(filename_R);
         
    // If the user has specified a crop at the command line, we go
    // with the cropped region instead.
    BBox2i crop_bbox(crop_bounds[0],crop_bounds[1],crop_bounds[2],crop_bounds[3]);
    if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
      vw_out(0) << "\t--> Cropping to bounding box: " << crop_bbox << "\n";

      // Do a quick check to make sure the user specified a reasonable crop region.
      if ((crop_bounds[0] < 0) || (crop_bounds[1] < 0) || 
          (crop_bounds[0] + crop_bounds[2] > left_disk_image.cols()) || 
          (crop_bounds[1] + crop_bounds[3] > left_disk_image.rows()) ) {
        vw_out(0) << "\nError: the specified crop region exceeds the dimensions of the original image.  \n Exiting.\n\n"; 
        exit(0); 
      }

      // Save a cropped version of the left image for reference.
      write_image(out_prefix+"-L-crop.tif", crop(left_disk_image, crop_bbox), TerminalProgressCallback(InfoMessage, "Left Image: "));
      write_image(out_prefix+"-R-crop.tif", crop(right_disk_image, crop_bbox), TerminalProgressCallback(InfoMessage, "Right Image: "));
    }
    
    ImageViewRef<PixelDisparity<float> > disparity_map;
    stereo::CorrelatorType cost_mode = ABS_DIFF_CORRELATOR;
    if (stereo_settings().cost_mode == 1) 
      cost_mode = SQR_DIFF_CORRELATOR;
    else if (stereo_settings().cost_mode == 2) 
      cost_mode = NORM_XCORR_CORRELATOR;

    if (vm.count("optimized-correlator")) {
      vw::stereo::OptimizedCorrelator correlator( search_range, 
                                                  stereo_settings().h_kern,
                                                  stereo_settings().xcorr_treshold,
                                                  stereo_settings().corrscore_rejection_treshold, // correlation score rejection threshold (1.0 disables, good values are 1.5 - 2.0)
                                                  stereo_settings().cost_blur, 
                                                  cost_mode);
      if (stereo_settings().slog) {
        std::cout << "\t--> Applying SLOG filter with width: " << stereo_settings().slogW << "\n";
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW));
      } else if (stereo_settings().log) {
        std::cout << "\t--> Applying LOG filter with width: " << stereo_settings().slogW << "\n";
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::LogStereoPreprocessingFilter(stereo_settings().slogW));
      } else {
        std::cout << "\t--> Applying BLUR filter with width: " << stereo_settings().slogW << "\n";
        disparity_map = correlator( left_disk_image, right_disk_image, stereo::NullStereoPreprocessingFilter());
      }
      
    } else {
      if (stereo_settings().slog) {
        std::cout << "\t--> Using SLOG pre-processing filter with " << stereo_settings().slogW << " sigma blur.\n";

        // Set up the CorrelatorView Object
        CorrelatorView<PixelGray<float>, uint8, stereo::SlogStereoPreprocessingFilter> corr_view(left_disk_image, right_disk_image, 
                                                                                                 Lmask, Rmask,
                                                                                                 stereo::SlogStereoPreprocessingFilter(stereo_settings().slogW));
        
        corr_view.set_search_range(search_range);
        corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern, stereo_settings().v_kern));
        corr_view.set_cross_corr_threshold(stereo_settings().xcorr_treshold);
        corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_treshold);
        corr_view.set_correlator_options(stereo_settings().cost_blur, cost_mode);
        if (vm.count("draft-mode"))
          corr_view.set_debug_mode(corr_debug_prefix);
        vw_out(0) << "\t--> Setting stereo options:\n";
        vw_out(0) << "\t    Cost Mode: " << cost_mode << "\n";
        vw_out(0) << "\t    Cost Blur: " << stereo_settings().cost_blur << "\n";
        vw_out(0) << "\t    Kernel Size: " << stereo_settings().h_kern << "x" << stereo_settings().v_kern << "\n";
        vw_out(0) << "\t    Crosscorr Threshold: " << stereo_settings().xcorr_treshold << "\n";
        vw_out(0) << "\t    Corrscore Rejection: " << stereo_settings().corrscore_rejection_treshold << "\n";
        
        vw_out(0) << "\t--> Building Disparity map.\n";
        if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
          disparity_map = crop(corr_view, crop_bbox);
        } else {
          disparity_map = corr_view;
        }

      } else if (stereo_settings().log) {
        std::cout << "\t--> Using LOG pre-processing filter with " << stereo_settings().slogW << " sigma blur.\n";

        // Set up the CorrelatorView Object
        CorrelatorView<PixelGray<float>, uint8, stereo::LogStereoPreprocessingFilter> corr_view(left_disk_image, right_disk_image, 
                                                                                                Lmask, Rmask,
                                                                                                stereo::LogStereoPreprocessingFilter(stereo_settings().slogW));
        
        corr_view.set_search_range(search_range);
        corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern, stereo_settings().v_kern));
        corr_view.set_cross_corr_threshold(stereo_settings().xcorr_treshold);
        corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_treshold);
        
        corr_view.set_correlator_options(stereo_settings().cost_blur, cost_mode);
        if (vm.count("draft-mode"))
          corr_view.set_debug_mode(corr_debug_prefix);
        std::cout << corr_view;
        
        std::cout<< "Building Disparity map...\n";
        if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
          disparity_map = crop(corr_view, crop_bbox);
        } else {
          disparity_map = corr_view;
        }

      } else { // Neither slog or log
        std::cout << "\t--> Using BLUR pre-processing filter.\n";

        CorrelatorView<PixelGray<float>, uint8, stereo::BlurStereoPreprocessingFilter> corr_view(left_disk_image, right_disk_image, 
                                                                                                 Lmask, Rmask,
                                                                                                 stereo::BlurStereoPreprocessingFilter(stereo_settings().slogW));
        
        corr_view.set_search_range(search_range);
        corr_view.set_kernel_size(Vector2i(stereo_settings().h_kern, stereo_settings().v_kern));
        corr_view.set_cross_corr_threshold(stereo_settings().xcorr_treshold);
        corr_view.set_corr_score_threshold(stereo_settings().corrscore_rejection_treshold);
        
        corr_view.set_correlator_options(stereo_settings().cost_blur, cost_mode);
        if (vm.count("draft-mode"))
          corr_view.set_debug_mode(corr_debug_prefix);
        std::cout << corr_view;
        
        std::cout<< "Building Disparity map...\n";
        if ( vm.count("crop-min-x") && vm.count("crop-min-y") && vm.count("crop-width") && vm.count("crop-height") ) {
          disparity_map = crop(corr_view, crop_bbox);
        } else {
          disparity_map = corr_view;
        }

      }
    }

    // Apply the Mask to the disparity map 
    disparity_map = disparity::mask(disparity_map, Lmask, Rmask);

    // Do some basic outlier rejection
    ImageViewRef<PixelDisparity<float> > proc_disparity_map = disparity::clean_up(disparity_map,
                                                                                  stereo_settings().rm_h_half_kern, stereo_settings().rm_v_half_kern,
                                                                                  stereo_settings().rm_treshold, stereo_settings().rm_min_matches/100.0);
    
    // Create a disk image resource and prepare to write a tiled
    // OpenEXR.
    DiskImageResourceOpenEXR disparity_map_rsrc(out_prefix + "-D.exr", disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(2048,disparity_map.cols()),std::min(2048, disparity_map.rows()));
    block_write_image( disparity_map_rsrc, disparity_map );
  }

  /*********************************************************************************/
  /*                            Subpixel Refinement                                */
  /*********************************************************************************/
  if( entry_point <= REFINEMENT ) {
    if (entry_point == REFINEMENT) 
      cout << "\nStarting at the REFINEMENT stage.\n";
    vw_out(0) << "\n[ " << current_posix_time_string() << " ] : Stage 2 --> REFINEMENT \n";
    
    try {
      string filename_L, filename_R;
      if (MEDIAN_FILTER == 1){
        filename_L = out_prefix+"-median-L.tif";
        filename_R = out_prefix+"-median-R.tif";
      } else {
        filename_L = out_prefix+"-L.tif";
        filename_R = out_prefix+"-R.tif";
      }
      DiskImageView<PixelGray<float> > left_disk_image(filename_L);
      DiskImageView<PixelGray<float> > right_disk_image(filename_R);
      DiskImageView<PixelDisparity<float> > disparity_disk_image(out_prefix + "-D.exr");
      ImageViewRef<PixelDisparity<float> > disparity_map = disparity_disk_image;
          
      // Affine and Bayes subpixel refinement always use the
      // LogPreprocessingFilter...
      if (stereo_settings().subpixel_mode > 0) {
        vw_out(0) << "\t--> Using affine adaptive subpixel mode " << stereo_settings().subpixel_mode << "\n";
        disparity_map = SubpixelView<LogStereoPreprocessingFilter>(disparity_disk_image, 
                                                                   channels_to_planes(left_disk_image), 
                                                                   channels_to_planes(right_disk_image),
                                                                   stereo_settings().subpixel_h_kern, stereo_settings().subpixel_v_kern, 
                                                                   stereo_settings().do_h_subpixel, 
                                                                   stereo_settings().do_v_subpixel,   // h and v subpixel
                                                                   stereo_settings().subpixel_mode,
                                                                   LogStereoPreprocessingFilter(stereo_settings().slogW),
                                                                   false);

      // Parabola subpixel refinement uses the same preprocessing
      // filter as the discrete correlation stage above.
      } else {
        vw_out(0) << "\t--> Using parabola subpixel mode.\n";
        if (stereo_settings().slog) {
          vw_out(0) << "\t    SLOG preprocessing width: " << stereo_settings().slogW << "\n";
          disparity_map = SubpixelView<SlogStereoPreprocessingFilter>(disparity_disk_image, 
                                                                      channels_to_planes(left_disk_image), 
                                                                      channels_to_planes(right_disk_image),
                                                                      stereo_settings().h_kern, stereo_settings().v_kern, 
                                                                      stereo_settings().do_h_subpixel, 
                                                                      stereo_settings().do_v_subpixel,   // h and v subpixel
                                                                      stereo_settings().subpixel_mode,
                                                                      SlogStereoPreprocessingFilter(stereo_settings().slogW),
                                                                      false);
        } else if (stereo_settings().log) {
          vw_out(0) << "\t    LOG preprocessing width: " << stereo_settings().slogW << "\n";
          disparity_map = SubpixelView<LogStereoPreprocessingFilter>(disparity_disk_image, 
                                                                     channels_to_planes(left_disk_image), 
                                                                     channels_to_planes(right_disk_image),
                                                                     stereo_settings().h_kern, stereo_settings().v_kern, 
                                                                     stereo_settings().do_h_subpixel, 
                                                                     stereo_settings().do_v_subpixel,   // h and v subpixel
                                                                     stereo_settings().subpixel_mode,
                                                                     LogStereoPreprocessingFilter(stereo_settings().slogW),
                                                                     false);
      
        } else {
          vw_out(0) << "\t    BLUR preprocessing width: " << stereo_settings().slogW << "\n";
          disparity_map = SubpixelView<BlurStereoPreprocessingFilter>(disparity_disk_image, 
                                                                      channels_to_planes(left_disk_image), 
                                                                      channels_to_planes(right_disk_image),
                                                                      stereo_settings().h_kern, stereo_settings().v_kern, 
                                                                      stereo_settings().do_h_subpixel, 
                                                                      stereo_settings().do_v_subpixel,   // h and v subpixel
                                                                      stereo_settings().subpixel_mode,
                                                                      BlurStereoPreprocessingFilter(stereo_settings().slogW),
                                                                      false);
        }
      }

      // Create a disk image resource and prepare to write a tiled
      // OpenEXR.
      DiskImageResourceOpenEXR disparity_map_rsrc2(out_prefix + "-R.exr", disparity_map.format() );
      disparity_map_rsrc2.set_tiled_write(std::min(256,disparity_map.cols()),std::min(256, disparity_map.rows()));
      block_write_image( disparity_map_rsrc2, disparity_map, TerminalProgressCallback(InfoMessage, "\t--> Refinement :") );    

      // For debugging...
      //        crop(disparity_map,100,150,200,200) = 
      //           crop(AffineSubpixelView(disparity_disk_image, 
      //                                   channels_to_planes(left_disk_image), 
      //                                   channels_to_planes(right_disk_image), 
      //                                   stereo_settings().h_kern, stereo_settings().v_kern, 
      //                                   stereo_settings().do_h_subpixel, 
      //                                   stereo_settings().do_v_subpixel,   // h and v subpixel
      //                                   false),
      //                100,150,200,200);

    } catch (IOErr &e) { 
      vw_out(ErrorMessage) << "\nUnable to start at refinement stage -- could not read input files.\n" << e.what() << "\nExiting.\n\n";
      exit(0);
    }
  }

  /***************************************************************************/
  /*                      Disparity Map Filtering                            */
  /***************************************************************************/
  if(entry_point <= FILTERING) {
    if (entry_point == FILTERING)
        cout << "\nStarting at the FILTERING stage.\n";
    vw_out(0) << "\n[ " << current_posix_time_string() << " ] : Stage 3 --> FILTERING \n";


    std::string post_correlation_fname;
    session->pre_filtering_hook(out_prefix+"-R.exr", post_correlation_fname);

    try {
      DiskImageView<PixelDisparity<float> > disparity_disk_image(post_correlation_fname);
      ImageViewRef<PixelDisparity<float> > disparity_map = disparity_disk_image;

      // DEBUG CODE:
      //
      //       std::cout << "\n\n---------> Making a hole! <--------------\n\n";
      //       ImageView<PixelDisparity<float> > test = disparity_disk_image;
      //       fill(crop(test, 80,150,100,100),PixelDisparity<float>());
      //       ImageViewRef<PixelDisparity<float> > disparity_map = test;

      vw_out(0) << "\t--> Cleaning up disparity map prior to filtering processes (" << stereo_settings().rm_cleanup_passes << " passes).\n";
      for (int i = 0; i < stereo_settings().rm_cleanup_passes; ++i) {
        disparity_map = disparity::clean_up(disparity_map,
                                            stereo_settings().rm_h_half_kern, stereo_settings().rm_v_half_kern,
                                            stereo_settings().rm_treshold, stereo_settings().rm_min_matches/100.0);
      }

      // Rasterize the results so far to a temporary file on disk.
      // This file is deleted once we complete the second half of the
      // disparity map filtering process.
      std::cout << "\t--> Rasterizing filtered disparity map to disk. \n";
      DiskCacheImageView<PixelDisparity<float> > filtered_disparity_map(disparity_map, "exr", 
                                                                        TerminalProgressCallback(ErrorMessage, "\t--> Writing: "));

      // Write out the extrapolation mask imaege
      std::cout << "\t--> Creating \"Good Pixel\" image: " << (out_prefix + "-GoodPixelMap.tif") << "\n";
      write_image(out_prefix + "-GoodPixelMap.tif", disparity::missing_pixel_image(filtered_disparity_map), 
                  TerminalProgressCallback(ErrorMessage, "\t    Writing: "));

      // Call out to NURBS hole filling code.  The hole filling is
      // done with a subsampled (by 4) images and then the hole filled
      // values are upsampled to the full resolution of the image
      // using bicubic interpolation.
      ImageViewRef<PixelDisparity<float> > hole_filled_disp_map = filtered_disparity_map;
      
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.tif");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.tif");
      if(stereo_settings().fill_holes_NURBS) {
        std::cout << "\t--> Filling holes with bicubicly interpolated B-SPLINE surface.\n";
        hole_filled_disp_map = HoleFillView(disparity::mask(filtered_disparity_map,Lmask,Rmask), 2);
      } 

      DiskImageResourceOpenEXR disparity_map_rsrc(out_prefix + "-F.exr", hole_filled_disp_map.format() );
      disparity_map_rsrc.set_tiled_write(std::min(1024,hole_filled_disp_map.cols()),std::min(1024, hole_filled_disp_map.rows()));
      block_write_image(disparity_map_rsrc, hole_filled_disp_map, TerminalProgressCallback(InfoMessage, "\t--> Filtering: ") ); 
    } catch (IOErr &e) { 
      cout << "\nUnable to start at filtering stage -- could not read input files.\n" << e.what() << "\nExiting.\n\n";
      exit(0);
    }
  }


  /******************************************************************************/
  /*                               TRIANGULATION                                */
  /******************************************************************************/
  if (entry_point <= POINT_CLOUD) {
    if (entry_point == POINT_CLOUD) 
      std::cout << "\nStarting at the TRIANGULATION stage.\n";
    vw_out(0) << "\n[ " << current_posix_time_string() << " ] : Stage 4 --> TRIANGULATION \n";

    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      session->camera_models(camera_model1, camera_model2);

      // If the user has generated a set of position and pose
      // corrections using the bundle_adjust program, we read them in
      // here and incorporate them into our camera model.
      Vector3 position_correction;
      Quaternion<double> pose_correction;
      if (fs::exists(prefix_from_filename(in_file1)+".adjust")) {
        read_adjustments(prefix_from_filename(in_file1)+".adjust", position_correction, pose_correction);
        camera_model1 = boost::shared_ptr<CameraModel>(new AdjustedCameraModel(camera_model1, 
                                                                               position_correction,
                                                                               pose_correction));
      }
      if (fs::exists(prefix_from_filename(in_file2)+".adjust")) {
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
      std::cout << "\t--> Generating a 3D point cloud.   \n";
      StereoView<ImageView<PixelDisparity<float> > > stereo_image(disparity_map, *camera_model1, *camera_model2);

      // For debugging...
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

      DiskImageResourceGDAL point_cloud_rsrc(out_prefix + "-PC.tif", point_cloud.format() );
      point_cloud_rsrc.set_block_size(Vector2i(std::min(1024,point_cloud.cols()),
					       std::min(1024, point_cloud.rows())));

      write_image(point_cloud_rsrc, point_cloud, TerminalProgressCallback(ErrorMessage, "\t--> Triangulating: "));
      vw_out(0) << "\t--> " << universe_radius_func;
    } catch (IOErr &e) { 
      cout << "\nUnable to start at point cloud stage -- could not read input files.\n" << e.what() << "\nExiting.\n\n";
      exit(0);
    }
    
  }

  return(EXIT_SUCCESS);
}

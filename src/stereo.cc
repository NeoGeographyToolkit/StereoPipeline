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
#include <vw/Image/Statistics.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Stereo/PyramidCorrelator.h>
#include <vw/Stereo/MultiresolutionCorrelator.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::cartography;

#include "stereo.h"
#include "file_lib.h"
#include "StereoSession.h"
#include "SurfaceNURBS.h"
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
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
}

//***********************************************************************
// MAIN
//***********************************************************************

int main(int argc, char* argv[]) {

  // Definition of data and structures
  int argstart;
  F_HD hd;         /* parameters read in header file & argv[] */
  DFT_F dft;       /* parameters read in stereo.default */
  TO_DO execute;   /* whether or not to execute specific parts of the program */
  int nn;          /* Reuseable counter variable */
  
  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int entry_point;
  unsigned cache_size;
  std::string stereo_session_string;
  std::string stereo_default_filename;
  std::string in_file1, in_file2, cam_file1, cam_file2, extra_arg1, extra_arg2;
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
    ("session-type,t", po::value<std::string>(&stereo_session_string)->default_value("pinhole"), "Select the stereo session type to use for processing. [default: pinhole]")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)");


  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("left-camera-model", po::value<std::string>(&cam_file1), "Left Camera Model File")
    ("right-camera-model", po::value<std::string>(&cam_file2), "Right Camera Model File")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames")
    ("extra_argument1", po::value<std::string>(&extra_arg1), "Extra Argument 1")
    ("extra_argument2", po::value<std::string>(&extra_arg2), "Extra Argument 2");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("left-camera-model", 1);
  positional_options_desc.add("right-camera-model", 1);
  positional_options_desc.add("output-prefix", 1);
  positional_options_desc.add("extra_argument1", 1);
  positional_options_desc.add("extra_argument2", 1);

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

  // Support for legacy code 
  init_dft_struct(&dft, &execute);
  argstart = 1;
  init_header_struct(&dft, &hd, argv[0], "dummy", "dummy");

  // Read all of the options out of the stereo default file. 
  read_default_file(&dft, &execute, stereo_default_filename.c_str());

  // Set search range from stereo.default file
  BBox2i search_range(Vector<int,2>(dft.h_corr_min, dft.v_corr_min),
                      Vector<int,2>(dft.h_corr_max, dft.v_corr_max));

  // Set the Vision Workbench debug level
  set_debug_level(DebugMessage-1);
  Cache::system_cache().resize( cache_size*1024*1024 ); // Set cache to 1Gb

  // Create a fresh stereo session and query it for the camera models.
  StereoSession* session = StereoSession::create(stereo_session_string);
  session->initialize(in_file1, in_file2, cam_file1, cam_file2, 
                      out_prefix, extra_arg1, extra_arg2);
  boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
  session->camera_models(camera_model1, camera_model2);
    
  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {

    std::string pre_preprocess_file1, pre_preprocess_file2;
    session->pre_preprocessing_hook(in_file1, in_file2, pre_preprocess_file1, pre_preprocess_file2);

    std::cout << "Pre_Preprocess_Files: " << pre_preprocess_file1 << "   " << pre_preprocess_file2 << "\n";
    DiskImageView<PixelGray<uint8> > left_rectified_image(pre_preprocess_file1);
    DiskImageView<PixelGray<uint8> > right_rectified_image(pre_preprocess_file2);
    
    // The generate_mask() routine returns an ImageView<bool>, so we
    // currently multiply by 255 to scale the bool value to a uint8.
    if(execute.w_mask) {  
      cout << "\nGenerating image masks...";
      int mask_buffer = std::max(dft.h_kern, dft.v_kern);
      ImageViewRef<uint8> Lmask = channel_cast_rescale<uint8>(disparity::generate_mask(left_rectified_image, mask_buffer));
      ImageViewRef<uint8> Rmask = channel_cast_rescale<uint8>(disparity::generate_mask(right_rectified_image, mask_buffer));
      printf("Done.\n");
      write_image(out_prefix + "-lMask.tif", Lmask);
      write_image(out_prefix + "-rMask.tif", Rmask);
    }
  }

  /*********************************************************************************/
  /*                            correlation step                                   */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    if (entry_point == CORRELATION) 
        cout << "\nStarting at the CORRELATION stage.\n";

    DiskImageView<PixelGray<float> > left_disk_image(out_prefix+"-L.tif");
    DiskImageView<PixelGray<float> > right_disk_image(out_prefix+"-R.tif");
            
    std::cout << "------------------------- correlation ----------------------\n";
    std::cout << "\tsearch range: " << search_range << "\n";
    std::cout << "\tkernel size : " << dft.h_kern << "x" << dft.v_kern << "\n";
    std::cout << "\txcorr thresh: " << dft.xcorr_treshold << "\n";
    std::cout << "\tslog stddev : " << dft.slogW << "\n\n";
    CorrelationSettings corr_settings(search_range.min().x(), search_range.max().x(), 
                                      search_range.min().y(), search_range.max().y(),
                                      dft.h_kern, dft.v_kern, 
                                      true,         // verbose
                                      dft.xcorr_treshold,
                                      1.5,          // correlation score rejection threshold (1.0 disables, good values are 1.5 - 2.0)
                                      dft.slogW,
                                      true, true,   // h and v subpixel
                                      true);        // bit image
    
    std::cout<< "Building Disparity map... " << std::flush;
    ImageViewRef<PixelDisparity<float> > disparity_map = CorrelatorView<PixelGray<float> >(left_disk_image, right_disk_image, corr_settings);
   
    ImageViewRef<PixelDisparity<float> > proc_disparity_map = disparity::clean_up(disparity_map,
                                                                                  dft.rm_h_half_kern, dft.rm_v_half_kern,
                                                                                  dft.rm_treshold, dft.rm_min_matches/100.0);

    // Create a disk image resource and prepare to write a tiled
    // OpenEXR.
    DiskImageResourceOpenEXR disparity_map_rsrc(out_prefix + "-D.exr", disparity_map.format() );
    disparity_map_rsrc.set_tiled_write(std::min(2048,disparity_map.cols()),std::min(2048, disparity_map.rows()));
    write_image( &disparity_map_rsrc, disparity_map, FileMetadataCollection::create(), TerminalProgressCallback() );
    DiskImageView<PixelDisparity<float> > disk_disparity_map(out_prefix + "-D.exr");
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

      // This seems to be more cleanup than is needed with the new
      // pyramid correlator... skipping it for now.
      //
      std::cout << "\tcleaning up disparity map prior to filtering processes.\n";
      ImageViewRef<PixelDisparity<float> > disparity_map = disparity::clean_up(disparity_disk_image,
                                                                               dft.rm_h_half_kern, dft.rm_v_half_kern,
                                                                               dft.rm_treshold, dft.rm_min_matches/100.0);

      // Apply the Mask to the disparity map 
      std::cout << "\tapplying mask.\n";
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.tif");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.tif");
      if(execute.apply_mask){
        disparity_map = disparity::mask(disparity_map, Lmask, Rmask);
      }

      // Rasterize the results so far to a temporary file on disk.
      // This file is deleted once we complete the second half of the
      // disparity map filtering process.
      std::cout << "\trasterizing filtered disparity map to disk. \n" << std::flush;
      std::string temp_filename = out_prefix + "-FiltF.exr";
      write_image( temp_filename, disparity_map, TerminalProgressCallback() );
      DiskImageView<PixelDisparity<float> > filtered_disparity_map(temp_filename);

      // Write out the extrapolation mask imaege
      if(execute.w_extrapolation_mask) 
        write_image(out_prefix + "-GoodPixelMap.tif", disparity::missing_pixel_image(filtered_disparity_map), TerminalProgressCallback());

      // Call out to NURBS hole filling code.     
      if(execute.fill_holes_NURBS) {
        std::cout << "Filling holes with B-SPLINE surface... " << std::flush;
        int nurbs_iterations = 10;
        ImageView<PixelDisparity<float> > final_disparity = MBASurfaceNURBS(filtered_disparity_map, nurbs_iterations);
        std::cout << "done.  Writing results to disk.\n";
        write_image(out_prefix + "-F.exr", disparity::mask(final_disparity,Lmask, Rmask), TerminalProgressCallback() ); 
      } else {
        write_image(out_prefix + "-F.exr", filtered_disparity_map, TerminalProgressCallback() ); 
      }

      // Delete the temporory file on disk.
      unlink(temp_filename.c_str());

      DiskImageView<PixelDisparity<float> > final_disparity_map(out_prefix + "-F.exr");
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
      std::string prehook_filename;
      session->pre_pointcloud_hook(out_prefix+"-F.exr", prehook_filename);
     
      DiskImageView<PixelDisparity<float> > disparity_map(prehook_filename);

      // Apply the stereo model.  This yields a image of 3D points in
      // space.  We build this image and immediately write out the
      // results to disk.
      std::cout << "Generating a 3D point cloud.   \n";
      StereoView<ImageView<PixelDisparity<float> > > stereo_image(disparity_map, *camera_model1, *camera_model2);

      // If the distance from the left camera center to a point is
      // greater than the universe radius, we remove that pixel and
      // replace it with a zero vector, which is the missing pixel value
      // in the point_image.
      //
      // We apply the universe radius here and then write the result
      // directly to a file on disk.
      UniverseRadiusFunc universe_radius_func(camera_model1->camera_center(Vector2(0,0)), dft.near_universe_radius, dft.far_universe_radius);
      write_image(out_prefix + "-PC.exr", per_pixel_filter(stereo_image, universe_radius_func), TerminalProgressCallback());
      std::cout << universe_radius_func;

    } catch (IOErr&) { 
      cout << "\n Unable to start at point cloud stage.\n\tCould not read input files. Exiting.\n\n";
      exit(0);
    }
    
  }

  free (hd.cmd_name);
  return(EXIT_SUCCESS);
}

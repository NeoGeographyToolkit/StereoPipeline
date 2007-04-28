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
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "OrthoRasterizer.h"
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
  DFT_F dft;	     /* parameters read in stereo.default */
  TO_DO execute;   /* whether or not to execute specific parts of the program */
  int nn;          /* Reuseable counter variable */
  
  // Set the Vision Workbench debug level
  set_debug_level(VerboseDebugMessage+1);

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int entry_point;
  std::string stereo_session_string;
  std::string stereo_default_filename;
  std::string in_file1, in_file2, cam_file1, cam_file2, extra_arg1, extra_arg2;
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("session-type,t", po::value<std::string>(&stereo_session_string)->default_value("pinhole"), "Select the stereo session type to use for processing. [default: pinhole]")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)")
    ("multiresolution,m", "Use the prototype multiresolution correlator instead of the old area-based correlator");

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
              << "	the extensions are automaticaly added to the output files\n"
              << "	the parameters should be in stereo.default\n\n";
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

//   // Set up the stereo engine object
//   stereo_engine.do_horizontal_pyramid_search = execute.autoSetCorrParam;
//   stereo_engine.do_vertical_pyramid_search = dft.autoSetVCorrParam;
//   stereo_engine.kernel_width = dft.h_kern;
//   stereo_engine.kernel_height = dft.v_kern;
//   stereo_engine.cross_correlation_threshold = dft.xcorr_treshold;
  
//   stereo_engine.do_slog = execute.slog;
//   stereo_engine.do_log = execute.log;
//   stereo_engine.slog_stddev = dft.slogW;
  
//   stereo_engine.do_cleanup = true;
//   stereo_engine.cleanup_vertical_half_kernel = dft.rm_v_half_kern;
//   stereo_engine.cleanup_horizontal_half_kernel = dft.rm_h_half_kern;
//   stereo_engine.cleanup_min_matches = dft.rm_min_matches;
//   stereo_engine.cleanup_rm_threshold = (float)dft.rm_treshold;
  
//   stereo_engine.do_nurbs_hole_filling = execute.fill_holes_NURBS;
//   stereo_engine.nurbs_iterations = 10;

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
    write_image(out_prefix + "-T.jpg", left_rectified_image);
    
    // The generate_mask() routine returns an ImageView<bool>, so we
    // currently multiply by 255 to scale the bool value to a uint8.
    if(execute.w_mask) {  
      cout << "\nGenerating image masks...";
      int mask_buffer = std::max(dft.h_kern, dft.v_kern);
      ImageViewRef<uint8> Lmask = channel_cast_rescale<uint8>(disparity::generate_mask(left_rectified_image, mask_buffer));
      ImageViewRef<uint8> Rmask = channel_cast_rescale<uint8>(disparity::generate_mask(right_rectified_image, mask_buffer));
      printf("Done.\n");
      write_image(out_prefix + "-lMask.png", Lmask);
      write_image(out_prefix + "-rMask.png", Rmask);
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
    
    // Call the stereo engine to perform the correlation
    // determine the search window parameters 
//     if((execute.autoSetCorrParam || dft.autoSetVCorrParam) && !vm.count("multiresolution")) {
//       std::cout << "\n ------------ Pyramid Search for Correlation Space --------- \n";
//       printf("Starting pyramid search with initial search dimensions H: [ %d, %d ] V: [ %d, %d ]\n",
//              search_range.min().x(), search_range.max().x(), search_range.min().y(), search_range.max().y());
      
//       try {
//         vw::stereo::PyramidCorrelator pyramid_correlator(search_range.min().x(), search_range.max().x(),
//                                                          search_range.min().y(), search_range.max().y(),
//                                                          dft.h_kern, dft.v_kern, 
//                                                          true, dft.xcorr_treshold,
//                                                          dft.slogW);
//         //          pyramid_correlator.enable_debug_mode("pyramid");
//         search_range = pyramid_correlator(left_image, right_image, execute.autoSetCorrParam, dft.autoSetVCorrParam);
          
//       } catch (vw::stereo::CorrelatorErr &e) {
//           std::cout << "Pyramid correlation failed:\n";
//           e.what();
//           std::cout << "Exiting.\n\n";
//           exit(1);
//       }
//     }
            
    std::cout << "------------------------- correlation ----------------------\n";
    CorrelationSettings corr_settings(search_range.min().x(), search_range.max().x(), 
                                      search_range.min().y(), search_range.max().y(),
                                      dft.h_kern, dft.v_kern, 
                                      true,         // verbose
                                      dft.xcorr_treshold,
                                      true, true,   // h and v subpixel
                                      true);        // bit image
    
    // perform the sign of laplacian of gaussian filter (SLOG) on the images 
    //    if(execute.slog) {
      std::cout << "Applying SLOG filter.\n";
      ImageView<PixelGray<uint8> > left_bit_image = channel_cast<uint8>(threshold(laplacian_filter(gaussian_filter(left_disk_image,dft.slogW)), 0.0));
      ImageView<PixelGray<uint8> > right_bit_image = channel_cast<uint8>(threshold(laplacian_filter(gaussian_filter(right_disk_image,dft.slogW)), 0.0));
      std::cout<< "Building Disparity map... " << std::flush;
      ImageViewRef<PixelDisparity<float> > disparity_map = CorrelatorView(left_bit_image, right_bit_image, corr_settings);
      //    }
//  else if (execute.log) {
//       corr_settings.m_bit_image = false;
//       std::cout << "Applying LOG filter.\n";
//       ImageViewRef<PixelGray<float> > left_log_image = vw::laplacian_filter(vw::gaussian_filter(Limg, dft.slogW));
//       ImageViewRef<PixelGray<float> > right_log_image = vw::laplacian_filter(vw::gaussian_filter(Rimg, dft.slogW));
//       disparity_map = correlator(left_log_image, right_log_image, false);
//     }

    std::cout << "\nCleaning up disparity map prior to filtering processes.\n";
    ImageView<PixelDisparity<float> > proc_disparity_map = disparity::clean_up(disparity_map,
                                                                               dft.rm_h_half_kern, dft.rm_v_half_kern,
                                                                               dft.rm_treshold, dft.rm_min_matches/100.0);

    write_image( out_prefix + "-D.exr", proc_disparity_map, TerminalProgressCallback() );
    DiskImageView<PixelDisparity<float> > disk_disparity_map(out_prefix + "-D.exr");

    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disk_disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp);
    write_image( out_prefix + "-DH.jpg", normalize(clamp(select_channel(disk_disparity_map,0), min_h_disp, max_h_disp)));
    write_image( out_prefix + "-DV.jpg", normalize(clamp(select_channel(disk_disparity_map,1), min_v_disp, max_v_disp)));
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

      //  The multiresolution correlate does not tend to have
      //  outliers(!), so we skip outlier detection in this case.
      if (!vm.count("multiresolution")) {
        std::cout << "\nCleaning up disparity map prior to filtering processes.\n";
        disparity_map = disparity::clean_up(disparity_map,
                                            dft.rm_h_half_kern, dft.rm_v_half_kern,
                                            dft.rm_treshold, dft.rm_min_matches/100.0);
      } 

      // Apply the Mask to the disparity map 
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.png");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.png");
      if(execute.apply_mask){
        disparity_map = disparity::mask(disparity_map, Lmask, Rmask);
      }

      // Rasterize the results so far to a temporary file on disk.
      // This file is deleted once we complete the second half of the
      // disparity map filtering process.
      std::cout << "Rasterizing filtered disparity map to disk. \n" << std::flush;
      std::string temp_filename = out_prefix + "-FiltF.exr";
      write_image( temp_filename, disparity_map, TerminalProgressCallback() );
      DiskImageView<PixelDisparity<float> > filtered_disparity_map(temp_filename);

      // Write out the extrapolation mask image
      if(execute.w_extrapolation_mask) 
        write_image(out_prefix + "-GoodPixelMap.png", disparity::missing_pixel_image(filtered_disparity_map));

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
      double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
      disparity::get_disparity_range(filtered_disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp);
      write_image( out_prefix + "-FH.jpg", normalize(clamp(select_channel(final_disparity_map,0), min_h_disp, max_h_disp)));
      write_image( out_prefix + "-FV.jpg", normalize(clamp(select_channel(final_disparity_map,1), min_v_disp, max_v_disp)));

    } catch (IOErr &e) { 
      cout << "\n An file IO error occurred during the filtering stage.  Exiting.\n\n";
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
      DiskImageView<PixelDisparity<float> > disparity_map(out_prefix+"-F.exr");

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

  /************************************************************************************/
  /*                            dot-cloud to wire mesh                                */
  /************************************************************************************/
  if(entry_point <= WIRE_MESH) {
    if (entry_point == WIRE_MESH)
      std::cout << "\nStarting at WIRE_MESH stage.\n\n";

    try {
      std::cout << "Using file " << out_prefix + "-PC.exr" << " as pointcloud map.\n";
      DiskImageView<Vector3> point_image(out_prefix + "-PC.exr");

      if (execute.mesh) {
        std::cout << "\nGenerating 3D mesh from point cloud:\n";
        Mesh mesh_maker;
        if(execute.adaptative_meshing) {
          mesh_maker.build_adaptive_mesh(point_image, dft.mesh_tolerance, dft.max_triangles);
        } else {
          mesh_maker.build_simple_mesh(point_image, dft.nff_h_step, dft.nff_v_step);
        }
        if (execute.write_ive) 
          mesh_maker.write_osg(out_prefix+".ive", out_prefix+"-T.jpg");
        if(execute.inventor)
          mesh_maker.write_inventor(out_prefix+".iv", out_prefix+"-T.jpg");
        if(execute.vrml)
          mesh_maker.write_vrml(out_prefix+".vrml", out_prefix+"-T.jpg");
      }        
      
      // Write out the DEM, texture, and extrapolation mask as
      // georeferenced files.  The DEM height is taken as the third
      // component (Z) of the pointcloud.  The other two components (X
      // and Y) determine the lateral extent of the DEM.
      if(execute.write_dem) {
        cout << "Creating a DEM.\n";
        
        
        // Write out the DEM, texture, and extrapolation mask
        // as georeferenced files.
        vw::cartography::OrthoRasterizer<Vector3> rasterizer(point_image);
        rasterizer.set_dem_spacing(dft.dem_spacing);
        rasterizer.use_minz_as_default = false;
        rasterizer.set_default_value(0);    
        BBox<float,3> dem_bbox = rasterizer.bounding_box();
        ImageView<PixelGrayA<float> > ortho_image = rasterizer(vw::select_channel(point_image, 2));    

        // Set up the georeferencing information
        vw::cartography::GeoReference georef;
        georef.set_transform(rasterizer.geo_transform());
        write_georeferenced_image(out_prefix + "-DEM.tif", ortho_image, georef);
        write_georeferenced_image(out_prefix + "-DEM-normalized.tif", channel_cast_rescale<uint8>(ortho_image), georef);

        // Work in progress - mbroxton (07-03-08)
        //       ImageView<PixelGrayA<float> > nurbs_output = MBASurfaceNURBS(ortho_alpha_image, 9);
        //       for (int j = 0; j < ortho_image.rows(); ++j) {
        //         for (int i = 0; i < ortho_image.cols(); ++i) {
        //           if (ortho_alpha_image(i,j).a() != 0) {
        //             std::cout << "D";
        //             ortho_alpha_image(i,j) = PixelGrayA<float>(nurbs_output(i,j).v(), 1.0);
        //           }
        //         }
        //       }
        //       write_image(out_prefix + "-DEM-nurbs.tif", ortho_alpha_image);


        // Write out a georeferenced orthoimage of the DTM with alpha.
        DiskImageView<PixelGray<float> > texture(out_prefix + "-L.tif");
        ortho_image = rasterizer(select_channel(texture, 0));
        write_georeferenced_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(ortho_image), georef);

        // Write out the offset files
        std::cout << "Offset: " << dem_bbox.min().x()/rasterizer.dem_spacing() << "   " << dem_bbox.max().y()/rasterizer.dem_spacing() << "\n";
        std::string offset_filename = out_prefix + "-DRG.offset";
        FILE* offset_file = fopen(offset_filename.c_str(), "w");
        fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.dem_spacing()), -int(dem_bbox.max().y()/rasterizer.dem_spacing()));
        fclose(offset_file);
        offset_filename = out_prefix + "-DEM-normalized.offset";
        offset_file = fopen(offset_filename.c_str(), "w");
        fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.dem_spacing()), -int(dem_bbox.max().y()/rasterizer.dem_spacing()));
        fclose(offset_file);
      }
    }catch (IOErr&) { 
      cout << "\nFailed to start at wire mesh phase.\n\tCould not read input files. Exiting.\n\n";
      exit(0);
    }
  }

  free (hd.cmd_name);
  return(EXIT_SUCCESS);
}

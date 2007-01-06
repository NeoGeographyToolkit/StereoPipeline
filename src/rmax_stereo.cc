/************************************************************************
 *     File: stereo.cc
 *     Date: April 2005
 *       By: Michael Broxton and Larry Edwards
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group 
 * Function: Main program for the stereo pipeline	
 *          				
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

#include <vw/Core/Debugging.h>
#include <vw/Image/ImageView.h>
#include <vw/Image/PixelTypes.h>
#include <vw/FileIO.h>
#include <vw/Image/Filter.h>
#include <vw/Core/Exception.h>
#include <vw/Image/Transform.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/CAHVModel.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/CameraTransform.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Stereo/StereoModel.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;

#include "stereo.h"
#include "file_lib.h"
#include "mask.h"
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "StereoEngine.h"
#include "OrthoRasterizer.h"
#include "RMAX/RMAX.h"
using namespace vw::cartography;

using namespace std;

// The stereo pipeline has several stages, which are enumerated below.

enum { PREPROCESSING = 0, 
       CORRELATION, 
       FILTERING, 
       POINT_CLOUD, 
       WIRE_MESH, 
       NUM_STAGES};

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
  StereoEngine stereo_engine;  /* The stereo engine class does most of the heavy lifting */
  
  // Image buffers 
  ImageView<PixelGray<float> > Limg, Rimg;
  ImageView<PixelGray<float> > texture;
  ImageView<PixelDisparity<float> > disparity_map;
  ImageView<bool> Lmask, Rmask;
  ImageView<Vector3> point_image;  
  Matrix<double> alignMatrix;

  // Set the Vision Workbench debug level
  set_debug_level(InfoMessage);
  
  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use here
  // to specify the type, size, help string, etc, of the command line
  // arguments.
  int entry_point;
  std::string stereo_default_filename;
  std::string in_file1, in_file2;
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("output-prefix", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if( vm.count("help") ||
      !vm.count("left-input-image") || !vm.count("right-input-image") || 
      !vm.count("output-prefix")) {
    std::cout << "\nUsage: stereo [options] <Left_input_image> <Right_input_image> <output_file_prefix>\n"
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

  // Set up the stereo engine object
  stereo_engine.do_horizontal_pyramid_search = execute.autoSetCorrParam;
  stereo_engine.do_vertical_pyramid_search = dft.autoSetVCorrParam;
  stereo_engine.search_range = BBox<int,2>(Vector<int,2>(dft.h_corr_min, dft.v_corr_min),
                                           Vector<int,2>(dft.h_corr_max, dft.v_corr_max));
  stereo_engine.kernel_width = dft.h_kern;
  stereo_engine.kernel_height = dft.v_kern;
  stereo_engine.cross_correlation_threshold = dft.xcorr_treshold;
  
  stereo_engine.do_slog = execute.slog;
  stereo_engine.do_log = execute.log;
  stereo_engine.slog_stddev = dft.slogW;
  
  stereo_engine.do_cleanup = true;
  stereo_engine.cleanup_vertical_half_kernel = dft.rm_v_half_kern;
  stereo_engine.cleanup_horizontal_half_kernel = dft.rm_h_half_kern;
  stereo_engine.cleanup_min_matches = dft.rm_min_matches;
  stereo_engine.cleanup_rm_threshold = (float)dft.rm_treshold;
  
  stereo_engine.do_nurbs_hole_filling = execute.fill_holes_NURBS;
  stereo_engine.nurbs_iterations = 10;

  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {
    // Read the input files 
    try {
      std::cout << "Loading image " << in_file1 << " as primary image:\n";
      read_image(Limg, in_file1);
      std::cout << "Loading image " << in_file2 << " as secondary image:\n";
      read_image(Rimg, in_file2);
      std::cout << std::endl;    
    } catch (IOErr& e) { 
      std::cout << e.desc() << "\n";
      std::cout << "\n Could not read one or more input files. Exiting.\n\n";
      exit(0);
    }
    
    texture = copy(Limg);
    
    if(execute.w_texture) {
      try{
        write_image(out_prefix + "-T.jpg", texture);
      } catch (IOErr &e) {
        std::cout << e.what() << "\n";
        std::cout << "\nWARNING: Could not write texture buffer to disk.\n";
      }
    }
    
    // Read Camera Models, removing lens distortion if necessary.
    CAHVModel left_cahv, right_cahv;

    CAHVORModel left_cahvor = rmax_image_camera_model(in_file1);
    CAHVORModel right_cahvor = rmax_image_camera_model(in_file2);

    left_cahv = linearize_camera(left_cahvor, Limg.cols(), Limg.rows(), Limg.cols(), Limg.rows());
    right_cahv = linearize_camera(right_cahvor, Limg.cols(), Limg.rows(), Limg.cols(), Limg.rows());
    std::cout << "Removing lens distortion from images.\n";

    Limg = transform(copy(Limg), CameraTransform<CAHVORModel, CAHVModel>(left_cahvor, left_cahv));
    Rimg = transform(copy(Rimg), CameraTransform<CAHVORModel, CAHVModel>(right_cahvor, right_cahv));
    write_image(out_prefix + "-L-lin.tif", Limg);
    write_image(out_prefix + "-R-lin.tif", Rimg);
 
    // Do epipolar rectification and warp the camera images into
    // epipolar alignment.
    if (execute.do_alignment && execute.epipolar_alignment) {
      std::cout << "\nAligning images using EPIPOLAR based alignment technique\n";
      
      CAHVModel left_epipolar_cahv, right_epipolar_cahv;
      ImageView<PixelGray<float> > left_epipolar_image, right_epipolar_image;
      epipolar(left_cahv, right_cahv, left_epipolar_cahv, right_epipolar_cahv);
      Limg = transform(copy(Limg), CameraTransform<CAHVModel, CAHVModel>(left_cahv, left_epipolar_cahv));
      Rimg = transform(copy(Rimg), CameraTransform<CAHVModel, CAHVModel>(right_cahv, right_epipolar_cahv));
    }
    
    // Align images using keypoint homography.
    else if (execute.keypoint_alignment && execute.do_alignment) {
      std::cout << "\nAligning images using KEYPOINT based automated alignment technique\n";
      Matrix<double> align_matrix = keypoint_align(Rimg, Limg);
      std::cout << "Alignment Matrix:\n" << alignMatrix << "\n";
      write_matrix(out_prefix + "-align.exr", align_matrix);
      ImageView<PixelGray<float> > aligned_image = transform(Rimg, HomographyTransform(align_matrix),
                                                             Limg.cols(), Limg.rows());
      Rimg = aligned_image;
    }
    
    /*******************************************************************/
    /* Save the Left linearized and epipolar-zed image as the texture  */
    /*******************************************************************/
    texture = copy(Limg);
    if(execute.w_texture) {
      try{
        write_image(out_prefix + "-T.jpg", texture);
      } catch (IOErr &e) {
        std::cout << e.what() << "\n";
        std::cout << "\nWARNING: Could not write texture buffer to disk.\n";
      }
    }
  
    /*******************************************************************/
    /* Save both L/R linearized and epipolar-zed images to disk        */
    /*******************************************************************/ 
    write_image(out_prefix + "-L.tif", channel_cast<uint8>(Limg*255));
    write_image(out_prefix + "-R.tif", channel_cast<uint8>(Rimg*255));
    
    // Mask any pixels that are black and 
    // appear on the edges of the image.
    printf("\nGenerating image masks...");
    Lmask = disparity::generate_mask(Limg);
    Rmask = disparity::generate_mask(Rimg);
    printf("Done.\n");
    if(execute.w_mask) {      
      write_mask(Lmask, out_prefix + "-lMask.png");
      write_mask(Rmask, out_prefix + "-rMask.png");
    }
  }

  /*********************************************************************************/
  /*                            correlation step                                   */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    
    // If we are jump-starting the code in the post-correlation phase,
    // we must initialize the disparity map object here with the 
    // twe files that were supplied at the command line.
    if (entry_point == CORRELATION) {
      try {
        std::cout << "\nStarting at the CORRELATION stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-L.tif" << " as primary image:\n";
        read_image(Limg, out_prefix + "-L.tif");
        std::cout << "\nLoading image " << out_prefix + "-R.tif" << " as secondary image:\n";
        read_image(Rimg, out_prefix + "-R.tif");
        std::cout << "\nLoading image " << out_prefix + "-T.jpg" << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        std::cout << std::endl;
      } catch (IOErr&) { 
        std::cout << "\nUnable to start at the correlation stage.  Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    std::cout << "\n" << stereo_engine << "\n";
    disparity_map = stereo_engine.correlate(Limg, Rimg);

    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image mask...");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }
    
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp,true);
    write_image( out_prefix + "-DH.jpg", normalize(clamp(select_channel(disparity_map,0), min_h_disp, max_h_disp)));
    write_image( out_prefix + "-DV.jpg", normalize(clamp(select_channel(disparity_map,1), min_v_disp, max_v_disp)));
    write_image( out_prefix + "-D.exr", channels_to_planes(disparity_map) );
  }

  /***************************************************************************/
  /*                      Disparity Map Filtering                            */
  /***************************************************************************/

  if(entry_point <= FILTERING) {

    if (entry_point == FILTERING) {
      try {
        std::cout << "\nStarting at the FILTERING stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-D.exr" << " as disparity map image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-D.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
        std::cout << "\nLoading image " << out_prefix + "-T.jpg" << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        std::cout << std::endl;
      } catch (IOErr&) { 
        std::cout << "\n Unable to start code at the filtering stage.  Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    stereo_engine.filter(disparity_map);

    // Write out the extrapolation mask image
    if(execute.w_extrapolation_mask) {  
      ImageView<PixelRGB<float> > extrapolationMask = disparity::rgb_missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap-color.jpg", extrapolationMask);
      ImageView<PixelGray<float> > extrapolationMaskGray = disparity::missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap.png", extrapolationMaskGray);
    }

    // Do the NURBS interpolation
    stereo_engine.interpolate(disparity_map);

//     // Smooth the disparity map slightly
//     disparity_map = crop(disparity_map, 5340,610,20,20);
//     for (int i = 0; i < disparity_map.cols(); i++) {
//       for (int j = 0; j < disparity_map.cols(); j++) {
//         if (disparity_map(i,j).missing() ) { std::cout << "[" << disparity_map(i,j).h() << " " << disparity_map(i,j).v() << "] ";} 
//         else { std::cout << "[" << disparity_map(i,j).h() << " " << disparity_map(i,j).v() << "] " ;}
//       }
//       std::cout << "\n";
//     }

//     std::cout << "\tSmoothing... " << std::flush;
//     disparity_map = gaussian_filter(copy(disparity_map), 2.0);
//     std::cout << "done.\n";

//     for (int i = 0; i < disparity_map.cols(); i++) {
//       for (int j = 0; j < disparity_map.cols(); j++) {
//         if (disparity_map(i,j).missing() ) { std::cout << "[" << disparity_map(i,j).h() << " " << disparity_map(i,j).v() << "] ";} 
//         else { std::cout << "[" << disparity_map(i,j).h() << " " << disparity_map(i,j).v() << "] ";}
//       }
//       std::cout << "\n";
//     }
    
    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image mask...");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }

    // Write the filtered disp map 
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp, true);
    write_image( out_prefix + "-FH.jpg", normalize(clamp(select_channel(disparity_map,0), min_h_disp, max_h_disp)));
    write_image( out_prefix + "-FV.jpg", normalize(clamp(select_channel(disparity_map,1), min_v_disp, max_v_disp)));
    write_image( out_prefix + "-F.exr", channels_to_planes(disparity_map) );
  }

  /******************************************************************************/
  /*                           disparity to dot-cloud                           */
  /******************************************************************************/
  if (entry_point <= POINT_CLOUD) {
    /* 
     * If we are jump-starting the code in the post-correlation phase,
     * we must initialize the disparity map object here with the 
     * twe files that were supplied at the command line.  Right now
     * this is purely for debugging.
     */
    if (entry_point == POINT_CLOUD) {
      /* Read the input files */
      try {
        std::cout << "\nStarting code at POINT_CLOUD stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-F.exr" << " as disparity image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-F.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
        std::cout << "\nLoading image " << out_prefix + "-T.jpg" << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        std::cout << std::endl;	
      } catch (IOErr&) { 
        std::cout << "\n Unable to start at point cloud stage.\n\tCould not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    // Read Camera Models, removing lens distortion if necessary.
    std::cout << "Loading camera models:\n";
    CAHVModel left_cahv, right_cahv;
    
    CAHVORModel left_cahvor = rmax_image_camera_model(in_file1);
    CAHVORModel right_cahvor = rmax_image_camera_model(in_file2);

    left_cahv = linearize_camera(left_cahvor, disparity_map.cols(), disparity_map.rows(), disparity_map.cols(), disparity_map.rows());
    right_cahv = linearize_camera(right_cahvor, disparity_map.cols(), disparity_map.rows(), disparity_map.cols(), disparity_map.rows());

    // Do epipolar rectification and warp the camera images into epipolar alignment.
    if (execute.do_alignment && execute.epipolar_alignment) {
      
      CAHVModel left_epipolar_cahv, right_epipolar_cahv;
      epipolar(left_cahv, right_cahv, left_epipolar_cahv, right_epipolar_cahv);

      StereoModel stereo_model(left_epipolar_cahv, right_epipolar_cahv);
      ImageView<double> error;
      std::cout << "Generating a 3D point cloud.   \n";
      point_image = stereo_model(disparity_map, error);
      write_image(out_prefix + "-error.png", normalize(error));
    }

    // Align images using keypoint homography.
    if (execute.keypoint_alignment && execute.do_alignment) {
      try {
        read_matrix(alignMatrix, out_prefix + "-align.exr");
      } catch (vw::IOErr &e) {
        std::cout << "Could not read in aligment matrix: " << out_prefix << "-align.exr.  Exiting. \n\n";
        exit(1);
      }
      
      // Apply the inverse of the transform matrix
      vw::Matrix<double> inv_align_matrix = inverse(alignMatrix);
      disparity_linear_transform(disparity_map, inv_align_matrix);
      
      // Apply the inverse transform to the disparity map 
      StereoModel stereo_model(left_cahv, right_cahv);
      std::cout << "Generating a 3D point cloud.   \n";
      ImageView<double> error;
      point_image = stereo_model(disparity_map, error);
      write_image(out_prefix + "-error.png", normalize(error));
    }

    // Write out the results to disk
    write_image(out_prefix + "-PC.exr", channels_to_planes(point_image));
  }

  /************************************************************************************/
  /*                            dot-cloud to wire mesh                                */
  /************************************************************************************/
  if(entry_point <= WIRE_MESH) {
    

    // If we are jump-starting the code in the wire mesh phase, we want 
    // to simply read in the point cloud from disk and start from here.
    if (entry_point == WIRE_MESH) {
      try {
        std::cout << "\nStarting at WIRE_MESH stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-PC.exr" << " as pointcloud map:\n";
        ImageView<double> points;
        read_image(points, out_prefix + "-PC.exr");
        point_image.set_size(points.cols(), points.rows());
        channels_to_planes(point_image) = points;
        std::cout << "\nLoading image " << out_prefix + "-T.jpg" << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");
        std::cout << std::endl;
        std::cout << "\nLoading image " << out_prefix + "-F.exr" << " as disparity map image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-F.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
      } catch (IOErr&) { 
        std::cout << "\nFailed to start at wire mesh phase.\n\tCould not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    if (execute.mesh) {
      Mesh mesh_maker;
      if(execute.adaptative_meshing) {
        mesh_maker.build_adaptive_mesh(point_image, dft.mesh_tolerance, dft.max_triangles);
      } else {
        mesh_maker.build_simple_mesh(point_image, dft.nff_h_step, dft.nff_v_step);
      }
      if(execute.inventor){
        mesh_maker.write_inventor(out_prefix+".iv", out_prefix+"-T.jpg");
      }
      if(execute.vrml){
        mesh_maker.write_vrml(out_prefix+".vrml", out_prefix+"-T.jpg");
      }
    }
     
    // Write out the DEM, texture, and extrapolation mask as
    // georeferenced files.  The DEM height is taken as the third
    // component (Z) of the pointcloud.  The other two components (X
    // and Y) determine the lateral extent of the DEM.
    if(execute.write_dem) {
      cout << "Creating a DEM from the Z points.\n";

      // Write out the DEM, texture, and extrapolation mask
      // as georeferenced files.
      ImageView<double> dem_texture = vw::select_channel(point_image, 2);
      vw::cartography::OrthoRasterizer<Vector3> rasterizer(point_image);
      rasterizer.use_minz_as_default = false;
      rasterizer.set_default_value(0);
      ImageView<PixelGray<float> > ortho_image = rasterizer(vw::select_channel(point_image, 2));
      write_image(out_prefix + "-DEM-debug.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Create a DEM with alpha
      ImageView<PixelGrayA<float> > ortho_alpha_image(ortho_image.cols(), ortho_image.rows());
      for (int j = 0; j < ortho_image.rows(); ++j) {
        for (int i = 0; i < ortho_image.cols(); ++i) {
          if (ortho_image(i,j) == 0) {
            ortho_alpha_image(i,j) = PixelGrayA<float>();
          } else {
            ortho_alpha_image(i,j) = PixelGrayA<float>(ortho_image(i,j));
          }
        }
      }
      write_image(out_prefix + "-DEM.tif", ortho_alpha_image);

      // Write out a georeferenced orthoimage of the DTM
      rasterizer.set_dem_spacing(dft.dem_spacing);
      rasterizer.use_minz_as_default = false;
      rasterizer.set_default_value(0);
      ortho_image = rasterizer(select_channel(texture, 0));
      write_image(out_prefix + "-DRG-debug.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Create a DRG with alpha
      ImageView<PixelGrayA<uint8> > drg_ortho_alpha_image(ortho_image.cols(), ortho_image.rows());
      for (int j = 0; j < ortho_image.rows(); ++j) {
        for (int i = 0; i < ortho_image.cols(); ++i) {
          if (ortho_image(i,j) == 0) {
            drg_ortho_alpha_image(i,j) = PixelGrayA<uint8>();
          } else {
            drg_ortho_alpha_image(i,j) = PixelGrayA<uint8>(uint8(ortho_image(i,j)*255),255);
          }
        }
      }
      write_image(out_prefix + "-DRG.tif", drg_ortho_alpha_image);


      // Write out the offset files
      BBox<float,3> dem_bbox = rasterizer.bounding_box();
      std::cout << "Offset: " << dem_bbox.min().x()/dft.dem_spacing << "   " << dem_bbox.max().y()/dft.dem_spacing << "\n";
      std::string dem_offset_filename = out_prefix + "-DEM.offset";
      std::string drg_offset_filename = out_prefix + "-DRG.offset";
      FILE* offset_file = fopen(dem_offset_filename.c_str(), "w");
      fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/dft.dem_spacing), -int(dem_bbox.max().y()/dft.dem_spacing));
      fclose(offset_file);
      offset_file = fopen(drg_offset_filename.c_str(), "w");
      fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/dft.dem_spacing), -int(dem_bbox.max().y()/dft.dem_spacing));
      fclose(offset_file);
    }
  }

  free (hd.cmd_name);
  return(EXIT_SUCCESS);
}

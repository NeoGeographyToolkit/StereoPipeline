//
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
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

//
// linescan_stereo.h
// 
// Stereo Pipeline for processing linescan stereo imagery from MGS and MRO.
// 
// Created 13 July 2005 by mbroxton.
//
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
#include <vw/Image/Filter.h>
#include <vw/Core/Exception.h>
#include <vw/Image/Transform.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;

#include "stereo.h"
#include "file_lib.h"
#include "mask.h"
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "SurfaceNURBS.h"
#include "StereoEngine.h"
#include "MOC/Ephemeris.h"
#include "MOC/Metadata.h"
#include "MOC/MOLA.h"
#include "Spice.h" 
#include "OrthoRasterizer.h"
#include "DEM.h"

#include <vector> 
#include <string>
#include <algorithm>
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
  StereoEngine stereo_engine; // Does most of the heavy lifting

  //Image buffers 
  ImageView<PixelGray<float> > Limg, Rimg;
  ImageView<PixelGray<float> > texture;
  ImageView<PixelDisparity<float> > disparity_map;
  ImageView<bool> Lmask, Rmask;
  ImageView<Vector3> point_image;  
  Matrix<double> align_matrix;
  
  // Set the Vision Workbench debug level
  set_debug_level(InfoMessage);

  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use
  // here to specify the type, size, help string, etc, of the command
  // line arguments.
  int entry_point;
  std::string stereo_default_filename;
  std::string description_tab_filename;
  std::string in_file1, in_file2, cam_file1, cam_file2;
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("description-file,d", po::value<std::string>(&description_tab_filename)->default_value("./description.tab"), "Explicitly specify the tabulated description file to use.")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("left-camera-model", po::value<std::string>(&cam_file1), "Left Camera Model File")
    ("right-camera-model", po::value<std::string>(&cam_file2), "Right Camera Model File")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("left-camera-model", 1);
  positional_options_desc.add("right-camera-model", 1);
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
      !vm.count("left-camera-model") || !vm.count("right-camera-model") || 
      !vm.count("output-prefix")) {
    std::cout << "\nUsage: stereo [options] <Left_input_image> <Right_input_image> <Left_camera_file> <Right_camera_file> <output_file_prefix>\n"
              << "	the extensions are automaticaly added to the output files\n"
              << "	the parameters should be in stereo.default\n\n"
              << " Note: The linescan stereo pipeline expects the u dimension (columns)\n"
              << "  of the image to correspond to the perspective projection axis of the\n"
              << "  pushbroom imager.  The v dimension (rows) correspond to the orthographic\n"
              << "  axis; that is, the distinct scanlines as the pushbroom imagers moves as\n"
              << "  a function of time.\n\n";
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
      cout << "\nLoading image " << in_file1 << " as primary image:\n";
      read_image(Limg, in_file1);
      cout << "\nLoading image " << in_file2 << " as secondary image:\n";
      read_image(Rimg, in_file2);
      cout << endl;
    } catch (IOErr& e) { 
      cout << e.what() << "\n";
      cout << "\n Could not read one or more input files. Exiting.\n\n";
      exit(0);
    }

    // Normalize the images, stretching the contrast if necessary.
    Limg = normalize(copy(Limg));
    Rimg = normalize(copy(Rimg));
    write_image(out_prefix + "-input-L.tif", channel_cast_rescale<uint8>(Limg));
    write_image(out_prefix + "-input-R.tif", channel_cast_rescale<uint8>(Rimg));

    texture = vw::copy(Limg);
    
    if(execute.w_texture) {
      try{
        write_image(out_prefix + "-T.jpg", texture);
      } catch (vw::IOErr &e) {
        cout << e.what() << "\n";
        cout << "\nWARNING: Could not write texture buffer to disk.\n";
      }
    }

    /*******************************************
     * Read description.tab file ( if available)
     *******************************************/ 
    
    MOCImageMetadata moc_metadata_1(in_file1);
    MOCImageMetadata moc_metadata_2(in_file2);

    // Read in the tabulated description entry
    std::cout << "Attempting to read data from " << description_tab_filename << ".\n";
    moc_metadata_1.read_tabulated_description(description_tab_filename);
    moc_metadata_2.read_tabulated_description(description_tab_filename);
    moc_metadata_1.write_viz_site_frame(out_prefix);

    try {
      cout << "Attempting to read MOC telemetry from supplementary ephemeris files... " << flush;
      moc_metadata_1.read_ephemeris_supplement(cam_file1);
      moc_metadata_2.read_ephemeris_supplement(cam_file2);
    } catch (EphemerisErr &e) {
      cout << "Failed to open the supplementary ephemeris file:\n\t";
      cout << e.what() << "\n";
      cout << "\tWarning: Proceeding without supplementary ephemeris information.\n";
    }
    
    // If the spice kernels are available, try to use them directly to
    // read in MOC telemetry.  
    try {
      cout << "Attempting to read MOC telemetry from SPICE kernels... " << flush;
      load_moc_kernels();
      moc_metadata_1.read_spice_data();
      moc_metadata_2.read_spice_data();
      cout << "success.\n";
    } catch (spice::SpiceErr &e) {
      cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
    }


    // Image Alignment
    cout << "\nPerforming image alignment.\n";
    ImageView<PixelGray<float> > adj_image, fully_adj_image;
    
    // Align images using keypoint homography.
    if (execute.keypoint_alignment && execute.do_alignment) {
      cout << "\nAligning images using KEYPOINT based automated alignment technique\n";
      align_matrix = keypoint_align(Rimg, Limg);
      write_matrix(out_prefix + "-align.exr", align_matrix);
      fully_adj_image = transform(Rimg, HomographyTransform(align_matrix),
                                  Limg.cols(), Limg.rows());
      Rimg = fully_adj_image;
      
    // Align images using ephemeris data 
    } else if (execute.ephemeris_alignment && execute.do_alignment) {
      cout << "\nERROR: MOC Ephemeris based image alignment is currently disabled. -mbroxton\n\n";
      exit(1);
      //      try {
//         cout << "\nAligning images using EPHEMERIS based automated alignment technique\n";
//         align_matrix = MOCImageAlign(Limg, Rimg, adj_image, fully_adj_image,
//                                     moc_metadata_1, moc_metadata_2,
//                                     dft.ephem_align_kernel_x, dft.ephem_align_kernel_y,
//                                     dft.ephem_align_kernel_width, dft.ephem_align_kernel_height,
//                                     out_prefix);
//       } catch (MOCEphemerisErr &e) {
//         cout << "Could not perform ephemeris-based alignment.\n\t";
//         cout << e.what() << "\n";
//         ExitFailure("\nExiting.\n\n");
//       }
//       Rimg = fully_adj_image;
//       write_matrix(out_prefix + "-align.exr", align_matrix);

    // Fall back on trying to read the alignment matrix manually.  
    } else if (execute.do_alignment) {      
      cout << "No automated technique for image registration has been selected.\n" <<
              "Attempting to find alignment transform in " << "align.exr.\n";
      try {
        read_matrix(align_matrix, "align.exr");
        cout << "Alignment Matrix:\n";
        cout << align_matrix << endl;
        
        // Take the transformation and apply it now to the 
        // original secondary image.
        vw::Matrix<double> invH = vw::math::inverse(align_matrix);
        fully_adj_image = vw::transform(Rimg, HomographyTransform(invH), Limg.cols(), Limg.rows()); 
        Rimg = fully_adj_image;
        
      } catch (IOErr&) { 
        cout << "\n\nFailed to open alignment file.  Could not align images.\n";
        cout << "\nExiting.\n\n";
        exit(1);
      }  
    }

    // Write the alignment matrix and aligned images to files
    write_image(out_prefix + "-L.tif", channel_cast<uint8>(Limg*255));
    write_image(out_prefix + "-R.tif", channel_cast<uint8>(Rimg*255));
   
    // Mask any pixels that are black and 
    // appear on the edges of the image.
    cout << "\nGenerating image masks...";
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
    
    /* 
     * If we are jump-starting the code in the post-correlation phase,
     * we must initialize the disparity map object here with the 
     * twe files that were supplied at the command line.  Right now
     * this is purely for debugging.
     */
    if (entry_point == CORRELATION) {
      try {
        cout << "\nStarting at the CORRELATION stage.\n";
        cout << "\nLoading image " << out_prefix + "-L.tif" << " as primary image:\n";
        read_image(Limg, out_prefix + "-L.tif");
        cout << "\nLoading image " << out_prefix + "-R.tif" << " as secondary image:\n";
        read_image(Rimg, out_prefix + "-R.tif");
        cout << "\nLoading image " << out_prefix + "-T.jpg" << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        cout << endl;
      } catch (IOErr&) { 
        cout << "\n Unable to start code at the correlation stage.  Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    // Call the stereo engine to perform the correlation
    std::cout << "\n" << stereo_engine << "\n";
    disparity_map = stereo_engine.correlate(Limg, Rimg);
    
    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image masks...");
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

    /* 
     * If we are jump-starting the code in the post-correlation phase,
     * we must initialize the disparity map object here with the 
     * twe files that were supplied at the command line.  Right now
     * this is purely for debugging.
     */
    if (entry_point == FILTERING) {
      /* Read the input files */
      try {
        cout << "\nStarting at the FILTERING stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-D.exr" << " as disparity map image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-D.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        std::cout << std::endl;
      } catch (IOErr&) { 
        cout << "\n Unable to start code at the filtering stage.  Could not read input files. Exiting.\n\n";
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

    stereo_engine.interpolate(disparity_map);

    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image mask...");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }

    // Write the filtered disp map 
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp,true);
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
        cout << "\n Unable to start at point cloud stage.\n\tCould not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    /*******************************************
     * Read description.tab file ( if available)
     *******************************************/ 
    MOCImageMetadata moc_metadata_1(in_file1);
    MOCImageMetadata moc_metadata_2(in_file2);

    // Read in the tabulated description entry
    std::cout << "Attempting to read data from " << description_tab_filename << ".\n";
    moc_metadata_1.read_tabulated_description(description_tab_filename);
    moc_metadata_2.read_tabulated_description(description_tab_filename);
    moc_metadata_1.write_viz_site_frame(out_prefix);

    try {
      cout << "Attempting to read MOC telemetry from supplementary ephemeris files... " << flush;
      moc_metadata_1.read_ephemeris_supplement(cam_file1);
      moc_metadata_2.read_ephemeris_supplement(cam_file2);
    } catch (EphemerisErr &e) {
      cout << "Failed to open the supplementary ephemeris file:\n\t";
      cout << e.what() << "\n";
      cout << "\tWarning: Proceeding without supplementary ephemeris information.\n";
    }
    
    // If the spice kernels are available, try to use them directly to
    //    read in MOC telemetry.  
    try {
      cout << "Attempting to read MOC telemetry from SPICE kernels... " << flush;
//       load_moc_kernels();
//       moc_metadata_1.read_spice_data();
//       moc_metadata_2.read_spice_data();
      cout << "success.\n";
    } catch (spice::SpiceErr &e) {
      cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
    }
    
    // Create the camera models and stereo models
    camera::OrbitingPushbroomModel left_camera_model = moc_metadata_1.camera_model();
    camera::OrbitingPushbroomModel right_camera_model = moc_metadata_2.camera_model();
    std::cout << left_camera_model << "\n";
    std::cout << right_camera_model << "\n";

    // We used a homography to line up the images, we may want 
    // to generate pre-alignment disparities before passing this information
    // onto the camera model in the next stage of the stereo pipeline.
    if (execute.do_alignment) {
      try {
        read_matrix(align_matrix, out_prefix + "-align.exr");
        std::cout << "Alignment Matrix: " << align_matrix << "\n";
      } catch (vw::IOErr &e) {
        std::cout << "Could not read in aligment matrix: " << out_prefix << "-align.exr.  Exiting. \n\n";
        exit(1);
      }

      vw::Matrix<double> inv_align_matrix = inverse(align_matrix);
      disparity_linear_transform(disparity_map, inv_align_matrix);
    }

    // Write a VRML file that depicts the positions and poses of the
    // spacecraft in a scene with a large Mars ellipse.for reference.
    write_orbital_reference_model(out_prefix + "-OrbitViz.vrml", left_camera_model, right_camera_model);
    
    // Apply the stereo model.  This yields a image of 3D points in space.
    StereoModel stereo_model(left_camera_model, right_camera_model);
    std::cout << "Generating a 3D point cloud.   \n";
    ImageView<double> error;
    point_image = stereo_model(disparity_map, error);
    write_image(out_prefix + "-error.png", normalize(error));

//     // Do the MOLA comparison
//     ImageView<Vector3> lat_lot_alt = cartography::xyz_to_latlon(point_image);
//     do_mola_comparison(lat_lot_alt, moc_metadata_1, out_prefix);

    // Write out the results to disk
    write_image(out_prefix + "-PC.exr", channels_to_planes(point_image));

    // Free up memory for the next stage
    disparity_map.reset();  
  }

  /************************************************************************************/
  /*                            dot-cloud to wire mesh                                */
  /************************************************************************************/

  if(entry_point <= WIRE_MESH) {
    
    /* 
     * If we are jump-starting the code in the wire mesh phase, we want 
     * to simply read in the point cloud from disk and start from here.
     */
    if (entry_point == WIRE_MESH) {
      /* Read the input files */
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
      } catch (IOErr&) { 
        cout << "\nFailed to start at wire mesh phase.\n\tCould not read input files. Exiting.\n\n";
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

    // Map the pointcloud coordinates onto the MARS areoid 
    if (execute.write_dem) {
      cout << "Reprojecting points and subtracting the Mars areoid.\n";
      ImageView<Vector3> lat_lon_alt = cartography::xyz_to_latlon(point_image);

      // Subtract off the equitorial radius in preparation for writing the data out to a DEM
      for (int i = 0; i < lat_lon_alt.cols(); i++) 
        for (int j = 0; j < lat_lon_alt.rows(); j++) 
          if (lat_lon_alt(i,j) != Vector3()) 
            lat_lon_alt(i,j).z() -= MOLA_PEDR_EQUATORIAL_RADIUS;

      // Write out the DEM, texture, and extrapolation mask
      // as georeferenced files.
      ImageView<double> dem_texture = vw::select_channel(lat_lon_alt, 2);
      vw::cartography::OrthoRasterizer<Vector3, double> rasterizer(lat_lon_alt, dem_texture, true);
      ImageView<PixelGray<float> > ortho_image = rasterizer.rasterize();

      vw::Matrix<double,3,3> affine = rasterizer.geo_transform();
      vw::cartography::GeoReference geo;
      geo.set_transform(affine);

      write_GMT_script(out_prefix, ortho_image.cols(), ortho_image.rows(), 
                       *(std::min_element(ortho_image.begin(), ortho_image.end())), 
                       *(std::max_element(ortho_image.begin(), ortho_image.end())), geo);
      write_georeferenced_image(out_prefix+"-DEM.dem", ortho_image, geo);
      write_image(out_prefix + "-DEM-debug.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Write out a georeferenced orthoimage of the DTM
      rasterizer = vw::cartography::OrthoRasterizer<Vector3, double>(lat_lon_alt, select_channel(texture, 0), true);
      rasterizer.use_minz_as_default = false;
      ortho_image = rasterizer.rasterize();
      write_georeferenced_image(out_prefix+"-DRG.dem", ortho_image, geo);
      write_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Write out a georeferenced orthoimage of the pixel extrapolation mask
      ImageView<PixelGray<float> > extrapolation_mask;
      try {
        read_image(extrapolation_mask, out_prefix + "-ExMap.png");
        rasterizer = vw::cartography::OrthoRasterizer<Vector3, double>(lat_lon_alt, select_channel(extrapolation_mask, 0),true);
        rasterizer.use_minz_as_default = false;
        ortho_image = rasterizer.rasterize();
        write_image(out_prefix + "-ExMap.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));
      } catch (IOErr &e) {
        std::cout << "Warning: an error occurred when reading the cached extrapolation map \"" << (out_prefix + "-ExMap.png") << "\" on disk.";
      }
    }
  }
  free (hd.cmd_name);
  return(EXIT_SUCCESS);
}

/*******/
/* END */
/*******/

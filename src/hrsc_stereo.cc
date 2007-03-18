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
// hrsc_stereo.h
// 
// Stereo Pipeline for processing linescan stereo imagery from Mars
// Express.
// 
// Created 31 October 2006 by mbroxton.
//
#include <boost/shared_ptr.hpp>
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
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;

#include "stereo.h"
#include "file_lib.h"
#include "mask.h"
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "StereoEngine.h"
#include "HRSC/HRSC.h"
#include "MOC/Metadata.h"
#include "Spice.h" 
#include "OrthoRasterizer.h"
#include "SIFT.h"
#include "DEM.h"

#include <vector> 
#include <string>
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
  StereoEngine stereo_engine; // Does most of the heavy lifting

  // Set the Vision Workbench debug level
  set_debug_level(InfoMessage+1);
  
  /*************************************/
  /* Parsing of command line arguments */
  /*************************************/

  // Boost has a nice command line parsing utility, which we use
  // here to specify the type, size, help string, etc, of the command
  // line arguments.
  int entry_point;
  std::string stereo_default_filename;
  std::string description_tab_filename;
  std::string in_file1, in_file2;
  std::string extori_file1, extori_file2;  
  std::string out_prefix;

  po::options_description visible_options("Options");
  visible_options.add_options()
    ("help,h", "Display this help message")
    ("stereo-file,s", po::value<std::string>(&stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("description-file,d", po::value<std::string>(&description_tab_filename)->default_value("./description.tab"), "Explicitly specify the tabulated description file to use.")
    ("entry-point,e", po::value<int>(&entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)")
    ("multiresolution,m", "Use the prototype multiresolution correlator instead of the old area-based correlator");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("left-input-image", po::value<std::string>(&in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(&in_file2), "Right Input Image")
    ("output-prefix", po::value<std::string>(&out_prefix), "Prefix for output filenames")
    ("left-extori-file", po::value<std::string>(&extori_file1), "Left Extori File")
    ("right-extori-file", po::value<std::string>(&extori_file2), "Right Extori File");
  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("output-prefix", 1);
  positional_options_desc.add("left-extori-file", 1);
  positional_options_desc.add("right-extori-file", 1);

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
    std::cout << "\nUsage: hrsc_stereo [options] <Left_input_image> <Right_input_image> <output_file_prefix> [left_extori_file] [right_extori_file]\n"
              << "\n Optional parameters are in [brackets]. "
              << " Stereo parameters should be in stereo.default\n\n"
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
  stereo_engine.use_multiresolution_correlator = (vm.count("multiresolution")>0);
  
  stereo_engine.do_slog = execute.slog;
  stereo_engine.do_log = execute.log;
  stereo_engine.slog_stddev = dft.slogW;
  
  stereo_engine.do_cleanup = true;
  stereo_engine.cleanup_passes = 2;
  stereo_engine.cleanup_vertical_half_kernel = dft.rm_v_half_kern;
  stereo_engine.cleanup_horizontal_half_kernel = dft.rm_h_half_kern;
  stereo_engine.cleanup_min_matches = dft.rm_min_matches;
  stereo_engine.cleanup_rm_threshold = (float)dft.rm_treshold;
  
  stereo_engine.do_nurbs_hole_filling = execute.fill_holes_NURBS;
  stereo_engine.nurbs_iterations = 10;

  // Load the two images
  cout << "Loading image " << in_file1 << " as primary image.\n";
  DiskImageView<PixelGray<float> > left_disk_image(in_file1);
  cout << "Loading image " << in_file2 << " as secondary image.\n";
  DiskImageView<PixelGray<float> > right_disk_image(in_file2);
  cout << endl;

  // Build the input prefix path by removing the filename suffix
  std::string in_prefix1 = prefix_from_filename(in_file1);
  std::string in_prefix2 = prefix_from_filename(in_file2);

  boost::shared_ptr<StereoImageMetadata> metadata1, metadata2;

  // Initialize the HRSC metadata object
  HRSCImageMetadata hrsc_metadata1(in_file1);
  HRSCImageMetadata hrsc_metadata2(in_file2);
  try {
    std::cout << "Loading HRSC Metadata.\n";
    hrsc_metadata1.read_line_times(in_prefix1 + ".txt");
    hrsc_metadata2.read_line_times(in_prefix2 + ".txt");
    hrsc_metadata1.read_ephemeris_supplement(in_prefix1 + ".sup");
    hrsc_metadata2.read_ephemeris_supplement(in_prefix2 + ".sup");
    
    if (vm.count("left-extori-file")) 
      hrsc_metadata1.read_extori_file(extori_file1,"S1");
    if (vm.count("right-extori-file")) 
      hrsc_metadata2.read_extori_file(extori_file2,"S2");
    
  } catch (IOErr &e) {
    std::cout << "An error occurred when loading metadata:\n\t" << e.what();
    std::cout << "\nExiting.\n\n";
    exit(1);
  }

  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {
    
    // Image Alignment
    //
    // Images are aligned by computing interest points, matching
    // them using a standard 2-Norm nearest-neighor metric, and then
    // rejecting outliers by fitting a similarity between the
    // putative matches using RANSAC.
    
    // Interest points are matched in image chunk of <= 2048x2048
    // pixels to conserve memory.
    std::cout << "\nInterest Point Detection\n";
    static const int MAX_KEYPOINT_IMAGE_DIMENSION = 2048;
    std::vector<InterestPoint> ip1 = interest_points(left_disk_image, LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
    std::vector<InterestPoint> ip2 = interest_points(right_disk_image, LoweDetector(), MAX_KEYPOINT_IMAGE_DIMENSION);
    
    // The basic interest point matcher does not impose any
    // constraints on the matched interest points.
    std::cout << "\nInterest Point Matching\n";
    InterestPointMatcher<L2NormMetric,NullConstraint> matcher;
    std::vector<InterestPoint> matched_ip1, matched_ip2;
    matcher.match(ip1, ip2, matched_ip1, matched_ip2);
    std::cout << "Found " << matched_ip1.size() << " putative matches.\n";
    
    // RANSAC is used to fit a similarity transform between the
    // matched sets of points
    Matrix<double> align_matrix = ransac(matched_ip2, matched_ip1, 
                                         vw::math::SimilarityFittingFunctor(),
                                         KeypointErrorMetric());
    write_matrix(out_prefix + "-align.exr", align_matrix);
    ImageView<PixelGray<float> > Rimg = transform(right_disk_image, HomographyTransform(align_matrix),
                                                  left_disk_image.cols(), left_disk_image.rows());
    ImageView<PixelGray<float> > Limg = left_disk_image;
    write_image(out_prefix + "-R.tif", channel_cast_rescale<uint8>(Rimg));
    write_image(out_prefix + "-L.tif", channel_cast_rescale<uint8>(Limg));
    write_image(out_prefix + "-T.jpg", channel_cast_rescale<uint8>(Limg));
  
    // Mask any pixels that are black and 
    // appear on the edges of the image.
    if(execute.w_mask) {  
      cout << "\nGenerating image masks...";
      int mask_buffer = std::max(dft.h_kern, dft.v_kern);
      ImageView<bool> Lmask = disparity::generate_mask(Limg, mask_buffer);
      ImageView<bool> Rmask = disparity::generate_mask(Rimg, mask_buffer);
      printf("Done.\n");
      write_mask(Lmask, out_prefix + "-lMask.png");
      write_mask(Rmask, out_prefix + "-rMask.png");
    }
  }
  /*********************************************************************************/
  /*                            correlation step                                   */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    if (entry_point == CORRELATION) 
        cout << "\nStarting at the CORRELATION stage.\n";

    try {
      ImageView<PixelGray<float> > Limg, Rimg;
      cout << "\nLoading image " << out_prefix + "-L.tif" << " as primary image:\n";
      read_image(Limg, out_prefix + "-L.tif");
      cout << "\nLoading image " << out_prefix + "-R.tif" << " as secondary image:\n";
      read_image(Rimg, out_prefix + "-R.tif");
      cout << endl;
    
      // Call the stereo engine to perform the correlation
      std::cout << "\n" << stereo_engine << "\n";
      ImageView<PixelDisparity<float> > disparity_map = stereo_engine.correlate(Limg, Rimg);
    
      // Apply the Mask to the disparity map 
      if(execute.apply_mask){
        printf("\nApplying image masks...");
        ImageView<bool> Lmask = read_mask(out_prefix + "-lMask.png");
        ImageView<bool> Rmask = read_mask(out_prefix + "-rMask.png");
        disparity::mask(disparity_map, Lmask, Rmask);
        printf("Done.\n");
      }
    
      double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
      disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp,true);
      write_image( out_prefix + "-DH.jpg", normalize(clamp(select_channel(disparity_map,0), min_h_disp, max_h_disp)));
      write_image( out_prefix + "-DV.jpg", normalize(clamp(select_channel(disparity_map,1), min_v_disp, max_v_disp)));
      write_image( out_prefix + "-D.exr", channels_to_planes(disparity_map) );
    } catch (IOErr&) { 
      cout << "\n A File IO error occurred during the correlation stage. Exiting.\n\n";
      exit(0);
    }
  }

  /***************************************************************************/
  /*                      Disparity Map Filtering                            */
  /***************************************************************************/

  if(entry_point <= FILTERING) {
    if (entry_point == FILTERING)
        cout << "\nStarting at the FILTERING stage.\n";
    
    try {
      std::cout << "\nUsing image " << out_prefix + "-D.exr" << " as disparity map image.\n";
      DiskImageView<PixelDisparity<float> > disparity_map_file(out_prefix + "-D.exr");
      ImageView<PixelDisparity<float> > disparity_map = disparity_map_file;
      
      // The multiresolution correlate does not tend to have outliers(!)
      if (!vm.count("multiresolution"))
        stereo_engine.filter(disparity_map);
      else 
        vw::stereo::disparity::sparse_disparity_filter(disparity_map, 100, 0.5);
      
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
        ImageView<bool> Lmask = read_mask(out_prefix + "-lMask.png");
        ImageView<bool> Rmask = read_mask(out_prefix + "-rMask.png");
        disparity::mask(disparity_map, Lmask, Rmask);
        printf("Done.\n");
      }

      // Before we apply the inverse alignment, we quickly write out
      // some debug images showing the fully filtered disparity map.
      double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
      disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp,true);
      write_image( out_prefix + "-FH.jpg", normalize(clamp(select_channel(disparity_map,0), min_h_disp, max_h_disp)));
      write_image( out_prefix + "-FV.jpg", normalize(clamp(select_channel(disparity_map,1), min_v_disp, max_v_disp)));

      // We used a homography to line up the images, we may want 
      // to generate pre-alignment disparities before passing this information
      // onto the camera model in the next stage of the stereo pipeline.
      if (execute.do_alignment) {
        try {
          Matrix<double> align_matrix;
          read_matrix(align_matrix, out_prefix + "-align.exr");
          std::cout << "Alignment Matrix: " << align_matrix << "\n";
          vw::Matrix<double> inv_align_matrix = inverse(align_matrix);
          disparity_linear_transform(disparity_map, inv_align_matrix);
        } catch (vw::IOErr &e) {
          std::cout << "Could not read in aligment matrix: " << out_prefix << "-align.exr.  Exiting. \n\n";
          exit(1);
        }
      }
      
      // Write the filtered disp map 
      write_image( out_prefix + "-F.exr", channels_to_planes(disparity_map) );

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
      DiskImageView<PixelDisparity<float> > disparity_map_resource(out_prefix+"-F.exr");
      
      // Create the camera models and stereo models
      boost::shared_ptr<camera::CameraModel> left_camera_model(hrsc_metadata1.camera_model());
      boost::shared_ptr<camera::CameraModel> right_camera_model(hrsc_metadata2.camera_model());
      std::cout << left_camera_model << "\n";
      std::cout << right_camera_model << "\n";

      // Write a VRML file that depicts the positions and poses of the
      // spacecraft in a scene with a large Mars ellipse.for reference.
      write_orbital_reference_model(out_prefix + "-OrbitViz.vrml", *left_camera_model, *right_camera_model);
    
      // Apply the stereo model.  This yields a image of 3D points in
      // space.  We build this image and immediately write out the
      // results to disk.
      std::cout << "Generating a 3D point cloud.   \n";
      StereoView<ImageView<PixelDisparity<float> > > stereo_image(disparity_map_resource, *left_camera_model, *right_camera_model);
      write_image(out_prefix + "-PC.exr", channels_to_planes(stereo_image));
      
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
          mesh_maker.write_osg(out_prefix+".ive", out_prefix+"-T.jpg");
        } else {
          mesh_maker.build_simple_mesh(point_image, dft.nff_h_step, dft.nff_v_step);
          mesh_maker.write_osg(out_prefix+".ive", out_prefix+"-T.jpg");
        }
        
        if(execute.inventor){
          mesh_maker.write_inventor(out_prefix+".iv", out_prefix+"-T.jpg", true);
        }
        if(execute.vrml){
          mesh_maker.write_vrml(out_prefix+".vrml", out_prefix+"-T.jpg");
        }
      }

      // Map the pointcloud coordinates onto the MARS areoid 
      if (execute.write_dem) {

        // Switch to lat, lon, radius and free up the memory that had
        // been used for the cartesian coordinates.
        cout << "Reprojecting points and subtracting the Mars areoid.\n";
        ImageView<Vector3> lon_lat_alt = cartography::xyz_to_lon_lat_radius(point_image);

        // Subtract off the equitorial radius in preparation for writing the data out to a DEM
        for (int j = 0; j < lon_lat_alt.rows(); j++) 
          for (int i = 0; i < lon_lat_alt.cols(); i++) 
            if (lon_lat_alt(i,j) != Vector3()) 
              lon_lat_alt(i,j).z() -= MOLA_PEDR_EQUATORIAL_RADIUS;

        // Reproject into sinusoidal projection
        vw::cartography::OrthoRasterizer<Vector3> bbox_computer(lon_lat_alt, 1);
        double min_longitude = bbox_computer.bounding_box().min().x();
        double max_longitude = bbox_computer.bounding_box().max().x();
        double sinusoidal_lon0 = roundf((max_longitude + min_longitude)/2);
        if (min_longitude == max_longitude) 
          throw LogicErr() << "Error while reprojecting 3D points.  Georgraphic region has zero width.";
        
        // Set up the datum
        vw::cartography::GeoDatum mars_datum;
        mars_datum.name() = "IAU2000 Mars Spheroid";
        mars_datum.spheroid_name() = "IAU2000 Mars Spheroid";
        mars_datum.meridian_name() = "Mars Prime Meridian";
        mars_datum.semi_major_axis() = MOLA_PEDR_EQUATORIAL_RADIUS;
        mars_datum.semi_minor_axis() = MOLA_PEDR_EQUATORIAL_RADIUS;
        
        // Note: the affine transform doesn't matter when using
        // reproject_point_image, so we simply don't specify anything
        // here.
        vw::cartography::GeoReference src_geo(mars_datum);
        vw::cartography::GeoReference dst_geo(mars_datum);
        dst_geo.set_sinusoidal(sinusoidal_lon0);

        std::cout << "Reprojecting into sinusoidal projection... " << std::flush;
        cartography::reproject_point_image(lon_lat_alt, src_geo, dst_geo);
        std::cout << "done.\n";

        // Rasterizing the third component (radius) of the point cloud.  
        vw::cartography::OrthoRasterizer<Vector3> rasterizer(lon_lat_alt, dft.dem_spacing);
        
        // Rasterize the DEM.
        ImageView<PixelGray<float> > ortho_image = rasterizer(vw::select_channel(lon_lat_alt,2));
        

        // Georeferencing 
        //
        // 1. Set up the affine transform
        dst_geo.set_transform(rasterizer.geo_transform());

        // 2. Save various data products
        write_GMT_script(out_prefix, ortho_image.cols(), ortho_image.rows(), 
                         *(std::min_element(ortho_image.begin(), ortho_image.end())), 
                         *(std::max_element(ortho_image.begin(), ortho_image.end())), 
                         1, // Scale factor: empirically determined. 
                         dst_geo);
        write_georeferenced_image(out_prefix+"-DEM.dem", ortho_image, dst_geo);
        write_ENVI_header(out_prefix+"-DEM.hdr", rasterizer.default_value(), ortho_image, dst_geo);
        write_image(out_prefix + "-DEM-debug.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

        // Write out a georeferenced orthoimage of the DTM
        ImageView<PixelGray<float> > texture;
        std::cout << "\nLoading image " << out_prefix + "-L.tif" << " as texture image:\n";
        read_image(texture, out_prefix + "-L.tif");
        std::cout << std::endl;
        rasterizer.use_minz_as_default = false;
        rasterizer.set_default_value(0);
        ortho_image = rasterizer(vw::select_channel(texture,0));
        texture.reset();
        write_georeferenced_image(out_prefix+"-DRG.tif", channel_cast_rescale<uint8>(normalize(ortho_image)), dst_geo);
        write_georeferenced_image(out_prefix+"-DRG.dem", ortho_image, dst_geo);

        // Write out a georeferenced orthoimage of the pixel extrapolation mask
        ImageView<PixelGray<float> > extrapolation_mask;
        try {
          read_image(extrapolation_mask, out_prefix + "-ExMap.png");
          rasterizer.use_minz_as_default = false;
          rasterizer.set_default_value(0);
          ortho_image = rasterizer(select_channel(extrapolation_mask, 0));
          write_image(out_prefix + "-ExMap.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));
        } catch (IOErr &e) {
          std::cout << "Warning: an error occurred when reading the cached extrapolation map \"" << (out_prefix + "-ExMap.png") << "\" on disk.";
        }
      }
    

    } catch (IOErr&) { 
      cout << "\nFailed to start at wire mesh phase.\n\tCould not read input files. Exiting.\n\n";
      exit(0);
    }


  }
  free (hd.cmd_name);
  return(EXIT_SUCCESS);
}

/*******/
/* END */
/*******/

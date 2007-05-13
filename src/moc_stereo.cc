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

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>
#include <vw/InterestPoint.h>
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

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
#include "SIFT.h"
#include "DEM.h"
#include "BundleAdjust.h"

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
  ImageView<Vector3> point_image;  
  Matrix<double> align_matrix;
  
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
  stereo_engine.use_multiresolution_correlator = (vm.count("multiresolution")>0);
  
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
  
  // Build the input prefix path by removing the filename suffix
  std::string in_prefix1 = prefix_from_filename(in_file1);
  std::string in_prefix2 = prefix_from_filename(in_file2);
  
  MOCImageMetadata moc_metadata_1(in_file1);
  MOCImageMetadata moc_metadata_2(in_file2);
  
  // Read in the tabulated description entry
  std::cout << "Attempting to read data from " << description_tab_filename << ".\n";
  moc_metadata_1.read_tabulated_description(description_tab_filename);
  moc_metadata_2.read_tabulated_description(description_tab_filename);
  moc_metadata_1.write_viz_site_frame(out_prefix);
  
  try {
    cout << "Attempting to read MOC telemetry from supplementary ephemeris files... " << flush;
    moc_metadata_1.read_ephemeris_supplement(in_prefix1 + ".sup");
    moc_metadata_2.read_ephemeris_supplement(in_prefix2 + ".sup");
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

  /*********************************************************************************/
  /*                            preprocessing step                                 */
  /*********************************************************************************/
  if (entry_point <= PREPROCESSING) {

    // Load the two images
    cout << "Opening image " << in_file1 << " as primary image.\n";
    DiskImageView<PixelGray<float> > left_disk_image(in_file1);
    cout << "Opening image " << in_file2 << " as secondary image.\n";
    DiskImageView<PixelGray<float> > right_disk_image(in_file2);
    cout << endl;

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
    LoweDetector detector;
    std::vector<InterestPoint> ip1 = interest_points(left_disk_image, detector, MAX_KEYPOINT_IMAGE_DIMENSION);
    std::vector<InterestPoint> ip2 = interest_points(right_disk_image, detector, MAX_KEYPOINT_IMAGE_DIMENSION);
    
    // The basic interest point matcher does not impose any
    // constraints on the matched interest points.
    std::cout << "\nInterest Point Matching\n";
    InterestPointMatcher<L2NormMetric,NullConstraint> matcher;
    std::vector<InterestPoint> matched_ip1, matched_ip2;
    matcher.match(ip1, ip2, matched_ip1, matched_ip2);
    std::cout << "Found " << matched_ip1.size() << " putative matches.\n";
    
    // RANSAC is used to fit a similarity transform between the
    // matched sets of points
    align_matrix = ransac(matched_ip2, matched_ip1, 
                          vw::math::SimilarityFittingFunctor(),
                          KeypointErrorMetric());
    write_matrix(out_prefix + "-align.exr", align_matrix);
    ImageView<PixelGray<float> > aligned_right_image = 
      transform(right_disk_image, HomographyTransform(align_matrix), 
                left_disk_image.cols(), left_disk_image.rows());
    write_image(out_prefix + "-R.png", channel_cast_rescale<uint8>(normalize(aligned_right_image)));
    write_image(out_prefix + "-L.png", channel_cast_rescale<uint8>(normalize(left_disk_image)));
    if(execute.w_texture) 
      write_image(out_prefix + "-T.jpg", channel_cast_rescale<uint8>(normalize(left_disk_image)));

    // Mask any pixels that are black and 
    // appear on the edges of the image.
    if(execute.w_mask) {  
      cout << "\nGenerating image masks...";
      int mask_buffer = std::max(dft.h_kern, dft.v_kern);

      ImageView<bool> mask = disparity::generate_mask(left_disk_image, mask_buffer);
      write_mask(mask, out_prefix + "-lMask.png");
      mask = disparity::generate_mask(aligned_right_image, mask_buffer);
      write_mask(mask, out_prefix + "-rMask.png");
      printf("Done.\n");
    }
  }

  /*********************************************************************************/
  /*                            correlation step                                   */
  /*********************************************************************************/
  if( entry_point <= CORRELATION ) {
    if (entry_point == CORRELATION)
      cout << "\nStarting at the CORRELATION stage.\n";
    
    // Load the images necessary for this stage
    DiskImageView<PixelGray<float> > left_disk_image(out_prefix + "-L.png");
    DiskImageView<PixelGray<float> > right_disk_image(out_prefix + "-R.png");

    // Call the stereo engine to perform the correlation
    std::cout << "\n" << stereo_engine << "\n";
    disparity_map = stereo_engine.correlate(left_disk_image, right_disk_image);
    
    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image masks...");
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.png");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.png");
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
        std::cout << std::endl;
      } catch (IOErr&) { 
        cout << "\n Unable to start code at the filtering stage.  Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }
    
    //    stereo_engine.filter(disparity_map);

    // Write out the extrapolation mask image
    if(execute.w_extrapolation_mask) {  
      ImageView<PixelRGB<float> > extrapolationMask = disparity::rgb_missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap-color.jpg", extrapolationMask);
      ImageView<PixelGray<float> > extrapolationMaskGray = disparity::missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap.png", extrapolationMaskGray);
    } 

    //    stereo_engine.interpolate(disparity_map);

    // Apply the Mask to the disparity map 
    if(execute.apply_mask){
      printf("\nApplying image mask...");
      DiskImageView<uint8> Lmask(out_prefix + "-lMask.png");
      DiskImageView<uint8> Rmask(out_prefix + "-rMask.png");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }

    // Write the filtered disp map 
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp, min_v_disp, max_v_disp,true);
    write_image( out_prefix + "-FH.jpg", normalize(clamp(select_channel(disparity_map,0), min_h_disp, max_h_disp)));
    write_image( out_prefix + "-FV.jpg", normalize(clamp(select_channel(disparity_map,1), min_v_disp, max_v_disp)));
    write_image( out_prefix + "-F.exr", disparity_map);
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

    
    // Create the camera models and stereo models
    camera::OrbitingPushbroomModel left_camera_model = *((camera::OrbitingPushbroomModel*)(moc_metadata_1.camera_model()));
    camera::OrbitingPushbroomModel right_camera_model = *((camera::OrbitingPushbroomModel*)(moc_metadata_2.camera_model()));
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

    // Remove pixels that are outside the bounds of the secondary image.
    DiskImageView<PixelGray<double> > right_image_temp(in_file2);
    disparity::remove_invalid_pixels(disparity_map, right_image_temp.cols(), right_image_temp.rows());

    // Write a VRML file that depicts the positions and poses of the
    // spacecraft in a scene with a large Mars ellipse.for reference.
    write_orbital_reference_model(out_prefix + "-OrbitViz.vrml", left_camera_model, right_camera_model);

    // Attempt bundle adjustment to correct camera pointing angles
    std::vector<Vector2> bundle1, bundle2;
    create_bundle_adjustment_pixel_list(disparity_map, 70, bundle1, bundle2);
    std::cout << "Bundle adjusting camera model using " << bundle1.size() << " pixel samples.\n";

    // Seed the optimization with a zero rotation quaternion.
    Vector<double,8> seed;
    seed[0] = 1.0;
    seed[4] = 1.0;
    CameraPointingOptimizeFunc optimize_functor(left_camera_model, right_camera_model, bundle1, bundle2);
    int status;
    Vector<double,8> quaternion_adjustment = vw::math::nelder_mead(optimize_functor, seed, status, true, 3, 1, 300);
    Vector4 v1 = normalize(subvector(quaternion_adjustment,0,4));
    Vector4 v2 = normalize(subvector(quaternion_adjustment,4,4));
    Quaternion<double> q1(v1[0], v1[1], v1[2], v1[3]);
    Quaternion<double> q2(v2[0], v2[1], v2[2], v2[3]);
    TransformedCameraModel bundle_adjusted_camera1(left_camera_model);
    TransformedCameraModel bundle_adjusted_camera2(right_camera_model);
    bundle_adjusted_camera1.set_rotation(q1);
    bundle_adjusted_camera2.set_rotation(q2);
    std::cout << "\t" << q1 << "\n";
    std::cout << "\t" << q2 << "\n";

    // Test code
    std::cout << "TEST -- Generating a 3D point cloud.   \n";
    DiskImageView<PixelDisparity<float> > disparity_map_resource(out_prefix+"-F.tif");
    StereoView<ImageView<PixelDisparity<float> > > stereo_image(disparity_map_resource, bundle_adjusted_camera1, bundle_adjusted_camera2);
    write_image(out_prefix + "-PC.tif", channels_to_planes(stereo_image));

    // apply the stereo model.  This yields a image of 3D points in space.
    StereoModel stereo_model(bundle_adjusted_camera1, bundle_adjusted_camera2);
    std::cout << "Generating a 3D point cloud.   \n";
    ImageView<double> error;
    point_image = stereo_model(disparity_map, error);
    write_image(out_prefix + "-error.png", normalize(error));

    // Do the MOLA comparison
    try {
      load_moc_kernels();
      ImageView<Vector3> lon_lat_alt = cartography::xyz_to_lon_lat_radius(point_image);
      do_mola_comparison(lon_lat_alt, moc_metadata_1, out_prefix);
    } catch (spice::SpiceErr &e) {
      cout << "Warning: an error occurred when reading SPICE information.  Skipping MOLA track comparison.\n";
    }

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
        mesh_maker.write_osg(out_prefix+".ive", out_prefix+"-T.jpg");
      } else {
        mesh_maker.build_simple_mesh(point_image, dft.nff_h_step, dft.nff_v_step);
        mesh_maker.write_osg(out_prefix+".ive", out_prefix+"-T.jpg");
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

      // Switch to lat, lon, radius and free up the memory that had
      // been used for the cartesian coordinates.
      cout << "Reprojecting points and subtracting the Mars areoid.\n";
      ImageView<Vector3> lon_lat_alt = cartography::xyz_to_lon_lat_radius(point_image);
      point_image.reset();

      // Subtract off the equitorial radius in preparation for writing the data out to a DEM
      for (int j = 0; j < lon_lat_alt.rows(); j++) 
        for (int i = 0; i < lon_lat_alt.cols(); i++) 
          if (lon_lat_alt(i,j) != Vector3()) 
            lon_lat_alt(i,j).z() -= MOLA_PEDR_EQUATORIAL_RADIUS;

      // Write out the DEM, texture, and extrapolation mask
      // as georeferenced files.
      ImageView<double> dem_texture = vw::select_channel(lon_lat_alt, 2);
      vw::cartography::OrthoRasterizer<Vector3> rasterizer(lon_lat_alt);
      ImageView<PixelGray<float> > ortho_image = rasterizer(vw::select_channel(lon_lat_alt,2));

      // Georeferencing 
      //
      // 1. Set up the affine transform
      vw::Matrix<double,3,3> affine = rasterizer.geo_transform();

      // 2. Set up the datum
      vw::cartography::GeoDatum mars_datum;
      mars_datum.name() = "IAU2000 Mars Spheroid";
      mars_datum.spheroid_name() = "IAU2000 Mars Spheroid";
      mars_datum.meridian_name() = "Mars Prime Meridian";
      mars_datum.semi_major_axis() = MOLA_PEDR_EQUATORIAL_RADIUS;
      mars_datum.semi_minor_axis() = MOLA_PEDR_EQUATORIAL_RADIUS;
      
      // 3. Create georeference and save
      vw::cartography::GeoReference geo(mars_datum, affine);
      write_GMT_script(out_prefix, ortho_image.cols(), ortho_image.rows(), 
                       *(std::min_element(ortho_image.begin(), ortho_image.end())), 
                       *(std::max_element(ortho_image.begin(), ortho_image.end())), 
                       30, // Scale factor: empirically determined. 
                       geo);
      write_georeferenced_image(out_prefix+"-DEM.dem", ortho_image, geo);
      write_ENVI_header(out_prefix+"-DEM.hdr", 0, ortho_image, geo);
      write_image(out_prefix + "-DEM-debug.tif", channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Write out a georeferenced orthoimage of the DTM
      rasterizer.use_minz_as_default = false;
      ortho_image = rasterizer(vw::select_channel(texture,0));
      write_georeferenced_image(out_prefix+"-DRG.tif", channel_cast_rescale<uint8>(normalize(ortho_image)), geo);
      write_georeferenced_image(out_prefix+"-DRG.dem", ortho_image, geo);

      // Write out a georeferenced orthoimage of the pixel extrapolation mask
      ImageView<PixelGray<float> > extrapolation_mask;
      try {
        read_image(extrapolation_mask, out_prefix + "-ExMap.png");
        rasterizer.use_minz_as_default = false;
        ortho_image = rasterizer(select_channel(extrapolation_mask, 0));
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

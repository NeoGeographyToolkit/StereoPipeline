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
// File: ctx_stereo.cc
// 
// Stereo Pipeline for processing linescan stereo imagery from MRO CTX.
// 
// Authors: M. Broxton and L. Edwards, after the original stereo.c by
// E. Zbinden
//

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>

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

#include "stereo.h"
#include "file_lib.h"
#include "mask.h"
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "SurfaceNURBS.h"
#include "StereoEngine.h"
#include "MRO/CTXEphemeris.h"		   // for load_ctx_kernels()
#include "MRO/CTXMetadata.h"		   // for read_spice_data()
#include "MRO/DiskImageResourceDDD.h"	   // support for Malin DDD image files
#include "Spice.h" 
#include "OrthoRasterizer.h"

#include <vector> 
#include <string>
#include <algorithm>			   // debugging

using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace std;

//=======================================================================
// Type declarations
//=======================================================================

// The stereo pipeline has several stages, which are enumerated below.

enum { PREPROCESSING = 0, 
       CORRELATION, 
       FILTERING, 
       POINT_CLOUD, 
       WIRE_MESH, 
       NUM_STAGES };

typedef vw::cartography::OrthoRasterizer<Vector3, double> Rasterizer;

enum { eHistogramSize = 32768 };

struct fill_histogram
{
  fill_histogram(unsigned int *counts) { m_counts = counts; }
  void operator()(float x)
  {
    m_counts[int(float(eHistogramSize - 1) * x)] += 1;
  }
  private:
    unsigned int *m_counts;
};

//=======================================================================
// Functions
//=======================================================================

void
parse_command_line_args(int argc, char *argv[],
			std::string* stereo_default_filename,
			std::string* description_tab_filename,
			std::string* in_file1,
			std::string* in_file2,
			std::string* cam_file1,
			std::string* cam_file2,
			std::string* out_prefix,
			int* entry_point)
{
  // Boost has a nice command line parsing utility, which we use
  // here to specify the type, size, help string, etc, of the command
  // line arguments.

  po::options_description visible_options("Options");

  visible_options.add_options()
    ("help,h", "Display this help message")
    ("stereo-file,s", po::value<std::string>(stereo_default_filename)->default_value("./stereo.default"), "Explicitly specify the stereo.default file to use. [default: ./stereo.default]")
    ("description-file,d", po::value<std::string>(description_tab_filename)->default_value("./description.tab"), "Explicitly specify the tabulated description file to use.")
    ("entry-point,e", po::value<int>(entry_point)->default_value(0), "Pipeline Entry Point (an integer from 1-4)");

  po::options_description positional_options("Positional Options");

  positional_options.add_options()
    ("left-input-image", po::value<std::string>(in_file1), "Left Input Image")
    ("right-input-image", po::value<std::string>(in_file2), "Right Input Image")
    ("output-prefix", po::value<std::string>(out_prefix), "Prefix for output filenames");

  po::positional_options_description positional_options_desc;
  positional_options_desc.add("left-input-image", 1);
  positional_options_desc.add("right-input-image", 1);
  positional_options_desc.add("output-prefix", 1);

  po::options_description all_options;
  all_options.add(visible_options).add(positional_options);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all_options).positional(positional_options_desc).run(), vm);
  po::notify(vm);

  // If the command line wasn't properly formed or the user requested
  // help, we print an usage message.
  if (vm.count("help") ||
      !vm.count("left-input-image") || !vm.count("right-input-image") || 
      !vm.count("output-prefix"))
  {
    std::cout
      << "\nUsage: stereo [options] <Left_input_image> "
      << "<Right_input_image> <Left_camera_file> <Right_camera_file> "
      << "<output_file_prefix>\n"
      << "	the extensions are automaticaly added to the output files\n"
      << "	the parameters should be in stereo.default\n\n"
      << " Note: The linescan stereo pipeline expects the u dimension "
      << "(columns)\n"
      << "  of the image to correspond to the perspective projection axis "
      << "of the\n"
      << "  pushbroom imager.  The v dimension (rows) correspond to the "
      << "orthographic\n"
      << "  axis; that is, the distinct scanlines as the pushbroom imagers "
      << "moves as\n"
      << "  a function of time.\n\n";
    std::cout << visible_options << std::endl;

    exit(1);
  }
}

static float
calculate_stretch(ImageView<PixelGray<float> > image)
{
  // For some reason it seems that we often have a bunch of junk up at
  // the high end... we pre-scale things (the junk get's pushed off
  // the top) so that the normalization does something reasonable

  unsigned int histogram[eHistogramSize];
  fill(histogram, histogram + eHistogramSize, 0);

  for_each(image.begin(), image.end(), fill_histogram(histogram));

  // find the peak non-zero value
  unsigned int max_count = 0;
  unsigned index_peak = 0;
  // We start the following at 1 since there typically are a bunch of
  // black pixels, and that kind of skews things
  for (int i = 1; i < eHistogramSize; i++)
    if (histogram[i] > max_count)
    {
      index_peak = i;
      max_count = histogram[i];
    }

  // discard the top 0.0005%
  unsigned int empty_threshold = int(float(max_count) * 0.000005);
  unsigned int max_value_index = 0;

  for (int i = eHistogramSize - 1; i > 0; --i)
  {
    if (histogram[i] > empty_threshold)
    {
      max_value_index = i;
      break;
    }
  }

  float max_value = float(max_value_index) / float(eHistogramSize - 1);
  float scale_factor = 1.0/max_value;

//   cout << "\nIndex of peak = " << index_peak << endl;
//   cout << "Count at peak = " << max_count << endl;
//   cout << "Empty bin threshold = " << empty_threshold << endl;
//   cout << "Max pixel value = " << max_value << endl;
//   cout << "Scale factor = " << scale_factor << endl;

  return scale_factor;
}

//=======================================================================
// Main
//=======================================================================

int
main(int argc, char* argv[])
{
  DiskImageResource::register_file_type(".ddd",
					&DiskImageResourceDDD::construct_open,
					&DiskImageResourceDDD::construct_create);

  // File names and entry point
  string stereo_default_filename, description_tab_filename, in_file1, in_file2;
  string cam_file1, cam_file2, out_prefix;
  int entry_point;

  // Definition of data and structures
  int argstart;
  F_HD hd;		// parameters read in header file & argv[]
  DFT_F dft;		// parameters read in stereo.default
  TO_DO execute;	// specifies which parts of the program to execute
  int nn;				   // Reuseable counter variable
  StereoEngine stereo_engine;		   // Does most of the heavy lifting

  // Image buffers 
  ImageView<PixelGray<float> > Limg, Rimg;
  ImageView<PixelGray<float> > texture;
  ImageView<PixelDisparity<float> > disparity_map;
  ImageView<bool> Lmask, Rmask;
  ImageView<Vector3> point_image;  
  Matrix<double> align_matrix;
  
  // Set the Vision Workbench debug level
  set_debug_level(InfoMessage);

  // Parse command line arguments
  parse_command_line_args(argc, argv,
			  &stereo_default_filename, &description_tab_filename,
			  &in_file1, &in_file2, &cam_file1, &cam_file2,
			  &out_prefix,&entry_point);
  
  // - - - - - - - - - - - - - - - - - - - - - - -
  // Initialize structures required by legacy code
  // - - - - - - - - - - - - - - - - - - - - - - -
  init_dft_struct(&dft, &execute);
  argstart = 1;
  init_header_struct(&dft, &hd, argv[0], "dummy", "dummy");

  // Read all of the options out of the stereo default file. 
  read_default_file(&dft, &execute, stereo_default_filename.c_str());

  // Set up the stereo engine object
  stereo_engine.do_horizontal_pyramid_search = execute.autoSetCorrParam;
  stereo_engine.do_vertical_pyramid_search = dft.autoSetVCorrParam;
  stereo_engine.search_range =
    BBox<int,2>(Vector<int,2>(dft.h_corr_min, dft.v_corr_min),
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
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                        Preprocessing entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= PREPROCESSING)
  {
    try    // Read the input files 
    {
      cout << "\nLoading image " << in_file1 << " as primary image:\n";
      read_image(Limg, in_file1);
      cout << "\nLoading image " << in_file2 << " as secondary image:\n";
      read_image(Rimg, in_file2);
      cout << endl;
    }
    catch (IOErr& e)
    {
      cout << e.what() << "\n";
      cout << "\n Could not read one or more input files. Exiting.\n\n";
      exit(0);
    }

    // Normalize the images, stretching the contrast if necessary.
    cout << "Normalizing images... " << flush;
    Limg = clamp(Limg, 0.0, 1.0);
    Rimg = clamp(Rimg, 0.0, 1.0);
    Limg *= calculate_stretch(Limg);
    Rimg *= calculate_stretch(Rimg);
    Limg = clamp(Limg, 0.0, 1.0);
    Rimg = clamp(Rimg, 0.0, 1.0);
//     Limg = normalize(copy(Limg), 0.0, max_value);
//     Rimg = normalize(copy(Rimg), 0.0, max_value);
    cout << "done." << endl;

    cout << "\nSaving TIFFs of normalized input images... " << flush;
    write_image(out_prefix + "-input-L.tif",
		channel_cast_rescale<uint8>(Limg));
    write_image(out_prefix + "-input-R.tif",
		channel_cast_rescale<uint8>(Rimg));
    cout << "done." << endl;

    cout << "\nCopying left input image into texture buffer... " << flush;
    texture = vw::copy(Limg);
    cout << "done." << endl;
    
    if (execute.w_texture)
    {
      try
      {
	cout << "\nSaving texture buffer as JPEG... " << flush;
        write_image(out_prefix + "-T.jpg", texture);
	cout << "done." << endl << endl;
      }
      catch (vw::IOErr &e)
      {
        cout << e.what() << "\n";
        cout << "\nWARNING: Could not write texture buffer to disk.\n";
      }
    }

    CTXImageMetadata ctx_metadata_1(in_file1);
    CTXImageMetadata ctx_metadata_2(in_file2);

    // If the spice kernels are available, try to use them directly to
    // read in MRO CTX telemetry.  
    try
    {
      cout << "Attempting to read MRO CTX telemetry from SPICE kernels... "
	   << flush;
      load_ctx_kernels();
      ctx_metadata_1.read_spice_data();
      ctx_metadata_2.read_spice_data();
      cout << "success.\n";
    }
    catch (spice::SpiceErr &e)
    {
      cout << "Warning: an error occurred when reading SPICE information. "
	"Falling back to *.sup files\n";
    }

    // Image Alignment
    cout << "\nPerforming image alignment.\n";
    ImageView<PixelGray<float> > adj_image, fully_adj_image;
    
    if (execute.keypoint_alignment && execute.do_alignment)
    {
      // Align images using keypoint homography.
      cout << "\nAligning images using KEYPOINT based automated alignment "
	"technique\n";
      align_matrix = keypoint_align(Rimg, Limg);
      write_matrix(out_prefix + "-align.exr", align_matrix);
      fully_adj_image = transform(Rimg, HomographyTransform(align_matrix),
                                  Limg.cols(), Limg.rows());
      Rimg = fully_adj_image;
    }
    else if (execute.ephemeris_alignment && execute.do_alignment)
    {
      // Align images using ephemeris data
      cout << "\nERROR: MRO CTX Ephemeris based image alignment is disabled."
	   << endl << endl;
      exit(1);
    }
    else if (execute.do_alignment)
    {
      // Fall back on trying to read in an alignment matrix.
      cout << "No automated technique for image registration has been "
	   << "selected.\nWill attempt to read in a previously generated "
	   << "alignment matrix from a file named \"align.exr\".\n";
      try
      {
        read_matrix(align_matrix, "align.exr");
        cout << "Alignment Matrix:\n";
        cout << align_matrix << endl;
        
        // Take the transformation and apply it now to the 
        // original secondary image.
        vw::Matrix<double> invH = vw::math::inverse(align_matrix);
        fully_adj_image = vw::transform(Rimg, HomographyTransform(invH),
					Limg.cols(), Limg.rows()); 
        Rimg = fully_adj_image;
        
      }
      catch (IOErr&)
      {
        cout << "\n\nFailed to open alignment file. Could not align images.\n";
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
    if (execute.w_mask)
    {  
      write_mask(Lmask, out_prefix + "-lMask.png");
      write_mask(Rmask, out_prefix + "-rMask.png");
    }
  }
   
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                          Correlation entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= CORRELATION)
  {
    // If we are jump-starting the code in the post-correlation phase,
    // we must initialize the disparity map object here with the two
    // files that were supplied at the command line.  Right now this
    // is purely for debugging.
    if (entry_point == CORRELATION)
    {
      try {
        cout << "\nStarting at the CORRELATION stage.\n";
        cout << "\nLoading image " << out_prefix + "-L.tif"
	     << " as primary image:\n";
        read_image(Limg, out_prefix + "-L.tif");
        cout << "\nLoading image " << out_prefix + "-R.tif"
	     << " as secondary image:\n";
        read_image(Rimg, out_prefix + "-R.tif");
        cout << "\nLoading image " << out_prefix + "-T.jpg"
	     << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        cout << endl;
      }
      catch (IOErr&)
      { 
        cout << "\n Unable to start code at the correlation stage.  "
	  "Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    // Call the stereo engine to perform the correlation
    std::cout << "\n" << stereo_engine << "\n";
    disparity_map = stereo_engine.correlate(Limg, Rimg);
    
    // Apply the Mask to the disparity map 
    if (execute.apply_mask)
    {
      printf("\nApplying image masks...");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }
    
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp,
				   min_v_disp, max_v_disp,true);
    write_image(out_prefix + "-DH.jpg",
		normalize(clamp(select_channel(disparity_map,0),
				min_h_disp, max_h_disp)));
    write_image(out_prefix + "-DV.jpg",
		 normalize(clamp(select_channel(disparity_map,1),
				 min_v_disp, max_v_disp)));
    write_image(out_prefix + "-D.exr", channels_to_planes(disparity_map));
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                 Disparity map filtering entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= FILTERING)
  {
    // If we are jump-starting the code in the post-correlation phase,
    // we must initialize the disparity map object here with the two
    // files that were supplied at the command line.  Right now this
    // is purely for debugging.
    if (entry_point == FILTERING)
    {
      // Read the input files
      try
      {
        cout << "\nStarting at the FILTERING stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-D.exr"
		  << " as disparity map image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-D.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
        read_image(texture, out_prefix + "-T.jpg");  
        Lmask = read_mask(out_prefix + "-lMask.png");
        Rmask = read_mask(out_prefix + "-rMask.png");
        std::cout << std::endl;
      }
      catch (IOErr&)
      {
        cout << "\n Unable to start code at the filtering stage.  "
	  "Could not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    // Smooth the disparity map
    stereo_engine.filter(disparity_map);

    // Write out the extrapolation mask image
    if (execute.w_extrapolation_mask)
    {
      ImageView<PixelRGB<float> > extrapolationMask =
	disparity::rgb_missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap-color.jpg", extrapolationMask);
      ImageView<PixelGray<float> > extrapolationMaskGray =
	disparity::missing_pixel_image(disparity_map);
      write_image(out_prefix + "-ExMap.png", extrapolationMaskGray);
    } 

    stereo_engine.interpolate(disparity_map);

    // Apply the Mask to the disparity map 
    if (execute.apply_mask)
    {
      printf("\nApplying image mask...");
      disparity::mask(disparity_map, Lmask, Rmask);
      printf("Done.\n");
    }

    // Write the filtered disp map 
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp,
				   min_v_disp, max_v_disp,true);
    write_image(out_prefix + "-FH.jpg",
		 normalize(clamp(select_channel(disparity_map,0),
				 min_h_disp, max_h_disp)));
    write_image(out_prefix + "-FV.jpg",
		 normalize(clamp(select_channel(disparity_map,1),
				 min_v_disp, max_v_disp)));
    write_image(out_prefix + "-F.exr", channels_to_planes(disparity_map));
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                 Disparity map to point-cloud entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= POINT_CLOUD)
  {
    // If we are jump-starting the code in the post-correlation phase,
    // we must initialize the disparity map object here with the 
    // two files that were supplied at the command line.  Right now
    // this is purely for debugging.
    if (entry_point == POINT_CLOUD)
    {
      // Read the input files
      try
      {
        std::cout << "\nStarting code at POINT_CLOUD stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-F.exr"
		  << " as disparity image:\n";
        ImageView<float> disparities;
        read_image(disparities, out_prefix + "-F.exr");
        disparity_map.set_size(disparities.cols(), disparities.rows());
        channels_to_planes(disparity_map) = disparities;
        std::cout << "\nLoading image " << out_prefix + "-T.jpg"
		  << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");  
        std::cout << std::endl;	
      }
      catch (IOErr&)
      {
        cout << "\n Unable to start at point cloud stage.\n\tCould not read "
	  "input files. Exiting.\n\n";
        exit(0);
      }
    }

    CTXImageMetadata ctx_metadata_1(in_file1);
    CTXImageMetadata ctx_metadata_2(in_file2);
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // If the spice kernels are available, try to use them directly to
    //    read in MRO CTX telemetry.  
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    try
    {
      cout << "Attempting to read MRO CTX telemetry from SPICE kernels... "
	   << flush;
      load_ctx_kernels();
      ctx_metadata_1.read_spice_data();
      ctx_metadata_2.read_spice_data();
      cout << "success.\n";
    }
    catch (spice::SpiceErr &e)
    {
      cout << "Warning: an error occurred when reading SPICE information. "
	"Falling back to *.sup files\n";
    }
    
    // Create the camera models and stereo models
    camera::OrbitingPushbroomModel
      left_camera_model = ctx_metadata_1.camera_model();
    camera::OrbitingPushbroomModel
      right_camera_model = ctx_metadata_2.camera_model();
    std::cout << left_camera_model << "\n";
    std::cout << right_camera_model << "\n";

    // We used a homography to line up the images, we may want 
    // to generate pre-alignment disparities before passing this information
    // onto the camera model in the next stage of the stereo pipeline.
    if (execute.do_alignment)
    {
      try
      {
        read_matrix(align_matrix, out_prefix + "-align.exr");
        std::cout << "Alignment Matrix: " << align_matrix << "\n";
      }
      catch (vw::IOErr &e)
      {
        std::cout << "Could not read in aligment matrix: " << out_prefix
		  << "-align.exr.  Exiting. \n\n";
        exit(1);
      }

      vw::Matrix<double> inv_align_matrix = inverse(align_matrix);
      disparity_linear_transform(disparity_map, inv_align_matrix);
    }

    // Write a VRML file that depicts the positions and poses of the
    // spacecraft in a scene with a large Mars ellipse.for reference.
    write_orbital_reference_model(out_prefix + "-OrbitViz.vrml",
				  left_camera_model, right_camera_model);
    
    // Apply the stereo model.  This yields a image of 3D points in space.
    StereoModel stereo_model(left_camera_model, right_camera_model);
    std::cout << "Generating a 3D point cloud.   \n";
    ImageView<double> error;
    point_image = stereo_model(disparity_map, error);
    write_image(out_prefix + "-error.png", normalize(error));

    // Write out the results to disk
    write_image(out_prefix + "-PC.exr", channels_to_planes(point_image));

    // Free up memory for the next stage
    disparity_map.reset();  
  }


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                 Point-cloud to wire mesh entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= WIRE_MESH)
  {
    // If we are jump-starting the code in the wire mesh phase, we want 
    // to simply read in the point cloud from disk and start from here.
    if (entry_point == WIRE_MESH)
    {
      // Read the input files
      try
      {
        std::cout << "\nStarting at WIRE_MESH stage.\n";
        std::cout << "\nLoading image " << out_prefix + "-PC.exr"
		  << " as pointcloud map:\n";
        ImageView<double> points;
        read_image(points, out_prefix + "-PC.exr");
        point_image.set_size(points.cols(), points.rows());
        channels_to_planes(point_image) = points;
        std::cout << "\nLoading image " << out_prefix + "-T.jpg"
		  << " as texture image:\n";
        read_image(texture, out_prefix + "-T.jpg");
        std::cout << std::endl;
      }
      catch (IOErr&)
      {
        cout << "\nFailed to start at wire mesh phase.\n"
	  "\tCould not read input files. Exiting.\n\n";
        exit(0);
      }
    }

    if (execute.mesh)
    {
      Mesh mesh_maker;
      if (execute.adaptative_meshing)
      {
        mesh_maker.build_adaptive_mesh(point_image, dft.mesh_tolerance,
				       dft.max_triangles);
      }
      else
      {
        mesh_maker.build_simple_mesh(point_image, dft.nff_h_step,
				     dft.nff_v_step);
      }
      if (execute.inventor)
      {
        mesh_maker.write_inventor(out_prefix+".iv", out_prefix+"-T.jpg");
      }
      if (execute.vrml)
      {
        mesh_maker.write_vrml(out_prefix+".vrml", out_prefix+"-T.jpg");
      }
    }

    // Map the pointcloud coordinates onto the MARS areoid 
    if (execute.write_dem)
    {
      cout << "Reprojecting points and subtracting the Mars areoid.\n";
      ImageView<Vector3> lat_lon_alt = cartography::xyz_to_latlon(point_image);

      // Subtract off the equitorial radius in preparation for writing
      // the data out to a DEM
      for (int i = 0; i < lat_lon_alt.cols(); i++) 
        for (int j = 0; j < lat_lon_alt.rows(); j++) 
          if (lat_lon_alt(i,j) != Vector3()) 
            lat_lon_alt(i,j).z() -= MOLA_PEDR_EQUATORIAL_RADIUS;

      // Write out the DEM, texture, and extrapolation mask
      // as georeferenced files.
      ImageView<double> dem_texture = vw::select_channel(lat_lon_alt, 2);
      Rasterizer rasterizer(lat_lon_alt, dem_texture, true);
      ImageView<PixelGray<float> > ortho_image = rasterizer.rasterize();
      write_image(out_prefix + "-DEM-debug.tif",
		  channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Write out a georeferenced orthoimage of the DTM
      rasterizer = Rasterizer(lat_lon_alt, select_channel(texture, 0), true);
      rasterizer.use_minz_as_default = false;
      ortho_image = rasterizer.rasterize();
      write_image(out_prefix + "-DRG-debug.tif",
		  channel_cast_rescale<uint8>(normalize(ortho_image)));

      // Write out a georeferenced orthoimage of the pixel extrapolation mask
      ImageView<PixelGray<float> > extrapolation_mask;
      try
      {
        read_image(extrapolation_mask, out_prefix + "-ExMap.png");
        rasterizer = Rasterizer(lat_lon_alt,
				select_channel(extrapolation_mask, 0), true);
        rasterizer.use_minz_as_default = false;
        ortho_image = rasterizer.rasterize();
        write_image(out_prefix + "-ExMap.tif",
		    channel_cast_rescale<uint8>(normalize(ortho_image)));
      }
      catch (IOErr &e)
      {
        std::cout << "Warning: an error occurred when reading the cached "
		  << "extrapolation map \"" << (out_prefix + "-ExMap.png")
		  << "\" on disk." << std::flush;
      }
    }
  }

  free (hd.cmd_name);

  return(EXIT_SUCCESS);
}

//=====//
// End //
//=====//

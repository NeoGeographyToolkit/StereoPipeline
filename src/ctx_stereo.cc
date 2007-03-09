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
#include <vw/Math.h>

#include "stereo.h"
#include "file_lib.h"
#include "mask.h"
#include "nff_terrain.h"
#include "ImageAlign.h"
#include "SurfaceNURBS.h"
#include "MRO/CTXEphemeris.h"		   // for load_ctx_kernels()
#include "MRO/CTXMetadata.h"		   // for read_spice_data()
#include "MRO/DiskImageResourceDDD.h"	   // support for Malin DDD image files
#include "Spice.h" 
#include "OrthoRasterizer.h"
#include "StereoEngine.h"

#include <vector> 
#include <string>
#include <iostream>
#include <cctype>			   // for tolower()
#include <algorithm>			   // debugging

using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;
using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::cartography;
using namespace vw::math;
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

typedef vw::cartography::OrthoRasterizer<Vector3> Rasterizer;
typedef vw::math::BBox<float, 3> BBox3d;

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

static void
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
      << "\nUsage: stereo [options] <Left_image> <Right_image> "
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
  // For some reason it seems that we often have a bunch of random
  // pixels at the high end which seems to be noise... we calculate a
  // scale factor so valid pixel values fill the range

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

static bool
cpu_is_big_endian()
{
  union { short number; char bytes[2]; } short_endian_check;

  short_endian_check.number = 1;

  return (char(short_endian_check.number) == short_endian_check.bytes[1]);
}

// Write an ENVI compatible DEM header
static void
write_ENVI_header(const string& headerName, int width, int height,
		  double pixelScaling, BBox3d bBox,
		  ENVI_data_type data_type = ENVI_float_32bit)
{
  FILE *headerFile = fopen(headerName.c_str(), "w");

  fprintf(headerFile, "ENVI\n");
  fprintf(headerFile, "description = { \n");
  fprintf(headerFile, "   -- Digital Elevation Map generated by the NASA Ames Stereo Pipeline --\n");
  fprintf(headerFile, "   \n");
  fprintf(headerFile, "   The Ames Stereo Pipeline generates 3D coordinates in a planetocentric, \n");
  fprintf(headerFile, "   coordinate system as defined by the International Astronomical Union (IAU).  \n");
  fprintf(headerFile, "   The origin of this coordinate system is the planetary center of mass, and \n");
  fprintf(headerFile, "   coordinate system is right handed.\n");
  fprintf(headerFile, "   \n");
  fprintf(headerFile, "   The output of the stereo reconstruction process is are points in a cartesian \n");
  fprintf(headerFile, "   coordinate frame.  ");
  fprintf(headerFile, "   \n");
  fprintf(headerFile, "    This DEM was generated by converting the cartesion coordinates to spherical \n");
  fprintf(headerFile, "   (polar) coordinates.   Next, the radius of an areoid that defines Mars \"sea level\"\n");
  fprintf(headerFile, "   is subtracted from the radial value of each data point.  This yields a value for\n");
  fprintf(headerFile, "   elevation relative to this areoid.  The sea-level reference is given by the IAU200 Mars\n");
  fprintf(headerFile, "   areoid: a bi-axial ellipsoid with an equatorial radius of 3396 kilometers and a\n");
  fprintf(headerFile, "   polar radius of 3376.2 kilometers.\n");
  fprintf(headerFile, "   \n");
  fprintf(headerFile, "   Finally, the latitude/longitude values have been remapped using a sinusoidal (equal area)\n");
  fprintf(headerFile, "   projection.\n");
  fprintf(headerFile, "   \n");
  fprintf(headerFile, "   Bounding box:\n");
  fprintf(headerFile, "     Minimum X (left)    = %f\n", bBox.min()[0]);
  fprintf(headerFile, "     Minimum Y (top)     = %f\n", bBox.max()[1]);
  fprintf(headerFile, "     Maximum X (right)   = %f\n", bBox.max()[0]);
  fprintf(headerFile, "     Maximum Y (bottom)  = %f\n", bBox.min()[1]);
  fprintf(headerFile, "     Minimum Z           = %f\n", bBox.min()[2]);
  fprintf(headerFile, "     Maximum Z           = %f\n", bBox.max()[2]);
  fprintf(headerFile, "     Default Z           = %f\n", bBox.min()[2]);
  fprintf(headerFile, "}\n");
  fprintf(headerFile, "samples = %d\n", width);
  fprintf(headerFile, "lines   = %d\n", height);
  fprintf(headerFile, "bands   = 1\n");
  fprintf(headerFile, "header offset = 0\n");
  fprintf(headerFile, "map info = { Geographic Lat/Lon, 1.5, 1.5, %f, %f, %f, %f, Mars IAU 2000 Areoid, units=Degrees}\n",
	  bBox.min()[0], bBox.max()[1], pixelScaling, pixelScaling);
  fprintf(headerFile, "file type = ENVI Standard\n");
  fprintf(headerFile, "data type = %d\n", int(data_type));
  cout << "write_ENVI_header(): ENVI data type = " << data_type << endl;
  fprintf(headerFile, "interleave = bsq\n");
  // Byte order defs: 0 = little endian, 1 = big endian
  fprintf(headerFile, "byte order = %d\n",
	  (unsigned int)(cpu_is_big_endian()));
  cout << "write_ENVI_header(): CPU is big endian = "
       << cpu_is_big_endian() << endl;
  fprintf(headerFile, "\n");
  fclose(headerFile);
}

static bool
file_is_readable(string filename)
{
  ifstream temp_stream(filename.c_str(), ios::in);
  bool file_readable = temp_stream;

  temp_stream.close();

  return file_readable;
}

static bool
filename_has_TIFF_extension(string filename)
{
  return ((filename.rfind(".tif") != string::npos) ||
	  (filename.rfind(".tiff") != string::npos) ||
	  (filename.rfind(".TIF") != string::npos) ||
	  (filename.rfind(".TIFF") != string::npos));
}

static bool
filename_has_DDD_extension(string filename)
{
  return ((filename.rfind(".ddd") != string::npos) ||
	  (filename.rfind(".DDD") != string::npos));
}

void
generate_file_names(string &image_filename1, string &image_filename2,
		    string& metadata_filename1, string& metadata_filename2)
{
  metadata_filename1 = image_filename1;
  metadata_filename2 = image_filename2;

  // If it's a tiff file we need an associated DDD file for meta data
  if (filename_has_TIFF_extension(image_filename1))
  {
    int dot_index = metadata_filename1.rfind(".");
    metadata_filename1.erase(dot_index);
    metadata_filename1 += ".ddd";
    dot_index = metadata_filename2.rfind(".");
    metadata_filename2.erase(dot_index);
    metadata_filename2 += ".ddd";
  }
  else if (filename_has_DDD_extension(image_filename1))
  {
    // If it's a DDD file name create the corresponding TIFF image
    // file name.
    string tiff_name = image_filename1;
    tiff_name.erase(tiff_name.rfind("."));
    tiff_name += ".tif";
    image_filename1 = tiff_name;

    tiff_name = image_filename2;
    tiff_name.erase(tiff_name.rfind("."));
    tiff_name += ".tif";
    image_filename2 = tiff_name;
  }
  else
  {
    cout << "WARNING in generate_file_names(): "
	 << "input files were neither DDD or TIFF!" << endl;
  }
}

static void
create_tiff_copies(string tiff_filename1, string tiff_filename2,
		   string ddd_filename1, string ddd_filename2)
{
  // Create TIFFs of the DDDs to allow block correlation to work
  // effectively off a disk image resource
  if (!file_is_readable(tiff_filename1))
  {
    ImageView<PixelGray<short> > temp_image;
    read_image(temp_image, ddd_filename1);
    write_image(tiff_filename1, temp_image);
//     temp_image.reset();			   // release the image memory
  }
  if (!file_is_readable(tiff_filename2))
  {
    ImageView<PixelGray<short> > temp_image;
    read_image(temp_image, ddd_filename2);
    write_image(tiff_filename2, temp_image);
//     temp_image.reset();			   // release the image memory
  }
}

//=======================================================================
// Main
//=======================================================================

int
main(int argc, char* argv[])
{
  // Register the DDD file handler with the Vision Workbench
  // DiskImageResource system.
  DiskImageResource::register_file_type(".ddd",
					&DiskImageResourceDDD::construct_open,
					&DiskImageResourceDDD::construct_create);

  // File names and entry point
  string stereo_default_filename, description_tab_filename;
  string image_filename1, image_filename2, metadata_file1, metadata_file2;
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
  ImageView<PixelGray<float> > left_image, right_image;
  ImageView<PixelDisparity<float> > disparity_map;
  ImageView<bool> left_mask, right_mask;
  ImageView<Vector3> point_image;  
  Matrix<double> align_matrix;
  Matrix<double> scale_matrix = identity_matrix(3);
  Matrix<double> inv_scale_matrix = identity_matrix(3);
  
  // Set the Vision Workbench debug level
  set_debug_level(InfoMessage);

  // Parse command line arguments
  parse_command_line_args(argc, argv,
			  &stereo_default_filename, &description_tab_filename,
			  &image_filename1, &image_filename2,
			  &cam_file1, &cam_file2, &out_prefix,&entry_point);
  
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

  generate_file_names(image_filename1, image_filename2,
		      metadata_file1, metadata_file2);
  create_tiff_copies(image_filename1, image_filename2,
		     metadata_file1, metadata_file2);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //                        Preprocessing entry point
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (entry_point <= PREPROCESSING)
  {
    // Normalize the images, stretching the contrast if necessary.
    if (!file_is_readable(out_prefix + "-normalized-L.tif") ||
	!file_is_readable(out_prefix + "-normalized-R.tif"))
    {
      cout << "Normalizing images... " << flush;
      try    // Read the input files 
      {
	cout << "\nLoading left image: " << image_filename1 << endl;
	read_image(left_image, image_filename1);
	cout << "\nLoading right image: " << image_filename2 << endl;
	read_image(right_image, image_filename2);
	cout << endl;
      }
      catch (IOErr& e)
      {
	cout << e.what() << "\n";
	cout << "\n Could not read one or more input images. Exiting.\n\n";
	exit(1);
      }
      // The old way...
      //     left_image = normalize(copy(left_image), 0.0, max_value);
      //     right_image = normalize(copy(right_image), 0.0, max_value);
      // The new way...
      left_image = clamp(left_image, 0.0, 1.0);
      right_image = clamp(right_image, 0.0, 1.0);
      left_image *= calculate_stretch(left_image);
      right_image *= calculate_stretch(right_image);
      left_image = clamp(left_image, 0.0, 1.0);
      right_image = clamp(right_image, 0.0, 1.0);
      cout << "done." << endl;

      cout << "\nSaving TIFFs of normalized input images... " << flush;
      write_image(out_prefix + "-normalized-L.tif",
		  channel_cast_rescale<uint8>(left_image));
      write_image(out_prefix + "-normalized-R.tif",
		  channel_cast_rescale<uint8>(right_image));
      // Release the memory used during normalization, since the
      // alignment procedure can be memory intensive, and because we
      // will be creating a new aligned right image.
//       left_image.reset();
//       right_image.reset();
      cout << "done." << endl;
    }
    else
    {
      cout << "Normalized images exist, skipping normalization. " << endl;
    }

    if (execute.w_texture)
    {
      try
      {
	cout << "\nLoading image, " << out_prefix + "-normalized-L.tif"
	     << " as texture..." << endl;
	read_image(left_image, out_prefix + "-normalized-L.tif");
	cout << "\nSaving texture as JPEG... " << flush;
        write_image(out_prefix + "-T.jpg",
		    channel_cast_rescale<uint8>(left_image));
	cout << "done." << endl << endl;
// 	left_image.reset();		   // release the memory
      }
      catch (vw::IOErr &e)
      {
        cout << e.what() << "\n";
        cout << "\nWARNING: Could not write texture to disk.\n";
      }
    }

    // Image Alignment
    cout << "\nPerforming image alignment.\n";
    if (execute.do_alignment)
    {

      if (execute.ephemeris_alignment)
      {
	// Align images using ephemeris data
	cout << "\nERROR: MRO CTX Ephemeris based image alignment is disabled."
	     << endl << endl;
	exit(1);
      }
      else if (execute.keypoint_alignment)
      {
	const float subsample_scale = 0.5;
	string left_align_name(out_prefix + "-normalized-L.tif");
	string right_align_name(out_prefix + "-normalized-R.tif");
	if (/* dft.subsample_align_images */ true)
	{
	  ImageView<PixelGray<uint8> > temp_image;
	  read_image(temp_image, left_align_name);
	  left_align_name = out_prefix + "-normalized-sub-L.tif";
	  write_image(left_align_name, resample(temp_image, subsample_scale));
// 	  temp_image.reset();
	  read_image(temp_image, right_align_name);
	  right_align_name = out_prefix + "-normalized-sub-R.tif";
	  write_image(right_align_name, resample(temp_image, subsample_scale));
// 	  temp_image.reset();		      
	}
	cout << "Using image " << left_align_name
	     << " for image alignment.\n";
	DiskImageView<PixelGray<float> > left_disk_image(left_align_name);
	cout << "Using image " << right_align_name
	     << " for image alignment.\n";
	DiskImageView<PixelGray<float> > right_disk_image(right_align_name);
	cout << endl;

	// Align images using keypoint homography.
	cout << "\nAligning images using KEYPOINT based automated alignment "
	     << "technique\n";
	align_matrix = keypoint_align(right_disk_image, left_disk_image);
	if (/* dft.subsample_align_images */ true)
	{
	  scale_matrix(0, 0) = subsample_scale;
	  scale_matrix(1, 1) = subsample_scale;
	  inv_scale_matrix(0, 0) = 1.0 / subsample_scale;
	  inv_scale_matrix(1, 1) = 1.0 / subsample_scale;
	  align_matrix = align_matrix * scale_matrix;
	  align_matrix = inv_scale_matrix * align_matrix;
	  cout << "Scaled alignment Matrix:\n";
	  cout << align_matrix << endl;
	}
	// Write the alignment matrix to a file
	write_matrix(out_prefix + "-align.exr", align_matrix);
      }
      else				   // try to read an align matrix in
      {
	cout << "Attempting to read in a previously generated alignment matrix"
	     << " from file: \"" << out_prefix + "-align.exr" << "\".\n";
	try
	{
	  read_matrix(align_matrix, out_prefix + "-align.exr");
	}
	catch (IOErr&)
	{
	  cout << "\n\nFailed to open alignment matrix. Exiting.\n\n";
	  exit(1);
	}  
	cout << "Alignment Matrix:\n";
	cout << align_matrix << endl;
// 	vw::Matrix<double> invH = vw::math::inverse(align_matrix);
// 	align_matrix = invH;
      }
      DiskImageView<PixelGray<float> > left_disk_image(out_prefix +
						       "-normalized-L.tif");
      DiskImageView<PixelGray<float> > right_disk_image(out_prefix +
							"-normalized-R.tif");
      right_image = transform(right_disk_image,
			      HomographyTransform(align_matrix),
			      left_disk_image.cols(),
			      left_disk_image.rows());
      left_image = vw::copy(left_disk_image);
    }
    else
    {
      read_image(left_image, out_prefix + "-normalized-L.tif");
      read_image(right_image, out_prefix + "-normalized-R.tif");
    }

    // Write the aligned images to files
    cout << "Writing aligned images to disk... " << flush;
    write_image(out_prefix + "-L.tif", channel_cast<uint8>(left_image*255));
    write_image(out_prefix + "-R.tif", channel_cast<uint8>(right_image*255));
    cout << "done." << endl;
   
    // Mask any pixels that are black and 
    // appear on the edges of the image.
    cout << "\nGenerating image masks...";
    left_mask = disparity::generate_mask(left_image);
    right_mask = disparity::generate_mask(right_image);
    printf("Done.\n");
    if (execute.w_mask)
    {  
      write_mask(left_mask, out_prefix + "-lMask.png");
      write_mask(right_mask, out_prefix + "-rMask.png");
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
        read_image(left_image, out_prefix + "-L.tif");
        cout << "\nLoading image " << out_prefix + "-R.tif"
	     << " as secondary image:\n";
        read_image(right_image, out_prefix + "-R.tif");
        left_mask = read_mask(out_prefix + "-lMask.png");
        right_mask = read_mask(out_prefix + "-rMask.png");
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
    try
    {
      disparity_map = stereo_engine.correlate(left_image, right_image);
    }
    catch (std::exception& e)
    {
      cout << "ERROR: caught standard exception!" << endl;
      cout << "Exiting!" << endl;
      exit(1);
    }
    
    // Apply the Mask to the disparity map 
    if (execute.apply_mask)
    {
      printf("\nApplying image masks...");
      disparity::mask(disparity_map, left_mask, right_mask);
      printf("Done.\n");
    }

    // Write the raw disparity map out first in case we are short on memory...
    write_image(out_prefix + "-D.exr", channels_to_planes(disparity_map));

    // Write out visible versions of the disparity map
    double min_h_disp, min_v_disp, max_h_disp, max_v_disp;
    disparity::get_disparity_range(disparity_map, min_h_disp, max_h_disp,
				   min_v_disp, max_v_disp,true);
    write_image(out_prefix + "-DH.jpg",
		normalize(clamp(select_channel(disparity_map,0),
				min_h_disp, max_h_disp)));
    write_image(out_prefix + "-DV.jpg",
		 normalize(clamp(select_channel(disparity_map,1),
				 min_v_disp, max_v_disp)));
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
        left_mask = read_mask(out_prefix + "-lMask.png");
        right_mask = read_mask(out_prefix + "-rMask.png");
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
      disparity::mask(disparity_map, left_mask, right_mask);
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
      }
      catch (IOErr&)
      {
        cout << "\n Unable to start at point cloud stage.\n\tCould not read "
	  "input files. Exiting.\n\n";
        exit(0);
      }
    }

    CTXImageMetadata ctx_metadata_1(metadata_file1);
    CTXImageMetadata ctx_metadata_2(metadata_file2);
    
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
      cout << "\nWarning: an error occurred when reading SPICE information. "
	   << "The camera models will not be valid. Continuing." << endl;
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
	// Note, the following line allocates memory... probably should
	// check for running out of memory
	point_image.set_size(points.cols(), points.rows());
        channels_to_planes(point_image) = points;
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
      cout << "Reprojecting points and subtracting the Mars areoid... "
	   << flush;
      ImageView<Vector3> lon_lat_alt =
	cartography::xyz_to_lon_lat_radius(point_image);
      point_image.reset();		   // we're done with point_image...
      // subtract off the equitorial radius in preparation for writing
      // the data out to a DEM
      for (int i = 0; i < lon_lat_alt.cols(); i++) 
        for (int j = 0; j < lon_lat_alt.rows(); j++)
          if (lon_lat_alt(i,j) != Vector3()) 
            lon_lat_alt(i,j).z() -= MOLA_PEDR_EQUATORIAL_RADIUS;
      cout << "done." << endl;

      // Write out the DEM
      {
	cout << "Rasterizing DEM and saving to disk... " << flush;
	Rasterizer rasterizer(lon_lat_alt);
	ImageView<PixelGray<float> > ortho_image =
	  rasterizer(vw::select_channel(lon_lat_alt, 2));
	write_image(out_prefix + "-DEM-debug.tif",
		    channel_cast_rescale<uint8>(normalize(ortho_image)));

	GeoReference geo;
	Matrix<double, 3, 3> affine = rasterizer.geo_transform();
	geo.set_transform(affine);
	write_georeferenced_image(out_prefix+"-DEM.tif", ortho_image, geo);
	write_georeferenced_image(out_prefix+"-DEM.dem", ortho_image, geo);
	// The GDAL ENVI file handler does not embed geo-referencing
	// information... we overwrite it here
	BBox3d dem_bbox = rasterizer.bounding_box();
	cout << "\tDEM BBox found = "
	     << "Min[" << dem_bbox.min()[0] << "," << dem_bbox.min()[1] << "] "
	     << "Max[" << dem_bbox.max()[0] << "," << dem_bbox.max()[1] << "]"
	     << endl;
	printf("\tZ Range = [zmin, zmax] = [%lf, %lf]\n",
	       dem_bbox.min()[2], dem_bbox.max()[2]);
	write_ENVI_header(out_prefix+"-DEM.hdr", ortho_image.cols(),
			  ortho_image.rows(), affine(0, 0), dem_bbox);
	ortho_image.reset();		   // let go of the memory
	cout << "done." << endl;
      }
      // Write out a georeferenced orthoimage corresponding to the DEM
      {
	cout << "Rasterizing texture and saving to disk... " << flush;
	Rasterizer rasterizer(lon_lat_alt);
	rasterizer.use_minz_as_default = false;
	rasterizer.set_default_value(0);
	ImageView<PixelGray<float> > ortho_image =
	  rasterizer(select_channel(left_image, 0));
	write_image(out_prefix + "-DRG.tif",
		    channel_cast_rescale<uint8>(normalize(ortho_image)));
	ortho_image.reset();		   // let go of the memory
	cout << "done." << endl;
      }
      // Write out a georeferenced orthoimage of the pixel extrapolation mask
      {
	ImageView<PixelGray<float> > extrapolation_mask;
	try
	{
	  cout << "Rasterizing extrapolation mask and saving to disk... "
	       << flush;
	  read_image(extrapolation_mask, out_prefix + "-ExMap.png");
	  Rasterizer rasterizer(lon_lat_alt);
	  rasterizer.use_minz_as_default = false;
	  rasterizer.set_default_value(0);
	  ImageView<PixelGray<float> > ortho_image =
	    rasterizer(select_channel(extrapolation_mask, 0));
	  write_image(out_prefix + "-ExMap.tif",
		      channel_cast_rescale<uint8>(normalize(ortho_image)));
	  ortho_image.reset();		   // let go of the memory
	  cout << "done." << endl;
	}
	catch (IOErr &e)
	{
	  std::cout << "Warning: an error occurred when reading the cached "
		    << "extrapolation map \"" << (out_prefix + "-ExMap.png")
		    << "\" on disk." << std::flush;
	}
      }
    }
  }

  free (hd.cmd_name);

  return(EXIT_SUCCESS);
}

//=====//
// End //
//=====//

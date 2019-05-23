// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

// Create a pinhole or optical bar camera model based on intrinsics, image corner
// coordinates, and, optionally, a DEM of the area.

#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/EigenUtils.h>
#include <asp/Camera/OpticalBarModel.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace std;
using namespace vw::cartography;
using namespace asp::camera;

// Parse numbers or strings from a list where they are separated by commas or spaces.
template<class T>
void parse_values(std::string list, std::vector<T> & values){

  values.clear();

  // Replace commas with spaces
  std::string oldStr = ",", newStr = " ";
  size_t pos = 0;
  while((pos = list.find(oldStr, pos)) != std::string::npos){
    list.replace(pos, oldStr.length(), newStr);
    pos += newStr.length();
  }

  // Read the values one by one
  std::istringstream is(list);
  T val;
  while (is >> val)
    values.push_back(val);
}

struct Options : public vw::cartography::GdalWriteOptions {
  string image_file, camera_file, lon_lat_values_str, pixel_values_str, datum_str,
    reference_dem, frame_index, gcp_file, camera_type, sample_file;
  double focal_length, pixel_pitch, gcp_std, height_above_datum;
  Vector2 optical_center;
  std::vector<double> lon_lat_values, pixel_values;
  bool refine_camera; 
  Options(): focal_length(-1), pixel_pitch(-1), gcp_std(1), height_above_datum(0), refine_camera(false) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-camera-file,o", po::value(&opt.camera_file), "Specify the output camera file with a .tsai extension.")
    ("camera-type", po::value(&opt.camera_type)->default_value("pinhole"), "Specify the camera type. Options are: pinhole (default) and opticalbar.")
    ("lon-lat-values", po::value(&opt.lon_lat_values_str)->default_value(""), "A (quoted) string listing numbers, separated by commas or spaces, having the longitude and latitude (alternating and in this order) of each image corner. The corners are traversed in the order 0,0 w,0, w,h, 0,h where w and h are the image width and height.")
    ("pixel-values", po::value(&opt.pixel_values_str)->default_value(""), "A (quoted) string listing numbers, separated by commas or spaces, having the column and row (alternating and in this order) of each pixel in the raw image at which the longitude and latitude is known. By default this is empty, and will be populated by the image corners traversed as earlier.")
    ("reference-dem", po::value(&opt.reference_dem)->default_value(""),
     "Use this DEM to infer the heights above datum of the image corners.")
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the longitude and latitude, unless a DEM is given. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("height-above-datum", po::value(&opt.height_above_datum)->default_value(0),
     "Assume this height above datum in meters for the image corners unless read from the DEM.")
    ("sample-file", po::value(&opt.sample_file)->default_value(""), 
     "Read in the camera parameters from the example camera file.  Required for opticalbar type.")
    ("focal-length", po::value(&opt.focal_length)->default_value(0),
     "The camera focal length.")
    ("optical-center", po::value(&opt.optical_center)->default_value(Vector2i(0,0),"0 0"),
     "The camera optical center.")
    ("pixel-pitch", po::value(&opt.pixel_pitch)->default_value(0),
     "The pixel pitch.")
    ("refine-camera", po::bool_switch(&opt.refine_camera)->default_value(false),
     "After a rough initial camera is obtained, refine it using least squares.")
    ("frame-index", po::value(&opt.frame_index)->default_value(""),
     "A file used to look up the longitude and latitude of image corners based on the image name, in the format provided by the SkySat video product.")
    ("gcp-file", po::value(&opt.gcp_file)->default_value(""),
     "If provided, save the image corner coordinates and heights in the GCP format to this file.")
    ("gcp-std", po::value(&opt.gcp_std)->default_value(1),
     "The standard deviation for each GCP pixel, if saving a GCP file. A smaller value suggests a more reliable measurement, hence will be given more weight.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("image-file", po::value(&opt.image_file));

  po::positional_options_description positional_desc;
  positional_desc.add("image-file",1);

  string usage("[options] <image-file> -o <camera-file>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if ( opt.image_file.empty() )
    vw_throw( ArgumentErr() << "Missing the input image.\n"
              << usage << general_options );

  if ( opt.camera_file.empty() )
    vw_throw( ArgumentErr() << "Missing the output camera file name.\n"
              << usage << general_options );

  boost::to_lower(opt.camera_type);
  
  if (opt.camera_type != "pinhole" && opt.camera_type != "opticalbar")
    vw_throw( ArgumentErr() << "Only pinhole and opticalbar cameras are supported.\n");
  
  if ((opt.camera_type == "opticalbar") && (opt.sample_file == ""))
    vw_throw( ArgumentErr() << "opticalbar type must use a sample camera file.\n"
              << usage << general_options );

  std::string ext = get_extension(opt.camera_file);
  if (ext != ".tsai") 
    vw_throw( ArgumentErr() << "The output camera file must end with .tsai.\n"
              << usage << general_options );

  // If we cannot read the data from a DEM, must specify a lot of things.
  if (opt.reference_dem.empty() && opt.datum_str.empty())
    vw_throw( ArgumentErr() << "Must provide either a reference DEM or a datum.\n"
              << usage << general_options );

  if (opt.gcp_std <= 0) 
    vw_throw( ArgumentErr() << "The GCP standard deviation must be positive.\n"
              << usage << general_options );

  if (opt.frame_index != "" && opt.lon_lat_values_str != "") 
    vw_throw( ArgumentErr() << "Cannot specify both the frame index file and the lon-lat corners.\n"
              << usage << general_options );

  if (opt.frame_index != "") {
    // Parse the frame index to extract opt.lon_lat_values_str.
    // Look for a line having this image, and search for 
    boost::filesystem::path p(opt.image_file); 
    std::string image_base = p.stem().string(); // strip the directory name and suffix
    std::ifstream file( opt.frame_index.c_str() );
    std::string line;
    std::string beg = "POLYGON((";
    std::string end = "))";
    while ( getline(file, line, '\n') ) {
      if (line.find(image_base) == 0) {
        int beg_pos = line.find(beg);
        if (beg_pos == std::string::npos)
          vw_throw( ArgumentErr() << "Cannot find " << beg << " in line: " << line << ".\n");
        beg_pos += beg.size();
        int end_pos = line.find(end);
        if (end_pos == std::string::npos)
          vw_throw( ArgumentErr() << "Cannot find " << end << " in line: " << line << ".\n");
        opt.lon_lat_values_str = line.substr(beg_pos, end_pos - beg_pos);
        vw_out() << "Scanned the lon-lat corner values: " << opt.lon_lat_values_str << std::endl;
        break;
      }
    }
    if (opt.lon_lat_values_str == "")
      vw_throw( ArgumentErr() << "Could not parse the entry for " << image_base
                << " in file: " << opt.frame_index << ".\n");
  }
    
  // Parse the lon-lat values
  parse_values<double>(opt.lon_lat_values_str, opt.lon_lat_values);
  if (opt.lon_lat_values.size() < 6) 
    vw_throw( ArgumentErr() << "Expecting at least three longitude-latitude pairs.\n"
              << usage << general_options );

  // Parse the pixel values
  parse_values<double>(opt.pixel_values_str, opt.pixel_values);
  
  if (opt.pixel_values.empty() && opt.lon_lat_values.size() != 8) {
    // When no pixel values are given, we will use the corners, hence
    // we must have eight values.
    vw_throw( ArgumentErr() << "Expecting 8 values for the image corners.\n"
              << usage << general_options );
  }
  
  if ( opt.sample_file == "" && (opt.focal_length <= 0      || opt.optical_center[0] <= 0 ||
                                 opt.optical_center[1] <= 0 || opt.pixel_pitch <= 0))
    vw_throw( ArgumentErr() << "Must provide positive focal length, optical center, "
              << "and pixel pitch values OR a sample file.\n" << usage << general_options );
  
  // Create the output directory
  vw::create_out_dir(opt.camera_file);
} // End function handle_arguments

// Form a camera based on info the user provided
void manufacture_cam(Options const& opt, int wid, int hgt,
		     boost::shared_ptr<CameraModel> & out_cam){

  if (opt.camera_type == "opticalbar") {
    boost::shared_ptr<asp::camera::OpticalBarModel> opticalbar_cam;
    opticalbar_cam.reset(new asp::camera::OpticalBarModel(opt.sample_file));
    // Make sure the image size matches the input image file.
    opticalbar_cam->set_image_size(Vector2i(wid, hgt));
    opticalbar_cam->set_optical_center(Vector2(wid/2.0, hgt/2.0));
    out_cam = opticalbar_cam;
  } else {
    boost::shared_ptr<PinholeModel> pinhole_cam;
    if (opt.sample_file != "") {
      // Use the initial guess from file
      pinhole_cam.reset(new PinholeModel(opt.sample_file));
    } else {
      // Use the intrinsics from the command line. Use trivial rotation and translation.
      Vector3 ctr(0, 0, 0);
      Matrix<double, 3, 3> rotation;
      rotation.set_identity();
      pinhole_cam.reset(new PinholeModel(ctr, rotation, opt.focal_length, opt.focal_length,
					 opt.optical_center[0], opt.optical_center[1],
					 NULL, opt.pixel_pitch));
    }
    out_cam = pinhole_cam;
  }
}

int main( int argc, char *argv[] ) {
  Options opt;
  try {

    handle_arguments(argc, argv, opt);
    
    vw::cartography::Datum datum;
    GeoReference geo;
    ImageView<float> dem;
    float nodata_value = -std::numeric_limits<float>::max(); 
    bool has_dem = false;
    if (opt.reference_dem != "") {
      dem = DiskImageView<float>(opt.reference_dem);
      bool ans = read_georeference(geo, opt.reference_dem);
      if (!ans) 
        vw_throw( ArgumentErr() << "Could not read the georeference from dem: "
                  << opt.reference_dem << ".\n");

      datum = geo.datum(); // Read this in for completeness
      has_dem = true;
      vw::read_nodata_val(opt.reference_dem, nodata_value);
      vw_out() << "Using nodata value: " << nodata_value << std::endl;
    }else{
      datum = vw::cartography::Datum(opt.datum_str); 
      vw_out() << "No reference DEM provided. Will use a height of "
               << opt.height_above_datum << " above the datum:\n" 
               << datum << std::endl;
    }

    // Prepare the DEM for interpolation
    ImageViewRef< PixelMask<float> > interp_dem
      = interpolate(create_mask(dem, nodata_value),
		    BilinearInterpolation(), ZeroEdgeExtension());
    
    DiskImageView<float> img(opt.image_file);
    int wid = img.cols(), hgt = img.rows();
    if (wid <= 0 || hgt <= 0) 
      vw_throw( ArgumentErr() << "Could not read an image with positive dimensions from: "
                << opt.image_file << ".\n");

    size_t num_lon_lat_pairs = opt.lon_lat_values.size()/2;
    
    if (opt.pixel_values.empty()) {
      // populate the corners
      double arr[] = {0, 0, wid, 0, wid, hgt, 0, hgt};
      for (size_t it  = 0; it < sizeof(arr)/sizeof(double); it++) 
        opt.pixel_values.push_back(arr[it]);
    }
    
    Vector2 pix;
    Vector3 llh, xyz;
    std::vector<Vector3> xyz_vec;

    // If to write a gcp file
    std::ostringstream gcp;
    gcp.precision(17);
    bool write_gcp = (opt.gcp_file != "");

    for (size_t corner_it = 0; corner_it < num_lon_lat_pairs; corner_it++) {

      // Get the height from the DEM if possible
      llh[0] = opt.lon_lat_values[2*corner_it+0];
      llh[1] = opt.lon_lat_values[2*corner_it+1];
      if (llh[1] < -90 || llh[1] > 90) 
        vw_throw( ArgumentErr() << "Detected a latitude out of bounds. "
                  << "Perhaps the longitude and latitude are reversed?\n");
      double height = opt.height_above_datum;
      if (has_dem) {
        bool success = false; 
        pix = geo.lonlat_to_pixel(subvector(llh, 0, 2));
        int len =  BilinearInterpolation::pixel_buffer;
        if (pix[0] >= 0 && pix[0] <= interp_dem.cols() - 1 - len &&
            pix[1] >= 0 && pix[1] <= interp_dem.rows() - 1 - len) {
          PixelMask<float> masked_height = interp_dem(pix[0], pix[1]);
          if (is_valid(masked_height)) {
            height = masked_height.child();
            success = true;
          }
        }
        if (!success) 
          vw_out() << "Could not determine a valid height value for the next corner. "
                   << "Will use a height of " << height << ".\n";
      }
      
      llh[2] = height;
      vw_out() << "Lon-lat-height for corner ("
               << opt.pixel_values[2*corner_it] << ' ' << opt.pixel_values[2*corner_it+1]
               << ") is "
               << llh[0] << ' ' << llh[1] << ' ' << llh[2] << std::endl;
    
      xyz = datum.geodetic_to_cartesian(llh);
      xyz_vec.push_back(xyz);

      if (write_gcp)
        gcp << corner_it << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2] << ' '
            << 1 << ' ' << 1 << ' ' << 1 << ' ' << opt.image_file << ' '
            << opt.pixel_values[2*corner_it] << ' ' << opt.pixel_values[2*corner_it+1] << ' '
            << opt.gcp_std << ' ' << opt.gcp_std << std::endl;
    } // End loop through lon-lat pairs

    if (write_gcp) {
      vw_out() << "Writing: " << opt.gcp_file << std::endl;
      std::ofstream fs(opt.gcp_file.c_str());
      fs << gcp.str();
      fs.close();
    }
    
    // Form a camera based on info the user provided
    boost::shared_ptr<CameraModel> out_cam;
    manufacture_cam(opt, wid, hgt, out_cam);

    // Transform it and optionally refine it
    bool verbose = true;
    fit_camera_to_xyz(opt.camera_type, opt.refine_camera,  
		      xyz_vec, opt.pixel_values, verbose, out_cam);


    vw_out() << "Writing: " << opt.camera_file << std::endl;
    if (opt.camera_type == "opticalbar")
      ((asp::camera::OpticalBarModel*)out_cam.get())->write(opt.camera_file);
    else
      ((PinholeModel*)out_cam.get())->write(opt.camera_file);


  } ASP_STANDARD_CATCHES;
    
  return 0;
}

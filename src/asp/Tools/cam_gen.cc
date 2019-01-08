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
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
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

// Parse numbers from a list where they are separated by commas or spaces
void parse_numbers(std::string list, std::vector<double> & values){

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
  double val;
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
    ("camera-type", po::value(&opt.camera_type), "Either pinhole [default] or opticalbar")
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
  parse_numbers(opt.lon_lat_values_str, opt.lon_lat_values);
  if (opt.lon_lat_values.size() < 6) 
    vw_throw( ArgumentErr() << "Expecting at least three longitude-latitude pairs.\n"
              << usage << general_options );

  // Parse the pixel values
  parse_numbers(opt.pixel_values_str, opt.pixel_values);
  
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


// Pack a pinhole or optical bar model's rotation and camera center into a vector
template <class CAM>
void camera_to_vector(CAM const& P, Vector<double> & C){

  Vector3 ctr = P.camera_center();
  Vector3 axis_angle = P.camera_pose().axis_angle();
  C.set_size(6);
  subvector(C, 0, 3) = ctr;
  subvector(C, 3, 3) = axis_angle;
}

// Pack a vector into a pinhole or optical bar model. We assume the model
// already has set its optical center, focal length, etc. 
template <class CAM>
void vector_to_camera(CAM & P, Vector<double> const& C){

  if (C.size() != 6) 
    vw_throw( LogicErr() << "Expecting a vector of size 6.\n" );

  Vector3 ctr      = subvector(C, 0, 3);
  Quat    rotation = axis_angle_to_quaternion(subvector(C, 3, 3));
  P.set_camera_center(ctr);
  P.set_camera_pose(rotation);
}


/// Find the camera model that best projects given xyz points into given pixels
template <class CAM>
class CameraSolveLMA : public vw::math::LeastSquaresModelBase<CameraSolveLMA<CAM> > {
  std::vector<vw::Vector3> const& m_xyz;
  CAM m_camera_model;
  mutable size_t m_iter_count;
  
public:

  typedef vw::Vector<double>    result_type;   // pixel residuals
  typedef vw::Vector<double, 6> domain_type;   // camera parameters (camera center and axis angle)
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of xyz to pixel pairs and a pinhole model
  CameraSolveLMA(std::vector<vw::Vector3> const& xyz,
                 CAM const& camera_model):
    m_xyz(xyz),
    m_camera_model(camera_model), m_iter_count(0){}

  /// Given the camera, project xyz into it
  inline result_type operator()( domain_type const& C ) const {

    // Create the camera model
    CAM camera_model = m_camera_model; // make a copy local to this function
    vector_to_camera(camera_model, C);      // update its parameters
    
    const size_t result_size = m_xyz.size() * 2;
    result_type result;
    result.set_size(result_size);
    for (size_t i = 0; i < m_xyz.size(); i++) {
      Vector2 pixel = camera_model.point_to_pixel(m_xyz[i]);
      result[2*i  ] = pixel[0];
      result[2*i+1] = pixel[1];
    }
  
    ++m_iter_count;
    
    return result;
  }
}; // End class CameraSolveLMA


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
      = interpolate(create_mask(dem, nodata_value), BilinearInterpolation(), ZeroEdgeExtension());
    
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
    
    // Prepare a pinhole camera or optical bar model with desired intrinsics but with
    // trivial position and orientation.
    Vector3 ctr(0, 0, 0);
    Matrix<double, 3, 3> rotation;
    rotation.set_identity();
    boost::shared_ptr<PinholeModel>    pinhole_cam;
    boost::shared_ptr<asp::camera::OpticalBarModel> opticalbar_cam;
    CameraModel const* cam_ptr;
    
    if (opt.camera_type == "opticalbar") {
      opticalbar_cam.reset(new asp::camera::OpticalBarModel(opt.sample_file));
      // Make sure the image size matches the input image file.
      opticalbar_cam->set_image_size(Vector2i(img.cols(), img.rows()));
      opticalbar_cam->set_optical_center(Vector2(img.cols()/2.0, img.rows()/2.0));
      cam_ptr = opticalbar_cam.get();
    } else {
      if (opt.sample_file != "")
        pinhole_cam.reset(new PinholeModel(opt.sample_file));
      else // Command line values
        pinhole_cam.reset(new PinholeModel(ctr, rotation, opt.focal_length, opt.focal_length,
                                          opt.optical_center[0], opt.optical_center[1],
                                          NULL, opt.pixel_pitch));
      cam_ptr = pinhole_cam.get();
    }
    
    // This is needed only for weird reflections
    //Vector3 u_vec, v_vec, w_vec;
    //cam.coordinate_frame(u_vec, v_vec, w_vec);
    //cam.set_coordinate_frame(u_vec, v_vec, w_vec);
    
    // Create fake points in space at given distance from this camera's center
    // and corresponding actual points on the ground.
    const double ht = 500000; // 500 km, just some height not too far from actual satellite height 
    vw::Matrix<double> in, out;
    in.set_size(3, num_lon_lat_pairs);
    out.set_size(3, num_lon_lat_pairs);
    for (int col = 0; col < in.cols(); col++) {
      Vector3 a = cam_ptr->camera_center(Vector2(0, 0)) +
        ht*cam_ptr->pixel_to_vector(Vector2(opt.pixel_values[2*col], opt.pixel_values[2*col+1]));
      for (int row = 0; row < in.rows(); row++) {
        in(row, col)  = a[row];
        out(row, col) = xyz_vec[col][row];
      }
    }

    // Apply a transform to the camera so that the fake points are on top of the real points
    double scale;
    asp::find_3D_affine_transform(in, out, rotation, ctr, scale);
    if (opt.camera_type == "opticalbar")
      opticalbar_cam->apply_transform(rotation, ctr, scale);
    else
      pinhole_cam->apply_transform(rotation, ctr, scale);

    // Print out some errors
    vw_out() << "The error between the projection of each ground "
             << "corner point into the coarse camera and its pixel value:\n";
    for (size_t corner_it = 0; corner_it < num_lon_lat_pairs; corner_it++) {
      vw_out () << "Corner and error: ("
                << opt.pixel_values[2*corner_it] << ' ' << opt.pixel_values[2*corner_it+1]
                << ") " <<  norm_2(cam_ptr->point_to_pixel(xyz_vec[corner_it]) -
                                   Vector2( opt.pixel_values[2*corner_it],
                                            opt.pixel_values[2*corner_it+1]))
                << std::endl;
    }
    
    // Solve a little optimization problem to make the points on the ground project
    // as much as possible exactly into the image corners.
    if (opt.refine_camera) {
      Vector<double> pixel_vec; // must copy to this structure
      pixel_vec.set_size(opt.pixel_values.size());
      for (size_t corner_it = 0; corner_it < pixel_vec.size(); corner_it++) 
        pixel_vec[corner_it] = opt.pixel_values[corner_it];
      const double abs_tolerance  = 1e-24;
      const double rel_tolerance  = 1e-24;
      const int    max_iterations = 2000;
      int status = 0;
      Vector<double> final_params;
      Vector<double> seed;
      
      if (opt.camera_type == "opticalbar") {
        CameraSolveLMA<asp::camera::OpticalBarModel> lma_model(xyz_vec, *opticalbar_cam);
        camera_to_vector(*opticalbar_cam, seed);
        final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
                                                status, abs_tolerance, rel_tolerance,
                                                max_iterations);
        vector_to_camera(*opticalbar_cam, final_params);
      } else {
        CameraSolveLMA<PinholeModel> lma_model(xyz_vec, *pinhole_cam);
        camera_to_vector(*pinhole_cam, seed);
        final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
                                                status, abs_tolerance, rel_tolerance,
                                                max_iterations);
        vector_to_camera(*pinhole_cam, final_params);
      }
      if (status < 1)
        vw_out() << "The Levenberg-Marquardt solver failed. Results may be inaccurate.\n";
      

      vw_out() << "The error between the projection of each ground "
               << "corner point into the refined camera and its pixel value:\n";
      for (size_t corner_it = 0; corner_it < num_lon_lat_pairs; corner_it++) {
        vw_out () << "Corner and error: ("
                  << opt.pixel_values[2*corner_it] << ' ' << opt.pixel_values[2*corner_it+1]
                  << ") " <<  norm_2(cam_ptr->point_to_pixel(xyz_vec[corner_it]) -
                                     Vector2( opt.pixel_values[2*corner_it],
                                              opt.pixel_values[2*corner_it+1]))
                  << std::endl;
      }
    } // End camera refinement case
    
    vw_out() << "Writing: " << opt.camera_file << std::endl;
    if (opt.camera_type == "opticalbar")
      opticalbar_cam->write(opt.camera_file);
    else
      pinhole_cam->write(opt.camera_file);

    if (write_gcp) {
      vw_out() << "Writing: " << opt.gcp_file << std::endl;
      std::ofstream fs(opt.gcp_file.c_str());
      fs << gcp.str();
      fs.close();
    }
    
  } ASP_STANDARD_CATCHES;
    
  return 0;
}

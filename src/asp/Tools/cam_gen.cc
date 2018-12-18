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

// Create a pinhole camera model based on intrinsics, image corner
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

struct Options : public vw::cartography::GdalWriteOptions {
  string image_file, camera_file, lon_lat_corners, datum_str, reference_dem, frame_index, gcp_file;
  double focal_length;
  Vector2 optical_center;
  std::vector<double> image_corners;
  Options(): focal_length(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-camera-file,o", po::value(&opt.camera_file), "Specify the output camera file with a .tsai extension.")
    ("lon-lat-corners", po::value(&opt.lon_lat_corners)->default_value(""), "A (quoted) string listing eight numbers, separated by commas or spaces, having the longitude and latitude (in this order) of each image corner. The corners are traversed in the order 0,0 w,0, w,h, 0,h where w and h are the image width and height.")
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the longitude and latitude. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("reference-dem", po::value(&opt.reference_dem)->default_value(""),
     "Use this DEM to infer the heights above datum of the image corners. Assume zero height if not provided.")
    ("focal-length", po::value(&opt.focal_length)->default_value(0),
     "The camera focal length, in pixels.")
    ("optical-center", po::value(&opt.optical_center)->default_value(Vector2i(0,0),"0 0"),
     "The camera optical center, in pixels.")
    ("frame-index", po::value(&opt.frame_index)->default_value(""),
     "A file used to look up the longitude and latitude of image corners based on the image name, in the format provided by the SkyBox video product.")
    ("gcp-file", po::value(&opt.gcp_file)->default_value(""),
     "If provided, save the image corner coordinates and heights in the GCP format to this file.");
  
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

  std::string ext = get_extension(opt.camera_file);
  if (ext != ".tsai") 
    vw_throw( ArgumentErr() << "The output camera file must end with .tsai.\n"
	      << usage << general_options );

  // If we cannot read the data from a DEM, must specify a lot of things.
  if (opt.reference_dem.empty() && opt.datum_str.empty())
    vw_throw( ArgumentErr() << "Must provide either a reference DEM or a datum.\n"
	      << usage << general_options );
    
  // Parse the image corners
  opt.image_corners.clear();
  std::string oldStr = ",", newStr = " ";
  size_t pos = 0;
  while((pos = opt.lon_lat_corners.find(oldStr, pos)) != std::string::npos){
    opt.lon_lat_corners.replace(pos, oldStr.length(), newStr);
    pos += newStr.length();
  }
  std::istringstream is(opt.lon_lat_corners);
  double val;
  while (is >> val)
    opt.image_corners.push_back(val);
  if (opt.image_corners.size() != 8)
    vw_throw( ArgumentErr() << "Expecting 8 values for the image corners.\n"
	      << usage << general_options );


  if (opt.focal_length <= 0 || opt.optical_center[0] <= 0 || opt.optical_center[1] <= 0) 
    vw_throw( ArgumentErr() << "Must provide positive focal length and optical center values.\n"
	      << usage << general_options );
  
  // Create the output directory
  vw::create_out_dir(opt.camera_file);
}

// Pack a pinhole model's rotation and camera center into a vector
void camera_to_vector(PinholeModel const& P, Vector<double> & C){

  Vector3 ctr = P.camera_center();
  Vector3 axis_angle = P.camera_pose().axis_angle();
  C.set_size(6);
  subvector(C, 0, 3) = ctr;
  subvector(C, 3, 3) = axis_angle;
}

// Pack a vector into a pinhole model. We assume the model
// already has set its optical center, focal length, etc. 
void vector_to_camera(PinholeModel & P, Vector<double> const& C){

  if (C.size() != 6) 
    vw_throw( LogicErr() << "Expecting a vector of size 6.\n" );

  Vector3 ctr      = subvector(C, 0, 3);
  Quat    rotation = axis_angle_to_quaternion(subvector(C, 3, 3));
  P.set_camera_center(ctr);
  P.set_camera_pose(rotation);
}

/// Find the camera model that best projects given xyz points into given pixels
class CameraSolveLMA : public vw::math::LeastSquaresModelBase<CameraSolveLMA> {
  std::vector<vw::Vector3> const& m_xyz;
  PinholeModel m_camera_model;
  mutable size_t m_iter_count;
  
public:

  typedef vw::Vector<double>    result_type;   // pixel residuals
  typedef vw::Vector<double, 6> domain_type;   // camera parameters (camera center and axis angle)
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of xyz to pixel pairs and a pinhole model
  CameraSolveLMA(std::vector<vw::Vector3> const& xyz,
              PinholeModel const& camera_model):
    m_xyz(xyz),
    m_camera_model(camera_model), m_iter_count(0){}

  /// Given the camera, project xyz into it
  inline result_type operator()( domain_type const& C ) const {

    // Create the camera model
    PinholeModel camera_model = m_camera_model; // make a copy local to this function
    vector_to_camera(camera_model, C);          // update its parameters
    
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
    bool has_dem = false;
    if (opt.reference_dem != "") {
      dem = DiskImageView<float>(opt.reference_dem);
      bool ans = read_georeference(geo, opt.reference_dem);
      if (!ans) 
	vw_throw( ArgumentErr() << "Could not read the georeference from dem: "
		  << opt.reference_dem << ".\n");

      datum = geo.datum(); // Read this in for completeness
      has_dem = true;
    }else{
      datum = vw::cartography::Datum(opt.datum_str); 
      vw_out() << "No reference DEM provided. Will use zero heights above the datum:\n" <<
	datum << std::endl;
    }

    // Prepare the DEM for interpolation
    ImageViewRef<float> interp_dem = interpolate(dem, BilinearInterpolation(), ZeroEdgeExtension());
    
    //double v[] = {-122.389,37.6273,-122.354,37.626,-122.358,37.6125,-122.393,37.6138};
    //double v[] = {-122.386,37.626,-122.358,37.6223,-122.361,37.6128,-122.389,37.6165}; // img1
    //double v[] = {-122.387,37.6263,-122.357,37.6231,-122.36,37.6129,-122.39,37.6161};  // img2
    //double v[] = {-122.387,37.6268,-122.356,37.6241,-122.359,37.6131,-122.39,37.6157}; // img3
    //double  v[] = {-122.388,37.627,-122.355,37.625,-122.359,37.6128,-122.392,37.6149}; // img4
    //double v[] = {-122.389,37.6273,-122.354,37.626,-122.358,37.6125,-122.393,37.6138}; // img5
    //double v1[] = {-106.101,39.4792,-106.064,39.4809,-106.068,39.4680,-106.105,39.4665}; // v1
    //double v2[] = {-106.103,39.4786,-106.061,39.4844,-106.066,39.4683,-106.107,39.4641}; // v2
    //double v3[] = {-106.105,39.478,-106.0570,39.4885,-106.064,39.4683,-106.111,39.4596}; // v3
    //double v4[] = {-106.107,39.4788,-106.053,39.4941,-106.061,39.4687,-106.114,39.4558}; // v4
    //double v5[] = {-106.109,39.4808,-106.048,39.5016,-106.058,39.4689,-106.117,39.4526}; // v5

    DiskImageView<float> img(opt.image_file);
    int wid = img.cols(), hgt = img.rows();
    if (wid <= 0 || hgt <= 0) 
      vw_throw( ArgumentErr() << "Could not read an image with positive dimensions from: "
		<< opt.image_file << ".\n");

    Vector2 pix;
    Vector3 llh, xyz;
    std::vector<Vector3> xyz_vec;

    // If to write a gcp file
    std::ostringstream gcp;
    gcp.precision(17);
    bool write_gcp = (opt.gcp_file != "");

    double corner_x[] = {0, wid, wid, 0};
    double corner_y[] = {0, 0,   hgt, hgt};
  
    for (size_t corner_it = 0; corner_it < 4; corner_it++) {

      // Get the height from the DEM if possible
      llh[0] = opt.image_corners[2*corner_it+0];
      llh[1] = opt.image_corners[2*corner_it+1];
      double height = 0.0;
      if (has_dem) {
	pix = geo.lonlat_to_pixel(subvector(llh, 0, 2));
	if (pix[0] >= 0 && pix[0] <= interp_dem.cols() - 1 - BilinearInterpolation::pixel_buffer &&
	    pix[1] >= 0 && pix[1] <= interp_dem.rows() - 1 - BilinearInterpolation::pixel_buffer) {
	  height = interp_dem(pix[0], pix[1]);
	}else
	  vw_out() << "Out of bounds for the DEM, will use a height of 0 above the datum.";
      }
      
      llh[2] = height;
      vw_out() << "Lon-lat-height for corner: ("
	       << corner_x[corner_it] << ' ' << corner_y[corner_it] << ") is "
	       << llh[0] << ' ' << llh[1] << ' ' << llh[2] << std::endl;
    
      xyz = datum.geodetic_to_cartesian(llh);
      xyz_vec.push_back(xyz);

      if (write_gcp)
        gcp << corner_it << ' ' << llh[1] << ' ' << llh[0] << ' ' << llh[2] << ' '
            << 1 << ' ' << 1 << ' ' << 1 << ' ' << opt.image_file << ' '
            << corner_x[corner_it] << ' ' << corner_y[corner_it] << ' '
            << 1 << ' ' << 1 << std::endl;
    }
    
    // We know that the satellite is at 502.689 km above ground.
    // The swath length is 2.5 km which is 2560 pixels.
    // This implies a focal length in pixels of
    // f = 502.689 * 2560 / 2.5 = 514753.536
    // If we divide focal length = 3.6 m by pixel pitch of
    // of 6.5 * 1e-6 m, we get 553846.153846. We will use this one.
    
    // We scaled the focal length by pixel pitch, so we assume the pixel pitch to be 1.
    double pixel_pitch = 1.0; 

    // Prepare a pinhole camera model with desired intrinsics but with
    // trivial position and orientation.
    Vector3 ctr(0, 0, 0);
    Matrix<double, 3, 3> rotation;
    rotation.set_identity();
    PinholeModel cam(ctr, rotation, opt.focal_length, opt.focal_length,
		     opt.optical_center[0], opt.optical_center[1], NULL, pixel_pitch);

    // This is needed only for weird reflections
    //Vector3 u_vec, v_vec, w_vec;
    //cam.coordinate_frame(u_vec, v_vec, w_vec);
    //cam.set_coordinate_frame(u_vec, v_vec, w_vec);
    
    // Create fake points in space at given distance from this camera's center
    // and corresponding actual points on the ground.
    double ht = 500000; // 500 km, just some height not too far from actual satellite height 
    vw::Matrix<double, 3, 4> in, out;
    for (int col = 0; col < in.cols(); col++) {
      Vector3 a = cam.camera_center(Vector2(0, 0)) +
        ht*cam.pixel_to_vector(Vector2(corner_x[col], corner_y[col]));
      for (int row = 0; row < in.rows(); row++) {
	in(row, col) = a[row];
	out(row, col) = xyz_vec[col][row];
      }
    }

    // Apply a transform to the camera so that the fake points are on top of the real points
    double scale;
    asp::find_3D_affine_transform(in, out, rotation, ctr, scale);
    cam.apply_transform(rotation, ctr, scale);

    // Solve a little optimization problem to make the points on the ground project
    // as much as possible exactly into the image corners.
    Vector<double, 8> pixel_vec;
    for (size_t corner_it = 0; corner_it < 4; corner_it++) {
      pixel_vec[2*corner_it+0] = corner_x[corner_it];
      pixel_vec[2*corner_it+1] = corner_y[corner_it];
    }
    CameraSolveLMA lma_model(xyz_vec, cam);
    Vector<double> seed;
    camera_to_vector(cam, seed);
    const double abs_tolerance  = 1e-24;
    const double rel_tolerance  = 1e-24;
    const int    max_iterations = 2000;
    int status = 0;
    Vector<double> final_params;
    final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
                                             status, abs_tolerance, rel_tolerance,
                                             max_iterations);
    if (status < 1)
      vw_out() << "The Levenberg-Marquardt solver failed. Results may be inaccurate.\n";
    vector_to_camera(cam, final_params);

    vw_out() << "The error between the projection of each ground "
             << "corner point into the camera and its pixel value:\n";
    for (size_t corner_it = 0; corner_it < 4; corner_it++) {
      vw_out () << "Corner and error: ("
                << corner_x[corner_it] << ' ' << corner_y[corner_it]
                << ") " <<  norm_2(cam.point_to_pixel(xyz_vec[corner_it]) -
                                   Vector2( corner_x[corner_it],  corner_y[corner_it]))
                << std::endl;
    }
    
    std::cout << "Writing: " << opt.camera_file << std::endl;
    cam.write(opt.camera_file);

    if (write_gcp) {
      vw_out() << "Writing: " << opt.gcp_file << std::endl;
      std::ofstream fs(opt.gcp_file.c_str());
      fs << gcp.str();
      fs.close();
    }
    
  } ASP_STANDARD_CATCHES;
    
  return 0;
}

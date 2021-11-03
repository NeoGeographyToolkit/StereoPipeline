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
#include <vw/Cartography/CameraBBox.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/Geometry.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/EigenUtils.h>
#include <vw/Camera/OpticalBarModel.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

// Solve for best fitting camera that projects given xyz locations at
// given pixels. If cam_weight > 0, try to constrain the camera height
// above datum at the value of cam_height.
// TODO: The logic with the camera height did not work. Wipe it.
// Use the original functions in VW and move some of the newer
// code from here to there.
template <class CAM>
class CameraSolveLMA_Ht: public vw::math::LeastSquaresModelBase<CameraSolveLMA_Ht<CAM> > {
  std::vector<vw::Vector3> const& m_xyz;
  CAM m_camera_model;
  double m_cam_height, m_cam_weight;
  vw::cartography::Datum m_datum;
  mutable size_t m_iter_count;
  
public:

  typedef vw::Vector<double>    result_type;   // pixel residuals
  typedef vw::Vector<double, 6> domain_type;   // camera parameters (camera center and axis angle)
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of xyz to pixel pairs and a pinhole model
  CameraSolveLMA_Ht(std::vector<vw::Vector3> const& xyz,
		  CAM const& camera_model,
		  double cam_height, double cam_weight,
		  vw::cartography::Datum const& datum):
    m_xyz(xyz),
    m_camera_model(camera_model), m_iter_count(0),
    m_cam_height(cam_height), m_cam_weight(cam_weight),
    m_datum(datum) {}

  /// Given the camera, project xyz into it
  inline result_type operator()( domain_type const& C ) const {

    // Create the camera model
    CAM camera_model = m_camera_model;  // make a copy local to this function
    vector_to_camera(camera_model, C);  // update its parameters

    int xyz_len = m_xyz.size();
    size_t result_size = xyz_len * 2;
    if (m_cam_weight > 0) {
      result_size += 1;
    }

    // See where the xyz coordinates project into the camera.
    result_type result;
    result.set_size(result_size);
    for (size_t i = 0; i < xyz_len; i++) {
      Vector2 pixel = camera_model.point_to_pixel(m_xyz[i]);
      result[2*i  ] = pixel[0];
      result[2*i+1] = pixel[1];
    }

    // Try to make the camera stay at given height
    if (m_cam_weight > 0) {
      Vector3 cam_ctr = subvector(C, 0, 3);
      Vector3 llh = m_datum.cartesian_to_geodetic(cam_ctr);
      result[2*xyz_len] = m_cam_weight*(llh[2] - m_cam_height);
    }
    
    ++m_iter_count;
    
    return result;
  }
}; // End class CameraSolveLMA_Ht

/// Find the best camera that fits the current GCP
void fit_camera_to_xyz_ht(bool parse_ecef,
			  Vector3 const& parsed_camera_center,
			  std::string const& camera_type,
			  bool refine_camera, 
			  std::vector<Vector3> const& xyz_vec,
			  std::vector<double> const& pixel_values,
			  double cam_height, double cam_weight,
			  vw::cartography::Datum const& datum,
			  bool verbose,
			  boost::shared_ptr<CameraModel> & out_cam){
  
  // Create fake points in space at given distance from this camera's
  // center and corresponding actual points on the ground.  Use 500
  // km, just some height not too far from actual satellite height.
  const double ht = 500000; 
  int num_pts = pixel_values.size()/2;
  vw::Matrix<double> in, out;
  in.set_size(3, num_pts);
  out.set_size(3, num_pts);
  for (int col = 0; col < in.cols(); col++) {
    Vector3 a = out_cam->camera_center(Vector2(0, 0)) +
      ht*out_cam->pixel_to_vector(Vector2(pixel_values[2*col], pixel_values[2*col+1]));
    for (int row = 0; row < in.rows(); row++) {
      in(row, col)  = a[row];
      out(row, col) = xyz_vec[col][row];
    }
  }
  
  // Apply a transform to the camera so that the fake points are on top of the real points
  Matrix<double, 3, 3> rotation;
  Vector3 translation;
  double scale;
  find_3D_affine_transform(in, out, rotation, translation, scale);
  if (camera_type == "opticalbar")
    ((vw::camera::OpticalBarModel*)out_cam.get())->apply_transform(rotation,
								   translation, scale);
  else{
    ((PinholeModel*)out_cam.get())->apply_transform(rotation, translation, scale);
    if (parse_ecef) {
      // Overwrite the solved camera center with what is found from the
      // frame index file.
      ((PinholeModel*)out_cam.get())->set_camera_center(parsed_camera_center);
    }
  }
  
  // Print out some errors
  if (verbose) {
    vw_out() << "The error between the projection of each ground "
	     << "corner point into the coarse camera and its pixel value:\n";
    for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
      vw_out () << "Corner and error: ("
		<< pixel_values[2*corner_it] << ' ' << pixel_values[2*corner_it+1]
		<< ") " <<  norm_2(out_cam.get()->point_to_pixel(xyz_vec[corner_it]) -
				 Vector2( pixel_values[2*corner_it],
					  pixel_values[2*corner_it+1]))
		<< std::endl;
    }
  }
  
  // Solve a little optimization problem to make the points on the ground project
  // as much as possible exactly into the image corners.
  if (refine_camera) {
    Vector<double> out_vec; // must copy to this structure
    int residual_len = pixel_values.size();

    // If we enforce the camera height, add another residual
    if (cam_weight > 0.0) 
      residual_len += 1;

    // Copy the image pixels
    out_vec.set_size(residual_len);
    for (size_t corner_it = 0; corner_it < pixel_values.size(); corner_it++) 
      out_vec[corner_it] = pixel_values[corner_it];

    if (cam_weight > 0.0) 
      out_vec[residual_len - 1] = 0.0;
        
    const double abs_tolerance  = 1e-24;
    const double rel_tolerance  = 1e-24;
    const int    max_iterations = 2000;
    int status = 0;
    Vector<double> final_params;
    Vector<double> seed;
      
    if (camera_type == "opticalbar") {
      CameraSolveLMA_Ht<vw::camera::OpticalBarModel>
	lma_model(xyz_vec, *((vw::camera::OpticalBarModel*)out_cam.get()),
		  cam_height, cam_weight, datum);
      camera_to_vector(*((vw::camera::OpticalBarModel*)out_cam.get()), seed);
      final_params = math::levenberg_marquardt(lma_model, seed, out_vec,
					       status, abs_tolerance, rel_tolerance,
					       max_iterations);
      vector_to_camera(*((vw::camera::OpticalBarModel*)out_cam.get()), final_params);
    } else {
      CameraSolveLMA_Ht<PinholeModel> lma_model(xyz_vec, *((PinholeModel*)out_cam.get()),
		  cam_height, cam_weight, datum);
      camera_to_vector(*((PinholeModel*)out_cam.get()), seed);
      final_params = math::levenberg_marquardt(lma_model, seed, out_vec,
					       status, abs_tolerance, rel_tolerance,
					       max_iterations);
      vector_to_camera(*((PinholeModel*)out_cam.get()), final_params);
    }
    if (status < 1)
      vw_out() << "The Levenberg-Marquardt solver failed. Results may be inaccurate.\n";

    if (verbose) {
      vw_out() << "The error between the projection of each ground "
	       << "corner point into the refined camera and its pixel value:\n";
      for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
	vw_out () << "Corner and error: ("
		  << pixel_values[2*corner_it] << ' ' << pixel_values[2*corner_it+1]
		  << ") " <<  norm_2(out_cam.get()->point_to_pixel(xyz_vec[corner_it]) -
				     Vector2( pixel_values[2*corner_it],
					      pixel_values[2*corner_it+1]))
		  << std::endl;
      }
    }
    
  } // End camera refinement case
}

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
  std::string image_file, camera_file, lon_lat_values_str, pixel_values_str, datum_str,
    reference_dem, frame_index, gcp_file, camera_type, sample_file, input_camera,
    stereo_session, bundle_adjust_prefix, parsed_cam_ctr_str, parsed_cam_quat_str;
  double focal_length, pixel_pitch, gcp_std, height_above_datum,
    cam_height, cam_weight;
  Vector2 optical_center;
  std::vector<double> lon_lat_values, pixel_values;
  bool refine_camera, parse_eci, parse_ecef; 
  Options(): focal_length(-1), pixel_pitch(-1), gcp_std(1), height_above_datum(0), refine_camera(false), cam_height(0), cam_weight(0) {}
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
     "The standard deviation for each GCP pixel, if saving a GCP file. A smaller value suggests a more reliable measurement, hence will be given more weight.")
    ("cam-height", po::value(&opt.cam_height)->default_value(0),
     "If both this and --cam-weight are positive, enforce that the output camera is at this height above datum. For SkySat, if not set, read this from the frame index. Highly experimental.")
    ("cam-weight", po::value(&opt.cam_weight)->default_value(0),
     "If positive, try to enforce the option --cam-height with this weight (bigger weight means try harder to enforce). Highly experimental.")
    ("parse-eci", po::bool_switch(&opt.parse_eci)->default_value(false),
     "Create cameras based on ECI positions and orientations (not working).")
    ("parse-ecef", po::bool_switch(&opt.parse_ecef)->default_value(false),
     "Create cameras based on ECEF position (but not orientation).")
    ("input-camera", po::value(&opt.input_camera)->default_value(""),
     "Create the output pinhole camera approximating this camera.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     " Select the input camera model type. Normally this is auto-detected, but may need to be specified if the input camera model is in XML format. See the doc for options.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust when providing an input camera.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("image-file", po::value(&opt.image_file));

  po::positional_options_description positional_desc;
  positional_desc.add("image-file",1);

  std::string usage("[options] <image-file> -o <camera-file>");
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
    vw_throw( ArgumentErr() << "Cannot specify both the frame index file "
	      << "and the lon-lat corners.\n"
              << usage << general_options );

  if (opt.frame_index != "") {
    // Parse the frame index to extract opt.lon_lat_values_str.
    // Look for a line having this image, and search for "POLYGON" followed by spaces and "((".
    boost::filesystem::path p(opt.image_file); 
    std::string image_base = p.stem().string(); // strip the directory name and suffix
    std::ifstream file( opt.frame_index.c_str() );
    std::string line;
    std::string beg1 = "POLYGON";
    std::string beg2 = "((";
    std::string end = "))";
    while ( getline(file, line, '\n') ) {
      if (line.find(image_base) == 0) {
        // Find POLYGON first.
        int beg_pos = line.find(beg1);
        if (beg_pos == std::string::npos)
          vw_throw( ArgumentErr() << "Cannot find " << beg1 << " in line: " << line << ".\n");
        beg_pos += beg1.size();

        // Move forward skipping any spaces until finding "(("
        beg_pos = line.find(beg2, beg_pos);
        if (beg_pos == std::string::npos)
          vw_throw( ArgumentErr() << "Cannot find " << beg2 << " in line: " << line << ".\n");
        beg_pos += beg2.size();

        // Find "))"
        int end_pos = line.find(end, beg_pos);
        if (end_pos == std::string::npos)
          vw_throw( ArgumentErr() << "Cannot find " << end << " in line: " << line << ".\n");
        opt.lon_lat_values_str = line.substr(beg_pos, end_pos - beg_pos);
        vw_out() << "Parsed the lon-lat corner values: " << opt.lon_lat_values_str
		 << std::endl;

	if (opt.parse_eci && opt.parse_ecef)
	  vw_throw( ArgumentErr() << "Cannot parse both ECI end ECEF at the same time.\n");
	
	// Also parse the camera height constraint, unless manually specified
	if (opt.cam_weight > 0 || opt.parse_eci || opt.parse_ecef) {
	  std::vector<std::string> vals;
	  parse_values<std::string>(line, vals);
	  
	  if (vals.size() < 12) 
	    vw_throw( ArgumentErr() << "Could not parse 12 values from: " << line << ".\n");

	  // Extract the ECI or ECEF coordinates of camera
	  // center. Keep them as string until we can convert to
	  // height above datum.
	  
	  if (opt.parse_eci) {
	    std::string x = vals[5];
	    std::string y = vals[6];
	    std::string z = vals[7];
	    opt.parsed_cam_ctr_str = x + " " + y + " " + z;
	    vw_out() << "Parsed the ECI camera center in km: "
		     << opt.parsed_cam_ctr_str <<".\n";
	    
	    std::string q0 = vals[8];
	    std::string q1 = vals[9];
	    std::string q2 = vals[10];
	    std::string q3 = vals[11];
	    opt.parsed_cam_quat_str = q0 + " " + q1 + " " + q2 + " " + q3;
	    vw_out() << "Parsed the ECI quaternion: "
		     << opt.parsed_cam_quat_str <<".\n";
	  }
	  
	  if (opt.parse_ecef) {
	    if (vals.size() < 19) 
	      vw_throw( ArgumentErr() << "Could not parse 19 values from: " << line << ".\n");
	    
	    std::string x = vals[12];
	    std::string y = vals[13];
	    std::string z = vals[14];
	    opt.parsed_cam_ctr_str = x + " " + y + " " + z;
	    vw_out() << "Parsed the ECEF camera center in km: "
		     << opt.parsed_cam_ctr_str <<".\n";
	    
	    std::string q0 = vals[15];
	    std::string q1 = vals[16];
	    std::string q2 = vals[17];
	    std::string q3 = vals[18];
	    opt.parsed_cam_quat_str = q0 + " " + q1 + " " + q2 + " " + q3;
	    vw_out() << "Parsed the ECEF quaternion: "
		     << opt.parsed_cam_quat_str <<".\n";
	  }
	  
	}
	
        break;
      }
    }
    if (opt.lon_lat_values_str == "")
      vw_throw( ArgumentErr() << "Could not parse the entry for " << image_base
                << " in file: " << opt.frame_index << ".\n");
  }
    
  // Parse the pixel values
  parse_values<double>(opt.pixel_values_str, opt.pixel_values);

  // If none were provided, use the image corners
  if (opt.pixel_values.empty()) {
    DiskImageView<float> img(opt.image_file);
    int wid = img.cols(), hgt = img.rows();
    if (wid <= 0 || hgt <= 0) 
      vw_throw( ArgumentErr() << "Could not read an image with positive dimensions from: "
		<< opt.image_file << ".\n");
    
    // populate the corners
    double arr[] = {0.0, 0.0, (double)wid, 0.0, (double)wid, (double)hgt, 0.0, (double)hgt};
    for (size_t it  = 0; it < sizeof(arr)/sizeof(double); it++) 
      opt.pixel_values.push_back(arr[it]);

    // Add inner points for robustness
    if (opt.input_camera != "") {
      double b = 0.25, e = 0.75;
      double arr[] = {b*wid, b*hgt, e*wid, b*hgt, e*wid, e*hgt, b*wid, e*hgt};
      for (size_t it  = 0; it < sizeof(arr)/sizeof(double); it++) 
	opt.pixel_values.push_back(arr[it]);
    }
    
  }
    
  // Parse the lon-lat values
  if (opt.input_camera == "") {
    parse_values<double>(opt.lon_lat_values_str, opt.lon_lat_values);
    // Bug fix for some frame_index files repeating the first point at the end
    int len = opt.lon_lat_values.size();
    if (opt.frame_index != "" && opt.lon_lat_values.size() == opt.pixel_values.size() + 2 &&
        len >= 2 && opt.lon_lat_values[0] == opt.lon_lat_values[len - 2] &&
        opt.lon_lat_values[1] == opt.lon_lat_values[len - 1]) {
      opt.lon_lat_values.pop_back();
      opt.lon_lat_values.pop_back();
    }
  }
  
  // Note that optical center can be negative (for some SkySat products).
  if ( opt.sample_file == "" && (opt.focal_length <= 0 || opt.pixel_pitch <= 0))
    vw_throw( ArgumentErr() << "Must provide positive focal length"
              << "and pixel pitch values OR a sample file.\n");
  
  // Create the output directory
  vw::create_out_dir(opt.camera_file);
} // End function handle_arguments

// Form a camera based on info the user provided
void manufacture_cam(Options const& opt, int wid, int hgt,
		     boost::shared_ptr<CameraModel> & out_cam){

  if (opt.camera_type == "opticalbar") {
    boost::shared_ptr<vw::camera::OpticalBarModel> opticalbar_cam;
    opticalbar_cam.reset(new vw::camera::OpticalBarModel(opt.sample_file));
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

// TODO: Wipe this logic and use RayDEMIntersectionLMA from VW.
// That one is also terrible code which needs to be replaced with a
// proper root-finding algorithm
// and use it. And this code should be moved to VW.
// https://github.com/NeoGeographyToolkit/StereoPipeline/issues/267
namespace vw {
  namespace cartography {

  // Define an LMA model to solve for a DEM intersecting a ray. The
  // variable of optimization is position on the ray. The cost
  // function is difference between datum height and DEM height at
  // current point on the ray.
  template <class DEMImageT>
  class RayDEMIntersectionLMA2 : public math::LeastSquaresModelBase< RayDEMIntersectionLMA2< DEMImageT > > {

    // TODO: Why does this use EdgeExtension if Helper() restricts access to the bounds?
    InterpolationView<EdgeExtensionView<DEMImageT, ConstantEdgeExtension>,
                      BilinearInterpolation> m_dem;
    GeoReference m_georef;
    Vector3      m_camera_ctr;
    Vector3      m_camera_vec;
    bool         m_treat_nodata_as_zero;

    /// Provide safe interaction with DEMs that are scalar
    /// - If m_dem(x,y) is in bounds, return the interpolated value.
    /// - Otherwise return 0 or big_val()
    template <class PixelT>
    typename boost::enable_if< IsScalar<PixelT>, double >::type
    inline Helper( double x, double y ) const {
      if ( (0 <= x) && (x <= m_dem.cols() - 1) && // for interpolation
           (0 <= y) && (y <= m_dem.rows() - 1) ){
        PixelT val = m_dem(x, y);
        if (is_valid(val)) return val;
      }
      if (m_treat_nodata_as_zero) return 0;
      return big_val();
    }

    /// Provide safe interaction with DEMs that are compound
    template <class PixelT>
    typename boost::enable_if< IsCompound<PixelT>, double>::type
    inline Helper( double x, double y ) const {
      if ( (0 <= x) && (x <= m_dem.cols() - 1) && // for interpolation
           (0 <= y) && (y <= m_dem.rows() - 1) ){
        PixelT val = m_dem(x, y);
        if (is_valid(val)) return val[0];
      }
      if (m_treat_nodata_as_zero) return 0;
      return big_val();
    }

  public:
    typedef Vector<double, 1> result_type;
    typedef Vector<double, 1> domain_type;
    typedef Matrix<double>    jacobian_type; ///< Jacobian form. Auto.

    /// Return a very large error to penalize locations that fall off the edge of the DEM.
    inline double big_val() const {
      // Don't make this too big as in the LMA algorithm it may get squared and may cause overflow.
      return 1.0e+50;
    }

    /// Constructor
    RayDEMIntersectionLMA2(ImageViewBase<DEMImageT> const& dem_image,
                          GeoReference const& georef,
                          Vector3 const& camera_ctr,
                          Vector3 const& camera_vec,
                          bool treat_nodata_as_zero
                          )
      : m_dem(interpolate(dem_image)), m_georef(georef),
        m_camera_ctr(camera_ctr), m_camera_vec(camera_vec),
        m_treat_nodata_as_zero(treat_nodata_as_zero){}

    /// Evaluator. See description above.
    inline result_type operator()( domain_type const& len ) const {
      // The proposed intersection point
      Vector3 xyz = m_camera_ctr + len[0]*m_camera_vec;

      // Convert to geodetic coordinates, then to DEM pixel coordinates
      Vector3 llh = m_georef.datum().cartesian_to_geodetic( xyz );
      Vector2 pix = m_georef.lonlat_to_pixel( Vector2( llh.x(), llh.y() ) );
      
      // Return a measure of the elevation difference between the DEM and the guess
      // at its current location.
      result_type result;
      result[0] = Helper<typename DEMImageT::pixel_type >(pix.x(),pix.y()) - llh[2];
      return result;
    }
  };

    
  // Intersect the ray going from the given camera pixel with a DEM.
  // The return value is a Cartesian point. If the ray goes through a
  // hole in the DEM where there is no data, we return no-intersection
  // or intersection with the datum, depending on whether the variable
  // treat_nodata_as_zero is false or true.
  template <class DEMImageT>
  Vector3 camera_pixel_to_dem_xyz2(Vector3 const& camera_ctr, Vector3 const& camera_vec,
                                  ImageViewBase<DEMImageT> const& dem_image,
                                  GeoReference const& georef,
                                  bool treat_nodata_as_zero,
                                  bool & has_intersection,
                                  double height_error_tol = 1e-1,  // error in DEM height
                                  double max_abs_tol      = 1e-14, // abs cost fun change b/w iters
                                  double max_rel_tol      = 1e-14,
                                  int num_max_iter        = 100,
                                  Vector3 xyz_guess       = Vector3()
                                  ){

    // This is a very fragile function and things can easily go wrong. 
    try {
      has_intersection = false;
      RayDEMIntersectionLMA2<DEMImageT> model(dem_image, georef, camera_ctr,
                                             camera_vec, treat_nodata_as_zero);

      Vector3 xyz;
      if ( xyz_guess == Vector3() ){ // If no guess provided
        // Intersect the ray with the datum, this is a good initial guess.
        xyz = datum_intersection(georef.datum(), camera_ctr, camera_vec);

        if ( xyz == Vector3() ) { // If we failed to intersect the datum, give up!
          has_intersection = false;
          return Vector3();
        }
      }else{ // User provided guess
        xyz = xyz_guess;
      }

      // Length along the ray from camera center to datum intersection point
      Vector<double, 1> base_len, len;
      double smallest_error_pos = std::numeric_limits<double>::max();
      double best_len_pos = std::numeric_limits<double>::max();
      double smallest_error_neg = std::numeric_limits<double>::max();
      double best_len_neg = std::numeric_limits<double>::max();
      bool success_pos = false, success_neg = false;
      
      // If the ray intersects the datum at a point which does not
      // correspond to a valid location in the DEM, wiggle that point
      // along the ray until hopefully it does. Store the value that
      // is closest to where that ray will intersect the DEM. Once
      // that value is located, it is helpful to repeat this logic one
      // more time, this time around the best guess found so far.
      // Hence two outer passes. The value xyz is updated at each
      // pass. The idea here is that the closer one gets to the true
      // solution, the likelier the LM solver will converge.
      for (int outer_pass = 0; outer_pass <= 0; outer_pass++){
	
	base_len[0] = norm_2(xyz - camera_ctr);

      
	const double radius     = norm_2(xyz); // Radius from XYZ coordinate center
	const int    ITER_LIMIT = 10; // There are two solver attempts per iteration
	const double small      = radius*0.02/( 1 << (ITER_LIMIT-1) ); // Wiggle
	for (int i = 0; i <= ITER_LIMIT; i++){
	  // Gradually expand delta until on final iteration it is == radius*0.02
	  double delta = 0;
	  if (i > 0)
	    delta = small*( 1 << (i-1) );

	  for (int k = -1; k <= 1; k += 2){ // For k==-1, k==1
	    len[0] = base_len[0] + k*delta; // Ray guess length +/- 2% planetary radius
	    // Use our model to compute the height diff at this length

	    Vector<double, 1> height_diff = model(len);
	  
	    if ( std::abs(height_diff[0]) < (model.big_val()/10.0) ){
	      has_intersection = true;
	    }else{
	      continue;
	    }
	    //if (i == 0) break; // When k*delta==0, no reason to do both + and -!

	    if (height_diff[0] < 0 && std::abs(height_diff[0]) < smallest_error_neg){
	      
	      smallest_error_neg = std::abs(height_diff[0]);
	      best_len_neg = len[0];
	      xyz = camera_ctr + best_len_neg*camera_vec; // broken!!!
	      success_neg = true;
	    }else{
	    }

	    if (height_diff[0] >=0 && std::abs(height_diff[0]) < smallest_error_pos){
	      
	      smallest_error_pos = std::abs(height_diff[0]);
	      best_len_pos = len[0];
	      success_pos = true;
	      xyz = camera_ctr + best_len_pos*camera_vec; // broken!!!
	    }else{
	    }

	    
	  } // End k loop
	  if (has_intersection) {
	    // break;
	  }
	} // End i loop
      
	// Failed to compute an intersection in the hard coded iteration limit!
	if ( !has_intersection ) {
	  return Vector3();
	}
      }

      // Refining the intersection using Levenberg-Marquardt
      // - This will actually use the L-M solver to play around with the len
      //   value to minimize the height difference from the DEM.
      int status = 0;
      Vector<double, 1> observation;
      observation[0] = 0;
      Vector<double, 1> dem_height_neg;
      dem_height_neg[0] = std::numeric_limits<double>::max();
      Vector<double, 1> final_len_neg;
      if (success_neg) {
	len[0] = best_len_neg;
	final_len_neg = math::levenberg_marquardt(model, len, observation, status,
                                      max_abs_tol, max_rel_tol,
						  num_max_iter);
	dem_height_neg = model(final_len_neg);
	
	if (status < 0) 
	  success_neg = false;
      }
      

      status = 0;
      observation[0] = 0;
      len[0] = best_len_pos;
      Vector<double, 1> final_len_pos;
      Vector<double, 1> dem_height_pos;
      dem_height_pos[0] = std::numeric_limits<double>::max();
      if (success_pos) {
	final_len_pos = math::levenberg_marquardt(model, len, observation, status,
				    max_abs_tol, max_rel_tol,
				    num_max_iter
				    );
	dem_height_pos = model(final_len_pos);
	if (status < 0) 
	  success_pos = false;
      }

      Vector<double, 1> dem_height;
      if (success_pos && std::abs(dem_height_pos[0]) <= std::abs(dem_height_neg[0])) {
	dem_height = dem_height_pos;
	len = final_len_pos;
      }else if (success_neg && std::abs(dem_height_neg[0]) <= std::abs(dem_height_pos[0])){
	dem_height = dem_height_neg;
	len = final_len_neg;
      }
      
      vw_out() << "Height error: " << dem_height << std::endl;
      
      if (!success_pos && !success_neg) 
	status = -1;
      
      if ( (status < 0) || (std::abs(dem_height[0]) > height_error_tol) ){
        has_intersection = false;
        return Vector3();
      }

      has_intersection = true;
      xyz = camera_ctr + len[0]*camera_vec;
      return xyz;
    }catch(...){
      has_intersection = false;
    }
    return Vector3();
  }

}
}

// Trace rays from pixel corners to DEM to see where they intersect the DEM
void extract_lon_lat_from_camera(Options & opt, ImageViewRef< PixelMask<float> > const& interp_dem,
				 GeoReference const& geo){

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
  
  std::string out_prefix;
  typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
  SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session, // may change
						       opt,
						       opt.image_file, opt.image_file,
						       opt.input_camera, opt.input_camera,
						       out_prefix));

  boost::shared_ptr<CameraModel> camera_model = session->camera_model(opt.image_file,
								      opt.input_camera);

  // Store here pixel values for the rays emanating from the pixels at
  // which we could intersect with the DEM.
  std::vector<double> good_pixel_values;
  
  int num_points = opt.pixel_values.size()/2;
  opt.lon_lat_values.reserve(2*num_points);
  opt.lon_lat_values.clear();
  
  for (int it = 0; it < num_points; it++){

    Vector2 pix(opt.pixel_values[2*it], opt.pixel_values[2*it+1]);

    Vector3 camera_ctr = camera_model->camera_center(pix);
    Vector3 camera_vec = camera_model->pixel_to_vector(pix);

    bool treat_nodata_as_zero = false;
    bool has_intersection = false;
    double height_error_tol = 1.0; // error in DEM height
    
    double max_abs_tol = 1e-20;
    double max_rel_tol      = 1e-20;
    int num_max_iter        = 1000;
    Vector3 xyz_guess       = Vector3();

    Vector3 xyz = camera_pixel_to_dem_xyz2(camera_ctr, camera_vec,  
					  interp_dem, geo, treat_nodata_as_zero,
					   has_intersection, height_error_tol,
					   max_abs_tol, max_rel_tol, num_max_iter, xyz_guess);

    if (xyz == Vector3() || !has_intersection){
      vw_out() << "Could not intersect the DEM with a ray coming "
	       << "from the camera at pixel: " << pix << ". Skipping it.\n";
      continue;
    }

    Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
    
    opt.lon_lat_values.push_back(llh[0]);
    opt.lon_lat_values.push_back(llh[1]);
    good_pixel_values.push_back(opt.pixel_values[2*it]);
    good_pixel_values.push_back(opt.pixel_values[2*it+1]);
  }

  if (good_pixel_values.size() < 6) {
    vw_throw( ArgumentErr() << "Successful intersection happened for less than "
	      << "3 pixels. Will not be able to create a camera. Consider checking "
	      << "your inputs, or passing different pixels in --pixel-values."
	      << opt.reference_dem << ".\n");
  }

  // Update with the values at which we were successful
  opt.pixel_values = good_pixel_values;
}

int main(int argc, char * argv[]){
  
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

    // If we have camera center in ECI or ECEF coordinates in km, convert
    // to height above datum.
    Vector3 parsed_cam_ctr;
    if (opt.parsed_cam_ctr_str != "") {
      std::vector<double> vals;
      parse_values<double>(opt.parsed_cam_ctr_str, vals);
      if (vals.size() != 3) 
	vw_throw( ArgumentErr() << "Could not parse 3 values from: "
		  << opt.parsed_cam_ctr_str << ".\n");

      parsed_cam_ctr = Vector3(vals[0], vals[1], vals[2]);
      parsed_cam_ctr *= 1000.0;  // convert to meters
      vw_out().precision(18);
      vw_out() << "Parsed camera center (meters): " << parsed_cam_ctr << "\n";

      Vector3 llh = datum.cartesian_to_geodetic(parsed_cam_ctr);
      
      // If parsed_cam_ctr is in ECI coordinates, the lon and lat won't be accurate
      // but the height will be.
      if (opt.cam_weight > 0) 
	opt.cam_height = llh[2];
    }
    
    vw::Quat parsed_cam_quat;
    if (opt.parsed_cam_quat_str != "") {
      std::vector<double> vals;
      parse_values<double>(opt.parsed_cam_quat_str, vals);
      if (vals.size() != 4) 
	vw_throw( ArgumentErr() << "Could not parse 4 values from: "
		  << opt.parsed_cam_quat_str << ".\n");

      parsed_cam_quat = vw::Quat(vals[0], vals[1], vals[2], vals[3]);
      vw_out() << "Parsed camera quaternion: " << parsed_cam_quat << "\n";
    }
    
    if (opt.cam_weight > 0) {
      vw_out() << "Will attempt to find a camera center height above datum of "
	       << opt.cam_height
	       << " meters with a weight strength of " << opt.cam_weight << ".\n";
    }
    
    if (opt.input_camera != ""){
      // Extract lon and lat from tracing rays from the camera to the ground.
      // This can modify opt.pixel_values.
      extract_lon_lat_from_camera(opt, create_mask(dem, nodata_value), geo);
    }

    if (opt.lon_lat_values.size() < 3) 
      vw_throw( ArgumentErr() << "Expecting at least three longitude-latitude pairs.\n");

    if (opt.lon_lat_values.size() != opt.pixel_values.size()){
      vw_throw( ArgumentErr()
		<< "The number of lon-lat pairs must equal the number of pixel pairs.\n");
    }

    size_t num_lon_lat_pairs = opt.lon_lat_values.size()/2;
    
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
          vw_out() << "Could not determine a valid height value at lon-lat: "
		   << llh[0] << ' ' << llh[1] << ". Will use a height of " << height << ".\n";
      }
      
      llh[2] = height;
      //vw_out() << "Lon-lat-height for corner ("
      //         << opt.pixel_values[2*corner_it] << ", " << opt.pixel_values[2*corner_it+1]
      //         << ") is "
      //         << llh[0] << ", " << llh[1] << ", " << llh[2] << std::endl;
    
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
    DiskImageView<float> img(opt.image_file);
    int wid = img.cols(), hgt = img.rows();
    if (wid <= 0 || hgt <= 0) 
      vw_throw( ArgumentErr() << "Could not read an image with positive dimensions from: "
		<< opt.image_file << ".\n");
    manufacture_cam(opt, wid, hgt, out_cam);

    // Transform it and optionally refine it
    bool verbose = true;
    fit_camera_to_xyz_ht(opt.parse_ecef, parsed_cam_ctr,
			 opt.camera_type, opt.refine_camera,  
			 xyz_vec, opt.pixel_values, 
			 opt.cam_height, opt.cam_weight, datum,
			 verbose, out_cam);
    
    if ((opt.parse_eci || opt.parse_ecef) && opt.camera_type == "opticalbar") {
      vw_throw( ArgumentErr() << "Cannot parse ECI/ECEF data for an optical bar camera.\n");
    }

    // Code that is not working. 
    //((vw::camera::PinholeModel*)out_cam.get())->set_camera_center(parsed_cam_ctr); 
    //((vw::camera::PinholeModel*)out_cam.get())->set_camera_pose(parsed_cam_quat); 
    //if (opt.parse_eci)
    // ((vw::camera::PinholeModel*)out_cam.get())->set_camera_pose(parsed_cam_quat);
    //if (opt.parse_ecef) 
    // ((vw::camera::PinholeModel*)out_cam.get())->set_camera_pose
    //	(inverse(parsed_cam_quat));
    
    llh = datum.cartesian_to_geodetic(out_cam->camera_center(Vector2()));
    vw_out() << "Output camera center lon, lat, and height above datum: " << llh << std::endl;
    vw_out() << "Writing: " << opt.camera_file << std::endl;
    if (opt.camera_type == "opticalbar")
      ((vw::camera::OpticalBarModel*)out_cam.get())->write(opt.camera_file);
    else {
      ((vw::camera::PinholeModel*)out_cam.get())->write(opt.camera_file);
    }


  } ASP_STANDARD_CATCHES;
    
  return 0;
}

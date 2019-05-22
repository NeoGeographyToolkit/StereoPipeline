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
  std::string cam_files, gcp_files;
  bool refine_camera; 
  Options(): focal_length(-1), pixel_pitch(-1), gcp_std(1), height_above_datum(0), refine_camera(false) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("gcp-files", po::value(&opt.gcp_files), "Read these GCP files.")
    ("cam-files", po::value(&opt.cam_files), "Read these camera files.")
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

// Parse a GCP file. It as assumed that each GCP only corresponds to one image. 
void parse_gcp(std::string const& gcp_file,
	       vw::cartography::Datum const& datum,
	       std::vector<Vector3> & xyz,
	       std::vector<Vector2> & pix){

  xyz.clear();
  pix.clear();
  
  // Open input file
  std::ifstream handle(gcp_file.c_str());
  if( !handle )
    vw_throw( vw::IOErr() << "Unable to open file \"" << gcp_file << "\"" );
  
  // Parse each line
  std::string line;
  while ( std::getline(handle, line, '\n') ){

    double id, lat, lon, height, a, b, c, x, y;
    std::string img;

    std::istringstream is(line);
    if (! (is >> id >> lat >> lon >> height >> a >> b >> c >> img >> x >> y) ) 
      vw_out() << "Ignoring invalid line: " << line << " from file " << gcp_file << "\n";

    Vector3 llh(lon, lat, height);
    Vector3 curr_xyz = datum.geodetic_to_cartesian(llh);
    xyz.push_back(curr_xyz);

    Vector2 curr_pix(x, y);
    pix.push_back(curr_pix);
  }
  
  handle.close();
}

// Unpack a vector into a rotation + translation + scale
void vector_to_transform(Vector<double> const & C, 
			 Matrix3x3            & rotation,
			 Vector3              & translation,
			 double               & scale){

  if (C.size() != 7) 
    vw_throw( LogicErr() << "Expecting a vector of size 7.\n" );
  
  translation = subvector(C, 0, 3);
  rotation    = axis_angle_to_quaternion(subvector(C, 3, 3)).rotation_matrix();
  scale       = C[6];

  return;
}

// Pack a rotation + translation + scale into a vector
void transform_to_vector(Vector<double>  & C, 
			 Matrix3x3 const & rotation,
			 Vector3   const & translation,
			 double    const & scale){


  Vector3 axis_angle = Quat(rotation).axis_angle();

  C.set_size(7);
  subvector(C, 0, 3) = translation;
  subvector(C, 3, 3) = axis_angle;
  C[6] = scale;
  
  return;
}

// Apply a given rotation + translation + scale to a pinhole model. We
// assume the model already has set its optical center, focal length,
// etc.
template <class CAM>
void apply_rot_trans_scale(CAM & P, Vector<double> const& C){

  if (C.size() != 7) 
    vw_throw( LogicErr() << "Expecting a vector of size 7.\n" );

  // Parse the transform
  Matrix3x3 rotation;
  Vector3   translation;
  double    scale;
  vector_to_transform(C, rotation, translation, scale);

  // Apply the transform
  P.apply_transform(rotation, translation, scale);
}

/// Find the rotation + translation + scale that best projects given
/// xyz points into given pixels for the input pinhole cameras (each
/// pinhole camera has a few xyz and pixels).
template <class CAM>
class CameraSolveRotTransScale: public vw::math::LeastSquaresModelBase<CameraSolveRotTransScale<CAM> > {

  std::vector< std::vector<Vector3> > const& m_xyz;
  vw::Vector<double> const& m_pixel_vec; // for debugging
  std::vector<CAM> m_cameras;
  
public:

  typedef vw::Vector<double>    result_type;   // pixel residuals
  typedef vw::Vector<double, 7> domain_type;   // axis angle + translation + scale
  typedef vw::Matrix<double> jacobian_type;

  /// Instantiate the solver with a set of xyz to pixel pairs and pinhole models
  CameraSolveRotTransScale(std::vector< std::vector<Vector3> > const& xyz,
		  vw::Vector<double> const& pixel_vec,
		  std::vector<CAM> const& cameras):
    m_xyz(xyz), m_pixel_vec(pixel_vec), m_cameras(cameras){
    
    // Sanity check
    if (m_xyz.size() != m_cameras.size()) 
      vw_throw( ArgumentErr() << "Error in CameraSolveRotTransScale: "
		<< "There must be as many xyz sets as cameras.\n");
    
  }
  
  /// Given the cameras, project xyz into them
  inline result_type operator()(domain_type const& C, bool verbose = false) const {

    verbose = true;
    
    // Create the camera models
    std::vector<CAM> cameras = m_cameras; // make a copy local to this function
    for (size_t it = 0; it < cameras.size(); it++) {
      apply_rot_trans_scale(cameras[it], C); // update its parameters
    }
    
    int result_size = 0;
    for (size_t it = 0; it < m_xyz.size(); it++) {
      result_size += m_xyz[it].size() * 2;
    }
    
    result_type result;
    result.set_size(result_size);
    int count = 0;
    for (size_t it = 0; it < m_xyz.size(); it++) {
      
      for (size_t c = 0; c < m_xyz[it].size(); c++) {
	Vector2 pixel = cameras[it].point_to_pixel(m_xyz[it][c]);
	
	result[2*count  ] = pixel[0];
	result[2*count+1] = pixel[1];
	count++;
      }

    }
    
    if (2*count != result_size) {
      vw_throw( LogicErr() << "Book-keeping failure in CameraSolveRotTransScale.\n");
    }

    double cost = 0;
    for (int it = 0; it < result_size; it++) {
      double diff = result[it] - m_pixel_vec[it];
      cost += diff*diff;
      if (verbose)
	std::cout << "diff is "
		  << result[it]
		  << ' ' << m_pixel_vec[it]
		  << ' '
		  << std::abs(result[it] - m_pixel_vec[it]) << std::endl;
    }

    return result;
  }
  
}; // End class CameraSolveRotTransScale

// Find the best fit camera that satisfies given GCP (represented as pixels and xyz values).
void fit_camera_to_xyz(Options const& opt,
		       boost::shared_ptr<CameraModel> const& in_cam,
		       boost::shared_ptr<CameraModel> & out_cam,
		       std::vector<Vector3> const& xyz_vec,
		       std::vector<double> const& pixel_values,
		       int wid, int hgt){
  
  // Prepare a pinhole camera or optical bar model with desired intrinsics but with
  // trivial position and orientation.
  Vector3 ctr(0, 0, 0);
  Matrix<double, 3, 3> rotation;
  rotation.set_identity();
    
  if (opt.camera_type == "opticalbar") {
    boost::shared_ptr<asp::camera::OpticalBarModel> opticalbar_cam;
    opticalbar_cam.reset(new asp::camera::OpticalBarModel(opt.sample_file));
    // Make sure the image size matches the input image file.
    opticalbar_cam->set_image_size(Vector2i(wid, hgt));
    opticalbar_cam->set_optical_center(Vector2(wid/2.0, hgt/2.0));
    out_cam = opticalbar_cam;
  } else {
    boost::shared_ptr<PinholeModel> pinhole_cam;
    if (in_cam.get() != NULL) {
      // Use the initial guess provided as input
      pinhole_cam.reset(new PinholeModel(*(PinholeModel*)in_cam.get()));
    } else if (opt.sample_file != "") {
      // Use the initial guess from file
      pinhole_cam.reset(new PinholeModel(opt.sample_file));
    } else {
      // Use the intrinsics from the command line
      pinhole_cam.reset(new PinholeModel(ctr, rotation, opt.focal_length, opt.focal_length,
					 opt.optical_center[0], opt.optical_center[1],
					 NULL, opt.pixel_pitch));
    }
    out_cam = pinhole_cam;
  }
    
  // This is needed only for weird reflections
  //Vector3 u_vec, v_vec, w_vec;
  //cam.coordinate_frame(u_vec, v_vec, w_vec);
  //cam.set_coordinate_frame(u_vec, v_vec, w_vec);
    
  // Create fake points in space at given distance from this camera's center
  // and corresponding actual points on the ground.
  const double ht = 500000; // 500 km, just some height not too far from actual satellite height 
  vw::Matrix<double> in, out;
  int num_pts = pixel_values.size()/2;
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
  double scale;
  asp::find_3D_affine_transform(in, out, rotation, ctr, scale);
  if (opt.camera_type == "opticalbar")
    ((asp::camera::OpticalBarModel*)out_cam.get())->apply_transform(rotation, ctr, scale);
  else
    ((PinholeModel*)out_cam.get())->apply_transform(rotation, ctr, scale);

  // Print out some errors
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
    
  // Solve a little optimization problem to make the points on the ground project
  // as much as possible exactly into the image corners.
  if (opt.refine_camera) {
    Vector<double> pixel_vec; // must copy to this structure
    pixel_vec.set_size(pixel_values.size());
    for (size_t corner_it = 0; corner_it < pixel_vec.size(); corner_it++) 
      pixel_vec[corner_it] = pixel_values[corner_it];
    const double abs_tolerance  = 1e-24;
    const double rel_tolerance  = 1e-24;
    const int    max_iterations = 2000;
    int status = 0;
    Vector<double> final_params;
    Vector<double> seed;
      
    if (opt.camera_type == "opticalbar") {
      CameraSolveLMA<asp::camera::OpticalBarModel>
	lma_model(xyz_vec, *((asp::camera::OpticalBarModel*)out_cam.get()));
      camera_to_vector(*((asp::camera::OpticalBarModel*)out_cam.get()), seed);
      final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
					       status, abs_tolerance, rel_tolerance,
					       max_iterations);
      vector_to_camera(*((asp::camera::OpticalBarModel*)out_cam.get()), final_params);
    } else {
      CameraSolveLMA<PinholeModel> lma_model(xyz_vec, *((PinholeModel*)out_cam.get()));
      camera_to_vector(*((PinholeModel*)out_cam.get()), seed);
      final_params = math::levenberg_marquardt(lma_model, seed, pixel_vec,
					       status, abs_tolerance, rel_tolerance,
					       max_iterations);
      vector_to_camera(*((PinholeModel*)out_cam.get()), final_params);
    }
    if (status < 1)
      vw_out() << "The Levenberg-Marquardt solver failed. Results may be inaccurate.\n";
      

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
  } // End camera refinement case
}

void process_cameras(vw::cartography::Datum const& datum,
		     Options const& opt, int wid, int hgt){

  // Cameras that we want to align to ground as a group using GCP.
  // They are obtained from SfM and are self-consistent, but are in an
  // arbitrary coordinate system.
  std::vector<PinholeModel> sfm_cams;

  // Individually aligned cameras using GCP. They may not be
  // self-consistent, and are only used to give an idea of the
  // transform to apply to the unaligned cameras.
  std::vector<PinholeModel> aux_cams;

  std::vector< std::vector<Vector3> > xyz;
  std::vector< std::vector<Vector2> > pix;

  std::vector<std::string> gcp_files, cam_files;

  parse_values(opt.gcp_files, gcp_files);
  parse_values(opt.cam_files, cam_files);
  if (gcp_files.size() != cam_files.size()) 
    vw_throw( ArgumentErr() << "Expecting as many GCP files as camera files.\n");

  int num_cams = gcp_files.size();

  // Create the cameras individually aligned to ground. 
  for (int it = 0; it < num_cams; it++) {
    std::string gcp_file = gcp_files[it];
    std::string cam_file = cam_files[it];

    std::vector<Vector3> curr_xyz;
    std::vector<Vector2> curr_pix;
    parse_gcp(gcp_files[it], datum, curr_xyz, curr_pix);

    xyz.push_back(curr_xyz);
    pix.push_back(curr_pix);

    vw::camera::PinholeModel cam(cam_files[it]);
    sfm_cams.push_back(cam);

    // Export to the format used by the API
    std::vector<double> pixel_values;
    for (size_t c = 0; c < pix[it].size(); c++) {
      pixel_values.push_back(pix[it][c][0]);
      pixel_values.push_back(pix[it][c][1]);
    }

    boost::shared_ptr<CameraModel> in_cam(new PinholeModel(cam)), out_cam;
    fit_camera_to_xyz(opt, in_cam, out_cam, xyz[it], opt.pixel_values, wid, hgt);
    aux_cams.push_back(*((PinholeModel*)out_cam.get()));
  }

  
  // Estimate the scale ratio between cameras obtained from SfM and cameras
  // aligned to GCP.
  double world_scale = 1.0;
  double len1 = 1.0, len2 = 1.0;
  if (num_cams > 1) {
    len1 = norm_2(sfm_cams[0].camera_center() - sfm_cams[1].camera_center());
    len2 = norm_2(aux_cams[0].camera_center() - aux_cams[1].camera_center());
    world_scale = len2/len1;
  }

  vw_out() << "Estimated scale to apply when converting to world coordinates using GCP: "
	   << world_scale << ".\n";

  char * sptr = getenv("SCALE");
  if (sptr != NULL) {
    double s = atof(sptr);
    world_scale *= s;
  }

  std::cout << "--Scale with tweak factor: " << world_scale << std::endl;
  
  // So far we aligned both cameras individually to GCP and we got an idea of scale.
  // Yet we would like to align them without changing the relationship between them,
  // so using a single transform for all not an individual transform for each.
  // This way we will transform the SfM-computed cameras to the new coordinate system.

  // Start by estimating a such a transform.
  int num_pts = 0;
  for (int it = 0; it < num_cams; it++) {
    num_pts += pix[it].size();
  }
  
  vw::Matrix<double> in_pts, out_pts;
  in_pts.set_size(3, num_pts);
  out_pts.set_size(3, num_pts);
  
  int col = 0;
  for (int it = 0; it < num_cams; it++) {
    
    // For each camera, find xyz values in the input cameras
    // that map to GCP. Use the scale for that.
    for (int c = 0; c < xyz[it].size(); c++) {
      
      // Distance from camera center to xyz for the individually aligned cameras
      double len = norm_2(aux_cams[it].camera_center() - xyz[it][c]);
      len = len / world_scale;
      Vector3 trans_xyz = sfm_cams[it].camera_center()
	+ len * sfm_cams[it].pixel_to_vector(pix[it][c]);
      for (int row = 0; row < in_pts.rows(); row++) {
	in_pts(row, col)  = trans_xyz[row];
	out_pts(row, col) = xyz[it][c][row];
      }
      
      col++;
    }
  }

  if (col != num_pts) 
    vw_throw( LogicErr() << "Book-keeping failure in cam_gen.\n");

  // The initial transform to world coordinates
  Vector<double> C;
  {
    double local_scale;
    Matrix3x3 rotation; 
    Vector3 translation;
    asp::find_3D_affine_transform(in_pts, out_pts, rotation, translation, local_scale);
    //std::cout << "---solved scale: " << scale << std::endl;
    transform_to_vector(C, rotation, translation, local_scale); // copy into C
  }
  
  std::cout << "---input C: " << C << std::endl;
  
  // Form the pixel vector
  int pixel_vec_len = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    pixel_vec_len += pix[it].size() * 2;
  }

  Vector<double> pixel_vec;
  pixel_vec.set_size(pixel_vec_len);
  int count = 0;
  for (size_t it = 0; it < pix.size(); it++) {
    for (size_t c = 0; c < pix[it].size(); c++) {
      Vector2 pixel = pix[it][c];
      pixel_vec[2*count  ] = pixel[0];
      pixel_vec[2*count+1] = pixel[1];
      count++;
    }
  }
  if (2*count != pixel_vec_len)
    vw_throw( LogicErr() << "Book-keeping failure in cam_gen.\n");

  // Optimize the transform
  double abs_tolerance  = 1e-24;
  double rel_tolerance  = 1e-24;
  int    max_iterations = 2000;
  int status = 0;
  CameraSolveRotTransScale<PinholeModel> lma_model(xyz, pixel_vec, sfm_cams);
  Vector<double> final_params
    = vw::math::levenberg_marquardt(lma_model, C, pixel_vec,
				    status, abs_tolerance, rel_tolerance,
				    max_iterations);

  std::cout << "--final C: " << final_params << std::endl;
  //std::cout << "---final scale " << final_params[6] << std::endl;
  
  bool verbose = true; 
  Vector<double>  final_residual = lma_model(final_params, verbose);

  // Bring the cameras to world coordinates
  for (int it = 0; it < num_cams; it++) {
    std::string cam_file = cam_files[it];
    apply_rot_trans_scale(sfm_cams[it], final_params);
    cam_file += ".out.tsai";
    vw_out() << "Writing: " << cam_file << std::endl;
    sfm_cams[it].write(cam_file);
  }

  len2 = 1;
  if (num_cams > 1) {
    len2 = norm_2(sfm_cams[0].camera_center() - sfm_cams[1].camera_center());
  }
  world_scale = len2/len1;
  
  vw_out() << "Refined scale to apply when converting to world coordinates using GCP: "
	   << world_scale << ".\n";
  
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

    if (write_gcp) {
      vw_out() << "Writing: " << opt.gcp_file << std::endl;
      std::ofstream fs(opt.gcp_file.c_str());
      fs << gcp.str();
      fs.close();
    }
    
    // Temporary workaround
    if (opt.gcp_files != "") {
      process_cameras(datum, opt, wid, hgt);
      return 0;
    }

    boost::shared_ptr<CameraModel> in_cam, out_cam;
    fit_camera_to_xyz(opt, in_cam, out_cam, xyz_vec, opt.pixel_values, wid, hgt);
    vw_out() << "Writing: " << opt.camera_file << std::endl;
    if (opt.camera_type == "opticalbar")
      ((asp::camera::OpticalBarModel*)out_cam.get())->write(opt.camera_file);
    else
      ((PinholeModel*)out_cam.get())->write(opt.camera_file);


  } ASP_STANDARD_CATCHES;
    
  return 0;
}

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

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/EigenUtils.h>
#include <asp/Camera/CameraResectioning.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Core/ReportUtils.h>
#include <asp/Core/CameraTransforms.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/CameraUtilities.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/Geometry.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Cartography/DatumUtils.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string/predicate.hpp>

// For parsing .json files
#include <nlohmann/json.hpp>

#include <limits>
#include <cstring>

namespace fs = boost::filesystem;
namespace po = boost::program_options;
using json = nlohmann::json;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

/// Find the best camera that fits the current GCP
void fit_camera_to_xyz_ht(Vector3 const& parsed_camera_center, // may not be known
                          Vector3 const& input_camera_center, // may not be known
                          std::string const& camera_type,
                          bool refine_camera, 
                          std::vector<Vector3> const& xyz_vec,
                          std::vector<double> const& pixel_values,
                          double cam_height, double cam_weight, double cam_ctr_weight,
                          vw::cartography::Datum const& datum,
                          bool verbose,
                          boost::shared_ptr<CameraModel> & out_cam){

  // Create fake points in space at given distance from this camera's
  // center and corresponding actual points on the ground.  Use 500
  // km, just some height not too far from actual satellite height.
  double ht = 500000.0; 
  int num_pts = pixel_values.size()/2;
  vw::Matrix<double> in, out;
  std::vector<vw::Vector2>  pixel_vec;
  in.set_size(3, num_pts);
  out.set_size(3, num_pts);
  for (int col = 0; col < in.cols(); col++) {
    if (input_camera_center != Vector3(0, 0, 0)) {
      // We know the camera center. Use that.
      // TODO(oalexan1): Problem with RPC!
      ht = norm_2(xyz_vec[col] - input_camera_center);
    }
    Vector2 pix = Vector2(pixel_values[2*col], pixel_values[2*col+1]);
    Vector3 a = out_cam->camera_center(Vector2(0, 0)) + ht * out_cam->pixel_to_vector(pix);
    pixel_vec.push_back(pix);
    for (int row = 0; row < in.rows(); row++) {
      in(row, col)  = a[row];
      out(row, col) = xyz_vec[col][row];
    }
  }
  
  // Apply a transform to the camera so that the fake points are on top of the real points
  Matrix<double, 3, 3> rotation;
  Vector3 translation;
  double scale;
  find_3D_transform(in, out, rotation, translation, scale);
  if (camera_type == "opticalbar") {
    ((vw::camera::OpticalBarModel*)out_cam.get())->apply_transform(rotation,
								   translation, scale);
  } else if (camera_type == "pinhole") {

    // The logic based on fake points can give junk results. Use instead the
    // state-of-the-art OpenCV solver. TODO(oalexan1): Need to consider using
    // this solution also for OpticalBar.
    try {
      asp::findCameraPose(xyz_vec, pixel_vec, *(PinholeModel*)out_cam.get());
    } catch(std::exception const& e) {
      vw_out() << "Failed to find the camera pose using OpenCV. Falling back "
                << "to ASP's internal approach. This is not as robust. "
                << "Check your inputs and validate the produced camera.\n";
      // Fall back to previous logic
      ((PinholeModel*)out_cam.get())->apply_transform(rotation, translation, scale);
    }
    
    if (parsed_camera_center != Vector3(0, 0, 0)) {
      // Overwrite the solved camera center with what was passed in
      ((PinholeModel*)out_cam.get())->set_camera_center(parsed_camera_center);
    }
  }
  
  // Print out some errors
  if (verbose) {
    vw_out() << "Errors of pixel projection in the coarse camera:\n";
    for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
      vw::Vector2 pix1 = out_cam.get()->point_to_pixel(xyz_vec[corner_it]);
      vw::Vector2 pix2 = Vector2( pixel_values[2*corner_it], pixel_values[2*corner_it+1]);
      vw_out () << "Pixel and error: ("
                << pixel_values[2*corner_it] << ' ' << pixel_values[2*corner_it+1]
                << ") " << norm_2(pix1 - pix2) << std::endl;
    }
  }

  Vector3 xyz0 = out_cam.get()->camera_center(vw::Vector2());

  // Solve a little optimization problem to make the points on the ground project
  // as much as possible exactly into the image corners.
  if (refine_camera) {
    if (camera_type == "opticalbar")
      fitOpticalBar(xyz_vec, cam_height, cam_weight, cam_ctr_weight, datum, pixel_values, 
                             *((vw::camera::OpticalBarModel*)out_cam.get()));
    else if (camera_type == "pinhole")
      fitPinhole(xyz_vec, cam_height, cam_weight, cam_ctr_weight, datum, pixel_values, 
                          *((PinholeModel*)out_cam.get()));
    else 
      vw_throw(ArgumentErr() << "Unknown camera type: " << camera_type << ".\n");

    if (verbose) {
      vw_out() << "Errors of pixel projection in the camera with refined pose:\n";
      for (size_t corner_it = 0; corner_it < num_pts; corner_it++) {
        vw::Vector2 pix1 = out_cam.get()->point_to_pixel(xyz_vec[corner_it]);
        vw::Vector2 pix2 = Vector2(pixel_values[2*corner_it], pixel_values[2*corner_it+1]);
	      vw_out () << "Pixel and error: ("
		              << pixel_values[2*corner_it] << ' ' << pixel_values[2*corner_it+1]
		              << ") " <<  norm_2(pix1 - pix2) << std::endl;
      }
    }
    
  } // End camera refinement case
}

// Parse numbers or strings from a list where they are separated by commas or spaces.
template<class T>
void parse_values(std::string list, std::vector<T> & values) {

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

struct Options : public vw::GdalWriteOptions {
  std::string image_file, out_camera, lon_lat_values_str, pixel_values_str, datum_str,
    reference_dem, frame_index, gcp_file, camera_type, sample_file, input_camera,
    stereo_session, bundle_adjust_prefix, parsed_camera_center_str, parsed_cam_quat_str,
    distortion_str, distortion_type, refine_intrinsics, extrinsics_file;
  double focal_length, pixel_pitch, gcp_std, height_above_datum,
    cam_height, cam_weight, cam_ctr_weight;
  Vector2 optical_center;
  vw::Vector3 camera_center;
  std::vector<double> lon_lat_values, pixel_values, distortion;
  bool refine_camera, parse_eci, parse_ecef, planet_pinhole; 
  int num_pixel_samples;
  bool exact_tsai_to_csm_conv;
  vw::cartography::Datum datum;
  Options(): focal_length(-1), pixel_pitch(-1), gcp_std(1), height_above_datum(0), refine_camera(false), cam_height(0), cam_weight(0), cam_ctr_weight(0), 
  planet_pinhole(false), parse_eci(false), parse_ecef(false), num_pixel_samples(0), 
  exact_tsai_to_csm_conv(false)
  {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("");
  general_options.add_options()
    ("output-camera-file,o", po::value(&opt.out_camera), "Specify the output camera file with a .tsai or .json extension.")
    ("camera-type", po::value(&opt.camera_type)->default_value("pinhole"), "Specify the camera type. Options are: pinhole (default), opticalbar, and linescan.")
    ("lon-lat-values", po::value(&opt.lon_lat_values_str)->default_value(""),
    "A (quoted) string listing numbers, separated by commas or spaces, "
    "having the longitude and latitude (alternating and in this "
    "order) of each image corner or some other list of pixels given "
    "by ``--pixel-values``. If the corners are used, they are traversed "
    "in the order (0, 0) (w, 0) (w, h), (0, h) where w and h are the "
     "image width and height.")
    ("pixel-values", po::value(&opt.pixel_values_str)->default_value(""), "A (quoted) string listing numbers, separated by commas or spaces, having the column and row (alternating and in this order) of each pixel in the raw image at which the longitude and latitude is known and given by --lon-lat-values. By default this is empty, and will be populated by the image corners traversed as mentioned at the earlier option.")
    ("reference-dem", po::value(&opt.reference_dem)->default_value(""),
     "Use this DEM to infer the heights above datum of the image corners.")
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the longitude and latitude, unless a DEM is given. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("height-above-datum", po::value(&opt.height_above_datum)->default_value(0),
     "Assume this height above datum in meters for the image corners unless read from the DEM.")
    ("sample-file", po::value(&opt.sample_file)->default_value(""), 
     "Read the camera intrinsics from this file. Required for opticalbar bar cameras.")
    ("focal-length", po::value(&opt.focal_length)->default_value(0),
     "The camera focal length.")
    ("optical-center", po::value(&opt.optical_center)->default_value(Vector2(nan, nan),"NaN NaN"),
     "The camera optical center. If not specified for pinhole cameras, it will be set to image center (half of image dimensions) times the pixel pitch. The optical bar camera always uses the image center.")
    ("pixel-pitch", po::value(&opt.pixel_pitch)->default_value(0),
     "The pixel pitch. If set to 1, the focal length and optical center are in units "
     "of pixel.")
    ("distortion", po::value(&opt.distortion_str)->default_value(""),
     "Distortion model parameters. It is best to leave this blank and have the program "
     "determine them. By default, the OpenCV radial-tangential lens distortion "
     "model is used. Then, can specify 5 numbers, in quotes, in the order "
     "k1, k2, p1, p2, k3. Also supported is the transverse model, which needs "
     "20 values. These are the coefficients of a pair of polynomials of degree 3 "
     "in x and y. Only applicable when creating CSM cameras. The default is zero "
     "distortion. See also --distortion-type.")
    ("distortion-type", po::value(&opt.distortion_type)->default_value("radtan"),
     "Set the distortion type. Options: radtan (default) and transverse. Only applicable "
     "when creating CSM Frame cameras.")
    ("camera-center", 
     po::value(&opt.camera_center)->default_value(Vector3(nan, nan, nan),"NaN NaN NaN"),
     "The camera center in ECEF coordinates. If not set, the program will solve for it.")
    ("refine-camera", po::bool_switch(&opt.refine_camera)->default_value(false)->implicit_value(true),
     "After a rough initial camera is obtained, refine its pose using least squares.")
    ("refine-intrinsics", po::value(&opt.refine_intrinsics)->default_value(""),
     "Refine the camera intrinsics together with the camera pose. Specify, in quotes, or as comma as separator, one or more of: focal_length, optical_center, other_intrinsics (distortion). Also can set as 'all' or 'none'. In the latter mode only the camera pose is optimized. Applicable only with option --input-camera and when creating a CSM Frame camera model.")
    ("num-pixel-samples", po::value(&opt.num_pixel_samples)->default_value(10000),
     "Number of uniformly distributed pixel samples to use with option --refine-intrinsics.")
    ("frame-index", po::value(&opt.frame_index)->default_value(""),
     "A file used to look up the longitude and latitude of image corners based on the image name, in the format provided by the SkySat video product.")
    ("gcp-file", po::value(&opt.gcp_file)->default_value(""),
     "If provided, save the image corner coordinates and heights in the GCP format to this file.")
    ("gcp-std", po::value(&opt.gcp_std)->default_value(1),
     "The standard deviation for each GCP pixel, if saving a GCP file. A smaller value suggests a more reliable measurement, hence will be given more weight.")
    ("cam-height", po::value(&opt.cam_height)->default_value(0),
     "If both this and --cam-weight are positive, enforce that the output camera is at this height above datum. For SkySat, if not set, read this from the frame index. Highly experimental.")
    ("cam-weight", po::value(&opt.cam_weight)->default_value(0),
     "If positive, try to enforce the option --cam-height with this weight (bigger weight means try harder to enforce).")
    ("cam-ctr-weight", po::value(&opt.cam_ctr_weight)->default_value(0),
     "If positive, try to enforce that during camera refinement the camera center stays close to the initial value (bigger weight means try harder to enforce this; a value like 1000.0 is good enough).")
    ("parse-eci", po::bool_switch(&opt.parse_eci)->default_value(false)->implicit_value(true),
     "Create cameras based on ECI positions and orientations (not working).")
    ("parse-ecef", po::bool_switch(&opt.parse_ecef)->default_value(false)->implicit_value(true),
     "Create cameras based on ECEF position (but not orientation).")
    ("input-camera", po::value(&opt.input_camera)->default_value(""),
     "Create the output pinhole camera approximating this camera. If with a "
     "_pinhole.json suffix, read it verbatim, with no refinements or "
     "taking into account other input options.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the input camera model type. Normally this is auto-detected, but may need to be specified if the input camera model is in XML format. See the doc for options.")
    ("extrinsics", po::value(&opt.extrinsics_file)->default_value(""),
     "Read a file having on each line an image name and extrinsic parameters as "
     "longitude, latitude, height above datum, roll, pitch, and yaw. Write one .tsai "
     "camera file per image.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust "
     "when providing an input camera.")
  ;
  
  general_options.add(vw::GdalWriteOptionsDescription(opt));

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

  // If extrinsics file is set, must have datum and sample file
  if (opt.extrinsics_file != "") {
    if (opt.datum_str == "")
      vw_throw(ArgumentErr() << "Must provide a datum when reading extrinsics.\n");
    if (opt.sample_file == "")
      vw_throw(ArgumentErr() << "Must provide a sample file when reading extrinsics.\n");
    // The rest of the options do not apply      
    return;  
  }

  if (opt.image_file.empty())
    vw_throw(ArgumentErr() << "Missing the input image.\n");
              
  if (opt.out_camera.empty())
    vw_throw(ArgumentErr() << "Missing the output camera file name.\n");

  boost::to_lower(opt.camera_type);

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
  
  if (opt.camera_type != "pinhole" && opt.camera_type != "opticalbar" && 
      opt.camera_type != "linescan")
    vw_throw(ArgumentErr() << "Only pinhole, opticalbar, and linescan "
               << "cameras are supported.\n");
  
  if ((opt.camera_type == "opticalbar") && (opt.sample_file == ""))
    vw_throw(ArgumentErr() << "opticalbar type must use a sample camera file.\n");

  std::string out_ext = get_extension(opt.out_camera);
  if (out_ext != ".tsai" && out_ext != ".json") 
    vw_throw(ArgumentErr() << "The output camera file must end with .tsai or .json.\n");

  if (opt.camera_type == "linescan" && out_ext != ".json")
    vw_throw(ArgumentErr() << "An output linescan camera must end with .json.\n");
  
  if (opt.camera_type == "opticalbar" && out_ext != ".tsai")
    vw_throw(ArgumentErr() << "An output optical bar camera must be in .tsai format.\n");

  // Create the output directory
  vw::create_out_dir(opt.out_camera);

  // The rest of the logic does not apply to linescan cameras
  if (opt.camera_type == "linescan")
    return;

  // Planet's custom format  
  opt.planet_pinhole = boost::algorithm::ends_with(opt.input_camera, "_pinhole.json");
  if (opt.planet_pinhole && !opt.datum_str.empty()) {
    // Can only use WGS84 for Planet cameras
    vw::cartography::Datum datum1(opt.datum_str);
    vw::cartography::Datum datum2("WG84");
    bool warn_only = false;
    vw::checkDatumConsistency(datum1, datum2, warn_only);
  }
  if (opt.planet_pinhole && opt.datum_str.empty()) {
    opt.datum_str = "WGS84";
    vw::vw_out() << "Setting the datum to: " << opt.datum_str << std::endl;
  }
  
  if (opt.planet_pinhole && (opt.refine_camera || opt.refine_intrinsics != ""))
    vw_throw(ArgumentErr() << "Cannot refine the pose or intrinsics for a pinhole camera "
             "read from a Planet JSON file. Consider running this tool to first convert "
             "it to ASP's format, and later refine that using another invocation. "
             "Note that camera refinement will not preserve the camera center.\n");
  
  if ((opt.cam_weight > 0 || opt.cam_ctr_weight > 0) && opt.refine_intrinsics != "")
   vw_throw(ArgumentErr() << "The option for refining the intrinsics cannot, "
            "for the time being, constrain either the camera height or camera center.\n");
   
  // Set the datum from the string
  if (!opt.datum_str.empty())
    opt.datum.set_well_known_datum(opt.datum_str);

  if (!opt.planet_pinhole && !opt.input_camera.empty()) {
    // Guess the datum from the camera model    
    asp::SessionPtr session;
    vw::cartography::Datum cam_datum;
    bool found_cam_datum = asp::datum_from_camera(opt.image_file, opt.input_camera, 
                                                  // Outputs
                                                  opt.stereo_session, session, cam_datum);
    // For pinhole session the guessed datum may be unreliable, so warn only
    bool warn_only = (opt.stereo_session.find("pinhole") != std::string::npos);
    if (found_cam_datum && !opt.datum_str.empty())
      vw::checkDatumConsistency(cam_datum, opt.datum, warn_only);
    // opt.datum is more general than opt.datum_str, but set both
    if (found_cam_datum && opt.datum_str.empty()) {
      opt.datum = cam_datum;
      opt.datum_str = cam_datum.name();
    }
  }

  // Must do the same for opt.sample_file. This and opt.input_camera are used for
  // different things. opt.sample_file may not even be an orbital camera.
  if (!opt.sample_file.empty()) {
    // Guess the datum from the camera model    
    asp::SessionPtr session;
    vw::cartography::Datum cam_datum;
    bool found_cam_datum = false;
    try {
      found_cam_datum = asp::datum_from_camera(opt.image_file, opt.sample_file,
                                           // Outputs
                                           opt.stereo_session, session, cam_datum);
    } catch (std::exception const& e) {
      // Use here an empty session, as the sample file need not be the same type
      // as the input camera.
      std::string local_session = ""; 
      found_cam_datum = asp::datum_from_camera(opt.image_file, opt.sample_file,
                                               // Outputs
                                               local_session, session, cam_datum);
    }
    
    // For pinhole session the guessed datum may be unreliable, so warn only
    bool warn_only = (opt.stereo_session.find("pinhole") != std::string::npos);
    if (found_cam_datum && !opt.datum_str.empty())
      vw::checkDatumConsistency(cam_datum, opt.datum, warn_only);
    // opt.datum is more general than opt.datum_str, but set both
    if (found_cam_datum && opt.datum_str.empty()) {
      opt.datum = cam_datum;
      opt.datum_str = cam_datum.name();
    }
  }
   
  // If we cannot read the data from a DEM, must know the datum
  if (!opt.planet_pinhole && opt.reference_dem.empty() && opt.datum_str.empty())
    vw_throw(ArgumentErr() << "Must provide either a reference DEM or a datum.\n");

  if (opt.gcp_std <= 0) 
    vw_throw(ArgumentErr() << "The GCP standard deviation must be positive.\n");

  if (!opt.planet_pinhole && opt.frame_index != "" && opt.lon_lat_values_str != "") 
    vw_throw(ArgumentErr() << "Cannot specify both the frame index file "
	      << "and the lon-lat corners.\n");

  if (opt.cam_weight > 0 && opt.cam_ctr_weight > 0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "Cannot enforce the camera center constraint and camera "
                 << "height constraint at the same time.\n");
  
  // TODO(oalexan1): Break up this big loop
  if (!opt.planet_pinhole && opt.frame_index != "") {
    // Parse the frame index to extract opt.lon_lat_values_str.
    // Look for a line having this image, and search for "POLYGON" followed by spaces and "((".
    boost::filesystem::path p(opt.image_file); 
    std::string image_base = p.stem().string(); // strip the directory name and suffix
    std::ifstream file( opt.frame_index.c_str() );
    std::string line;
    std::string beg1 = "POLYGON";
    std::string beg2 = "((";
    std::string end = "))";
    while (getline(file, line, '\n')) {
      if (line.find(image_base) != std::string::npos) {
        // Find POLYGON first.
        int beg_pos = line.find(beg1);
        if (beg_pos == std::string::npos)
          vw_throw(ArgumentErr() << "Cannot find " << beg1 << " in line: " << line << ".\n");
        beg_pos += beg1.size();

        // Move forward skipping any spaces until finding "(("
        beg_pos = line.find(beg2, beg_pos);
        if (beg_pos == std::string::npos)
          vw_throw(ArgumentErr() << "Cannot find " << beg2 << " in line: " << line << ".\n");
        beg_pos += beg2.size();

        // Find "))"
        int end_pos = line.find(end, beg_pos);
        if (end_pos == std::string::npos)
          vw_throw(ArgumentErr() << "Cannot find " << end << " in line: " << line << ".\n");
        opt.lon_lat_values_str = line.substr(beg_pos, end_pos - beg_pos);
        vw_out() << "Parsed the lon-lat corner values: " 
                 << opt.lon_lat_values_str << std::endl;

        if (opt.parse_eci && opt.parse_ecef)
          vw_throw(ArgumentErr() << "Cannot parse both ECI end ECEF at the same time.\n");

        // Also parse the camera height constraint, unless manually specified
        if (opt.cam_weight > 0 || opt.parse_eci || opt.parse_ecef) {
          std::vector<std::string> vals;
          parse_values<std::string>(line, vals);
          
          if (vals.size() < 12) 
            vw_throw(ArgumentErr() << "Could not parse 12 values from: " << line << ".\n");

          // Extract the ECI or ECEF coordinates of camera
          // center. Keep them as string until we can convert to
          // height above datum.
          
          if (opt.parse_eci) {
            std::string x = vals[5];
            std::string y = vals[6];
            std::string z = vals[7];
            opt.parsed_camera_center_str = x + " " + y + " " + z;
            vw_out() << "Parsed the ECI camera center in km: "
                << opt.parsed_camera_center_str <<".\n";
            
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
              vw_throw(ArgumentErr() << "Could not parse 19 values from: " << line << ".\n");
            
            std::string x = vals[12];
            std::string y = vals[13];
            std::string z = vals[14];
            opt.parsed_camera_center_str = x + " " + y + " " + z;
            vw_out() << "Parsed the ECEF camera center in km: "
                << opt.parsed_camera_center_str <<".\n";
            
            std::string q0 = vals[15];
            std::string q1 = vals[16];
            std::string q2 = vals[17];
            std::string q3 = vals[18];
            opt.parsed_cam_quat_str = q0 + " " + q1 + " " + q2 + " " + q3;
            vw_out() << "Parsed the ECEF quaternion: "
                << opt.parsed_cam_quat_str <<".\n";
          }
        } // End parsing camera height constraint
        
        break;
      } // end of if we found the image name in the line
    } // end of loop through lines in frame index file
    
    if (opt.lon_lat_values_str == "")
      vw_throw(ArgumentErr() << "Could not parse the entry for " << image_base
                << " in file: " << opt.frame_index << ".\n");
  }
    
  // Parse the pixel values
  parse_values<double>(opt.pixel_values_str, opt.pixel_values);

  // If none were provided, use the image corners
  if (!opt.planet_pinhole && opt.pixel_values.empty()) {
    DiskImageView<float> img(opt.image_file);
    int wid = img.cols(), hgt = img.rows();
    if (wid <= 0 || hgt <= 0) 
      vw_throw(ArgumentErr() << "Could not read an image with positive dimensions from: "
                             << opt.image_file << ".\n");
    
    // populate the corners
    double arr[] = {0.0, 0.0, wid-1.0, 0.0, wid-1.0, hgt-1.0, 0.0, hgt-1.0};
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
  if (!opt.planet_pinhole && opt.input_camera == "") {
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

  std::string input_ext = get_extension(opt.input_camera);
  
  // Note that optical center can be negative (for some SkySat products).
  if (!opt.planet_pinhole &&
      opt.sample_file == "" &&
      (opt.focal_length <= 0 || opt.pixel_pitch <= 0)) {
  
      // A pinhole camera can be its own sample file
      if (input_ext == ".tsai") 
        opt.sample_file = opt.input_camera;
      else 
        vw_throw(ArgumentErr() << "Must provide positive focal length "
                << "and pixel pitch values, or a sample file to read these from.\n");
  }
  
  if (opt.sample_file != "" && 
      (!vm["focal-length"].defaulted() || !vm["optical-center"].defaulted() ||
        !vm["distortion"].defaulted()))
    vw::vw_throw(vw::ArgumentErr() << "Cannot specify both a sample file and "
              << "focal length, optical center, or distortion.\n");

  if ((opt.parse_eci || opt.parse_ecef) && opt.camera_type == "opticalbar") 
    vw_throw(ArgumentErr() << "Cannot parse ECI/ECEF data for an optical bar camera.\n");
  
  // Parse the lens distortion
  parse_values<double>(opt.distortion_str, opt.distortion);
  
  // Sanity checks for lens distortion  
  if (opt.camera_type != "pinhole" || out_ext != ".json") {
    if (opt.distortion.size() != 0)
      vw_throw(ArgumentErr() 
               << "Distortion coefficients are only supported for CSM Frame cameras.\n"); 
    if (opt.refine_intrinsics != "")
      vw_throw(ArgumentErr() 
               << "Refining intrinsics is only supported for CSM Frame cameras.\n");
  }
  
  // Populate the distortion parameters for CSM, if not set
  if (opt.distortion.size() == 0 && opt.camera_type == "pinhole" && out_ext == ".json") {
    if (opt.distortion_type == "radtan") {
      
      // Consider the special case when we can precisely convert tsai to json,
      // if the distortion is radtan (tsai) or no distortion.
      std::vector<double> local_distortion;
      if (input_ext == ".tsai") {
        vw::camera::PinholeModel pin(opt.input_camera);
        vw::camera::NullLensDistortion const* zero_dist =
          dynamic_cast<vw::camera::NullLensDistortion const*>(pin.lens_distortion());
        vw::camera::TsaiLensDistortion const* radtan_dist = 
          dynamic_cast<vw::camera::TsaiLensDistortion const*>(pin.lens_distortion());
        if (zero_dist != NULL) {
          // Zero distortion can be thought of as radtan distortion with all zeros
          local_distortion.resize(5, 0.0);
          opt.exact_tsai_to_csm_conv = true;
        } else if (radtan_dist != NULL) {
          vw::Vector<double> dist = radtan_dist->distortion_parameters();
          // Must have 5 parameters, else throw an error
          if (dist.size() != 5) 
            vw_throw(ArgumentErr() << "Expecting 5 distortion coefficients for tsai "
                      << "distortion, got: " << dist.size() << ".\n");
          // resize local dist to 5 elem and copy one by one
          local_distortion.resize(5, 0.0);
          for (size_t i = 0; i < 5; i++) 
            local_distortion[i] = dist[i];
          opt.exact_tsai_to_csm_conv = true;
        }
      }
      
      // See if can copy the exact distortion from the tsai file
      if (!local_distortion.empty()) 
        opt.distortion = local_distortion;
      else
        opt.distortion.resize(5, 0.0);
        
    } else if (opt.distortion_type == "transverse") {
      // This distortion model is a pair of polynomials of degree 3 in x and y.
      opt.distortion.resize(20, 0.0); 
      // Great care is needed below
      // Free term for first polynomial
      opt.distortion[0] = 0;
      opt.distortion[1] = 1; // Set the x term coeff of first poly to 1.0
      // Free term for second polynomial
      opt.distortion[10] = 0;
      opt.distortion[12] = 1; // Set the y term coeff of second poly to 1.0
    } else {
      vw_throw(ArgumentErr() << "Unknown distortion type: " << opt.distortion_type << ".\n");
    }
  }

  if (opt.distortion.size() != 0) {
    if (opt.distortion_type == "radtan" && opt.distortion.size() != 5)
      vw_throw(ArgumentErr() << "Expecting 5 distortion coefficients for radtan "
               << "distortion, got: " << opt.distortion.size() << ".\n");
    if (opt.distortion_type == "transverse" && opt.distortion.size() != 20)
      vw_throw(ArgumentErr() << "Expecting 20 distortion coefficients for transverse "
               << "distortion, got: " << opt.distortion.size() << ".\n"); 
  }
  
  if (opt.refine_intrinsics != "" && opt.input_camera == "")
    vw_throw(ArgumentErr() << "Must provide an input camera to refine its intrinsics.\n");
    
  if (opt.num_pixel_samples <= 0)
    vw_throw(ArgumentErr() << "The number of pixel samples must be positive.\n");

} // End function handle_arguments

// Form a camera based on info the user provided
void manufacture_cam(Options & opt, int wid, int hgt,
		     boost::shared_ptr<CameraModel> & out_cam) {

  if (opt.camera_type == "opticalbar") {
    boost::shared_ptr<vw::camera::OpticalBarModel> opticalbar_cam;
    opticalbar_cam.reset(new vw::camera::OpticalBarModel(opt.sample_file));
    // Make sure the image size matches the input image file.
    // TODO(oalexan1): This looks fishy if the pitch is not 1.
    opticalbar_cam->set_image_size(Vector2i(wid, hgt));
    opticalbar_cam->set_optical_center(Vector2(wid/2.0, hgt/2.0));
    out_cam = opticalbar_cam;
  } else if (opt.camera_type == "pinhole") { // csm frame comes here too

    std::string sample_ext = get_extension(opt.sample_file);
    if (opt.sample_file != "" && sample_ext == ".json") {
      // Read the intrinsics from the csm file. The distortion will be set later.
      asp::CsmModel csm(opt.sample_file);
      opt.focal_length = csm.focal_length();
      opt.optical_center = csm.optical_center(); 
      opt.distortion = csm.distortion();
      if (opt.distortion.size() == 5)
        opt.distortion_type = "radtan";
      else if (opt.distortion.size() == 20)
        opt.distortion_type = "transverse";
      else
        vw_throw(ArgumentErr() 
                 << "Unsupported distortion model in: " << opt.sample_file << ".\n");
    }
  
    boost::shared_ptr<PinholeModel> pinhole_cam;
    if (opt.sample_file != "" &&  sample_ext != ".json") {
      // Use the initial guess from file
      pinhole_cam.reset(new PinholeModel(opt.sample_file));
    } else {
      // Use the intrinsics from the command line or read from the CSM sample
      // file. Use trivial rotation and translation.
      Vector3 ctr(0, 0, 0);
      Matrix<double, 3, 3> rotation;
      rotation.set_identity();
      // When the user does not set the optical center, use the image center times pixel pitch
      Vector2 opt_ctr = opt.optical_center;
      if (std::isnan(opt_ctr[0]) || std::isnan(opt_ctr[1]))
        opt_ctr = Vector2(opt.pixel_pitch * wid/2.0, opt.pixel_pitch * hgt/2.0);
      
      vw::camera::LensDistortion const* distortion = NULL; // no distortion info yet
      pinhole_cam.reset(new PinholeModel(ctr, rotation, opt.focal_length, opt.focal_length,
                                         opt_ctr[0], opt_ctr[1], distortion, opt.pixel_pitch));
    }
    out_cam = pinhole_cam;
  }
}

// Intersect a ray with the DEM or the datum. Return the zero vector on failure.
void dem_or_datum_intersect(Options const& opt, vw::cartography::GeoReference const& geo,
                            ImageViewRef<PixelMask<float>> const& interp_dem,
                            vw::CamPtr input_camera_ptr,
                            vw::Vector2 const& pix, 
                            vw::Vector3 & xyz) { // output

  if (input_camera_ptr.get() == NULL)
    vw_throw(ArgumentErr() << "Input camera was not provided.\n");
  
  Vector3 camera_ctr = input_camera_ptr->camera_center(pix);
  Vector3 camera_vec = input_camera_ptr->pixel_to_vector(pix);

  bool treat_nodata_as_zero = false;
  bool has_intersection = false;
  double height_error_tol = 0.01; // error in DEM height, in meters
  double max_abs_tol = 1e-20; // this will not be reached due to height_error_tol
  double max_rel_tol = 1e-20;
  int num_max_iter   = 100;
  Vector3 xyz_guess  = Vector3();

  xyz = vw::cartography::camera_pixel_to_dem_xyz(camera_ctr, camera_vec,  
                                                 interp_dem, geo, treat_nodata_as_zero,
                                                 has_intersection, height_error_tol,
                                                 max_abs_tol, max_rel_tol, num_max_iter, 
                                                 xyz_guess);
  
  // If we could not intersect the DEM, use the datum to estimate the height
  if (xyz == Vector3() || !has_intersection) 
    xyz = datum_intersection(geo.datum().semi_major_axis() + opt.height_above_datum,
                             geo.datum().semi_minor_axis() + opt.height_above_datum,
                             camera_ctr, camera_vec);
}

// Trace rays from pixel corners to DEM to see where they intersect the DEM.
// Save a pointer to the camera, georef, and interp_dem, as we may need these later.
void extract_lon_lat_cam_ctr_from_camera(Options & opt,
                                         ImageViewRef<PixelMask<float>> const& interp_dem,
                                         GeoReference const& geo,
                                         // Outputs
                                         vw::CamPtr & input_camera_ptr,
                                         std::vector<double> & cam_heights,
                                         vw::Vector3 & cam_ctr) {

  cam_heights.clear();
  cam_ctr = Vector3(0, 0, 0);

  // Load the camera. By now --bundle-adjust-prefix should be set. 
  // We will need it outside of this function. 
  std::string out_prefix;
    asp::SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session, // may change
						       opt,
						       opt.image_file, opt.image_file,
						       opt.input_camera, opt.input_camera,
						       out_prefix));
  input_camera_ptr = session->camera_model(opt.image_file, opt.input_camera);

  // Store here pixel values for the rays emanating from the pixels at
  // which we could intersect with the DEM.
  std::vector<double> good_pixel_values;
  
  int num_points = opt.pixel_values.size()/2;
  opt.lon_lat_values.reserve(2*num_points);
  opt.lon_lat_values.clear();

  // Estimate camera center
  std::vector<vw::Vector3> ctrs, dirs;
  
  for (int it = 0; it < num_points; it++) {

    Vector2 pix(opt.pixel_values[2*it], opt.pixel_values[2*it+1]);
    vw::Vector3 xyz;                                                
    dem_or_datum_intersect(opt, geo, interp_dem, input_camera_ptr, pix, xyz);
     
    if (xyz == Vector3()) {
      vw_out() << "Could not intersect the ground with a ray coming "
	             << "from the camera at pixel: " << pix << ". Skipping it.\n";
      continue;
    }
    ctrs.push_back(input_camera_ptr->camera_center(pix));
    dirs.push_back(input_camera_ptr->pixel_to_vector(pix));
    
    Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
    opt.lon_lat_values.push_back(llh[0]);
    opt.lon_lat_values.push_back(llh[1]);
    good_pixel_values.push_back(opt.pixel_values[2*it]);
    good_pixel_values.push_back(opt.pixel_values[2*it+1]);
    cam_heights.push_back(llh[2]); // will use it later
  }

  if (good_pixel_values.size() < 6) {
    vw_throw(ArgumentErr() << "Successful intersection happened for less than "
	      << "3 pixels. Will not be able to create a camera. Consider checking "
	      << "your inputs, or passing different pixels in --pixel-values. DEM: "
	      << opt.reference_dem << ".\n");
  }

  // Estimate camera center by triangulating back to the camera. This is necessary
  // for RPC, which does not store a camera center
  int num = 0;
  for (size_t it1 = 0; it1 < ctrs.size(); it1++) {
    for (size_t it2 = it1 + 1; it2 < ctrs.size(); it2++) {
      vw::Vector3 err;
      vw::Vector3 pt = vw::stereo::triangulate_pair(dirs[it1], ctrs[it1],
                                                    dirs[it2], ctrs[it2], err);
      if (pt != Vector3(0, 0, 0)) {
        cam_ctr += pt;
        num += 1;
      }
    }
  }
  if (num > 0) 
    cam_ctr = cam_ctr / num;
  
  // Update with the values at which we were successful
  opt.pixel_values = good_pixel_values;
}

vw::Matrix<double> vec2matrix(int rows, int cols, std::vector<double> const& vals) {
  int len = vals.size();
  if (len != rows * cols) 
    vw::vw_throw(vw::ArgumentErr() << "Size mis-match.\n");

  vw::Matrix<double> M;
  M.set_size(rows, cols);

  int count = 0;
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      M(row, col) = vals[count];
      count++;
    }
  }
  return M;
}

// Refine the camera intrinsics and pose. Only applicable to CSM Frame cameras.
void refineIntrinsics(Options const& opt, vw::cartography::GeoReference const& geo,
                      ImageViewRef<PixelMask<float>> & interp_dem,
                      vw::CamPtr const& input_camera_ptr, 
                      int width, int height, asp::CsmModel & csm) {
  
  // Sanity checks
  if (opt.reference_dem.empty() && opt.datum_str.empty())
    vw_throw(ArgumentErr() << "Must provide a DEM or a datum to refine the camera.\n");
  if (input_camera_ptr.get() == NULL)
    vw_throw(ArgumentErr() << "Must provide an input camera to refine its intrinsics.\n");
  
  std::vector<vw::Vector2> all_pix_samples;
  asp::createPixelSamples(width, height, opt.num_pixel_samples, all_pix_samples);  
  
  // Populate the corresponding ground points
  std::vector<vw::Vector2> pix_samples;
  std::vector<vw::Vector3> xyz_samples;
  for (size_t i = 0; i < all_pix_samples.size(); i++) {

    vw::Vector2 pix = all_pix_samples[i];
    vw::Vector3 xyz;
    dem_or_datum_intersect(opt, geo, interp_dem, input_camera_ptr, pix, xyz);
    if (xyz == Vector3()) 
      continue;
      
    pix_samples.push_back(pix);
    xyz_samples.push_back(xyz);
  }
    
  // Must have at least 1 pixel sample
  if (pix_samples.empty()) 
    vw_throw(ArgumentErr() << "Could not find any valid pixel samples.\n");

  asp::refineCsmFrameFit(pix_samples, xyz_samples, opt.refine_intrinsics, csm);
}

// Read a matrix in json format. This will throw an error if the json object
// does not have the expected data.
vw::Matrix<double> json_mat(json const& j, int rows, int cols) {

  vw::Matrix<double> M;
  M.set_size(rows, cols);
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      M(row, col) = j[row][col].get<double>();
    }
  }
  return M;
}

// Create a camera using user-specified options. Keep record
// of the georeference and the datum.
void form_camera(Options & opt, vw::cartography::GeoReference & geo,
                 ImageViewRef<PixelMask<float>> & interp_dem,
                 vw::CamPtr & input_camera_ptr,
                 boost::shared_ptr<CameraModel> & out_cam) {

  ImageView<float> dem;
  float nodata_value = -std::numeric_limits<float>::max(); 
  bool has_dem = false;
  if (opt.reference_dem != "") {
    dem = DiskImageView<float>(opt.reference_dem);
    bool ans = read_georeference(geo, opt.reference_dem);
    if (!ans) 
      vw_throw(ArgumentErr() << "Could not read the georeference from dem: "
                << opt.reference_dem << ".\n");
    has_dem = true;
    vw::read_nodata_val(opt.reference_dem, nodata_value);
    vw_out() << "Using nodata value: " << nodata_value << std::endl;
    
    // For pinhole the datum may be unreliable, so warn only
    bool warn_only = (opt.stereo_session.find("pinhole") != std::string::npos);
    if (!opt.datum_str.empty()) 
      vw::checkDatumConsistency(geo.datum(), opt.datum, warn_only);
    
    if (opt.datum_str.empty()) {
      // Set for completeness
      opt.datum = geo.datum();
      opt.datum_str = geo.datum().name();
    }
      
  } else {
    // Keep track of the datum
    geo.set_datum(opt.datum);
    vw_out() << "No reference DEM provided. Will use a height of "
             << opt.height_above_datum << " above the datum:\n" 
             << geo.datum() << std::endl;
  }

  // Prepare the DEM for interpolation. It may be empty if not provided.
  interp_dem = interpolate(create_mask(dem, nodata_value),
                  BilinearInterpolation(), ZeroEdgeExtension());

  // If we have camera center in ECI or ECEF coordinates in km, convert
  // it to meters, then find the height above datum.
  Vector3 parsed_camera_center(0, 0, 0);
  if (opt.parsed_camera_center_str != "") {
    std::vector<double> vals;
    parse_values<double>(opt.parsed_camera_center_str, vals);
    if (vals.size() != 3) 
      vw_throw(ArgumentErr() << "Could not parse 3 values from: "
                << opt.parsed_camera_center_str << ".\n");

    parsed_camera_center = Vector3(vals[0], vals[1], vals[2]);
    parsed_camera_center *= 1000.0;  // convert to meters
    vw_out() << "Parsed camera center (meters): " << parsed_camera_center << "\n";
  }
  
  // The camera center can be also set on the command line
  if (!std::isnan(opt.camera_center[0])) {
    parsed_camera_center = opt.camera_center;
    vw_out() << "Using the camera center set on the command line: " 
      << std::setprecision(17) << parsed_camera_center << "\n";
  }
  
  if (parsed_camera_center != Vector3() && opt.cam_weight > 0) {
    // If parsed_camera_center is in ECI coordinates, the lon and lat won't be accurate
    // but the height will be.
    Vector3 llh = geo.datum().cartesian_to_geodetic(parsed_camera_center);
    opt.cam_height = llh[2];
  }
    
  vw::Quat parsed_cam_quat;
  if (opt.parsed_cam_quat_str != "") {
    std::vector<double> vals;
    parse_values<double>(opt.parsed_cam_quat_str, vals);
    if (vals.size() != 4) 
      vw_throw(ArgumentErr() << "Could not parse 4 values from: "
                << opt.parsed_cam_quat_str << ".\n");

    parsed_cam_quat = vw::Quat(vals[0], vals[1], vals[2], vals[3]);
    vw_out() << "Parsed camera quaternion: " << parsed_cam_quat << "\n";
  }
    
  if (opt.cam_weight > 0) {
    vw_out() << "Will attempt to find a camera center height above datum of "
             << opt.cam_height
             << " meters with a weight strength of " << opt.cam_weight << ".\n";
  }
  if (opt.cam_ctr_weight > 0 && opt.refine_camera)  
    vw_out() << "Will constrain the camera center with the camera weight.\n"; 

  Vector3 input_cam_ctr(0, 0, 0); // estimated camera center from input camera
  std::vector<double> cam_heights;
  if (opt.input_camera != "") {
    // Extract lon and lat from tracing rays from the camera to the ground.
    // This can modify opt.pixel_values. Also calc the camera center.
    extract_lon_lat_cam_ctr_from_camera(opt, create_mask(dem, nodata_value), 
                                        geo, 
                                        // Outputs
                                        input_camera_ptr, cam_heights, input_cam_ctr);
  }

  // Overwrite the estimated center with what is parsed from vendor's data,
  // if this data exists.
  if (parsed_camera_center != Vector3(0, 0, 0))
    input_cam_ctr = parsed_camera_center;
    
  if (opt.lon_lat_values.size() < 3) 
    vw_throw(ArgumentErr() << "Expecting at least three longitude-latitude pairs.\n");

  if (opt.lon_lat_values.size() != opt.pixel_values.size()) {
    vw_throw(ArgumentErr()
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

  // TODO(oalexan1): Make this into a function
  for (size_t corner_it = 0; corner_it < num_lon_lat_pairs; corner_it++) {

    // Get the height from the DEM if possible
    llh[0] = opt.lon_lat_values[2*corner_it+0];
    llh[1] = opt.lon_lat_values[2*corner_it+1];

    if (llh[1] < -90 || llh[1] > 90) 
      vw_throw(ArgumentErr() << "Detected a latitude out of bounds. "
                << "Perhaps the longitude and latitude are reversed?\n");

    double height = opt.height_above_datum; 
    if (opt.input_camera != "") {
      height = cam_heights[corner_it]; // already computed
    } else {
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
    }
      
    llh[2] = height;
    xyz = geo.datum().geodetic_to_cartesian(llh);
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
  DiskImageView<float> img(opt.image_file);
  int wid = img.cols(), hgt = img.rows();
  if (wid <= 0 || hgt <= 0) 
    vw_throw(ArgumentErr() << "Could not read an image with positive dimensions from: "
              << opt.image_file << ".\n");
  manufacture_cam(opt, wid, hgt, out_cam);

  // Transform it and optionally refine it
  bool verbose = true;
  fit_camera_to_xyz_ht(parsed_camera_center, input_cam_ctr,
                       opt.camera_type, opt.refine_camera,  
                       xyz_vec, opt.pixel_values, 
                       opt.cam_height, opt.cam_weight, opt.cam_ctr_weight, geo.datum(),
                       verbose, out_cam);
    
  return;
}

// Read a pinhole camera from Planet's json file format (*_pinhole.json). Then
// the WGS84 datum is assumed.
void read_pinhole_from_json(Options & opt, vw::cartography::GeoReference & geo,
                            boost::shared_ptr<CameraModel> & out_cam) {

  if (opt.datum_str.empty()) 
    vw_throw(ArgumentErr() << "Must provide a datum to read a pinhole camera.\n");
    
  // Set the datum
  geo.set_datum(opt.datum);
  
  std::ifstream f(opt.input_camera);
  json j = json::parse(f);

  // Parse the focal length and optical center. Negate the focal
  // length to make it positive. We adjust for that later.
  json const& cam = j["P_camera"];
  double fx = -cam[0][0].get<double>();
  double fy = -cam[1][1].get<double>();
  double ox = cam[0][2].get<double>();
  double oy = cam[1][2].get<double>();

  json const& exterior = j["exterior_orientation"];
  double ecef_x = exterior["x_ecef_meters"].get<double>();
  double ecef_y = exterior["y_ecef_meters"].get<double>();
  double ecef_z = exterior["z_ecef_meters"].get<double>();

  // Following the Planet convention of naming things
  vw::Matrix<double> extrinsic = json_mat(j["P_extrinsic"], 4, 4);
  vw::Matrix<double> intrinsic = json_mat(j["P_intrinsic"], 4, 4);

  // Adjust for the fact that Planet likes negative focal lengths, while
  // vw::camera::PinholeModel uses positive values.
  vw::Matrix<double> flip;
  flip.set_identity(4);
  flip(0, 0) = -1;
  flip(1, 1) = -1;
      
  // Create a blank pinhole model and get an alias to it
  out_cam.reset(new vw::camera::PinholeModel());
  PinholeModel & pin = *((PinholeModel*)out_cam.get());

  // Populate the model
  pin.set_pixel_pitch(1.0); // not necessary, but better be explicit
  pin.set_focal_length(vw::Vector2(fx, fy));
  pin.set_point_offset(vw::Vector2(ox, oy));

  pin.set_camera_center(vw::Vector3(ecef_x, ecef_y, ecef_z));

  vw::Matrix<double> world2cam = flip * intrinsic * extrinsic;
  vw::Matrix<double> cam2world = inverse(world2cam);
  pin.set_camera_pose(submatrix(cam2world, 0, 0, 3, 3));
}

void save_linescan(Options & opt) {
  
  // Set this before loading an ASTER camera, as otherwise the camera will not
  // be of linescan type.
  asp::stereo_settings().aster_use_csm = true;

  // Load the cameras. By now bundle-adjust-prefix should be set in stereo settings.
  std::string out_prefix;
    asp::SessionPtr session(asp::StereoSessionFactory::create
                     (opt.stereo_session, // may change
                      opt,
                      opt.image_file, opt.image_file,
                      opt.input_camera, opt.input_camera,
                      out_prefix));

  boost::shared_ptr<CameraModel> camera_model = session->camera_model(opt.image_file,
								      opt.input_camera);
  
  // Get a pointer to the underlying CSM model. It is owned by camera_model.
  asp::CsmModel * csm_cam = asp::csm_model(camera_model, opt.stereo_session);
  
  // This is a bit of a convoluted way of applying the adjustment from the base
  // camera model to the CSM model. To be revisited once all ASP linescan cameras
  // are CSM rather than having CSM as a member.
  if (asp::stereo_settings().bundle_adjust_prefix != "") 
     asp::applyAdjustmentToCsmCamera(opt.image_file,
                                     opt.input_camera,
                                     asp::stereo_settings().bundle_adjust_prefix,
                                     camera_model,
                                     csm_cam);

  vw_out() << "Writing: " << opt.out_camera << std::endl;
  csm_cam->saveState(opt.out_camera);
}

// See --extrinsics.
void camerasFromExtrinsics(Options const& opt) {
  
  // Read the extrinsics
  std::set<std::string> str_col_names = {"image"};  
  std::set<std::string> num_col_names = {"lon", "lat", "height_above_datum", 
                                            "roll", "pitch", "yaw"};
  std::map<std::string, std::vector<std::string>> str_map;
  std::map<std::string, std::vector<double>> num_map;
  asp::readReportFile(opt.extrinsics_file, str_col_names, num_col_names, str_map, num_map); 
  
  // Read the intrinsics
  vw::camera::PinholeModel pinhole(opt.sample_file);
  
  // Read the datum
  vw::cartography::Datum datum(opt.datum_str);
  
  int num_cameras = str_map["image"].size();
  if (num_cameras == 0) 
    vw_throw(ArgumentErr() << "No extrinsics found in: " << opt.extrinsics_file << ".\n");
  
  // Iterate over cameras
  for (int i = 0; i < num_cameras; i++) {
    
    // Read the extrinsics
    double lon = num_map["lon"][i];
    double lat = num_map["lat"][i];
    double height_above_datum = num_map["height_above_datum"][i];
    double roll = num_map["roll"][i];
    double pitch = num_map["pitch"][i];
    double yaw = num_map["yaw"][i];
    
    // Convert camera center to ECEF
    vw::Vector3 llh(lon, lat, height_above_datum);
    vw::Vector3 P = datum.geodetic_to_cartesian(llh);
    // Camera-to-world rotation
    vw::Matrix3x3 ned = datum.lonlat_to_ned_matrix(llh);
    vw::Matrix3x3 R = ned * asp::rollPitchYaw(roll, pitch, yaw) * asp::rotationXY();
    // Form the camera
    pinhole.set_camera_center(P);
    pinhole.set_camera_pose(R);

    // Write the camera
    std::string imageFile = str_map["image"][i];
    std::string camFile = fs::path(imageFile).replace_extension(".tsai").string();
    vw::create_out_dir(camFile);
    vw::vw_out() << "Writing: " << camFile << std::endl;
    pinhole.write(camFile);
  }
} 
  
int main(int argc, char * argv[]) {
  
  Options opt;
  try {
    
    handle_arguments(argc, argv, opt);

    if (opt.camera_type == "linescan") {
      // The linescan workflow is very different than the rest of the code,
      // as only camera conversion is done.
      save_linescan(opt);
      return 0;
    }

    if (opt.extrinsics_file != "") {
      camerasFromExtrinsics(opt);
      return 0;
    }
    
    // Some of the numbers we print need high precision
    vw_out().precision(17);
  
    // Interpolated dem. May not always exist.
    ImageViewRef<PixelMask<float>> interp_dem;
    // Georeference. At least the datum will exist.
    vw::cartography::GeoReference geo;
    // Input camera
    vw::CamPtr input_camera_ptr(NULL);
    // Output camera
    boost::shared_ptr<CameraModel> out_cam;

    if (!opt.planet_pinhole && !opt.exact_tsai_to_csm_conv) {
      // Create a camera using user-specified options. Read geo and interp_dem.
      form_camera(opt, geo, interp_dem, input_camera_ptr, out_cam);
    } else if (!opt.exact_tsai_to_csm_conv) {
      // Read a pinhole camera from Planet's json file format (*_pinhole.json). Then
      // the WGS84 datum is assumed. Ignore all other input options. Read geo datum.
      read_pinhole_from_json(opt, geo, out_cam);
    }
    
    if (opt.camera_type == "opticalbar") {
      ((vw::camera::OpticalBarModel*)out_cam.get())->write(opt.out_camera);
    } else if (opt.camera_type == "pinhole") {
      vw::camera::PinholeModel* pin = NULL;
      boost::shared_ptr<vw::camera::PinholeModel> input_pin;
      std::string out_ext = get_extension(opt.out_camera);
      if (opt.exact_tsai_to_csm_conv) {
        // Get the input pinhole model, to be converted below exactly to CSM
        input_pin.reset(new vw::camera::PinholeModel(opt.input_camera));
        pin = input_pin.get();
        // Must make another copy to satisfy another path in the logic below
        input_camera_ptr.reset(new vw::camera::PinholeModel(opt.input_camera));
      } else { 
        // Use the manufactured camera      
        pin = (vw::camera::PinholeModel*)out_cam.get();
      }
     
      if (out_ext == ".tsai") {
        // Print these only for pinhole, as they may not exist for csm
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(out_cam->camera_center(Vector2()));
        vw_out() << "Output camera center lon, lat, and height above datum: " 
                 << llh << std::endl;
        vw_out() << "Writing: " << opt.out_camera << std::endl;
        pin->write(opt.out_camera);
        
      } else if (out_ext == ".json") {
        DiskImageView<float> img(opt.image_file);
        int width = img.cols(), height = img.rows();
        asp::CsmModel csm;
        csm.createFrameModel(*pin, width, height, 
                             geo.datum().semi_major_axis(), geo.datum().semi_minor_axis(), 
                             opt.distortion_type, opt.distortion);
        if (opt.refine_intrinsics != "")
          refineIntrinsics(opt, geo, interp_dem, input_camera_ptr, width, height, csm);
          
        csm.saveState(opt.out_camera);
      } else {
        vw_throw(ArgumentErr() << "Unknown output camera file extension: " 
                 << out_ext << ".\n");
      }
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

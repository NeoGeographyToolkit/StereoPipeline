// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

/// \file CameraGenUtils.cc

#include <asp/Camera/CameraGenUtils.h>
#include <asp/Camera/CsmModel.h>

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Camera/PinholeModel.h>
#include <vw/Camera/OpticalBarModel.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/LensDistortion.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Image/Interpolation.h>
#include <vw/FileIO/FileTypes.h>
#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>

#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

using namespace vw;
using json = nlohmann::json;

namespace asp {

// Tokenize a string on commas and whitespace into string tokens, keeping each
// token as a string (so numeric precision is preserved until a later parse).
static void splitStringTokens(std::string list, std::vector<std::string> & tokens) {
  tokens.clear();
  std::replace(list.begin(), list.end(), ',', ' ');
  std::istringstream is(list);
  std::string val;
  while (is >> val)
    tokens.push_back(val);
}

// Read an N x M matrix from a json node. Throws if the node lacks the expected data.
static vw::Matrix<double> jsonToMatrix(json const& j, int rows, int cols) {

  vw::Matrix<double> M;
  M.set_size(rows, cols);
  double sum = 0;
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      M(row, col) = j[row][col].get<double>();
      sum += M(row, col);
    }
  }
  return M;
}

// Build an initial camera (pinhole or optical bar) from the given intrinsics or
// a sample camera file. For a CSM (.json) sample, read its intrinsics back.
void manufactureCam(std::string const& camera_type,
                    std::string const& sample_file,
                    int wid, int hgt,
                    double & focal_length,
                    double & pixel_pitch,
                    vw::Vector2 & optical_center,
                    std::vector<double> & distortion,
                    std::string & distortion_type,
                    boost::shared_ptr<vw::CameraModel> & out_cam) {
  if (camera_type == "opticalbar") {
    boost::shared_ptr<vw::OpticalBarModel> opticalbar_cam =
      boost::make_shared<vw::OpticalBarModel>(sample_file);
    // Make sure the image size matches the input image file.
    // TODO(oalexan1): This looks fishy if the pitch is not 1.
    opticalbar_cam->set_image_size(Vector2i(wid, hgt));
    opticalbar_cam->set_optical_center(Vector2(wid/2.0, hgt/2.0));
    out_cam = opticalbar_cam;
  } else if (camera_type == "pinhole") { // csm frame comes here too

    std::string sample_ext = vw::get_extension(sample_file);
    if (sample_file != "" && sample_ext == ".json") {
      // Read the intrinsics from the csm file. The distortion will be set later.
      asp::CsmModel csm(sample_file);
      focal_length = csm.focal_length();
      pixel_pitch = csm.frame_pixel_pitch();
      optical_center = csm.optical_center() * csm.frame_pixel_pitch();
      // TODO(oalexan1): Must copy the target name from CSM
      distortion = csm.distortion();
      if (distortion.size() == 5)
        distortion_type = "radtan";
      else if (distortion.size() == 3)
        distortion_type = "radial";
      else if (distortion.size() == 20)
        distortion_type = "transverse";
      else
        vw_throw(ArgumentErr()
                 << "Unsupported distortion model in: " << sample_file << ".\n");
    }

    boost::shared_ptr<PinholeModel> pinhole_cam;
    if (sample_file != "" &&  sample_ext != ".json") {
      // Use the initial guess from file
      pinhole_cam = boost::make_shared<PinholeModel>(sample_file);
    } else {
      // Use the intrinsics from the command line or read from the CSM sample
      // file. Use trivial rotation and translation.
      Vector3 ctr(0, 0, 0);
      Matrix<double, 3, 3> rotation;
      rotation.set_identity();
      // When the user does not set the optical center, use the image center
      Vector2 opt_ctr = optical_center;
      if (std::isnan(opt_ctr[0]) || std::isnan(opt_ctr[1]))
        opt_ctr = Vector2(pixel_pitch * wid/2.0, pixel_pitch * hgt/2.0);

      vw::LensDistortion const* dist = NULL; // no distortion info yet
      pinhole_cam = boost::make_shared<PinholeModel>
                      (ctr, rotation, focal_length, focal_length,
                       opt_ctr[0], opt_ctr[1], dist,
                       pixel_pitch);
    }
    out_cam = pinhole_cam;
  }
}

// For each lon-lat pair, compute the ground xyz point and its lon-lat-height.
void cornersToGroundXyz(std::vector<double> const& lon_lat_values,
                        std::vector<double> const& cam_heights,
                        bool has_input_camera,
                        vw::ImageViewRef<vw::PixelMask<float>> const& interp_dem,
                        bool has_dem,
                        vw::GeoReference const& geo,
                        double height_above_datum,
                        std::vector<vw::Vector3> & xyz_vec,
                        std::vector<vw::Vector3> & llh_vec) {

  xyz_vec.clear();
  llh_vec.clear();
  size_t num_lon_lat_pairs = lon_lat_values.size()/2;

  Vector2 pix;
  Vector3 llh, xyz;
  for (size_t corner_it = 0; corner_it < num_lon_lat_pairs; corner_it++) {

    // Get the height from the DEM if possible
    llh[0] = lon_lat_values[2*corner_it+0];
    llh[1] = lon_lat_values[2*corner_it+1];

    if (llh[1] < -90 || llh[1] > 90)
      vw_throw(ArgumentErr() << "Detected a latitude out of bounds. "
                << "Perhaps the longitude and latitude are reversed?\n");

    double height = height_above_datum;
    if (has_input_camera) {
      height = cam_heights[corner_it]; // already computed
    } else {
      if (has_dem) {
        bool success = false;
        pix = geo.lonlat_to_pixel(subvector(llh, 0, 2));
        int len = BilinearInterpolation::pixel_buffer;
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
    llh_vec.push_back(llh);
  }
}

// Parse the frame index to extract lon_lat_values_str. Look for a line
// having this image, and search for "POLYGON" followed by spaces and "((".
void parseFrameIndex(std::string const& frame_index,
                     std::string const& image_file,
                     bool parse_ecef,
                     double cam_weight,
                     // Outputs
                     std::string & lon_lat_values_str,
                     std::string & parsed_camera_center_str,
                     std::string & parsed_cam_quat_str) {

  boost::filesystem::path p(image_file);
  std::string image_base = p.stem().string(); // strip the directory name and suffix
  std::ifstream file(frame_index.c_str());
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
      lon_lat_values_str = line.substr(beg_pos, end_pos - beg_pos);
      vw_out() << "Parsed the lon-lat corner values: "
                << lon_lat_values_str << "\n";

      // Also parse the camera height constraint, unless manually specified
      if (cam_weight > 0 || parse_ecef) {
        std::vector<std::string> vals;
        splitStringTokens(line, vals);

        if (vals.size() < 12)
          vw_throw(ArgumentErr() << "Could not parse 12 values from: " << line << ".\n");

        // Extract the ECEF coordinates of the camera center. Keep them as a
        // string until we can convert to height above datum.
        if (parse_ecef) {
          if (vals.size() < 19)
            vw_throw(ArgumentErr() << "Could not parse 19 values from: " << line << ".\n");

          std::string x = vals[12];
          std::string y = vals[13];
          std::string z = vals[14];
          parsed_camera_center_str = x + " " + y + " " + z;
          vw_out() << "Parsed the ECEF camera center in km: "
                   << parsed_camera_center_str <<".\n";

          std::string q0 = vals[15];
          std::string q1 = vals[16];
          std::string q2 = vals[17];
          std::string q3 = vals[18];
          parsed_cam_quat_str = q0 + " " + q1 + " " + q2 + " " + q3;
          vw_out() << "Parsed the ECEF quaternion: "
                   << parsed_cam_quat_str <<".\n";
        }
      } // End parsing camera height constraint

      break;
    } // end of if we found the image name in the line
  } // end of loop through lines in frame index file

  if (lon_lat_values_str == "")
    vw_throw(ArgumentErr() << "Could not parse the entry for " << image_base
              << " in file: " << frame_index << ".\n");

}

// Fetch metadata from an ISIS cube for saving to the output CSM camera.
void isisCubMetadata(boost::shared_ptr<vw::CameraModel> in_cam,
                     // Outputs
                     double & ephem_time,
                     vw::Vector3 & sun_pos,
                     std::string & serial_number,
                     std::string & target_name) {

  // Initialize the output variables
  ephem_time    = 0.0;
  sun_pos       = vw::Vector3(0, 0, 0);
  serial_number = "";
  target_name   = "";

#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1

  IsisCameraModel const * isis_cam = NULL;
  if (in_cam.get() != NULL)
    isis_cam = dynamic_cast<IsisCameraModel const*>(vw::unadjusted_model(in_cam.get()));

  if (isis_cam != NULL) {
    ephem_time    = isis_cam->ephemeris_time();
    sun_pos       = isis_cam->sun_position();
    serial_number = isis_cam->serial_number();
    target_name   = isis_cam->target_name();
  }
#endif

}

// Average the pairwise triangulated intersection of rays (given by centers and
// directions) to estimate a single camera center. Needed for models like RPC
// that do not store a camera center.
vw::Vector3 triangulateCameraCenter(std::vector<vw::Vector3> const& ctrs,
                                    std::vector<vw::Vector3> const& dirs) {

  vw::Vector3 cam_ctr(0, 0, 0);
  int num = 0;
  for (size_t it1 = 0; it1 < ctrs.size(); it1++) {
    for (size_t it2 = it1 + 1; it2 < ctrs.size(); it2++) {
      vw::Vector3 err;
      vw::Vector3 pt = vw::triangulate_pair(dirs[it1], ctrs[it1],
                                            dirs[it2], ctrs[it2], err);
      if (pt != Vector3(0, 0, 0)) {
        cam_ctr += pt;
        num += 1;
      }
    }
  }
  if (num > 0)
    cam_ctr = cam_ctr / num;


  return cam_ctr;
}

// Read a pinhole camera from Planet's json file format (*_pinhole.json). Then
// the WGS84 datum is assumed.
void readPlanetPinhole(std::string const& input_camera,
                       vw::Datum const& datum,
                       // Outputs
                       vw::GeoReference & geo,
                       boost::shared_ptr<vw::CameraModel> & out_cam) {

  if (datum.name().empty())
    vw_throw(ArgumentErr() << "Must provide a datum to read a pinhole camera.\n");

  // Set the datum
  geo.set_datum(datum);

  std::ifstream f(input_camera);
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
  vw::Matrix<double> extrinsic = jsonToMatrix(j["P_extrinsic"], 4, 4);
  vw::Matrix<double> intrinsic = jsonToMatrix(j["P_intrinsic"], 4, 4);

  // Adjust for the fact that Planet likes negative focal lengths, while
  // vw::PinholeModel uses positive values.
  vw::Matrix<double> flip;
  flip.set_identity(4);
  flip(0, 0) = -1;
  flip(1, 1) = -1;

  // Create a blank pinhole model and get an alias to it
  out_cam = boost::make_shared<vw::PinholeModel>();
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

// Populate the CSM lens distortion parameters when not set, and validate the
// count for the given type. For a radtan target with a tsai input the exact
// distortion is copied and exact_tsai_to_csm_conv is set.
void setupCsmDistortion(std::string const& camera_type,
                        std::string const& out_ext,
                        std::string const& input_ext,
                        std::string const& input_camera,
                        bool csm_refit_pose,
                        std::string const& distortion_type,
                        // Outputs
                        std::vector<double> & distortion,
                        bool & exact_tsai_to_csm_conv) {

  exact_tsai_to_csm_conv = false;

  // Populate the distortion parameters for CSM, if not set. Not for --csm-refit-pose:
  // there, if no --distortion is given, the input camera's own distortion is kept as-is
  // (a no-op transplant), rather than being reset to zero.
  if (distortion.size() == 0 && camera_type == "pinhole" && out_ext == ".json" &&
      !csm_refit_pose) {
    if (distortion_type == "radtan") {

      // Consider the special case when we can precisely convert tsai to json,
      // if the distortion is radtan (tsai) or no distortion.
      std::vector<double> local_distortion;
      if (input_ext == ".tsai") {
        vw::PinholeModel pin(input_camera);
        vw::NullLensDistortion const* zero_dist =
          dynamic_cast<vw::NullLensDistortion const*>(pin.lens_distortion());
        vw::TsaiLensDistortion const* radtan_dist =
          dynamic_cast<vw::TsaiLensDistortion const*>(pin.lens_distortion());
        if (zero_dist != NULL) {
          // Zero distortion can be thought of as radtan distortion with all zeros
          local_distortion.resize(5, 0.0);
          exact_tsai_to_csm_conv = true;
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
          exact_tsai_to_csm_conv = true;
        }
      }

      // See if can copy the exact distortion from the tsai file
      if (!local_distortion.empty())
        distortion = local_distortion;
      else
        distortion.resize(5, 0.0);

    } else if (distortion_type == "radial") {
      distortion.resize(3, 0.0);
    } else if (distortion_type == "transverse") {
      // This distortion model is a pair of polynomials of degree 3 in x and y.
      distortion.resize(20, 0.0);
      // Great care is needed below
      // Free term for first polynomial
      distortion[0] = 0;
      distortion[1] = 1; // Set the x term coeff of first poly to 1.0
      // Free term for second polynomial
      distortion[10] = 0;
      distortion[12] = 1; // Set the y term coeff of second poly to 1.0
    } else {
      vw_throw(ArgumentErr() << "Unknown distortion type: " << distortion_type << ".\n");
    }
  }

  if (distortion.size() != 0) {
    if (distortion_type == "radtan" && distortion.size() != 5)
      vw_throw(ArgumentErr() << "Expecting 5 distortion coefficients for radtan "
               << "distortion, got: " << distortion.size() << ".\n");
    if (distortion_type == "radial" && distortion.size() != 3)
      vw_throw(ArgumentErr() << "Expecting 3 distortion coefficients for radial "
               << "distortion, got: " << distortion.size() << ".\n");
    if (distortion_type == "transverse" && distortion.size() != 20)
      vw_throw(ArgumentErr() << "Expecting 20 distortion coefficients for transverse "
               << "distortion, got: " << distortion.size() << ".\n");
  }

}

} // end namespace asp

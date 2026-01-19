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

// Tool to compare two camera models for the same image. For example,
// compare ISIS to CSM, linescan to RPC (for DG, PeruSat, or
// Pleiades), Optical bar vs pinhole (with the latter created with
// convert_pinhole_model).

// For each camera model find the camera center and ray direction at a
// set of sampled pixels, then by projecting pixels to the ground
// using the cam1 camera and back-projecting the resulting points into
// the cam2 camera, then doing this in reverse.

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Sessions/CameraUtils.h>

#include <asp/asp_config.h> // defines ASP_HAVE_PKG_ISIS
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/IsisCameraModel.h>
#endif // ASP_HAVE_PKG_ISIS

#include <vw/Cartography/DatumUtils.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/Math/NewtonRaphson.h>
#include <vw/Core/Stopwatch.h>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options: vw::GdalWriteOptions {
  std::string image_file, cam1_file, cam2_file, session1, session2, bundle_adjust_prefix,
  cam1_bundle_adjust_prefix, cam2_bundle_adjust_prefix, datum, bathy_plane;
  int sample_rate;
  double subpixel_offset, height_above_datum, refraction_index;
  bool print_per_pixel_results, aster_use_csm, aster_vs_csm, test_error_propagation;
  vw::Vector2 single_pixel;
  std::vector<vw::BathyPlane> bathy_plane_vec;

  Options() {}
};

// A class to manage a local tangent plane at a given ECEF point. The tangent
// plane is tangent to datum at that point. For an ellipsoid datum, this plane
// is perpendicular to the geodetic normal (not the geocentric radius vector).
// The X and Y axes are East and North respectively.
class BathyFunctor {
public:
  BathyFunctor(vw::Vector3 const& ecef_point,
               vw::cartography::Datum const& datum,
               vw::CamPtr const& cam,
               vw::BathyPlane const& bathy_plane,
               double refraction_index): m_cam(cam), m_origin(ecef_point),
                                         m_bathy_plane(bathy_plane),
                                         m_refraction_index(refraction_index) {
    // Bathy planes require WGS_1984 datum
    if (datum.name() != "WGS_1984")
      vw::vw_throw(vw::ArgumentErr() << "Bathy plane processing requires WGS_1984 datum.\n");

    // Convert ECEF to geodetic
    vw::Vector3 llh = datum.cartesian_to_geodetic(ecef_point);

    // Get the NED matrix at this location
    // Columns are: [North | East | Down] in ECEF coordinates
    vw::Matrix3x3 ned_matrix = datum.lonlat_to_ned_matrix(llh);

    // X axis is East (column 1 of NED matrix)
    m_x_axis = vw::math::select_col(ned_matrix, 1);

    // Y axis is North (column 0 of NED matrix)
    m_y_axis = vw::math::select_col(ned_matrix, 0);

    // Normal is Down (column 2 of NED matrix), but we want it pointing up
    m_normal = -vw::math::select_col(ned_matrix, 2);
  }

  // Intersect a ray with the tangent plane and return 2D coordinates in the tangent
  // plane coordinate system.
  vw::Vector2 rayPlaneIntersect(vw::Vector3 const& ray_pt,
                                 vw::Vector3 const& ray_dir) const {
    // Intersect the ray with the tangent plane at m_origin
    // Plane equation: dot(X - m_origin, m_normal) = 0
    // Ray equation: X = ray_pt + t * ray_dir
    // Solve: dot(ray_pt + t * ray_dir - m_origin, m_normal) = 0
    double denom = vw::math::dot_prod(ray_dir, m_normal);
    if (std::abs(denom) < 1e-10)
      vw::vw_throw(vw::ArgumentErr() << "Ray is parallel to tangent plane.\n");

    double t = vw::math::dot_prod(m_origin - ray_pt, m_normal) / denom;
    vw::Vector3 intersection = ray_pt + t * ray_dir;

    // Project intersection onto tangent plane axes
    vw::Vector3 offset = intersection - m_origin;
    double x = vw::math::dot_prod(offset, m_x_axis);
    double y = vw::math::dot_prod(offset, m_y_axis);

    return vw::Vector2(x, y);
  }

  // Operator for use with Newton-Raphson solver
  // Input: pix is a pixel in the camera
  // Output: 2D coordinates in the tangent plane after ray bending
  vw::Vector2 operator()(vw::Vector2 const& pix) const {
    // Get ray from camera (without bathy correction)
    vw::Vector3 cam_ctr = m_cam->camera_center(pix);
    vw::Vector3 cam_dir = m_cam->pixel_to_vector(pix);

    // Apply Snell's law to bend the ray at the water surface
    vw::Vector3 refracted_pt, refracted_dir;
    bool success = vw::curvedSnellLaw(cam_ctr, cam_dir,
                                      m_bathy_plane.bathy_plane,
                                      m_bathy_plane.plane_proj,
                                      m_refraction_index,
                                      refracted_pt, refracted_dir);

    if (!success)
      vw::vw_throw(vw::ArgumentErr() << "Snell's law refraction failed.\n");

    // Intersect refracted ray with tangent plane and return 2D coordinates
    return rayPlaneIntersect(refracted_pt, refracted_dir);
  }

  vw::CamPtr m_cam;                 // Camera pointer
  vw::Vector3 m_origin;             // Origin of tangent plane (ECEF point P)
  vw::BathyPlane m_bathy_plane;     // Bathy plane for refraction
  double m_refraction_index;        // Index of refraction
  vw::Vector3 m_x_axis;             // East direction (tangent plane X axis)
  vw::Vector3 m_y_axis;             // North direction (tangent plane Y axis)
  vw::Vector3 m_normal;             // Up direction (perpendicular to plane)
};

// Project an ECEF point to pixel, accounting for bathymetry if the point
// is below the bathy plane (water surface).
vw::Vector2 point_to_pixel(vw::CamPtr const& cam,
                           vw::cartography::Datum const& datum,
                           vw::BathyPlane const& bathy_plane,
                           double refraction_index,
                           vw::Vector3 const& ecef_point) {

  // Get camera pixel as if there was no refraction
  vw::Vector2 pix = cam->point_to_pixel(ecef_point);

  // Convert ECEF point to projected coordinates for distance check
  vw::Vector3 proj_pt = vw::proj_point(bathy_plane.plane_proj, ecef_point);

  // Check signed distance to bathy plane
  double dist = vw::signed_dist_to_plane(bathy_plane.bathy_plane, proj_pt);

  // If point is above the water surface (positive distance), no refraction
  if (dist >= 0)
    return pix;

  // Point is below water surface - need to account for refraction
  // Set up the BathyFunctor for Newton-Raphson iteration
  BathyFunctor bathy_func(ecef_point, datum, cam, bathy_plane, refraction_index);

  // Set up Newton-Raphson solver with numerical Jacobian
  vw::math::NewtonRaphson nr(bathy_func);

  // Solve: find pixel such that bathy_func(pix) projects to (0, 0) in tangent plane
  // since ecef_point is the origin of the tangent plane
  vw::Vector2 target(0, 0);
  double step = 0.5;    // 0.5 meter step for numerical differentiation
  double tol = 1e-6;    // 1e-6 pixels tolerance

  pix = nr.solve(pix, target, step, tol);

  return pix;
}

void handle_arguments(int argc, char *argv[], Options& opt) {

  double nan = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("image", po::value(&opt.image_file),  "Image file.")
    ("cam1",  po::value(&opt.cam1_file),   "Camera 1 file.")
    ("cam2",  po::value(&opt.cam2_file),   "Camera 2 file.")
    ("session1", po::value(&opt.session1),
     "Session to use for camera 1 (if not provided it will be guessed).")
    ("session2", po::value(&opt.session2),
     "Session to use for camera 2 (if not provided it will be guessed).")
    ("sample-rate",   po::value(&opt.sample_rate)->default_value(100),
     "Use one out of these many pixels when sampling the image.")
    ("subpixel-offset",   po::value(&opt.subpixel_offset)->default_value(0.0),
     "Add to each integer pixel this offset (in x and y) when sampling the image.")
    ("single-pixel",   po::value(&opt.single_pixel)->default_value(Vector2(nan, nan)),
     "Instead of sampling pixels from the image use only this pixel.")
    ("print-per-pixel-results", po::bool_switch(&opt.print_per_pixel_results)->default_value(false)->implicit_value(true),
     "Print the results at each pixel.")
    ("height-above-datum",   po::value(&opt.height_above_datum)->default_value(0.0),
     "Let the ground surface be at this height above the datum (measured in meters).")
    ("datum", po::value(&opt.datum),
     "Set the datum. This will override the datum from the input cameras. Usually needed "
     "only for Pinhole cameras, when the camera does not have the datum information. "
     "Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA "
     "(3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars "
     "(=D_MARS), Moon (=D_MOON).")
    ("aster-use-csm",
     po::bool_switch(&opt.aster_use_csm)->default_value(false)->implicit_value(true),
     "Use the CSM model with ASTER cameras (-t aster).")
    ("aster-vs-csm",
     po::bool_switch(&opt.aster_vs_csm)->default_value(false)->implicit_value(true),
     "Compare projecting into the camera without and with using the CSM model for ASTER.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Adjust the cameras using this prefix.")
    ("cam1-bundle-adjust-prefix", po::value(&opt.cam1_bundle_adjust_prefix),
     "Adjust the first camera using this prefix.")
    ("cam2-bundle-adjust-prefix", po::value(&opt.cam2_bundle_adjust_prefix),
     "Adjust the first camera using this prefix.")
    ("test-error-propagation", po::bool_switch(&opt.test_error_propagation)->default_value(false)->implicit_value(true),
     "Test computing the stddev (see --propagate-errors). This is an undocumented "
     "developer option.")
    ("bathy-plane", po::value(&opt.bathy_plane),
     "Read from this file a bathy plane, so a water surface which is a plane in local "
     "projected coordinates. A ray from the camera to the ellipsoid determined by "
     "--height-above-datum that encounters this bathy plane along the way will get bent "
     "according to Snell's law. Same for a ray going in reverse.")
    ("refraction-index", po::value(&opt.refraction_index)->default_value(1.0),
     "The index of refraction of water to be used in bathymetry correction. "
     "Must be bigger than 1. This index can be computed with refr_index.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--image <image file> --cam1 <camera 1 file> --cam2 <camera 2 file> "
                    "[other options]");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.image_file == "" || opt.cam1_file == "" || opt.cam2_file == "")
    vw_throw(ArgumentErr() << "Not all inputs were specified.\n" << usage << general_options);

  if (opt.sample_rate <= 0)
    vw_throw(ArgumentErr() << "The sample rate must be positive.\n" << usage << general_options);

  // If we have to deal with the CSM model, ensure it is loaded. Set this early.
  if (opt.aster_vs_csm)
    opt.aster_use_csm = true;
  asp::stereo_settings().aster_use_csm = opt.aster_use_csm;

  // If either cam1_adjust_prefix or cam2_adjust_prefix is set, then
  // must not set bundle_adjust_prefix. Throw an error then.
  if (opt.cam1_bundle_adjust_prefix != "" || opt.cam2_bundle_adjust_prefix != "") {
    if (opt.bundle_adjust_prefix != "")
      vw_throw(ArgumentErr() << "Cannot set both --bundle-adjust-prefix and "
               "--cam1-bundle-adjust-prefix or --cam2-bundle-adjust-prefix.\n");
  } else {
    // Need this to be able to load adjusted camera models. This must be set
    // before loading the cameras.
    asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
  }

  if (opt.test_error_propagation)
    asp::stereo_settings().propagate_errors = true;

  // Validate bathymetry options
  bool have_bathy_plane = (opt.bathy_plane != "");
  bool have_refraction = (opt.refraction_index > 1.0);

  if (have_bathy_plane && !have_refraction)
    vw_throw(ArgumentErr() << "When --bathy-plane is set, --refraction-index must be "
             << "specified and bigger than 1.\n");

  if (!have_bathy_plane && have_refraction)
    vw_throw(ArgumentErr() << "When --refraction-index is set, --bathy-plane must "
             << "also be specified.\n");

  // Load bathy plane if specified
  if (have_bathy_plane) {
    int num_images = 1;
    vw::readBathyPlanes(opt.bathy_plane, num_images, opt.bathy_plane_vec);
    vw_out() << "Loaded bathy plane: " << opt.bathy_plane << "\n";
    vw_out() << "Refraction index: " << opt.refraction_index << "\n";
  }

}

// Sort the diffs and print some stats
void print_diffs(std::string const& tag, std::vector<double> & diffs) {
  std::sort(diffs.begin(), diffs.end());

  vw_out() << "\n";

  if (diffs.empty()) {
    vw_out() << "Empty list of diffs for: " << tag << "\n";
    return;
  }

  vw_out() << tag << "\n";
  vw_out() << "Min:    " << diffs[0] << "\n";
  vw_out() << "Median: " << diffs[diffs.size()/2] << "\n";
  vw_out() << "Max:    " << diffs.back() << "\n";
}

void testErrorPropagation(Options const& opt,
                          vw::cartography::Datum const& datum,
                          vw::CamPtr cam1_model,
                          vw::CamPtr cam2_model) {

  double major_axis = datum.semi_major_axis() + opt.height_above_datum;
  double minor_axis = datum.semi_minor_axis() + opt.height_above_datum;

  // Try to find a pair of pixels corresponding to same tri point,
  // within image bounds
  vw::Vector2 pix1, pix2;
  Vector3 triPt;
  for (int i = 0; i < 20; i++) {

    pix1 = Vector2(i * 1000, i * 1000);
    Vector3 cam1_dir = cam1_model->pixel_to_vector(pix1);
    Vector3 cam1_ctr = cam1_model->camera_center(pix1);

    // Shoot a ray from the cam1 camera, intersect it with the
    // given height above datum
    triPt = vw::cartography::datum_intersection(major_axis, minor_axis, cam1_ctr, cam1_dir);

    // Skip invalid intersections
    if (triPt == Vector3(0, 0, 0))
      continue;

    // Project to second camera
    pix2 = cam2_model->point_to_pixel(triPt);

    if (pix2.x() > 0 && pix2.y() > 0)
      break;
  }

  vw::vw_out() << "Left pixel:  " << pix1 << std::endl;
  vw::vw_out() << "Right pixel: " << pix2 << std::endl;
  auto const& v = asp::stereo_settings().horizontal_stddev; // alias
  vw::Vector2 ans = asp::propagateCovariance(triPt, datum,
                                             v[0], v[1],
                                             cam1_model.get(), cam2_model.get(),
                                             pix1, pix2);
  vw::vw_out() << "Horizontal and vertical stddev: " << ans << std::endl;
}

// This is an important sanity check for RPC cameras, which have only a limited
// range of valid heights above datum.
void rpc_datum_sanity_check(std::string const& cam_file,
                            double height_above_datum,
                            vw::camera::CameraModel const* cam) {
  // Cast the camera to RPC
  asp::RPCModel const* rpc_cam = dynamic_cast<asp::RPCModel const*>(cam);
  if (rpc_cam == NULL)
    vw::vw_throw(vw::ArgumentErr() << "Expecting an RPC camera model.\n");

  vw::Vector3 lonlatheight_offset = rpc_cam->lonlatheight_offset();
  vw::Vector3 lonlatheight_scale  = rpc_cam->lonlatheight_scale();
  double mid_ht = lonlatheight_offset[2];
  double min_ht = mid_ht - lonlatheight_scale[2];
  double max_ht = mid_ht + lonlatheight_scale[2];

  if (height_above_datum < min_ht || height_above_datum > max_ht) {
    vw::vw_out() << "\n";
    vw::vw_out(vw::WarningMessage)
      << "For RPC camera file: " << cam_file
      << ", the range of valid heights above datum is "
      << min_ht << " to " << max_ht
      << " meters. The provided height above datum is "
      << height_above_datum
      << " meters. The results may be inaccurate. Set appropriately "
      << "the --height-above-datum option.\n\n";
  }
}

void run_cam_test(Options & opt) {

  bool indiv_adjust_prefix = (opt.cam1_bundle_adjust_prefix != "" ||
                              opt.cam2_bundle_adjust_prefix != "");

  // Load cam1
  std::string out_prefix;
  std::string default_session1 = opt.session1; // save it before it changes
  if (indiv_adjust_prefix)
    asp::stereo_settings().bundle_adjust_prefix = opt.cam1_bundle_adjust_prefix;
  asp::SessionPtr cam1_session(asp::StereoSessionFactory::create
                              (opt.session1, // may change
                              opt,
                              opt.image_file, opt.image_file,
                              opt.cam1_file, opt.cam1_file,
                              out_prefix));
  boost::shared_ptr<vw::camera::CameraModel> cam1_model
    = cam1_session->camera_model(opt.image_file, opt.cam1_file);

  // Load cam2
  std::string default_session2 = opt.session2; // save it before it changes
  if (indiv_adjust_prefix)
    asp::stereo_settings().bundle_adjust_prefix = opt.cam2_bundle_adjust_prefix;
  asp::SessionPtr cam2_session(asp::StereoSessionFactory::create
                          (opt.session2, // may change
                          opt,
                          opt.image_file, opt.image_file,
                          opt.cam2_file, opt.cam2_file,
                          out_prefix));
  boost::shared_ptr<vw::camera::CameraModel> cam2_model
    = cam2_session->camera_model(opt.image_file, opt.cam2_file);

  if (indiv_adjust_prefix)
    asp::stereo_settings().bundle_adjust_prefix = ""; // reset

  bool found_datum = false;
  vw::cartography::Datum datum;
  if (opt.datum != "") {
    // Use the datum specified by the user
    datum.set_well_known_datum(opt.datum);
    found_datum = true;
  }

  // See if the first camera has a datum    
  vw::cartography::Datum cam_datum;
  bool warn_only = true; // warn about differences in the datums
  bool found_cam_datum = asp::datum_from_camera(opt.image_file, opt.cam1_file,
                                                // Outputs
                                                opt.session1, cam1_session, cam_datum);
  if (found_datum && found_cam_datum)
    vw::checkDatumConsistency(datum, cam_datum, warn_only);

    if (!found_datum && found_cam_datum) {
    datum = cam_datum;
    found_datum = true;
  }

  // Same for the second camera
  found_cam_datum = asp::datum_from_camera(opt.image_file, opt.cam2_file,
                                            // Outputs
                                            opt.session2, cam2_session, cam_datum);
  if (found_datum && found_cam_datum)
    vw::checkDatumConsistency(datum, cam_datum, warn_only);

  if (!found_datum && found_cam_datum) {
    datum = cam_datum;
    found_datum = true;
  }

  if (!found_datum)
    vw_throw(ArgumentErr() << "Could not find the datum. Set --datum or use "
              << "the nadirpinhole session (for cameras relative to a planet).\n");
  vw_out() << "Using datum: " << datum << std::endl;

  // Sanity check
  vw::Vector3 llh1 = datum.cartesian_to_geodetic(cam1_model->camera_center(Vector2()));
  vw::Vector3 llh2 = datum.cartesian_to_geodetic(cam2_model->camera_center(Vector2()));
  if (llh1[2] < 0 || llh2[2] < 0)
          vw::vw_out(vw::WarningMessage) << "First or second camera center is below "
          << "the zero datum surface. Check your data. Consider using "
          << "the --datum and/or --height-above-datum options.\n";

  if (opt.session1 == opt.session2 && (default_session1 == "" || default_session2 == "") &&
     (opt.session1 == "dg"))
    vw::vw_out(vw::WarningMessage) << "The session names for both cameras "
              << "were guessed as: '" << opt.session1 << "'. It is suggested that they be "
              << "explicitly specified using --session1 and --session2, as a DigitalGlobe "
              << "camera file may contain both exact linescan and RPC cameras.\n";

  // Sanity checks for RPC cameras
  if (opt.session1 == "rpc")
    rpc_datum_sanity_check(opt.cam1_file, opt.height_above_datum,
                            vw::camera::unadjusted_model(cam1_model.get()));
  if (opt.session2 == "rpc")
    rpc_datum_sanity_check(opt.cam2_file, opt.height_above_datum,
                            vw::camera::unadjusted_model(cam2_model.get()));

  if (opt.test_error_propagation && opt.session1 == "dg" && opt.session2 == "dg") {
    testErrorPropagation(opt, datum, cam1_model, cam2_model);
    return;
  }

  // Find the input image dimensions
  int image_cols = 0, image_rows = 0;
  try {
    DiskImageView<float> image(opt.image_file);
    image_cols = image.cols();
    image_rows = image.rows();
  } catch(const std::exception& e) {
    // For CSM-to-CSM ground-to-image and image-to-ground comparisons only,
    // the camera has the dimensions if the .cub image is missing.
    asp::CsmModel * csm_model
      = dynamic_cast<asp::CsmModel*>(vw::camera::unadjusted_model(cam1_model.get()));
    if (csm_model != NULL) {
      image_cols = csm_model->get_image_size()[0];
      image_rows = csm_model->get_image_size()[1];
    } else {
      vw::vw_throw(ArgumentErr() << e.what());
    }
  }
  vw_out() << "Image dimensions: " << image_cols << ' ' << image_rows << std::endl;

  Stopwatch sw;
  sw.start();

  double major_axis = datum.semi_major_axis() + opt.height_above_datum;
  double minor_axis = datum.semi_minor_axis() + opt.height_above_datum;

  bool have_bathy_plane = (opt.bathy_plane != "");

  // Iterate over the image
  bool single_pix = !std::isnan(opt.single_pixel[0]) && !std::isnan(opt.single_pixel[1]);
  std::vector<double> ctr_diff, dir_diff, cam1_to_cam2_diff, cam2_to_cam1_diff;
  std::vector<double> nocsm_vs_csm_diff;
  int num_failed = 0;
  for (int col = 0; col < image_cols; col += opt.sample_rate) {
    for (int row = 0; row < image_rows; row += opt.sample_rate) {

      try {
        Vector2 image_pix(col + opt.subpixel_offset, row + opt.subpixel_offset);
        if (single_pix)
          image_pix = opt.single_pixel;

        if (opt.print_per_pixel_results || single_pix)
          vw_out() << "Pixel: " << image_pix << "\n";

        Vector3 cam1_ctr = cam1_model->camera_center(image_pix);
        Vector3 cam2_ctr = cam2_model->camera_center(image_pix);
        ctr_diff.push_back(norm_2(cam1_ctr - cam2_ctr));

        if (opt.print_per_pixel_results)
          vw_out() << "Camera center diff: " << ctr_diff.back() << std::endl;

        Vector3 cam1_dir = cam1_model->pixel_to_vector(image_pix);
        Vector3 cam2_dir = cam2_model->pixel_to_vector(image_pix);
        dir_diff.push_back(norm_2(cam1_dir - cam2_dir));

        if (opt.print_per_pixel_results)
          vw_out() << "Camera direction diff: " << dir_diff.back() << std::endl;

        // Shoot a ray from the cam1 camera, intersect it with the
        // given height above datum, and project it back into the cam2
        // camera.
        Vector3 xyz;
        if (!have_bathy_plane)

          xyz = vw::cartography::datum_intersection(major_axis, minor_axis,
                                                    cam1_ctr, cam1_dir);
        else
          xyz = vw::datumBathyIntersection(cam1_ctr, cam1_dir,
                                           major_axis, minor_axis,
                                           opt.bathy_plane_vec[0],
                                           opt.refraction_index);
        // Skip invalid intersections
        if (xyz == Vector3(0, 0, 0))
          continue;

        Vector2 cam2_pix;
        if (!have_bathy_plane)
          cam2_pix = cam2_model->point_to_pixel(xyz);
        else
          cam2_pix = point_to_pixel(cam2_model, datum, opt.bathy_plane_vec[0],
                                    opt.refraction_index, xyz);
        cam1_to_cam2_diff.push_back(norm_2(image_pix - cam2_pix));

        if (opt.print_per_pixel_results)
          vw_out() << "cam1 to cam2 pixel diff: " << image_pix - cam2_pix << std::endl;

        // Turn CSM on and off and see the effect on the pixel
        if (opt.aster_vs_csm) {
          asp::stereo_settings().aster_use_csm = !asp::stereo_settings().aster_use_csm;
          Vector2 cam2_pix2;
          if (!have_bathy_plane)
            cam2_pix2 = cam2_model->point_to_pixel(xyz);
          else
            cam2_pix2 = point_to_pixel(cam2_model, datum, opt.bathy_plane_vec[0],
                                       opt.refraction_index, xyz);

          asp::stereo_settings().aster_use_csm = !asp::stereo_settings().aster_use_csm;
          nocsm_vs_csm_diff.push_back(norm_2(cam2_pix - cam2_pix2));
        }

        // Shoot a ray from the cam2 camera, intersect it with the
        // given height above the datum, and project it back into the
        // cam1 camera.
        if (!have_bathy_plane)
          xyz = vw::cartography::datum_intersection(major_axis, minor_axis,
                                                    cam2_ctr, cam2_dir);
        else
          xyz = vw::datumBathyIntersection(cam2_ctr, cam2_dir,
                                           major_axis, minor_axis,
                                           opt.bathy_plane_vec[0],
                                           opt.refraction_index);
        // Skip invalid intersections
        if (xyz == Vector3(0, 0, 0))
          continue;

        Vector2 cam1_pix;
        if (!have_bathy_plane)
          cam1_pix = cam1_model->point_to_pixel(xyz);
        else
          cam1_pix = point_to_pixel(cam1_model, datum, opt.bathy_plane_vec[0],
                                    opt.refraction_index, xyz);
        cam2_to_cam1_diff.push_back(norm_2(image_pix - cam1_pix));

        if (opt.print_per_pixel_results)
          vw_out() << "cam2 to cam1 pixel diff: " << image_pix - cam1_pix << "\n\n";

        if (opt.aster_vs_csm) {
          asp::stereo_settings().aster_use_csm = !asp::stereo_settings().aster_use_csm;
          Vector2 cam1_pix2;
          if (!have_bathy_plane)
            cam1_pix2 = cam1_model->point_to_pixel(xyz);
          else
            cam1_pix2 = point_to_pixel(cam1_model, datum, opt.bathy_plane_vec[0],
                                       opt.refraction_index, xyz);
          asp::stereo_settings().aster_use_csm = !asp::stereo_settings().aster_use_csm;
          nocsm_vs_csm_diff.push_back(norm_2(cam1_pix - cam1_pix2));
        }
      } catch (...) {
        // Failure to compute these operations can occur for pinhole cameras
        num_failed++;
      }

      if (single_pix)
        break;
    }

    if (single_pix)
      break;
  }

  sw.stop();
  vw_out() << "Number of samples: " << ctr_diff.size() << "\n";
  if (num_failed > 0)
    vw_out() << "Number of failed samples (not counted above): " << num_failed << "\n";

  print_diffs("cam1 to cam2 camera direction diff norm", dir_diff);
  print_diffs("cam1 to cam2 camera center diff (meters)", ctr_diff);
  print_diffs("cam1 to cam2 pixel diff", cam1_to_cam2_diff);
  print_diffs("cam2 to cam1 pixel diff", cam2_to_cam1_diff);
  if (opt.aster_vs_csm)
    print_diffs("No-csm vs csm pixel diff", nocsm_vs_csm_diff);

  // If either session is rpc, warn that the camera center may not be accurate
  if (opt.session1 == "rpc" || opt.session2 == "rpc") {
    vw::vw_out() << "\n"; // separate from the diffs
    vw::vw_out(vw::WarningMessage)
      << "For RPC cameras, the concept of camera center is not well-defined, "
      << "so the result for that should be ignored.\n";
  }

  double elapsed_sec = sw.elapsed_seconds();
  vw_out() << "\nElapsed time per sample: " << 1e+6 * elapsed_sec/ctr_diff.size()
            << " milliseconds. (More samples will increase the accuracy.)\n";
}

int main(int argc, char *argv[]) {

  Options opt;
  try {

    handle_arguments(argc, argv, opt);
    run_cam_test(opt);

  } ASP_STANDARD_CATCHES;

  return 0;
}

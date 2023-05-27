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

// Tool to create simulated satellite images and/or pinhole cameras for them.
// See the manual for details.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Sessions/StereoSession.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Geometry/baseUtils.h>
#include <vw/Cartography/CameraBBox.h>

#if 0
// Temporary debugging code. 
// TODO(oalexan1): Remove this in due time. It is so much more convenient to debug
// here, however, compared to the code in CameraBBox.h, which requires recompilation
// of a large number of files.
#ifndef __VW_CARTOGRAPHY_CAMERABBOX2_H__
#define __VW_CARTOGRAPHY_CAMERABBOX2_H__

/// \file CameraBBox.h Contains bounding box, pixel intersection, and misc utilities.

#include <vw/config.h>
#if defined(VW_HAVE_PKG_CAMERA)

#include <vw/Image/ImageViewRef.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/Interpolation.h>
#include <vw/Math/LevenbergMarquardt.h>
#include <vw/Math/BresenhamLine.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Cartography/GeoReference.h>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

// TODO(oalexan1): Move most of this code to .cc. Use it all with
// ImageViewRef<float> for the DEM.
namespace vw { namespace cartography2 {

  using namespace vw::cartography;

  // Define an LMA model to solve for a DEM intersecting a ray. The
  // variable of optimization is position on the ray. The cost
  // function is difference between datum height and DEM height at
  // current point on the ray.
  template <class DEMImageT>
  class RayDEMIntersectionLMA2:
    public math::LeastSquaresModelBase<RayDEMIntersectionLMA2<DEMImageT>> {

    // TODO: Why does this use EdgeExtension if Helper() restricts access to the bounds?
    InterpolationView<EdgeExtensionView<DEMImageT, ConstantEdgeExtension>,
                      BilinearInterpolation> m_dem;
    GeoReference const& m_georef; // use an alias, as making a copy will likely leak memory
    Vector3      m_camera_ctr;
    Vector3      m_camera_vec;
    bool         m_treat_nodata_as_zero;

    /// Provide safe interaction with DEMs that are scalar
    /// - If m_dem(x,y) is in bounds, return the interpolated value.
    /// - Otherwise return 0 or big_val()
    template <class PixelT>
    typename boost::enable_if<IsScalar<PixelT>, double>::type
    inline Helper(double x, double y) const {
      if ((0 <= x) && (x <= m_dem.cols() - 1) && // for interpolation
           (0 <= y) && (y <= m_dem.rows() - 1)){
        PixelT val = m_dem(x, y);
        if (is_valid(val)) {
        return val;
        }
      }
      if (m_treat_nodata_as_zero)
        return 0;

      return big_val();
    }

    /// Provide safe interaction with DEMs that are compound
    template <class PixelT>
    typename boost::enable_if<IsCompound<PixelT>, double>::type
    inline Helper(double x, double y) const {
      if ((0 <= x) && (x <= m_dem.cols() - 1) && // for interpolation
           (0 <= y) && (y <= m_dem.rows() - 1)){
        PixelT val = m_dem(x, y);
        if (is_valid(val))  {
          return val[0];
        }
      }
      if (m_treat_nodata_as_zero) {
        return 0;
      }

      return big_val();
    }

  public:
    typedef Vector<double, 1> result_type;
    typedef Vector<double, 1> domain_type;
    typedef Matrix<double>    jacobian_type; ///< Jacobian form. Auto.

    /// Return a very large error to penalize locations that fall off the edge of the DEM.
    inline double big_val() const {
      // Don't make this too big as in the LMA algorithm it may get
      // squared and may cause overflow.
      return 1.0e+50;
    }

    /// Constructor
    RayDEMIntersectionLMA2(ImageViewBase<DEMImageT> const& dem_image,
                          GeoReference const& georef,
                          Vector3 const& camera_ctr,
                          Vector3 const& camera_vec,
                          bool treat_nodata_as_zero):
      m_dem(interpolate(dem_image)), // create an interpolation object
      m_georef(georef),
      m_camera_ctr(camera_ctr), m_camera_vec(camera_vec),
      m_treat_nodata_as_zero(treat_nodata_as_zero){}
    
    /// Evaluator. See description above.
    inline result_type operator()(domain_type const& len) const {
      // The proposed intersection point
      Vector3 xyz = m_camera_ctr + len[0]*m_camera_vec;

      // Convert to geodetic coordinates, then to DEM pixel coordinates
      Vector3 llh = m_georef.datum().cartesian_to_geodetic(xyz);
      Vector2 pix = m_georef.lonlat_to_pixel(Vector2(llh.x(), llh.y()));

      // Return a measure of the elevation difference between the DEM and the guess
      // at its current location.
      result_type result;
      result[0] = Helper<typename DEMImageT::pixel_type>(pix.x(), pix.y()) - llh[2];
      return result;
    }
  };

  // The first step in finding where a ray intersects the DEM. Find a position
  // on the ray where one is at least above the DEM. Do that by wiggling the
  // initial guess along the ray.
  template<class ModelT>
  void findInitPositionAboveDEM(ModelT & model,
                                Vector3 const& camera_ctr,
                                Vector3 const& xyz,
                                // outputs
                                bool & has_intersection,
                                Vector<double, 1> & len) {

    Vector<double, 1> len0;
    len0[0] = norm_2(xyz - camera_ctr);

    // Initialize the outputs
    len = len0;
    has_intersection = false;

    // If the ray intersects the datum at a point which does not
    // correspond to a valid location in the DEM, wiggle that point
    // along the ray until hopefully it does.
    const double radius     = norm_2(xyz); // Radius from XZY coordinate center
    const int    ITER_LIMIT = 10; // There are two solver attempts per iteration
    const double small      = radius*0.02/(1 << ITER_LIMIT); // Wiggle
    for (int i = 0; i <= ITER_LIMIT; i++) {

      // Gradually expand delta magnitude until on final iteration it is == radius*0.02.
      // We flip between positive and negative values of ever-increasing magnitude.
      double delta = small*(1 << i);
      if (i == 0)
        delta = 0.0; // In first try, start at the initial guess

      for (int k = -1; k <= 1; k += 2) { // For k==-1, k==1
        // Use below -k because we want len to increase first time
        len[0] = len0[0] - k*delta; // Ray guess length +/- 2% planetary radius
        //  std::cout << "len is " << len[0] << "\n";

        // Use our model to compute the height diff at this length
        Vector<double, 1> height_diff = model(len);
        // TODO: This is a very lenient threshold. big_val()/10.0 == 1.0e+49.
        // The effect of this is to stop this loop when we get over valid DEM terrain.
        if (std::abs(height_diff[0]) < (model.big_val()/10.0)) {
          has_intersection = true;
          break;
        }
      } // End k loop

      if (has_intersection) {
        break;
      }
    } // End i loop
  } // End function

  // A very customized secant method function, for this specific application.
  // Make several attempts to improve robustness. Return the solved length along
  // the ray (which is also passed in as an initial guess) and the
  // has_intersection flag.
  template <class ModelT>
  void secantMethod(ModelT & model, Vector3 const& camera_ctr, 
    Vector3 const& camera_vec, double height_error_tol,
    // outputs
    bool & has_intersection,  Vector<double, 1> & len) {
    
    // Initialize the output
    has_intersection = false;

    // Do several attempts with different starting values for x1. The value of j
    // will control the step size.
    int num_j = 100; // will use this variable in two places below
    Vector<double, 1> len1, len2; 

    for (int j = 0; j <= num_j; j++) {

      double x0 = len[0];
      double f0 = model(len)[0];
    
      if (std::abs(f0) < height_error_tol) {
        // As a robustness check, return early if we are already close enough.
        // Otherwise may end up with a denominator close to 0 below.
        len[0] = x0;
        has_intersection = true;
        return; // Done
      }

      double x1 = len[0] + 10.0 * (j + 1); 
      len1[0] = x1;
      double f1 = model(len1)[0];

      if (std::abs(f0 - f1) < 1e-6 && j < num_j)
        continue; // Try next j, as the f values are too close

      // Do 100 iterations of secant method
      for (int i = 0; i < 100; i++) {
        double x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        len2[0] = x2;
        double f2 = model(len2)[0];
        if (std::abs(f2) < height_error_tol) {
          x0 = x2;
          f0 = f2;
          break;
        }
        x0 = x1; f0 = f1;
        x1 = x2; f1 = f2;
      }

      if (std::abs(f0) < height_error_tol) {
        len[0] = x0;
        has_intersection = true;
        return; // Done
      } else {
        has_intersection = false;
        // Try again
      }
    }

    return;
  }

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
                                  Vector3 xyz_guess       = Vector3(),
                                  double height_guess     = 0.0) {
    
    // This is a very fragile function and things can easily go wrong. 
    try {
      has_intersection = false;
      RayDEMIntersectionLMA2<DEMImageT> model(dem_image, georef, camera_ctr,
                                             camera_vec, treat_nodata_as_zero);

      Vector3 xyz;
      if (xyz_guess == Vector3()){ // If no guess provided
        // Intersect the ray with the datum, this is a good initial guess.
        xyz = datum_intersection(georef.datum().semi_major_axis() + height_guess,
                                 georef.datum().semi_minor_axis() + height_guess,
                                 camera_ctr, camera_vec);

        if (xyz == Vector3()) { // If we failed to intersect the datum, give up.
          has_intersection = false;
          return Vector3();
        }
      } else { // User provided guess
        xyz = xyz_guess;
      }

      // Now wiggle xyz along the ray until it is somewhere above the DEM.
      // Will return not xyz, but the 'len' along the ray for it.
      Vector<double, 1> len;
      findInitPositionAboveDEM(model, camera_ctr, xyz, 
        // outputs
        has_intersection, len);

      // Call the secant method function to find the intersection with the
      // ground. Return has_intersection and len. This is 10x faster and more
      // robust than the Levenberg-Marquardt method used below (which used to be
      // the original method).
      Vector<double, 1> len_secant = len; // will change
      secantMethod(model, camera_ctr, camera_vec, height_error_tol,
                   has_intersection, len_secant);
      if (has_intersection) {
        xyz = camera_ctr +  len_secant[0] * camera_vec;
        return xyz;
      }

      // If no luck, fallback to using Levenberg-Marquardt, with the original
      // value of len.
      int status = 0;
      Vector<double, 1> observation; observation[0] = 0;
      len = math::levenberg_marquardt(model, len, observation, status,
        max_abs_tol, max_rel_tol, num_max_iter);
      Vector<double, 1> dem_height = model(len);
      //std::cout << "dem_height is " << dem_height[0] << "\n";

#if 0
      // We don't really care of the status of the minimization algorithm, since
      // we use it as a root solver. The good test is whether the height difference
      // is small enough. This test can fail even if we are close enough to the root.
      if (status < 0) {
        has_intersection = false;
        return Vector3();
      }
#endif

      if (std::abs(dem_height[0]) <= height_error_tol) {
          has_intersection = true;
          xyz = camera_ctr + len[0]*camera_vec;
          return xyz;
      } else {
        // Failed
        has_intersection = false;
        return Vector3();
      }
    }catch(...){
      has_intersection = false;
    }
    return Vector3();
  }
 
} // namespace cartography
} // namespace vw

#endif // VW_HAVE_PKG_CAMERA

#endif // __VW_CARTOGRAPHY_CAMERABBOX_H__
#endif

using namespace vw::cartography;
using namespace vw::math;
using namespace vw::geometry;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::GdalWriteOptions {
  std::string dem_file, ortho_file, out_prefix, camera_list;
  vw::Vector3 first, last; // dem pixel and height above dem datum
  int num_cameras;
  vw::Vector2 optical_center, image_size, first_ground_pos, last_ground_pos;
  double focal_length, dem_height_error_tol;
  double roll, pitch, yaw;
  bool no_images;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  double NaN = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("dem", po::value(&opt.dem_file)->default_value(""), "Input DEM file.")
    ("ortho", po::value(&opt.ortho_file)->default_value(""), "Input georeferenced image file.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix. All the "
    "files that are saved will start with this prefix.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of pinhole cameras to create synthetic images for. "
     "Then these cameras will be used instead of generating them. Specify one file "
     "per line. The options --first, --last, --num, --focal-length, "
     "and --optical-center will be ignored.")
    ("first", po::value(&opt.first)->default_value(vw::Vector3(), ""),
    "First camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("last", po::value(&opt.last)->default_value(vw::Vector3(), ""),
    "Last camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("num", po::value(&opt.num_cameras)->default_value(0),
    "Number of cameras to generate, including the first and last ones. Must be positive. "
    "The cameras are uniformly distributed along the straight edge from first to last (in "
    "projected coordinates).")
    ("first-ground-pos", po::value(&opt.first_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of first camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("last-ground-pos", po::value(&opt.last_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of last camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("focal-length", po::value(&opt.focal_length)->default_value(NaN),
     "Output camera focal length in units of pixel.")
    ("optical-center", po::value(&opt.optical_center)->default_value(vw::Vector2(NaN, NaN),"NaN NaN"),
     "Output camera optical center (image column and row).")
    ("image-size", po::value(&opt.image_size)->default_value(vw::Vector2(NaN, NaN),
      "NaN NaN"),
      "Output camera image size (width and height).")
    ("roll", po::value(&opt.roll)->default_value(NaN),
    "Camera roll angle, in degrees. See the documentation for more details.")
    ("pitch", po::value(&opt.pitch)->default_value(NaN),
    "Camera pitch angle, in degrees.")
    ("yaw", po::value(&opt.yaw)->default_value(NaN),
    "Camera yaw angle, in degrees.")
    ("no-images", po::bool_switch(&opt.no_images)->default_value(false)->implicit_value(true),
     "Create only cameras, and no images. Cannot be used with --camera-list.")
    ("dem-height-error-tol", po::value(&opt.dem_height_error_tol)->default_value(0.001),
     "When intersecting a ray with a DEM, use this as the height error tolerance "
     "(measured in meters). It is expected that the default will be always good enough.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--dem <dem file> --ortho <ortho image file> "
                    "[other options]");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  if (opt.dem_file == "" || opt.ortho_file == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing input DEM and/or ortho image.\n");
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing output prefix.\n");
  if (std::isnan(opt.image_size[0]) || std::isnan(opt.image_size[1]))
    vw::vw_throw(vw::ArgumentErr() << "The image size must be specified.\n");

  if (opt.camera_list != "" && opt.no_images)
    vw::vw_throw(vw::ArgumentErr() << "The --camera-list and --no-images options "
      "cannot be used together.\n");

  if (opt.camera_list == "") {
    if (opt.first == vw::Vector3() || opt.last == vw::Vector3())
      vw::vw_throw(vw::ArgumentErr() << "The first and last camera positions must be "
        "specified.\n");

    if (opt.num_cameras < 2)
      vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 2.\n");

    // Validate focal length, optical center, and image size
    if (std::isnan(opt.focal_length))
      vw::vw_throw(vw::ArgumentErr() << "The focal length must be positive.\n");
    if (std::isnan(opt.optical_center[0]) || std::isnan(opt.optical_center[1]))
      vw::vw_throw(vw::ArgumentErr() << "The optical center must be specified.\n");

    // Either both first and last ground positions are specified, or none.
    if (std::isnan(norm_2(opt.first_ground_pos)) != 
      std::isnan(norm_2(opt.last_ground_pos)))
      vw::vw_throw(vw::ArgumentErr() << "Either both first and last ground positions "
        "must be specified, or none.\n");

    // Check that either all of roll, pitch, and yaw are specified, or none.
    int ans = int(std::isnan(opt.roll)) +
              int(std::isnan(opt.pitch)) +
              int(std::isnan(opt.yaw));
    if (ans != 0 && ans != 3)
      vw::vw_throw(vw::ArgumentErr() << "Either all of roll, pitch, and yaw must be "
        "specified, or none.\n");
  }

  // Create the output directory based on the output prefix
  vw::create_out_dir(opt.out_prefix);

  return;
}

// A function that will read a geo-referenced image, its nodata value,
// and the georeference, and will return a PixelMasked image, the nodata
// value, and the georeference.
// TODO(oalexan1): May need to move this to a more general place.
void readGeorefImage(std::string const& image_file, 
  float & nodata_val, vw::cartography::GeoReference & georef,
  vw::ImageViewRef<vw::PixelMask<float>> & masked_image) {

  // Initial value, in case the image has no nodata field
  nodata_val = std::numeric_limits<float>::quiet_NaN();
  if (!vw::read_nodata_val(image_file, nodata_val))
        vw::vw_out() << "Warning: Could not read the nodata value for: "
                      << image_file << "\nUsing: " << nodata_val << ".\n";

    // Read the image
    vw::vw_out() << "Reading: " << image_file << std::endl;
    vw::DiskImageView<float> image(image_file);
    // Create the masked image
    masked_image = vw::create_mask(image, nodata_val);

    // Read the georeference, and throw an exception if it is missing
    bool has_georef = vw::cartography::read_georeference(georef, image_file);
    if (!has_georef)
      vw::vw_throw(vw::ArgumentErr() << "Missing georeference in: "
                                     << image_file << ".\n");
}

// A function to convert from projected coordinates to ECEF
vw::Vector3 projToEcef(vw::cartography::GeoReference const& georef,
                vw::Vector3                          const& proj) {
  vw::Vector3 llh = georef.point_to_geodetic(proj);
  vw::Vector3 ecef = georef.datum().geodetic_to_cartesian(llh);
  return ecef;
}

// A function that will take as input the endpoints and will compute the
// satellite trajectory and along track/across track/down directions in ECEF,
// which will give the camera to world rotation matrix.
// The key observation is that the trajectory will be a straight edge in
// projected coordinates so will be computed there first.
void calcTrajectory(Options const& opt, 
                    vw::cartography::GeoReference const& dem_georef,
                    vw::ImageViewRef<vw::PixelMask<float>> dem,
                    // Outputs
                    std::vector<vw::Vector3> & trajectory,
                    // the vector of camera to world
                    // rotation matrices
                    std::vector<vw::Matrix3x3> & cam2world) {

  // Convert the first and last camera center positions to projected coordinates
  vw::Vector3 first_proj, last_proj;
  subvector(first_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.first, 0, 2)); // x and y
  first_proj[2] = opt.first[2]; // z
  subvector(last_proj, 0, 2) = dem_georef.pixel_to_point
      (vw::math::subvector(opt.last,  0, 2)); // x and y
  last_proj[2] = opt.last[2]; // z

  // Validate one more time that we have at least two cameras
  if (opt.num_cameras < 2)
    vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 2.\n");

  // Create interpolated DEM with bilinear interpolation with invalid pixel 
  // edge extension
  vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
  nodata_mask.invalidate();
  auto interp_dem = vw::interpolate(dem, vw::BilinearInterpolation(),
    vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));

  // Direction along the edge in proj coords (along track direction)
  vw::Vector3 proj_along = last_proj - first_proj;
  // Sanity check
  if (proj_along == vw::Vector3())
    vw::vw_throw(vw::ArgumentErr()
      << "The first and last camera positions are the same.\n");

  // Normalize
  proj_along = proj_along / norm_2(proj_along);

  // One more sanity check
  if (std::max(std::abs(proj_along[0]), std::abs(proj_along[1])) < 1e-6)
    vw::vw_throw(vw::ArgumentErr()
      << "It appears that the satellite is aiming for the ground. "
      <<  "Correct the orbit end points.\n");

  // Find the across-track direction, parallel to the ground, in projected coords
  vw::Vector3 proj_across = vw::math::cross_prod(proj_along, vw::Vector3(0, 0, 1));
  proj_across = proj_across / norm_2(proj_across);

  bool have_ground_pos = !std::isnan(norm_2(opt.first_ground_pos)) &&  
      !std::isnan(norm_2(opt.last_ground_pos));

  // Find the trajectory, as well as points in the along track and across track 
  // directions in the projected space with a spacing of 0.1 m. Do not use
  // a small spacing as in ECEF these will be large numbers and we may
  // have precision issues.
  double delta = 0.1; 
  std::vector<vw::Vector3> along_track(opt.num_cameras), across_track(opt.num_cameras);
  trajectory.resize(opt.num_cameras);
  cam2world.resize(opt.num_cameras);
  for (int i = 0; i < opt.num_cameras; i++) {
    double t = double(i) / double(opt.num_cameras - 1);
    vw::Vector3 P = first_proj * (1.0 - t) + last_proj * t; // traj
    vw::Vector3 L = P + delta * proj_along; // along track point
    vw::Vector3 C = P + delta * proj_across; // across track point

    // Convert to cartesian
    P = projToEcef(dem_georef, P);
    L = projToEcef(dem_georef, L);
    C = projToEcef(dem_georef, C);

    // Create the along track and across track vectors
    vw::Vector3 along = L - P;
    vw::Vector3 across = C - P;

    if (have_ground_pos) {
      // The camera won't look straight down, but constrained by ground positions
      vw::Vector2 ground_pix = opt.first_ground_pos * (1.0 - t) + opt.last_ground_pos * t;

      // Find the projected position along the ground path
      vw::Vector3 ground_proj_pos;
      subvector(ground_proj_pos, 0, 2) = dem_georef.pixel_to_point(ground_pix); // x and y
      auto val = interp_dem(ground_pix[0], ground_pix[1]);
      if (!is_valid(val))
        vw::vw_throw(vw::ArgumentErr() 
          << "Could not interpolate into the DEM along the ground path.\n");
      ground_proj_pos[2] = val.child(); // z

      // Convert the ground point to ECEF
      vw::Vector3 G = projToEcef(dem_georef, ground_proj_pos);

      // Find the ground direction
      vw::Vector3 ground_dir = G - P;
      if (norm_2(ground_dir) < 1e-6)
        vw::vw_throw(vw::ArgumentErr()
          << "The ground position is too close to the camera.\n");

      // Normalize      
      along = along / norm_2(along);
      ground_dir = ground_dir / norm_2(ground_dir);

      // Adjust the along-track direction to make it perpendicular to ground dir
      along = along - dot_prod(ground_dir, along) * ground_dir;

      // Find 'across' as y direction, given that 'along' is x, and 'ground_dir' is z
      across = -vw::math::cross_prod(along, ground_dir);
    }

    // Normalize
    along = along / norm_2(along);
    across = across / norm_2(across);
    // Ensure that across is perpendicular to along
    across = across - dot_prod(along, across) * along;
    // Normalize again
    across = across / norm_2(across);

    // Find the z vector as perpendicular to both along and across
    vw::Vector3 down = vw::math::cross_prod(along, across);
    down = down / norm_2(down);

    // Trajectory
    trajectory[i] = P;

    // The camera to world rotation has these vectors as the columns
    for (int row = 0; row < 3; row++) {
      cam2world[i](row, 0) = along[row];
      cam2world[i](row, 1) = across[row];
      cam2world[i](row, 2) = down[row];
    }

    // if to apply a roll, pitch, yaw rotation
    if (!std::isnan(opt.roll) && !std::isnan(opt.pitch) && !std::isnan(opt.yaw)) {
      vw::Matrix3x3 R = asp::rollPitchYaw(opt.roll, opt.pitch, opt.yaw);
      cam2world[i] = cam2world[i] * R;
    }

#if 0
    // Useful for debugging purposes
    // Find the lon-lat from cartesian
    vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(P);
    // Find NED matrix
    vw::Matrix3x3 ned = dem_georef.datum().lonlat_to_ned_matrix(subvector(llh, 0, 2));
    //std::cout << "ned: " << ned << std::endl;
#endif

  }
  return;
}

// Generate a prefix that will be used for image names and camera names
std::string genPrefix(Options const& opt, int i) {
  return opt.out_prefix + "-" + num2str(10000 + i);
}

// A function to read the pinhole cameras from disk
void readCameras(Options const& opt, 
    std::vector<std::string> & cam_names,
    std::vector<vw::camera::PinholeModel> & cams) {

  // Read the camera names
  vw::vw_out() << "Reading: " << opt.camera_list << std::endl;
  asp::read_list(opt.camera_list, cam_names);

  // Sanity check
  if (cam_names.empty())
    vw::vw_throw(vw::ArgumentErr() << "No cameras were found.\n");

  cams.resize(cam_names.size());
  for (int i = 0; i < int(cam_names.size()); i++)
    cams[i].read(cam_names[i]);
  
  return;
}

// A function to create and save the cameras. Assume no distortion, and pixel
// pitch = 1.
void genCameras(Options const& opt, std::vector<vw::Vector3> const & trajectory,
                  std::vector<vw::Matrix3x3> const & cam2world,
                  // outputs
                  std::vector<std::string> & cam_names,
                  std::vector<vw::camera::PinholeModel> & cams) {

  // Ensure we have as many camera positions as we have camera orientations
  if (trajectory.size() != cam2world.size())
    vw::vw_throw(vw::ArgumentErr()
      << "Expecting as many camera positions as camera orientations.\n");

    cams.resize(trajectory.size());
    cam_names.resize(trajectory.size());
    for (int i = 0; i < int(trajectory.size()); i++) {

      cams[i] = vw::camera::PinholeModel(trajectory[i], cam2world[i],
                                   opt.focal_length, opt.focal_length,
                                   opt.image_size[0], opt.image_size[1]);
      
      std::string camName = genPrefix(opt, i) + ".tsai";
      cam_names[i] = camName;
      vw::vw_out() << "Writing: " << camName << std::endl;
      cams[i].write(camName);
    }
  
  return;
}

// Generate images by projecting rays from the sensor to the ground
void genImages(Options const& opt,
    bool external_cameras,
    std::vector<std::string> const& cam_names,
    std::vector<vw::camera::PinholeModel> const& cams,
    vw::cartography::GeoReference const& dem_georef,
    vw::ImageViewRef<vw::PixelMask<float>> dem,
    vw::cartography::GeoReference const& ortho_georef,
    vw::ImageViewRef<vw::PixelMask<float>> ortho,
    float ortho_nodata_val) {

    // Generate image names from camera names by replacing the extension
    std::vector<std::string> image_names;
    image_names.resize(cam_names.size());
    for (int i = 0; i < int(cam_names.size()); i++) {
      if (external_cameras)
       image_names[i] = opt.out_prefix + "-" 
        + fs::path(cam_names[i]).filename().replace_extension(".tif").string();
      else
        image_names[i] = genPrefix(opt, i) + ".tif";

      vw::vw_out() << "Writing: " << image_names[i] << std::endl;
    }

  // Create interpolated image with bicubic interpolation with invalid pixel 
  // edge extension
  vw::PixelMask<float> nodata_mask = vw::PixelMask<float>(); // invalid value
  nodata_mask.invalidate();
  auto interp_ortho = vw::interpolate(ortho, vw::BicubicInterpolation(),
                                      vw::ValueEdgeExtension<vw::PixelMask<float>>(nodata_mask));
  int cols = opt.image_size[0];
  int rows = opt.image_size[1];
  vw::vw_out() << "Generating images.\n";

  // The location where the ray intersects the ground. We will use each obtained
  // location as initial guess for the next ray. This may not be always a great
  // guess, but it is better than starting nowhere. It should work decently
  // if the camera is high, and with a small footprint on the ground.
  vw::Vector3 xyz(0, 0, 0);

  for (size_t i = 0; i < cams.size(); i++) {
    vw::ImageView<vw::PixelMask<float>> image(cols, rows);

    vw::TerminalProgressCallback tpc("", image_names[i] + ": ");
    tpc.report_progress(0);
    double inc_amount = 1.0 / double(cols);

    for (int col = 0; col < cols; col++) {
      for (int row = 0; row < rows; row++) {

        // Start with an invalid pixel
        image(col, row) = vw::PixelMask<float>();
        image(col, row).invalidate();
        vw::Vector2 pix(col, row);

        vw::Vector3 cam_ctr = cams[i].camera_center(pix);
        vw::Vector3 cam_dir = cams[i].pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM.
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double max_abs_tol = std::min(opt.dem_height_error_tol, 1e-14);
        double max_rel_tol = max_abs_tol;
        int num_max_iter = 100;

        // Find xyz, and use it for the next guess
        xyz = vw::cartography::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, dem,
           dem_georef, treat_nodata_as_zero,
           has_intersection, opt.dem_height_error_tol, 
           max_abs_tol, max_rel_tol, 
           num_max_iter, xyz);

        //if (!has_intersection) {
        //  //Temporary debug code
        //  std::cout << "Failed for pixel " << pix << "\n";
        //  exit(0);
        //}

        if (!has_intersection) 
          continue;

        // Find the texture value at the intersection point by interpolation.
        // This will result in an invalid value if if out of range or if the
        // image itself has invalid pixels.
        vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
        vw::Vector2 ortho_pix = ortho_georef.lonlat_to_pixel(vw::Vector2(llh[0], llh[1]));
        image(col, row) = interp_ortho(ortho_pix[0], ortho_pix[1]);
      }

      tpc.report_incremental_progress(inc_amount);
    } 
    tpc.report_finished();

    // Save the image using the block write function
    vw::vw_out() << "Writing: " << image_names[i] << std::endl;
    bool has_georef = false; // the produced image is raw, it has no georef
    bool has_nodata = true;

    block_write_gdal_image(image_names[i], 
      vw::apply_mask(image, ortho_nodata_val), 
      has_georef, ortho_georef, // the ortho georef will not be used
      has_nodata, ortho_nodata_val, // borrow the nodata from ortho
      opt, vw::TerminalProgressCallback("", "\t--> "));
  }  
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Read the DEM
    vw::ImageViewRef<vw::PixelMask<float>> dem;
    float dem_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference dem_georef;
    readGeorefImage(opt.dem_file, dem_nodata_val, dem_georef, dem);

    // Read the ortho image
    vw::ImageViewRef<vw::PixelMask<float>> ortho;
    float ortho_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference ortho_georef;
    readGeorefImage(opt.ortho_file, ortho_nodata_val, ortho_georef, ortho);

    std::vector<std::string> cam_names;
    std::vector<vw::camera::PinholeModel> cams;
    bool external_cameras = false;
    if (!opt.camera_list.empty()) {
      // Read the cameras
      readCameras(opt, cam_names, cams);
      external_cameras = true;
    } else {
     // Generate the cameras   
      std::vector<vw::Vector3> trajectory(opt.num_cameras);
      // vector of rot matrices
      std::vector<vw::Matrix3x3> cam2world(opt.num_cameras);
      calcTrajectory(opt, dem_georef, dem,
        trajectory, cam2world);
      // Generate cameras
      genCameras(opt, trajectory, cam2world, cam_names, cams);
    }

    // Generate images
    if (!opt.no_images)
      genImages(opt, external_cameras, cam_names, cams, dem_georef, dem, 
        ortho_georef, ortho, ortho_nodata_val);

  } ASP_STANDARD_CATCHES;

  return 0;
}

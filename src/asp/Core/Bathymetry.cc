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

#include <asp/Core/Bathymetry.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Core/Exception.h>
#include <vw/Math/LevenbergMarquardt.h>

// Turn this on to verify if Snell's law holds given the incoming and
// outgoing rays to a plane. Then stereo_tri needs to be run with one
// thread as this will print a lot of text.
#define DEBUG_BATHY 0

namespace asp {

  using namespace vw;
  using namespace vw::stereo;

  void read_bathy_plane(std::string const& bathy_plane_file,
                        std::vector<double> & bathy_plane,
                        bool & use_curved_water_surface,
                        vw::cartography::GeoReference & water_surface_projection) {

    vw_out() << "Reading the water surface from: " << bathy_plane_file << "\n";
    
    std::ifstream handle;
    handle.open(bathy_plane_file.c_str());
    if (handle.fail()) 
      vw_throw(vw::IOErr() << "Unable to open bathy plane file: " << bathy_plane_file << "\n");
    
    std::vector<std::string> lines;
    std::string line;
    while ( getline(handle, line, '\n') ){
      lines.push_back(line);
    }

    if (lines.empty())
      vw_throw(vw::IOErr() << "Invalid bathy plane file: " << bathy_plane_file << "\n");
      
    // Read the four values
    bathy_plane.resize(4);
    {
      std::istringstream iss(lines[0]);
      if (!(iss >> bathy_plane[0] >> bathy_plane[1] >> bathy_plane[2] >> bathy_plane[3])) 
        vw_throw(vw::IOErr() << "Could not read four values from first "
                 << "line of the bathy plane.\n");
    }
    
    // The second line must start with a comment for the plane to be a curved surface
    if (lines.size() <= 1 || lines[1][0] != '#')
      use_curved_water_surface = false;
    else 
     use_curved_water_surface = true;
    
    if (!use_curved_water_surface && bathy_plane[3] >= 0) 
      vw_throw(vw::IOErr() << "For a planar water surface, the fourth value "
               << "must be negative.\n");

    if (!use_curved_water_surface)
      return; // nothing more to do in this case
    
    if (bathy_plane[2] <= 0) 
      vw_throw(vw::IOErr() << "For a curved water surface, the third value "
               << "must be positive.\n");

    double scale = 1.0;
    double proj_lat, proj_lon;
    {
      if (lines.size() == 2)
        vw_throw(vw::IOErr() << "Invalid bathy plane file: " << bathy_plane_file << "\n");

      std::istringstream iss(lines[2]);
      if (!(iss >> proj_lat >> proj_lon)) 
        vw_throw(vw::IOErr() << "Could not read the projection latitude and longitude.\n");
    }
    vw::cartography::Datum datum("WGS_1984");
    water_surface_projection.set_datum(datum);
    water_surface_projection.set_stereographic(proj_lat, proj_lon, scale);
    vw_out() << "Read projection: " <<  water_surface_projection.overall_proj4_str()
             << std::endl;
  }

  // Read left and right bathy plane settings and associated data.
  // More often than not they will be identical.
  void read_bathy_plane_set(std::string const& bathy_plane_files,
                            std::vector<BathyPlaneSettings> & bathy_plane_set) {

    bathy_plane_set.clear();
    
    std::string bathy_plane_file;
    std::istringstream iss(bathy_plane_files);
    while (iss >> bathy_plane_file) {

      bathy_plane_set.push_back(BathyPlaneSettings());
      read_bathy_plane(bathy_plane_file,
                       bathy_plane_set.back().bathy_plane,  
                       bathy_plane_set.back().use_curved_water_surface,  
                       bathy_plane_set.back().water_surface_projection);
    }

    if (bathy_plane_set.size() != 1 && bathy_plane_set.size() != 2) 
      vw_throw(vw::ArgumentErr() << "One or two bathy planes expected.\n");

    // Clone the bathy plane if there's only one
    if (bathy_plane_set.size() == 1)
      bathy_plane_set.push_back(bathy_plane_set[0]);
  }
    
  // Given a plane as four values a, b, c, d, with the plane being a * x + b * y + c * z + d = 0,
  // find how far off a point (x, y, z) is from the plane by evaluating the above expression.
  double signed_dist_to_plane(std::vector<double> const& plane, vw::Vector3 const& point) {

    double ans = 0.0;
    for (unsigned coord_it = 0; coord_it < 3; coord_it++) {
      ans += plane[coord_it] * point[coord_it];
    }
    ans += plane[3];
    
    return ans;
  }

  // Compute the projected coordinates of an ECEF point
  inline Vector3 proj_point(vw::cartography::GeoReference const& projection,
                            Vector3 const& xyz) {
    return projection.geodetic_to_point(projection.datum().cartesian_to_geodetic(xyz));
  }

  // Reverse this operation
  inline Vector3 unproj_point(vw::cartography::GeoReference const& projection,
                              Vector3 const& proj_pt) {
    return projection.datum().geodetic_to_cartesian(projection.point_to_geodetic(proj_pt));
  }

  // Given a ECEF point xyz, and two planes, find if xyz is above or below each of the
  // plane by finding the signed distances to them.
  void signed_distances_to_planes(bool use_curved_water_surface,
                                  std::vector<BathyPlaneSettings> const& bathy_set,
                                  vw::Vector3 const& xyz,
                                  std::vector<double> & distances) {
    
    if (bathy_set.size() != 2) 
      vw_throw(vw::ArgumentErr() << "Two bathy planes expected.\n");
    
    distances.resize(2);
    for (size_t it = 0; it < 2; it++) {
      // For a curved water surface need to first convert xyz to projected coordinates
      if (use_curved_water_surface)
        distances[it] = signed_dist_to_plane(bathy_set[it].bathy_plane,
                                             proj_point(bathy_set[it].water_surface_projection,
                                                        xyz));
      else
        distances[it] = signed_dist_to_plane(bathy_set[it].bathy_plane, xyz);
    }
  }

  // Test Snell's law in projected and unprojected coordinates
  void test_snells_law(std::vector<double> const& plane,
                       vw::cartography::GeoReference const& water_surface_projection,
                       double refraction_index,
                       vw::Vector3 const& out_unproj_xyz,
                       vw::Vector3 const& in_unproj_dir, vw::Vector3 const& out_unproj_dir,
                       vw::Vector3 const& out_proj_xyz, 
                       vw::Vector3 const& in_proj_dir, vw::Vector3 const& out_proj_dir) {
  
    // Verify Snell's law.
    
    // 1. In projected coordinates
    Vector3 proj_normal(plane[0], plane[1], plane[2]);
    double sin_in = sin(acos(dot_prod(proj_normal, -in_proj_dir)));
    double sin_out = sin(acos(dot_prod(-proj_normal, out_proj_dir)));
    std::cout << "proj sin_in, sin_out, sin_in - index * sin_out "
              << sin_in << ' ' << sin_out << ' '
              << sin_in - refraction_index * sin_out << std::endl;
  
    // 2. In unprojected coordinates
    Vector3 proj_xyz_above_normal = out_proj_xyz + 1.0 * proj_normal; // go 1 m along the normal
    Vector3 unproj_xyz_above_normal = unproj_point(water_surface_projection,
                                                   proj_xyz_above_normal);
    Vector3 unproj_normal = unproj_xyz_above_normal - out_unproj_xyz;
    unproj_normal /= norm_2(unproj_normal); // normalize
    sin_in = sin(acos(dot_prod(unproj_normal, -in_unproj_dir)));
    sin_out = sin(acos(dot_prod(-unproj_normal, out_unproj_dir)));
    std::cout << "unproj sin_in, sin_out, sin_in - index * sin_out "
              << sin_in << ' ' << sin_out << ' '
              << sin_in - refraction_index * sin_out << std::endl;
  
    // Verify that the incoming ray, outgoing ray, and the
    // normal are in the same plane in projected coordinates
  
    // 1. In projected coordinates
    Vector3 in_out_normal = vw::math::cross_prod(in_proj_dir, out_proj_dir);
    double plane_error = dot_prod(in_out_normal, proj_normal);
    std::cout << "proj plane error " << plane_error << std::endl;
  
    // 2. In unprojected coordinates
    in_out_normal = vw::math::cross_prod(in_unproj_dir, out_unproj_dir);
    plane_error = dot_prod(in_out_normal, unproj_normal);
    std::cout << "unproj plane error " << plane_error << std::endl;
  
    std::cout << std::endl;
  }

  // See the .h file for more info
  bool snells_law(Vector3 const& in_xyz, Vector3 const& in_dir,
                  std::vector<double> const& plane,
                  double refraction_index, 
                  Vector3 & out_xyz, Vector3 & out_dir) {

    // The ray is given as in_xyz + alpha * in_dir, where alpha is real.
    // See where it intersects the plane.
    double cn = 0.0, dn = 0.0; // Dot product of in_xyz and in_dir with plane normal n
    for (size_t it = 0; it < 3; it++) {
      cn += plane[it] * in_xyz[it];
      dn += plane[it] * in_dir[it];
    }

    // The ray must descend to the plane, or else something is not right
    if (dn >= 0.0)
      return false;
    
    double alpha = -(plane[3] + cn)/dn;
  
    // The intersection with the plane
    out_xyz = in_xyz + alpha * in_dir;

    // Let n be the plane normal pointing up (the first three components
    // of the plane vector). Let out_dir be the outgoing vector after the
    // ray hits the water, according to Snell's law, with in_dir being the
    // incoming ray. Let a1 be the angles between -in_dir and n, a2 be the
    // angle between out_dir and -n.
  
    // Then sin(a1) = refraction_index * sin(a2) per Snell's law.
    // Square this. Note that cos^2 (x) + sin^2 (x) = 1.
    // So, 1 - cos(a1)^2 = refraction_index^2 * (1 - cos(a2)^2).
    // But cos(a1) = dot_product(-in_dir, n) = -dn.
    // So, cos(a2)^2 = 1 - (1 - dn^2)/refraction_index^2
    // Call the left-hand value cos_sq.

    double cos_sq = 1.0 - (1.0 - dn * dn)/refraction_index/refraction_index;
  
    // The outgoing vector out_dir will be a linear combination of -n and d1,
    // normalized to unit length. Let alpha > 0 be the value which will
    // produce the linear combination.  So,
    // out_dir = (-n + alpha * in_dir)/norm(-n + alpha * in_dir)
    // But dot(out_dir, -n) = cos(a2). Hence, if we dot the above with n and square it,
    // we get 
    // cos(a2)^2 = (-1 + alpha * dn)^2 / dot( -n + alpha * in_dir, -n + alpha * in_dir)
    // or 
    // cos(a2)^2 * dot( -n + alpha * in_dir, -n + alpha * in_dir) = (-1 + alpha * dn)^2  
    // or
    // cos_sq * (1 - 2 * alpha * dn + alpha^2) = ( 1 - 2*alpha * dn + alpha^2 * dn^2)
    //
    // Note that we computed cos_sq from Snell's law above.
  
    // Move everything to the left and find the coefficients of the
    // quadratic equation in alpha, so u * alpha^2 + v * alpha + w = 0.
    double u = cos_sq - dn * dn;  // this is cos(a2)^2 - cos(a1)^2 > 0 as a2 < a1
    double v = -2 * dn * cos_sq + 2.0 * dn;
    double w = cos_sq - 1.0;
    double delta = v * v - 4 * u * w; // discriminant
    if (u <= 0.0 || delta < 0.0) 
      return false; // must not happen
  
    alpha = (-v + sqrt(delta)) / (2.0 * u); // pick the positive quadratic root

    if (alpha < 0) 
      return false; // must not happen
    
    // The normalized direction after the ray is bent
    out_dir = -Vector3(plane[0], plane[1], plane[2]) + alpha * in_dir;
    out_dir = out_dir / norm_2(out_dir);

    return true;
  }  

  // Consider a stereographic projection and a plane
  // a * x + b * y + c * z + d = 0 for (x, y, z) in this projection.
  // Intersect it with a ray given in ECEF coordinates.
  // If the values a and b are 0, that is the same as intersecting
  // the ray with the spheroid of values -d/c above the datum.
  // This solver was not used as it was too slow. An approximate
  // solution was instead found.
  class SolveCurvedPlaneIntersection:
    public vw::math::LeastSquaresModelBase<SolveCurvedPlaneIntersection> {
    vw::Vector3 const& m_ray_pt;
    vw::Vector3 const& m_ray_dir;
    vw::cartography::GeoReference const& m_projection;
    std::vector<double> const& m_proj_plane;
  public:

    // This is a one-parameter problem, yet have to use a vector (of size 1)
    // as required by the API.
    typedef vw::Vector<double, 1> result_type;   // residual
    typedef vw::Vector<double, 1> domain_type;   // parameter giving the position on the ray
    typedef vw::Matrix<double>    jacobian_type;

    /// Instantiate the solver with a set of xyz to pixel pairs and a pinhole model
    SolveCurvedPlaneIntersection(vw::Vector3 const& ray_pt, vw::Vector3 const& ray_dir,
                                 vw::cartography::GeoReference const& projection,
                                 std::vector<double> const& proj_plane):
      m_ray_pt(ray_pt), m_ray_dir(ray_dir), m_projection(projection), m_proj_plane(proj_plane) {}

    /// Given the camera, project xyz into it
    inline result_type operator()(domain_type const& t) const {

      // Get the current point along the ray
      Vector3 xyz = m_ray_pt + t[0] * m_ray_dir;

      // Convert to projected coordinates
      Vector3 proj_pt = proj_point(m_projection, xyz);

      result_type ans;
      ans[0] = signed_dist_to_plane(m_proj_plane, proj_pt);
      return ans;
    }
  }; // End class SolveCurvedPlaneIntersection

  // Given a ray in ECEF and a water surface which is a plane only in
  // a local stereographic projection, compute how the ray bends under
  // Snell's law. Use the following approximate logic. Find where the
  // ray intersects the datum with the mean water height, as then it
  // is close to the water surface, since the water surface is almost
  // horizontal in projected coordinates. Find a point on that ray 1 m
  // before that. Convert both of these points from ECEF to the
  // projected coordinate system. Do Snell's law in that coordinate
  // system for the ray going through those two projected points. Find
  // a point on the outgoing ray in projected coordinates Find another
  // close point further along it. Undo the projection for these two
  // points. That will give the outgoing direction in ECEF.
  bool snells_law_curved(Vector3 const& in_xyz, Vector3 const& in_dir,
                         std::vector<double> const& plane,
                         vw::cartography::GeoReference const& water_surface_projection,
                         double refraction_index, 
                         Vector3 & out_xyz, Vector3 & out_dir) {
        
    // Find the mean water surface
    double mean_ht = -plane[3] / plane[2];
    double major_radius
      = water_surface_projection.datum().semi_major_axis() + mean_ht;
    double minor_radius
      = water_surface_projection.datum().semi_minor_axis() + mean_ht;
          
    // Intersect the ray with the mean water surface, this will
    // give us the initial guess for intersecting with that
    // surface. The precise value of this is not important, as
    // long as it is rather close to the plane and on that ray.
    Vector3 guess_xyz = vw::cartography::datum_intersection(major_radius, minor_radius,
                                                            in_xyz, in_dir);

    // Move a little up the ray
    Vector3 prev_xyz = guess_xyz - 1.0 * in_dir;
          
    Vector3 in_proj_xyz = proj_point(water_surface_projection, guess_xyz);
    Vector3 prev_proj_xyz = proj_point(water_surface_projection, prev_xyz);
          
    Vector3 in_proj_dir = in_proj_xyz - prev_proj_xyz;
    in_proj_dir /= norm_2(in_proj_dir);

    // Snell's law in projected coordinates
    Vector3 out_proj_xyz, out_proj_dir;
    bool ans = snells_law(in_proj_xyz, in_proj_dir,
                          plane, refraction_index,
                          out_proj_xyz, out_proj_dir);

    // If Snell's law failed to work, exit early
    if (!ans)
      return ans;
          
    Vector3 next_proj_xyz = out_proj_xyz + 1.0 * out_proj_dir;

    // Convert back to ECEF
    out_xyz = unproj_point(water_surface_projection, out_proj_xyz);
    Vector3 next_xyz = unproj_point(water_surface_projection, next_proj_xyz);

    // Finally get the outgoing direction according to Snell's law in ECEF
    out_dir = next_xyz - out_xyz;
    out_dir /= norm_2(out_dir);

#if 0
    // Refine out_xyz with a solver, with 1e-16 tolerance, after using
    // that 1 m perturbation, as above. The parameter determining the
    // position on the ray changes by under 2.6e-8, so it is not worth
    // it. It is rather slow too.  Then one would need to still
    // recompute out_dir somehow, if doing things this way.
    SolveCurvedPlaneIntersection model(in_xyz, in_dir, water_surface_projection, plane);
    vw::Vector<double> objective(1), start(1);
    start[0] = norm_2(out_xyz - in_xyz);
    objective[0] = 0.0; 
    int status = -1; // will change
    Vector<double> solution = math::levenberg_marquardt(model, start, objective, status);

    out_xyz = in_xyz + solution[0] * in_dir; 
#endif
    
#if 0
    // Sanity check
    test_snells_law(plane,  
                    water_surface_projection,  
                    refraction_index,
                    out_xyz, in_dir, out_dir,
                    out_proj_xyz, in_proj_dir, out_proj_dir);
#endif

    return true;
  }
  
  // Settings used for bathymetry correction
  void BathyStereoModel::set_bathy(double refraction_index,
                                   std::vector<BathyPlaneSettings> const& bathy_set) {
    
    m_bathy_correct = true;
    m_refraction_index = refraction_index;
    m_bathy_set = bathy_set;
    
    if (m_refraction_index <= 1) 
      vw::vw_throw(vw::ArgumentErr() << "The water refraction index must be bigger than 1.");

    if (m_bathy_set.size() != 2) 
      vw::vw_throw(vw::ArgumentErr() << "Expecting two bathy planes (left and right).");

    for (int it = 0; it < 2; it++) {
      if (m_bathy_set[it].bathy_plane.size() != 4)
        vw::vw_throw(vw::ArgumentErr() << "The bathy plane must have 4 coefficients.");
    }

    if (m_bathy_set[0].use_curved_water_surface != m_bathy_set[1].use_curved_water_surface)
      vw::vw_throw(vw::ArgumentErr()
                   << "Either both or none of the bathy planes must model the "
                   << "curvature of the water surface.");

    // The default behavior is for the left and right bathy planes to be the same.
    // Yet we allow them to be different. Here need to check.
    m_single_bathy_plane = true;
    if (m_bathy_set[0].use_curved_water_surface != m_bathy_set[1].use_curved_water_surface)
      m_single_bathy_plane = false;
    if (m_bathy_set[0].water_surface_projection.proj4_str()
        != m_bathy_set[1].water_surface_projection.proj4_str())
       m_single_bathy_plane = false;
    if (m_bathy_set[0].bathy_plane != m_bathy_set[1].bathy_plane)
      m_single_bathy_plane = false;
  }

  // Compute the rays intersection. Note that even if we are in
  // bathymetry mode, so m_bathy_correct is true, for this particular
  // pair of rays we may have do_bathy false, and then we won't do the
  // correction.  Return also a flag saying if we did bathymetry
  // correction or not.
  Vector3 BathyStereoModel::operator()(std::vector<Vector2> const& pixVec,
                                       Vector3& errorVec, bool do_bathy,
                                       bool & did_bathy) const {
  
    // Initialize the outputs
    did_bathy = false;
    errorVec = Vector3();
    
    // It was verified beforehand that both bathy planes have the same
    // value for use_curved_water_surface.
    bool use_curved_water_surface = m_bathy_set[0].use_curved_water_surface;
    
    int num_cams = m_cameras.size();
    VW_ASSERT((int)pixVec.size() == num_cams,
              vw::ArgumentErr() << "the number of rays must match "
              << "the number of cameras.\n");
  
    try {

      std::vector<Vector3> camDirs(num_cams), camCtrs(num_cams);
      camDirs.clear(); camCtrs.clear();
    
      // Pick the valid rays
      for (int p = 0; p < num_cams; p++){
      
        Vector2 pix = pixVec[p];
        if (pix != pix || // i.e., NaN
            pix == camera::CameraModel::invalid_pixel() ) 
          continue;
      
        camDirs.push_back(m_cameras[p]->pixel_to_vector(pix));
        camCtrs.push_back(m_cameras[p]->camera_center(pix));
      }

      // Not enough valid rays
      if (camDirs.size() < 2) 
        return Vector3();

      if (are_nearly_parallel(m_least_squares, m_angle_tol, camDirs)) 
        return Vector3();

      // Determine range by triangulation
      Vector3 uncorr_tri_pt = triangulate_point(camDirs, camCtrs, errorVec);
      if ( m_least_squares ){
        if (num_cams == 2)
          refine_point(pixVec[0], pixVec[1], uncorr_tri_pt);
        else
          vw::vw_throw(vw::NoImplErr() << "Least squares refinement is not "
                       << "implemented for multi-view stereo.");
      }
    
      // Reflect points that fall behind one of the two cameras.  Do
      // not do this when bathymetry mode is on, as then we surely
      // have satellite images and there is no way a point would be
      // behind the camera.
      if (!m_bathy_correct) {
        bool reflect = false;
        for (int p = 0; p < (int)camCtrs.size(); p++)
          if (dot_prod(uncorr_tri_pt - camCtrs[p], camDirs[p]) < 0)
            reflect = true;
        if (reflect)
          uncorr_tri_pt = -uncorr_tri_pt + 2*camCtrs[0];
      }

      if (!do_bathy || camDirs.size() != 2) 
        return uncorr_tri_pt;
    
      // Continue with bathymetry correction
      
      if (!m_bathy_correct) 
        vw::vw_throw(vw::ArgumentErr()
                     << "Requested to do bathymetry correction while "
                     << "this mode was not set up.");

      // Find the rays after bending, according to Snell's law.
      std::vector<Vector3> waterDirs(2), waterCtrs(2);

      // When there's a single plane, things are simple.
      // Rays get bent or not, then they intersect, and done.
      if (m_single_bathy_plane) {
      
        if (!use_curved_water_surface) {
          
          double ht_val = signed_dist_to_plane(m_bathy_set[0].bathy_plane, uncorr_tri_pt);
          if (ht_val >= 0) {
            // the rays intersect above the water surface, no need to go on
            did_bathy = false;
            return uncorr_tri_pt;
          }
          
          // The simple case, when the water surface is a plane in ECEF
          for (size_t it = 0; it < 2; it++) {
            bool ans = snells_law(camCtrs[it], camDirs[it], m_bathy_set[it].bathy_plane,
                                  m_refraction_index, 
                                  waterCtrs[it], waterDirs[it]);
            // If Snell's law failed to work, return the result before it
            if (!ans) {
              did_bathy = false;
              return uncorr_tri_pt;
            }
          }
          
        } else{
          
          // The more complex case, the water surface is curved. It is
          // however flat (a plane) if we switch to proj coordinates.
          Vector3 proj_pt = proj_point(m_bathy_set[0].water_surface_projection, uncorr_tri_pt);
          double ht_val = signed_dist_to_plane(m_bathy_set[0].bathy_plane, proj_pt);
          if (ht_val >= 0) {
            // the rays intersect above the water surface
            did_bathy = false;
            return uncorr_tri_pt;
          }
          
          for (size_t it = 0; it < 2; it++) {
            // Bend each ray at the surface according to Snell's law.
            bool ans = snells_law_curved(camCtrs[it], camDirs[it],
                                         m_bathy_set[it].bathy_plane,  
                                         m_bathy_set[it].water_surface_projection,
                                         m_refraction_index,
                                         waterCtrs[it], waterDirs[it]);
            if (!ans) {
              did_bathy = false;
              return uncorr_tri_pt;
            }
          }
        }
        
        // Re-triangulate with the new rays
        Vector3 corr_tri_pt = triangulate_point(waterDirs, waterCtrs, errorVec);
        
        did_bathy = true;
        return corr_tri_pt;
      }

      // The case of left and right images having their own bathy planes
      
      // Bend the rays
      if (!use_curved_water_surface) {
        for (size_t it = 0; it < 2; it++) {
          bool ans = snells_law(camCtrs[it], camDirs[it],
                                m_bathy_set[it].bathy_plane,  
                                m_refraction_index,
                                waterCtrs[it], waterDirs[it]);
          if (!ans)
            return uncorr_tri_pt;
        }
      } else {
        for (size_t it = 0; it < 2; it++) {
          // Bend each ray at the surface according to Snell's law.
          bool ans = snells_law_curved(camCtrs[it], camDirs[it],
                                       m_bathy_set[it].bathy_plane,  
                                       m_bathy_set[it].water_surface_projection,
                                       m_refraction_index,
                                       waterCtrs[it], waterDirs[it]);
          if (!ans)
            return uncorr_tri_pt;
        }
      }
      
      // Each ray has two parts: before bending and after it. Two
      // bent rays can intersect on their unbent parts, the bent part
      // of one ray with unbent part of another ray, unbent part of
      // one ray with bent part of another ray, and bent parts of both
      // rays. Handle all these with much care. 

      Vector3 err, tri_pt;
      std::vector<double> signed_dists;

      // See if the unbent portions intersect above their planes
      tri_pt = triangulate_pair(camDirs[0], camCtrs[0], camDirs[1], camCtrs[1], err);
      signed_distances_to_planes(use_curved_water_surface, m_bathy_set, tri_pt, signed_dists);
      if (signed_dists[0] >= 0 && signed_dists[1] >= 0) {
        did_bathy = false; // since the rays did not reach the bathy plane
        errorVec = err;
        return tri_pt;
      }
      
      // See if the bent portions intersect below their planes
      tri_pt = triangulate_pair(waterDirs[0], waterCtrs[0], waterDirs[1], waterCtrs[1], err);
      signed_distances_to_planes(use_curved_water_surface, m_bathy_set, tri_pt, signed_dists);
      if (signed_dists[0] <= 0 && signed_dists[1] <= 0) {
        did_bathy = true; // the resulting point is at least under one plane
        errorVec = err;
        return tri_pt;
      }

      // See if the left unbent portion intersects the right bent portion,
      // above left's water plane and below right's water plane
      tri_pt = triangulate_pair(camDirs[0], camCtrs[0], waterDirs[1], waterCtrs[1], err);
      signed_distances_to_planes(use_curved_water_surface, m_bathy_set, tri_pt, signed_dists);
      if (signed_dists[0] >= 0 && signed_dists[1] <= 0) {
        did_bathy = true; // the resulting point is at least under one plane
        errorVec = err;
        return tri_pt;
      }
      
      // See if the left bent portion intersects the right unbent portion,
      // below left's water plane and above right's water plane
      tri_pt = triangulate_pair(waterDirs[0], waterCtrs[0], camDirs[1], camCtrs[1], err);
      signed_distances_to_planes(use_curved_water_surface, m_bathy_set, tri_pt, signed_dists);
      if (signed_dists[0] <= 0 && signed_dists[1] >= 0) {
        did_bathy = true; // the resulting point is at least under one plane
        errorVec = err;
        return tri_pt;
      }
      
    } catch (const camera::PixelToRayErr& /*e*/) {}

    // We arrive here only when there's bad luck
    did_bathy = false;
    errorVec = vw::Vector3();
    return vw::Vector3();
  }

  Vector3 BathyStereoModel::operator()(std::vector<Vector2> const& pixVec,
                                       double& error) const {
    vw::vw_throw(vw::NoImplErr() << "Not implemented for BathyStereoModel.");
    return Vector3();
  }
  
  Vector3 BathyStereoModel::operator()(Vector2 const& pix1,
                                       Vector2 const& pix2, Vector3& errorVec) const {
    vw::vw_throw(vw::NoImplErr() << "Not implemented for BathyStereoModel.");
    return Vector3();
  }


  Vector3 BathyStereoModel::operator()(Vector2 const& pix1, Vector2 const& pix2,
                                       double& error) const {
    vw::vw_throw(vw::NoImplErr() << "Not implemented for BathyStereoModel.");
    return Vector3();
  }
  
}

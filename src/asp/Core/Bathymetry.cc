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
      if (!(iss >> bathy_plane[0] >> bathy_plane[1] >> bathy_plane[2] >>bathy_plane[3])) 
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

  // Consider a stereographic projection and a plane
  // a * x + b * y + c * z + d = 0 for (x, y, z) in this projection.
  // Intersect it with a ray given in ECEF coordinates.
  // If the values a and b are 0, that is the same as intersecting
  // the ray with the spheroid of values -d/c above the datum.
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
    SolveCurvedPlaneIntersection(vw::Vector3 const& ray_pt,
                                 vw::Vector3 const& ray_dir,
                                 vw::cartography::GeoReference const& projection,
                                 std::vector<double> const& proj_plane):
      m_ray_pt(ray_pt),
      m_ray_dir(ray_dir),
      m_projection(projection),
      m_proj_plane(proj_plane) {}

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

  
  // Settings used for bathymetry correction
  void BathyStereoModel::set_bathy(double refraction_index,
                                   std::vector<double> const& bathy_plane,
                                   bool use_curved_water_surface,
                                   vw::cartography::GeoReference
                                   const& water_surface_projection) {
    
    m_bathy_correct = true;
    m_refraction_index = refraction_index;
    m_bathy_plane = bathy_plane;
    m_use_curved_water_surface = use_curved_water_surface;
    m_water_surface_projection = water_surface_projection;
    
    if (m_refraction_index <= 1) 
      vw::vw_throw(vw::ArgumentErr() << "The water refraction index must be bigger than 1.");

    if (m_bathy_plane.size() != 4)
      vw::vw_throw(vw::ArgumentErr() << "The bathymetry plane must have 4 coefficients.");
  
  }

  // Given a ray going down towards Earth, starting at point c and
  // with unit direction d, a plane 'p' to the water surface with four
  // coefficients such that the plane equation is p[0] * x + p[1] * y
  // + p[2] * z + p[3] = 0, the normal (p[0], p[1], p[2]) pointing
  // upwards away from Earth, and water refraction index, find where
  // this ray meets the water plane named c2, and the ray direction d2
  // after it bends according to Snell's law. Return true on success.
  bool BathyStereoModel::snells_law(Vector3 const& c, Vector3 const& d,
                                    std::vector<double> const& p,
                                    double refraction_index, 
                                    Vector3 & c2, Vector3 & d2) {

    // The ray is given as c + alpha * d, where alpha is real.
    // See where it intersects the plane.
    double cn = 0.0, dn = 0.0; // Dot product of c and d with plane normal n
    for (size_t it = 0; it < 3; it++) {
      cn += p[it] * c[it];
      dn += p[it] * d[it];
    }

    // The ray must descend to the plane, or else something is not right
    if (dn >= 0.0)
      return false;
    
    double alpha = -(p[3] + cn)/dn;
  
    // The intersection with the plane
    c2 = c + alpha * d;

    // Let n be the plane normal pointing up (the first three components
    // of the plane vector p). Let d2 be the outgoing vector after the
    // ray hits the water, according to Snell's law, with d being the
    // incoming ray. Let a1 be the angles between -d and n, a2 be the
    // angle between d2 and -n.
  
    // Then sin(a1) = refraction_index * sin(a2) per Snell's law.
    // Square this. Note that cos^2 (x) + sin^2 (x) = 1.
    // So, 1 - cos(a1)^2 = refraction_index^2 * (1 - cos(a2)^2).
    // But cos(a1) = dot_product(-d, n) = -dn.
    // So, cos(a2)^2 = 1 - (1 - dn^2)/refraction_index^2
    // Call the left-hand value cos_sq.

    double cos_sq = 1.0 - (1.0 - dn * dn)/refraction_index/refraction_index;
  
    // The outgoing vector d2 will be a linear combination of -n and d1,
    // normalized to unit length. Let alpha > 0 be the value which will
    // produce the linear combination.  So,
    // d2 = (-n + alpha * d)/norm(-n + alpha * d)
    // But dot(d2, -n) = cos(a2). Hence, if we dot the above with n and square it,
    // we get 
    // cos(a2)^2 = (-1 + alpha * dn)^2 / dot( -n + alpha * d, -n + alpha * d)
    // or 
    // cos(a2)^2 * dot( -n + alpha * d, -n + alpha * d) = (-1 + alpha * dn)^2  
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
    d2 = -Vector3(p[0], p[1], p[2]) + alpha * d;
    d2 = d2 / norm_2(d2);

    return true;
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
      Vector3 result = triangulate_point(camDirs, camCtrs, errorVec);
      if ( m_least_squares ){
        if (num_cams == 2)
          refine_point(pixVec[0], pixVec[1], result);
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
          if (dot_prod(result - camCtrs[p], camDirs[p]) < 0 ) reflect = true;
        if (reflect)
          result = -result + 2*camCtrs[0];
      }

      if (!do_bathy || camDirs.size() != 2) 
        return result;
    
      // Continue with bathymetry correction
      
      if (!m_bathy_correct) 
        vw::vw_throw(vw::ArgumentErr()
                     << "Requested to do bathymetry correction while "
                     << "this mode was not set up.");

      // Find the rays after bending, according to Snell's law
      
      std::vector<Vector3> waterDirs(2), waterCtrs(2);
      if (!m_use_curved_water_surface) {
        
        // The simple case, when the water surface is a plane in ECEF

        // If the intersection point is above the water plane, don't do
        // bathymetry correction.
        double ht_val = signed_dist_to_plane(m_bathy_plane, result);
        if (!(ht_val < 0)) // take into account also the NaN case
          return result;
        
        // Find where the rays intersect the plane and how the water bends
        // them
        for (size_t it = 0; it < 2; it++) {
          bool ans = snells_law(camCtrs[it], camDirs[it], m_bathy_plane,  
                                m_refraction_index,  
                                waterCtrs[it], waterDirs[it]);
          
          // If Snell's law failed to work, return the result before it
          if (!ans)
            return result;
        }
        
      }else{

        // The more complex case, the water surface is curved. It is
        // however flat (a plane) if we switch to projected
        // coordinates.
        
        // For that reason, use the following logic. Given a ray that
        // we want to bend according to Snell's law, find where it
        // intersects the datum with the mean water height. Find a
        // point on that ray 1 m before that. Convert both of these
        // points from ECEF to the projected coordinate system. Do
        // Snell's law in that coordinate system for the ray going
        // through them. Find the outgoing ray. Then do similar logic
        // to convert the ray back to ECEF. This should be accurate
        // enough.
        
        // If the rays intersect above the water surface, don't do the
        // correction.
        Vector3 proj_pt = proj_point(m_water_surface_projection, result);
        double ht_val = signed_dist_to_plane(m_bathy_plane, proj_pt);
        if (!(ht_val < 0)) // take into account also the NaN case
          return result;

        // Find the mean water surface
        double mean_ht = -m_bathy_plane[3] / m_bathy_plane[2];
        double major_radius = m_water_surface_projection.datum().semi_major_axis() + mean_ht;
        double minor_radius = m_water_surface_projection.datum().semi_minor_axis() + mean_ht;

        // Bend each ray at the surface according to Snell's law. Takes some work.
        for (size_t it = 0; it < 2; it++) {
          
          // Intersect the ray with the mean water surface, this will
          // give us the initial guess for intersecting with that
          // surface.
          Vector3 guess_xyz = vw::cartography::datum_intersection(major_radius, minor_radius,
                                                                  camCtrs[it], camDirs[it]);

# if 0
          // Use a solver. This is way too slow in practice.
          
          // Find the position on the ray and convert to projected coordinates
          double guess_t = dot_prod(guess_xyz - camCtrs[it], camDirs[it]);
          
          Vector3 proj_pt = proj_point(m_water_surface_projection, guess_xyz);
          double err = signed_dist_to_plane(m_bathy_plane, proj_pt);

          // std::cout << "Initial plane error " << err << "\n";

          // Refine t so see where the intersection on the surface happens
          Vector<double, 1> initial_t, observation;
          initial_t[0] = guess_t;
          observation[0] = 0.0; // At the end we will have cost_function_residual == observation
          int status = 0;
          const double abs_tolerance  = 1e-16;
          const double rel_tolerance  = 1e-16;
          const int    max_iterations = 10;
          SolveCurvedPlaneIntersection model(camCtrs[it], camDirs[it],
                                             m_water_surface_projection,
                                             m_bathy_plane);
          Vector<double, 1> solved_t
            = vw::math::levenberg_marquardt(model, initial_t, observation,
                                            status, abs_tolerance, rel_tolerance,
                                            max_iterations);

          // The solved-for point on the surface
          Vector3 in_pt
            = proj_point(m_water_surface_projection, camCtrs[it] + solved_t[0] * camDirs[it]);

          // std::cout << "Final plane error "
          // << signed_dist_to_plane(m_bathy_plane, in_pt) << "\n";

          Vector3 prev_pt
            = proj_point(m_water_surface_projection, camCtrs[it]
                         + (solved_t[0] - 1.0)* camDirs[it]);
#else
          // No solver, just a one-step solution, gives very similar results
          Vector3 prev_xyz = guess_xyz - 1.0 * camDirs[it];
          
          Vector3 in_pt = proj_point(m_water_surface_projection, guess_xyz);
          Vector3 prev_pt = proj_point(m_water_surface_projection, prev_xyz);
#endif
          
          Vector3 in_dir = in_pt - prev_pt;
          in_dir /= norm_2(in_dir);

          Vector3 out_pt, out_dir;
          bool ans = snells_law(in_pt, in_dir,
                                m_bathy_plane, m_refraction_index,
                                out_pt, out_dir);

          // If Snell's law failed to work, return the result before it
          if (!ans)
            return result;

          
          Vector3 next_pt = out_pt + 1.0 * out_dir;

          // Convert back to ECEF
          Vector3 out_xyz = unproj_point(m_water_surface_projection, out_pt);
          Vector3 next_xyz = unproj_point(m_water_surface_projection, next_pt);

          // Finally get the outgoing direction according to Snell's law in ECEF
          waterCtrs[it] = out_xyz;
          waterDirs[it] = next_xyz - out_xyz;
          waterDirs[it] /= norm_2(waterDirs[it]);

#if DEBUG_BATHY
          // Test Snell's law in projected and unprojected coordinates
          
          // Projected coordinates
          Vector3 plane_normal(m_bathy_plane[0], m_bathy_plane[1], m_bathy_plane[2]);
          double sin_in = sin(acos(dot_prod(plane_normal, -in_dir)));
          double sin_out = sin(acos(dot_prod(-plane_normal, out_dir)));
          std::cout << "proj sin_in, sin_out, sin_in - index * sin_out "
                    << sin_in << ' ' << sin_out << ' '
                    << sin_in - m_refraction_index * sin_out << std::endl;
          
          // Unprojected coordinates
          Vector3 pt_above_normal = out_pt + 1.0 * plane_normal; // go up a bit along the normal
          Vector3 xyz_above_normal = unproj_point(m_water_surface_projection, pt_above_normal);
          Vector3 unproj_normal = xyz_above_normal - out_xyz;
          unproj_normal /= norm_2(unproj_normal); // normalize
          sin_in = sin(acos(dot_prod(unproj_normal, -camDirs[it])));
          sin_out = sin(acos(dot_prod(-unproj_normal, waterDirs[it])));
          std::cout << "unproj sin_in, sin_out, sin_in - index * sin_out "
                    << sin_in << ' ' << sin_out << ' '
                    << sin_in - m_refraction_index * sin_out << std::endl;

          // Verify that the incoming ray, outgoing ray, and the
          // normal are in the same plane in projected coordinates
          Vector3 in_out_normal = cross_prod(in_dir, out_dir);
          double plane_error = dot_prod(in_out_normal, plane_normal);
          std::cout << "proj plane error " << plane_error << std::endl;

          // In unprojected coordinates
          in_out_normal = cross_prod(camDirs[it], waterDirs[it]);
          plane_error = dot_prod(in_out_normal, unproj_normal);
          std::cout << "unproj plane error " << plane_error << std::endl;
          
          std::cout << std::endl;
#endif
        }
        
      } 
      
      // Re-triangulate with the new rays
      result = triangulate_point(waterDirs, waterCtrs, errorVec);
      
      did_bathy = true;
      return result;

    } catch (const camera::PixelToRayErr& /*e*/) {
      return Vector3();
    }
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

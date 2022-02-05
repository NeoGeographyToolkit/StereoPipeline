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


/// \file Bathymetry.h
///

#ifndef __ASP_CORE_BATHYMETRY_H__
#define __ASP_CORE_BATHYMETRY_H__

#include <vw/Math/Vector.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/Cartography/GeoReference.h>

namespace asp {

  struct BathyPlaneSettings {
    std::vector<double> bathy_plane;
    bool use_curved_water_surface;
    vw::cartography::GeoReference water_surface_projection;
    
    BathyPlaneSettings(): use_curved_water_surface(false) {}
  };
    
  // Read left and right bathy plane settings and associated data.
  // More often than not they will be identical.
  void read_bathy_plane_set(std::string const& bathy_plane_files,
                            std::vector<BathyPlaneSettings> & bathy_plane_set);

  // Given a ray going down towards Earth, starting at point c and
  // with unit direction d, a plane 'p' to the water surface with four
  // coefficients such that the plane equation is p[0] * x + p[1] * y
  // + p[2] * z + p[3] = 0, the normal (p[0], p[1], p[2]) pointing
  // upwards away from Earth, and water refraction index, find where
  // this ray meets the water plane named c2, and the ray direction d2
  // after it bends according to Snell's law. Return true on success.
  bool snells_law(vw::Vector3 const& c, vw::Vector3 const& d,
                  std::vector<double> const& plane, double refraction_index,
                           vw::Vector3 & c2, vw::Vector3 & d2);
  
  // In this version of snells_law(), we are given given a point
  // further down on the incoming ray, where this ray may intersect
  // another ray if no correction due to refraction index happens. if
  // that further down point is above the water plane, there is no
  // need to bend the ray.
  bool snells_law_maybe(vw::Vector3 const& c, vw::Vector3 const& d,
                        std::vector<double> const& plane, double refraction_index,
                        vw::Vector3 & c2, vw::Vector3 & d2);
  
  class BathyStereoModel: public vw::stereo::StereoModel {
  public:
    
    //------------------------------------------------------------------
    // Constructors / Destructors
    //------------------------------------------------------------------
    BathyStereoModel(std::vector<const vw::camera::CameraModel *> const& cameras,
                     bool least_squares_refine = false,
                     double angle_tol = 0.0):
      vw::stereo::StereoModel(cameras, least_squares_refine, angle_tol), m_bathy_correct(false){}
    
    BathyStereoModel(vw::camera::CameraModel const* camera_model1,
                     vw::camera::CameraModel const* camera_model2,
                     bool least_squares_refine = false,
                     double angle_tol = 0.0):
      vw::stereo::StereoModel(camera_model1, camera_model2, least_squares_refine, angle_tol),
      m_bathy_correct(false), m_single_bathy_plane(true) {}
    
    virtual ~BathyStereoModel() {}
    
    //------------------------------------------------------------------
    // Public Methods
    //------------------------------------------------------------------
    
    /// Apply a stereo model to multiple or just two image coordinates.
    /// Returns an xyz point. The error is set to 0 if triangulation
    /// did not succeed, otherwise it is the vector between the closest points on the rays.
    virtual vw::Vector3 operator()(std::vector<vw::Vector2> const& pixVec,
                                   vw::Vector3& errorVec, bool do_bathy, bool & did_bathy) const;
    virtual vw::Vector3 operator()(std::vector<vw::Vector2> const& pixVec,
                                   double & error) const;
    virtual vw::Vector3 operator()(vw::Vector2 const& pix1, vw::Vector2 const& pix2,
                                   vw::Vector3& errorVec) const;
    virtual vw::Vector3 operator()(vw::Vector2 const& pix1, vw::Vector2 const& pix2,
                                   double & error) const;
    
    // Settings used for bathymetry correction. The left and right images
    // get individual bathy plane settings, but they may be identical.
    void set_bathy(double refraction_index,
                   std::vector<BathyPlaneSettings> const& bathy_set);
    
  private:
    // Used for bathymetry
    bool m_bathy_correct;                        // If to do bathy correction
    bool m_single_bathy_plane;                   // if the left and right images use same plane 
    double m_refraction_index;                   // Water refraction index
    std::vector<BathyPlaneSettings> m_bathy_set; // Bathy plane settings
  };
  
} // end namespace asp


#endif //__ASP_CORE_BATHYMETRY_H__

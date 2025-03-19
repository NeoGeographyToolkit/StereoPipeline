// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// RPC model and triangulation. Following the paper:

/// Jacek Grodecki, Gene Dial and James Lutes, "Mathematical Model for
/// 3D Feature Extraction from Multiple Satellite Images Described by
/// RPCs." Proceedings of ASPRS 2004 Conference, Denver, Colorado, May,
/// 2004.

/// Digital Globe images (and others) often include an RPC camera model
/// definition.  The RPC model is less complicated than the full Digital
/// Globe camera model.

#ifndef __STEREO_CAMERA_RPC_MODEL_H__
#define __STEREO_CAMERA_RPC_MODEL_H__

#include <vw/Math/Matrix.h>
#include <vw/Math/Vector.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/Datum.h>

#include <string>
#include <ostream>

namespace vw {
  class DiskImageResourceGDAL;
}

namespace asp {

  class RPCModel : public vw::camera::CameraModel {

  public:

    // These integers show up in a lot of RPC code
    static const int GEODETIC_COORD_SIZE = 3;
    static const int IMAGE_COORD_SIZE    = 2;
    static const int NUM_RPC_COEFFS = 78; // Total number of RPC coefficients

    typedef vw::Vector<double,20> CoeffVec;

    /// Construct from a GDAL image or an .RPB file.
    RPCModel(std::string const& filename);

    RPCModel(vw::DiskImageResourceGDAL* resource);
    RPCModel(vw::cartography::Datum const& datum,
             CoeffVec    const& line_num_coeff,
             CoeffVec    const& line_den_coeff,
             CoeffVec    const& samp_num_coeff,
             CoeffVec    const& samp_den_coeff,
             vw::Vector2 const& xy_offset,
             vw::Vector2 const& xy_scale,
             vw::Vector3 const& lonlatheight_offset,
             vw::Vector3 const& lonlatheight_scale,
             double err_bias = 0.0,
             double err_rand = 0.0,
             double terrain_height = std::numeric_limits<double>::quiet_NaN());

    virtual std::string type() const { return "RPC"; }
    virtual ~RPCModel() {}

    // Save the RPC model to an XML file. This is similar to the WorldView format.
    // GDAL can read it.
    void saveXML(std::string const& filename) const;
    
    /// Populate this object from a .RPB file.
    void load_rpb_file(std::string const& filename);

    // Standard Access Methods. The concept of camera_center does not
    // apply well to RPC, we just return an arbitrary point on the ray.
    virtual vw::Vector2 point_to_pixel (vw::Vector3 const& point) const;
    virtual vw::Vector3 pixel_to_vector(vw::Vector2 const& pix) const;
    virtual vw::Vector3 camera_center  (vw::Vector2 const& pix) const;

   // Apply a transform to an RPC model
   void applyTransform(vw::Matrix4x4 const& transform);

    static vw::Vector2 normalizedLlhToPix
      (vw::Vector3 const& normalized_geodetic,
       CoeffVec    const& line_num_coeff,   CoeffVec const& line_den_coeff,
       CoeffVec    const& sample_num_coeff, CoeffVec const& sample_den_coeff);

    // Normalized lon-lat-height to normalized pixel
    vw::Vector2 normalizedLlhToPix(vw::Vector3 const& normalized_geodetic) const;

    vw::Vector2 geodetic_to_pixel(vw::Vector3 const& geodetic) const;

    // Access to constants
    vw::cartography::Datum const& datum   () const { return m_datum;               }
    CoeffVec    const& line_num_coeff     () const { return m_line_num_coeff;      }
    CoeffVec    const& line_den_coeff     () const { return m_line_den_coeff;      }
    CoeffVec    const& sample_num_coeff   () const { return m_sample_num_coeff;    }
    CoeffVec    const& sample_den_coeff   () const { return m_sample_den_coeff;    }
    vw::Vector2 const& xy_offset          () const { return m_xy_offset;           }
    vw::Vector2 const& xy_scale           () const { return m_xy_scale;            }
    vw::Vector3 const& lonlatheight_offset() const { return m_lonlatheight_offset; }
    vw::Vector3 const& lonlatheight_scale () const { return m_lonlatheight_scale;  }

    /// Returns a vector containing the order of each of the terms a CoeffVec applies to.
    static vw::Vector<int, 20> get_coeff_order();

    // Helper methods used for triangulation and projection
    static CoeffVec calculate_terms(vw::Vector3 const& normalized_geodetic);
    static vw::Matrix<double, 20, 2> terms_Jacobian2(vw::Vector3 const& normalized_geodetic);
    static vw::Matrix<double, 20, 3> terms_Jacobian3(vw::Vector3 const& normalized_geodetic);
    static CoeffVec quotient_Jacobian(CoeffVec const& c, CoeffVec const& d,
                                       CoeffVec const& u);
    static vw::Matrix3x3 normalization_Jacobian(vw::Vector3 const& q);

    vw::Matrix<double, 2, 3> geodetic_to_pixel_Jacobian (vw::Vector3 const& geodetic) const;
    vw::Matrix<double, 2, 3> geodetic_to_pixel_numerical_Jacobian (vw::Vector3 const& geodetic, double tol) const;
    vw::Matrix<double, 2, 2> normalizedLlhToPixJac(vw::Vector3 const& normalized_geodetic) const;

    /// Given a pixel (the projection of a point in 3D space onto the camera image)
    /// and the value of the height of the point, find the lonlat of the point
    /// using Newton's method. The user may provide a guess for the lonlat.
    vw::Vector2 image_to_ground(vw::Vector2 const& pixel, double height,
                                vw::Vector2 lonlat_guess = vw::Vector2(0.0, 0.0)) const;

    /// Find a point which gets projected onto the current pixel,
    /// and the direction of the ray going through that point.
    void point_and_dir(vw::Vector2 const& pix, vw::Vector3 & P, vw::Vector3 & dir) const;

    // Will be read only for DG RPC camera models and set to 0 for the rest
    double m_err_bias, m_err_rand;
    
    // The height used for ortho images (for Maxar and Pleiades)
    double m_terrain_height;

  private:
    vw::cartography::Datum m_datum;

    // Scaling parameters
    CoeffVec    m_line_num_coeff,   m_line_den_coeff,
                m_sample_num_coeff, m_sample_den_coeff;
    vw::Vector2 m_xy_offset;
    vw::Vector2 m_xy_scale;
    vw::Vector3 m_lonlatheight_offset;
    vw::Vector3 m_lonlatheight_scale;

    void initialize(vw::DiskImageResourceGDAL* resource);
  };

  std::ostream& operator<<(std::ostream& os, const RPCModel& rpc);
}

#endif //__STEREO_CAMERA_RPC_MODEL_H__

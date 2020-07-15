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


// These are objects that relate directly to block in XML that we need
// to read. They only read and then store the raw values. Other
// objects will interpret the results.

#ifndef __STEREO_CAMERA_RPC_XML_H__
#define __STEREO_CAMERA_RPC_XML_H__

#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Camera/CameraModel.h>
#include <asp/Core/Common.h>

#include <vector>
#include <string>

#include <boost/smart_ptr/scoped_ptr.hpp>

// Special forward declare so we can hide the Xerces headers.
#include <xercesc/util/XercesDefs.hpp> // Needed for this XERCES macro
XERCES_CPP_NAMESPACE_BEGIN
  class DOMDocument;
  class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace asp {

  // Forward declare so that we can cut down on the headers.
  class RPCModel;


  /// Objects that represent read data from XML. These also provide a
  /// storage structure for modification later on.
  class ImageXML : public BitChecker {

    void parse_meta      ( xercesc::DOMElement* node );
    void parse_band_tdi  ( xercesc::DOMElement* node, int & out_tdi);
    void parse_tlc_list  ( xercesc::DOMElement* node );
    void parse_image_size( xercesc::DOMElement* node );

  public:
    ImageXML();

    void parse( xercesc::DOMElement* node );

    std::string  tlc_start_time;
    std::string  first_line_start_time;
    std::vector<std::pair<double,double> > tlc_vec; // Line -> time offset pairings
    std::string  sat_id, band_id;
    std::string  scan_direction;
    int          tdi;
    std::vector<int> tdi_multi; // for multi-spectral images
    double       avg_line_rate;
    vw::Vector2i image_size;
  };


  ///
  class GeometricXML : public BitChecker {

    void parse_principal_distance( xercesc::DOMElement* node );
    void parse_optical_distortion( xercesc::DOMElement* node );
    void parse_perspective_center( xercesc::DOMElement* node );
    void parse_camera_attitude   ( xercesc::DOMElement* node );
    void parse_detector_mounting ( xercesc::DOMElement* node );

  public:
    GeometricXML();

    void parse( xercesc::DOMElement* node );

    double      principal_distance;   // mm
    vw::int32   optical_polyorder;
    vw::Vector<double> optical_a, optical_b; // Don't currently support these
    vw::Vector3 perspective_center;  // meters in spacecraft frame
    vw::Quat    camera_attitude;
    vw::Vector2 detector_origin;     // mm
    double      detector_rotation;    // degrees about Z+ in camera frame
    double      detector_pixel_pitch; // mm
    
    void printDebugInfo() const; ///< Debug string
  };


  ///
  class EphemerisXML : public BitChecker {

    void parse_meta    ( xercesc::DOMElement* node );
    void parse_eph_list( xercesc::DOMElement* node );

  public:
    EphemerisXML();

    void parse( xercesc::DOMElement* node );

    std::string start_time;      // UTC
    double time_interval;        // seconds
    std::vector<vw::Vector3> position_vec, velocity_vec; // ECEF
    std::vector<vw::Vector<double,6> > covariance_vec;   // Tri-diagonal
  };


  /// 
  class AttitudeXML : public BitChecker {

    void parse_meta( xercesc::DOMElement* node );
    void parse_att_list( xercesc::DOMElement* node );

  public:
    AttitudeXML();

    void parse( xercesc::DOMElement* node );

    std::string start_time;
    double time_interval;
    std::vector<vw::Quat> quat_vec;
    std::vector<vw::Vector<double,10> > covariance_vec;
  };



  /// Reads from Digital Globe XML format
  class RPCXML : public BitChecker {
    boost::scoped_ptr<RPCModel> m_rpc;
    vw::BBox3 m_lat_lon_height_box;

    void parse_vector( xercesc::DOMElement* node,
                       vw::Vector<double,20> & vec );

    void parse_rpb( xercesc::DOMElement* node ); ///< Digital Globe XML
    void parse_rational_function_model( xercesc::DOMElement* node ); ///< Pleiades / Astrium
    void parse_perusat_model( xercesc::DOMElement* node ); ///< PeruSat-1

  public:
    RPCXML();
    void read_from_file( std::string const& name );
    void parse( xercesc::DOMElement* node ) { parse_rpb( node ); }

    // TODO: Why is this function in this class?
    void parse_bbox( xercesc::DOMElement* node ); ///< Read the valid sensor model bounds

    /// Return a pointer to the loaded RPC model.
    RPCModel* rpc_ptr() const;
    /// Get the GDC bounding box that the RPC model is valid for.
    vw::BBox3 get_lon_lat_height_box() const;
  };

  // Helper functions to allow us to fill the objects. This doesn't
  // really help with code reuse but I think it makes it easer to read.
  void read_xml( std::string const& filename,
                 GeometricXML& geo,
                 AttitudeXML & att,
                 EphemerisXML& eph,
                 ImageXML    & img,
                 RPCXML      & rpc );
  vw::Vector2i xml_image_size( std::string const& filename );


  /// Function to extract the four corners from the first band
  ///  of a Worldview XML file.
  /// - The order is [TOP_LEFT, TOP_RIGHT, BOT_RIGHT, BOT_LEFT]
  bool read_WV_XML_corners(std::string const& xml_path,
                           std::vector<vw::Vector2> &pixel_corners,
                           std::vector<vw::Vector2> &lonlat_corners);

  /// Attempts to approximate a georeference for a WorldView image using the
  ///  four corner points in the XML file.
  /// - This only works for unprojected WV images with four lonlat GCPs at the corner pixels.
  /// - The approximation is limited to using a perspective transform.
  bool approximate_wv_georeference(std::string const& wv_xml_path,
                                   vw::cartography::GeoReference & approx_georef);

  /// Reads a georeference from the WorldView file.
  /// - If the image is not georegistered but there are
  ///   corner coordinates in xml_path, an approximate
  ///   georef file will be created.
  bool read_wv_georeference(vw::cartography::GeoReference &georef,
                            std::string const &image_path,
                            std::string const &xml_path="");

} //end namespace asp

#endif//__STEREO_CAMERA_RPC_XML_H__

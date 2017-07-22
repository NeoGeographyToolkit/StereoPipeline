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

#ifndef __STEREO_CAMERA_SPOT_XML_H__
#define __STEREO_CAMERA_SPOT_XML_H__

#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Geometry.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Camera/Extrinsics.h>
#include <asp/Core/Common.h>

#include <vector>
#include <string>

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// Special forward declare so we can hide the Xerces headers.
#include <xercesc/util/XercesDefs.hpp> // Needed for this XERCES macro
XERCES_CPP_NAMESPACE_BEGIN
  class DOMDocument;
  class DOMElement;
  class XercesDOMParser;
  class ErrorHandler;
XERCES_CPP_NAMESPACE_END

namespace asp {


  // TODO: There is a duplicate in LinescanDG class!
  /// A functor that returns the difference in seconds from a reference time.
  /// - Uses boost::posix_time.
  class SecondsFromRef
  {
    boost::posix_time::ptime m_reference;
  public:
    inline SecondsFromRef() {}
    inline SecondsFromRef(boost::posix_time::ptime const& ref_time) : m_reference(ref_time) {}

    inline void set_base_time(boost::posix_time::ptime const& ref_time) {m_reference = ref_time;}

    inline double operator()( boost::posix_time::ptime const& time ) const {
      return double( (time - m_reference).total_microseconds() ) / 1e6;
    }
  };


  class SpotXML {
  public:
  
    /// Constructor
    /// - Sets the fixed reference time.
    SpotXML() : m_time_ref_functor(boost::posix_time::time_from_string("2002-05-04 00:00:00.00")) {}

    // The reader will populate these fields
    std::vector<vw::Vector2> lonlat_corners;
    std::vector<vw::Vector2> pixel_corners;
    std::vector<std::pair<int,         vw::Vector2> > look_angles;   // (column, psi_x/psi_y)
    std::list  <std::pair<std::string, vw::Vector3> > pose_logs;     // (time,   yaw/pitch/roll)
    std::list  <std::pair<std::string, vw::Vector3> > position_logs; // (time,   X/Y/Z)
    std::list  <std::pair<std::string, vw::Vector3> > velocity_logs; // (time,   dX/dY/dZ)
    vw::Vector2i image_size;
    double       line_period;
    std::string  center_time;
    int          center_line;
    int          center_col;

    /// Parse an XML file to populate the data
    void read_xml(std::string const& xml_path);
    
    /// Parse an XML tree to populate the data
    void parse_xml(xercesc::DOMElement* node);

    /// Load the estimated image lonlat bounds from the XML file
    static vw::BBox2 get_estimated_bounds(std::string const& xml_path);
    
    /// Load the estimated image lonlat corners from the XML file
    /// - Corners are returned in clockwise order.
    static std::vector<vw::Vector2> get_lonlat_corners(std::string const& xml_path);
    
    /// Faster overload for when the file has already been parsed.
    vw::BBox2 get_estimated_bounds() const;

    // Functions to setup functors which manage the raw input data.
    vw::camera::LagrangianInterpolation setup_position_func() const;
    vw::camera::LagrangianInterpolation setup_velocity_func() const;
    vw::camera::LinearTimeInterpolation setup_time_func    () const;
    vw::camera::LinearPiecewisePositionInterpolation setup_pose_func(
        vw::camera::LinearTimeInterpolation const& time_func) const; // (yaw/pitch/roll)        

  private: // The various XML data reading sections
  
    /// Just opens the XML file for reading and returns the root node.
    xercesc::DOMElement* open_xml_file(std::string const& xml_path);
  
    void read_look_angles(xercesc::DOMElement* look_angles_node);
    void read_ephemeris  (xercesc::DOMElement* ephemeris_node);
    void read_attitude   (xercesc::DOMElement* corrected_attitudes_node);
    void read_corners    (xercesc::DOMElement* dataset_frame_node);
    void read_image_size (xercesc::DOMElement* raster_dims_node);
    void read_line_times (xercesc::DOMElement* sensor_config_node);

    /// Converts a time from string to numeric format.
    /// - All times are in seconds relative to May 5th, 2002 (when SPOT5 launched)
    double convert_time(std::string const& s) const;

    boost::shared_ptr<xercesc::XercesDOMParser> m_parser;
    boost::shared_ptr<xercesc::ErrorHandler>    m_errHandler;
    SecondsFromRef m_time_ref_functor;

  }; // End class SpotXML


} //end namespace asp

#endif//__STEREO_CAMERA_SPOT_XML_H__

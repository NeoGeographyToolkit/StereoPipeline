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

#ifndef __STEREO_CAMERA_PLEIADES_XML_H__
#define __STEREO_CAMERA_PLEIADES_XML_H__

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

  class PleiadesXML {
  public:

    /// Constructor
    PleiadesXML(): m_start_time_is_set(false){}

    vw::Vector2i m_image_size;
    vw::Quaternion<double> m_instrument_biases;
    vw::Vector2 m_tan_psi_x, m_tan_psi_y;

    /// Parse an XML file to populate the data
    void read_xml(std::string const& xml_path);
    
    /// Parse an XML tree to populate the data
    void parse_xml(xercesc::DOMElement* node);

    // Functions to setup functors which manage the raw input data.
    vw::camera::LinearTimeInterpolation setup_time_func() const;
    vw::camera::LagrangianInterpolation setup_position_func
    (vw::camera::LinearTimeInterpolation const& time_func) const;
    vw::camera::LagrangianInterpolation setup_velocity_func
    (vw::camera::LinearTimeInterpolation const& time_func) const;
    vw::camera::SLERPPoseInterpolation         setup_pose_func
    (vw::camera::LinearTimeInterpolation const& time_func) const;
    
  private: // The various XML data reading sections
  
    /// Just opens the XML file for reading and returns the root node.
    xercesc::DOMElement* open_xml_file(std::string const& xml_path);

    void read_image_size  (xercesc::DOMElement* raster_data);
    void read_times       (xercesc::DOMElement* time);
    void read_ephemeris   (xercesc::DOMElement* ephemeris);
    void read_attitudes   (xercesc::DOMElement* attitudes);
    void read_look_angles (xercesc::DOMElement* look_angles);
    void read_instr_biases(xercesc::DOMElement* instr_biases);
    void read_center_data (xercesc::DOMElement* geom_values);

    /// Converts a time from string to double precision in seconds.
    /// All times are in seconds relative to the start time.
    /// When the start time is passed in, use is_start_time = true.
    double convert_time(std::string const& s, bool is_start_time);

    // Boost does not like a time string such as "2017-12-07 15:36:40.90795Z"
    // because it expects precisely 6 digits after the dot (hence for the millisecond).
    // Fix that.
    static std::string fix_millisecond(std::string const& in_str);

    // All times represented as doubles will be in seconds relative to m_start_time_stamp
    boost::posix_time::ptime m_start_time_stamp;

    bool   m_start_time_is_set;
    double m_start_time;
    double center_time;
    double m_line_period;
    double m_center_col;
    double m_center_row;

    std::list<std::pair<double, vw::Vector3>> m_positions;        // (time,   X/Y/Z)
    std::list<std::pair<double, vw::Vector3>> m_velocities;       // (time,   dX/dY/dZ)
    std::list<std::pair<double, vw::Quaternion<double>>> m_poses; // (time, quaternion)
    
    boost::shared_ptr<xercesc::XercesDOMParser> m_parser;
    boost::shared_ptr<xercesc::ErrorHandler>    m_err_handler;
    
  }; // End class PleiadesXML


} //end namespace asp

#endif//__STEREO_CAMERA_PLEIADES_XML_H__

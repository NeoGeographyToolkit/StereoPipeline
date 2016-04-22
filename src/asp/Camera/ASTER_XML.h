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

#ifndef __STEREO_CAMERA_ASTER_XML_H__
#define __STEREO_CAMERA_ASTER_XML_H__

#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/Log.h>
#include <vw/Math/Vector.h>
#include <vw/Math/Matrix.h>
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

  class ASTERXML {
  public:
  
    /// Constructor
    /// - Sets the fixed reference time.
    ASTERXML(){}

    // The reader will populate these fields
    std::vector< std::vector<vw::Vector2> > m_lattice_mat;
    std::vector< std::vector<vw::Vector3> > m_sight_mat;
    std::vector< std::vector<vw::Vector3> > m_world_sight_mat;
    std::vector<vw::Vector3>                m_sat_pos;
    vw::Vector2i                            m_image_size;
    
    /// Parse an XML file to populate the data
    void read_xml(std::string const& xml_path);
    
    /// Parse an XML tree to populate the data
    void parse_xml(xercesc::DOMElement* node);

  private: // The various XML data reading sections
  
    /// Just opens the XML file for reading and returns the root node.
    xercesc::DOMElement* open_xml_file(std::string const& xml_path);
  
    boost::shared_ptr<xercesc::XercesDOMParser> m_parser;
    boost::shared_ptr<xercesc::ErrorHandler>    m_errHandler;

  }; // End class ASTERXML


} //end namespace asp

#endif//__STEREO_CAMERA_ASTER_XML_H__

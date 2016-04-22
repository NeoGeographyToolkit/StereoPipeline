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

#include <vw/Core/Exception.h>          // for ArgumentErr, vw_throw, etc
#include <vw/Math/Quaternion.h>         // for Quat, Quaternion
#include <vw/Math/Vector.h>             // for Vector, Vector3, Vector4, etc
#include <vw/Cartography/Datum.h>       // for Datum
#include <vw/FileIO/DiskImageResourceGDAL.h>

#include <asp/Core/FileUtils.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/ASTER_XML.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/XMLBase.h>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/sax/SAXException.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/sax/ErrorHandler.hpp>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <iomanip>

using namespace vw;
using namespace vw::cartography;
using namespace xercesc;

using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

namespace asp {

DOMElement* ASTERXML::open_xml_file(std::string const& xml_path) {

  // Check if the file actually exists and throw a user helpful file.
  if ( !boost::filesystem::exists( xml_path ) )
    vw_throw( ArgumentErr() << "XML file \"" << xml_path << "\" does not exist." );

  std::string error_prefix = "XML file \"" + xml_path + "\" is invalid.\nException message is: \n";
  std::string err_message  = ""; // Filled in later on error

  try{
    //std::cout << "Set XML parser\n";
  
    // Set up the XML parser if we have not already done so
    if (!m_parser.get()) {
      m_parser.reset(new XercesDOMParser());
      m_errHandler.reset(new HandlerBase());
      m_parser->setValidationScheme(XercesDOMParser::Val_Always);
      m_parser->setDoNamespaces(true);   
      m_parser->setErrorHandler(m_errHandler.get());
    }

    //std::cout << "Load XML\n";

    // Load the XML file
    m_parser->parse( xml_path.c_str() );
    DOMDocument* xmlDoc      = m_parser->getDocument();
    DOMElement * elementRoot = xmlDoc->getDocumentElement();
    return elementRoot;

  } catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    err_message = error_prefix + message;
    XMLString::release(&message);
  } catch (const DOMException& toCatch) {
    char* message = XMLString::transcode(toCatch.msg);
    err_message = error_prefix + message;
    XMLString::release(&message);
  } catch (const SAXException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    err_message = error_prefix + message;
    XMLString::release(&message);
  } catch ( const std::exception& e ) {
    err_message = error_prefix + e.what();
  } catch (...) {
    err_message = "Unrecognized error in XML file \"" + xml_path + "\"\n";
  }
  vw_throw( ArgumentErr() << err_message); // Only get here on error
  return 0;
}

void ASTERXML::read_xml(std::string const& xml_path) {

  DOMElement * elementRoot = open_xml_file(xml_path);
  parse_xml(elementRoot);
}

void ASTERXML::parse_xml(xercesc::DOMElement* node) {

  xercesc::DOMElement* lattice_point_node = get_node<DOMElement>(node, "LATTICE_POINT");
  std::string lattice_point_txt( XMLString::transcode(lattice_point_node->getTextContent()) );
  asp::read_matrix_from_string(lattice_point_txt, m_lattice_mat);

  xercesc::DOMElement* sight_vector_node = get_node<DOMElement>(node, "SIGHT_VECTOR");
  std::string sight_vector_txt( XMLString::transcode(sight_vector_node->getTextContent()) );
  asp::read_matrix_from_string(sight_vector_txt, m_sight_mat);
  if (m_lattice_mat.empty() || m_sight_mat.empty()     ||
      m_lattice_mat.size()   != m_sight_mat.size()   ||
      m_lattice_mat[0].size() != m_sight_mat[0].size() ) {
    vw_throw( ArgumentErr() << "Inconsistent lattice point and sight vector information.\n");
  }

  xercesc::DOMElement* world_sight_vector_node = get_node<DOMElement>(node, "WORLD_SIGHT_VECTOR");
  std::string world_sight_vector_txt(XMLString::transcode(world_sight_vector_node->getTextContent()));
  asp::read_matrix_from_string(world_sight_vector_txt, m_world_sight_mat);
  if (m_lattice_mat.empty()   || m_world_sight_mat.empty()  ||
      m_lattice_mat.size()    != m_world_sight_mat.size()   ||
      m_lattice_mat[0].size() != m_world_sight_mat[0].size() ) {
    vw_throw( ArgumentErr() << "Inconsistent lattice point and world sight vector information.\n");
  }

  std::vector< std::vector<vw::Vector3> > sat_pos_mat;
  xercesc::DOMElement* sat_pos_node = get_node<DOMElement>(node, "SAT_POS");
  std::string sat_pos_txt( XMLString::transcode(sat_pos_node->getTextContent()) );
  asp::read_matrix_from_string(sat_pos_txt, sat_pos_mat);
  for (size_t row = 0; row < sat_pos_mat.size(); row++) {
    for (size_t col = 0; col < sat_pos_mat[row].size(); col++) {
      //std::cout << sat_pos_mat[row][col] << " ";
    }
    //std::cout << std::endl;
  }
  m_sat_pos = sat_pos_mat[0]; // Go from matrix with one row to a vector

  if (m_sat_pos.size() != m_sight_mat.size()) 
    vw_throw( ArgumentErr() << "Inconsistent satellite position and sight vector information.\n");
  if (m_sat_pos.size() != m_world_sight_mat.size()) 
    vw_throw( ArgumentErr() << "Inconsistent satellite position and world "
	      << "sight vector information.\n");

  xercesc::DOMElement* cols_node = get_node<DOMElement>(node, "IMAGE_COLS");
  std::string cols_txt( XMLString::transcode(cols_node->getTextContent()) );
  int cols = atoi(cols_txt.c_str());
  
  xercesc::DOMElement* rows_node = get_node<DOMElement>(node, "IMAGE_ROWS");
  std::string rows_txt( XMLString::transcode(rows_node->getTextContent()) );
  int rows = atoi(rows_txt.c_str());

  m_image_size = vw::Vector2(cols, rows);
}

} // end namespace asp



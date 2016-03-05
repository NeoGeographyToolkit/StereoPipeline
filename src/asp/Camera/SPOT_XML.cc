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

#include <asp/Camera/XMLBase.h>
#include <asp/Camera/SPOT_XML.h>
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

using namespace vw;
using namespace vw::cartography;
using namespace xercesc;

using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

namespace asp {

//========================================================================
// ImageXML class





/*
  Load the following elements from the .DIM file:
  - The four corner locations at the beginning of the file (<Dataset_Frame>)
  - Datum information (<Coordinate_Reference_System>)
  - Image info (<Image_Display>)
  - Date/time (<Scene_Source>)
  - Image size (<Raster_Dimensions>)
  - <Ephemeris> data:  ?
    - (GCC location / GCC velocity / time) 
  - DORIS points?  
  - <Satellite_Attitudes> ?
    - (time / yaw / pitch / roll)
  - Ignore angular speeds.
  - <Quaternion_List> ?
    - (time / Q0 / Q1 / Q2 / Q3)
  - <Corrected_Attitudes> ?
    - (time / yaw / pitch / roll)
  - <Instrument_Look_Angles_List> (one per sample)
    - (ID / PSI_X / PSI_Y) -> X is line angle, Y is sample angle.
    - Going to need some extra Linescan code to implement this.
  - Ignore dark current

  Also need a raw data loader, gdal can't load the SPOT data.
*/





void SpotXML::read_xml(std::string const& xml_path) {

  // Check if the file actually exists and throw a user helpful file.
  if ( !boost::filesystem::exists( xml_path ) )
    vw_throw( ArgumentErr() << "XML file \"" << xml_path << "\" does not exist." );

  std::string error_prefix = "XML file \"" + xml_path + "\" is invalid.\nException message is: \n";
  std::string err_message  = ""; // Filled in later on error

  try{
    //std::cout << "Set XML parser\n";
  
    // Set up the XML parser
    boost::scoped_ptr<XercesDOMParser> parser( new XercesDOMParser() );
    parser->setValidationScheme(XercesDOMParser::Val_Always);
    parser->setDoNamespaces(true);
    boost::scoped_ptr<ErrorHandler> errHandler( new HandlerBase() );
    parser->setErrorHandler(errHandler.get());

    //std::cout << "Load XML\n";

    // Load the XML file
    parser->parse( xml_path.c_str() );
    DOMDocument* xmlDoc      = parser->getDocument();
    DOMElement * elementRoot = xmlDoc->getDocumentElement();

    // Use the parser function to walk through the XML tree
    return parse_xml(elementRoot);

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
  
}

void SpotXML::parse_xml(xercesc::DOMElement* node) {

  //std::cout << "Find dataset\n";
  xercesc::DOMElement* dataset_frame_node       = get_node<DOMElement>(node, "Dataset_Frame"); 
  //xercesc::DOMElement* crs_node                 get_node<DOMElement>(node, "Coordinate_Reference_System");
  //xercesc::DOMElement* image_display_node       get_node<DOMElement>(node, "Image_Display");
  //xercesc::DOMElement* scene_source_node        get_node<DOMElement>(node, "Scene_Source");
  //std::cout << "Find dims\n";
  xercesc::DOMElement* raster_dims_node         = get_node<DOMElement>(node, "Raster_Dimensions");
  //std::cout << "Find ephem\n";
  xercesc::DOMElement* corrected_attitudes_node = get_node<DOMElement>(node, "Corrected_Attitudes");
  //std::cout << "Find angles\n";
  xercesc::DOMElement* look_angles_node         = get_node<DOMElement>(node, "Instrument_Look_Angles_List");
  xercesc::DOMElement* sensor_config_node       = get_node<DOMElement>(node, "Sensor_Configuration");

  //std::cout << "Parse dataset\n";
  read_corners(dataset_frame_node);
  //read_datum(crs_node);
  //read_display_info(image_display_node);
  //read_datetime(scene_source_node);
  //std::cout << "Parse dims\n";
  read_image_size(raster_dims_node);
  //std::cout << "Parse ephem\n";
  read_ephemeris(corrected_attitudes_node);
  //std::cout << "Parse angles\n";
  read_look_angles(look_angles_node);
  //std::cout << "Parse line times\n";
  read_line_times(sensor_config_node);
}

//void read_datum(xercesc::DOMElement* crs_node) {
// TODO  
//}

void SpotXML::read_look_angles(xercesc::DOMElement* look_angles_node) {

  // Set up the data storage
  const size_t num_cols = image_size.x();
  if (num_cols == 0)
    vw_throw(ArgumentErr() << "Did not load image size from SPOT XML file!\n");
  look_angles.resize(image_size.x());

  // Dig two levels down
  xercesc::DOMElement* look_angle_node
    = get_node<DOMElement>(look_angles_node, "Instrument_Look_Angles");
  xercesc::DOMElement* look_angle_list_node
    = get_node<DOMElement>(look_angle_node, "Look_Angles_List");

  // Pick out the "Angles" nodes
  DOMNodeList* children = look_angle_list_node->getChildNodes();
  size_t index = 0;
  for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
    // Check child node type
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() != DOMNode::ELEMENT_NODE )
      continue;

    // Check the node time
    DOMElement* curr_element = dynamic_cast<DOMElement*>( curr_node );
    std::string tag( XMLString::transcode(curr_element->getTagName()) );
    if (tag.find("Look_Angles") == std::string::npos)
      continue;

    if (index >= num_cols)
      vw_throw(ArgumentErr() << "More look angles than rows in SPOT XML file!\n");

    // Record the values  
    cast_xmlch( get_node<DOMElement>(curr_element, "DETECTOR_ID")->getTextContent(), look_angles[index].first );
    cast_xmlch( get_node<DOMElement>(curr_element, "PSI_X"      )->getTextContent(), look_angles[index].second.x() );
    cast_xmlch( get_node<DOMElement>(curr_element, "PSI_Y"      )->getTextContent(), look_angles[index].second.y() );
    ++index;

  } // End loop through corrected attitudes
}


void SpotXML::read_ephemeris(xercesc::DOMElement* corrected_attitudes_node) {

  pose_logs.clear(); // Reset data storage

  // Dig one level down
  xercesc::DOMElement* corrected_attitude_node
    = get_node<DOMElement>(corrected_attitudes_node, "Corrected_Attitude");

  // Pick out the "Angles" nodes
  DOMNodeList* children = corrected_attitude_node->getChildNodes();
  for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
    // Check child node type
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() != DOMNode::ELEMENT_NODE )
      continue;

    // Check the node time
    DOMElement* curr_element = dynamic_cast<DOMElement*>( curr_node );
    std::string tag( XMLString::transcode(curr_element->getTagName()) );
    if (tag.find("Angles") == std::string::npos)
      continue;
  
    std::pair<std::string, Vector3> data;
    cast_xmlch( get_node<DOMElement>(curr_element, "YAW"  )->getTextContent(), data.second.x() );
    cast_xmlch( get_node<DOMElement>(curr_element, "PITCH")->getTextContent(), data.second.y() );
    cast_xmlch( get_node<DOMElement>(curr_element, "ROLL" )->getTextContent(), data.second.z() );
    cast_xmlch( get_node<DOMElement>(curr_element, "TIME" )->getTextContent(), data.first );
    pose_logs.push_back(data);

  } // End loop through corrected attitudes
}


void SpotXML::read_corners(xercesc::DOMElement* dataset_frame_node) {

  // Set up storage
  const size_t NUM_CORNERS = 4;
  lonlat_corners.resize(NUM_CORNERS);
  pixel_corners.resize(NUM_CORNERS);

  // Look through the four vertex nodes
  DOMNodeList* children = dataset_frame_node->getChildNodes();
  size_t count = 0;
  for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
    // Check child node type
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() != DOMNode::ELEMENT_NODE )
      continue;

    // Check the node time
    DOMElement* curr_element = dynamic_cast<DOMElement*>( curr_node );

    // There should only be four vertex nodes here.
    // Parse the values.
    XmlUtils::cast_xmlch(XmlUtils::get_node<DOMElement>(curr_element, "FRAME_LON")->getTextContent(), lonlat_corners[count].x());
    XmlUtils::cast_xmlch(XmlUtils::get_node<DOMElement>(curr_element, "FRAME_LAT")->getTextContent(), lonlat_corners[count].y());
    XmlUtils::cast_xmlch(XmlUtils::get_node<DOMElement>(curr_element, "FRAME_ROW")->getTextContent(), pixel_corners[count].y());
    XmlUtils::cast_xmlch(XmlUtils::get_node<DOMElement>(curr_element, "FRAME_COL")->getTextContent(), pixel_corners[count].x());
    lonlat_corners[count].z() = 0;
    ++count;
    if (count == NUM_CORNERS)
      return;

  } // End loop through the vertex nodes
}


void SpotXML::read_image_size(xercesc::DOMElement* raster_dims_node) {
  cast_xmlch( get_node<DOMElement>(raster_dims_node, "NROWS")->getTextContent(), image_size[1] );
  cast_xmlch( get_node<DOMElement>(raster_dims_node, "NCOLS")->getTextContent(), image_size[0] );
}

void SpotXML::read_line_times(xercesc::DOMElement* sensor_config_node) {
  cast_xmlch( get_node<DOMElement>(sensor_config_node, "LINE_PERIOD"      )->getTextContent(), line_period);
  cast_xmlch( get_node<DOMElement>(sensor_config_node, "SCENE_CENTER_TIME")->getTextContent(), center_time);
  cast_xmlch( get_node<DOMElement>(sensor_config_node, "SCENE_CENTER_LINE")->getTextContent(), center_line);
  cast_xmlch( get_node<DOMElement>(sensor_config_node, "SCENE_CENTER_COL" )->getTextContent(), center_col);
}




} // end namespace asp



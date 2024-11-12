// __BEGIN_LICENSE__
//  Copyright (c) 2009-2024, United States Government as represented by the
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

// Utilties for parsing PRISM .DIMA files in XML format.

#include <vw/Core/Exception.h>          // for ArgumentErr, vw_throw, etc
#include <vw/Math/Quaternion.h>         // for Quat, Quaternion
#include <vw/Math/Vector.h>             // for Vector, Vector3, Vector4, etc

#include <asp/Camera/XMLBase.h>
#include <asp/Camera/PRISM_XML.h>
#include <asp/Camera/TimeProcessing.h>

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

using namespace xercesc;

using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

namespace asp {

using namespace xercesc;
using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

// Parse the prifle (satellite id)
void parse_profile(DOMElement* root, std::string & profile) {
  DOMElement* id_node = get_node<DOMElement>(root, "Metadata_Id");
  DOMElement* profile_node = get_node<DOMElement>(id_node, "METADATA_PROFILE");
  cast_xmlch(profile_node->getTextContent(), profile);
}

// Read the first and last image line times
void read_first_last_line_times(xercesc::DOMElement * data_node, 
                                double              & first_line_time, 
                                double              & last_line_time) {

  DOMElement* time_node = get_node<DOMElement>(data_node, "Satellite_Time");

  // First line time
  DOMElement* first_line_time_node = get_node<DOMElement>(time_node, "TIME_FIRST_LINE");
  std::string first_line_time_str, last_line_time_str;
  cast_xmlch(first_line_time_node->getTextContent(), first_line_time_str);

  // Last line time
  DOMElement* last_line_time_node = get_node<DOMElement>(time_node, "TIME_LAST_LINE");
  cast_xmlch(last_line_time_node->getTextContent(), last_line_time_str);

  first_line_time = asp::to_epoch(asp::parse_time(first_line_time_str));
  last_line_time  = asp::to_epoch(asp::parse_time(last_line_time_str));
}

// This function simplifies the logic when we know the info we need is surely in
// the first child.
DOMElement* getFirstChildByTagName(DOMElement* node, const std::string & tag) {
  
  DOMNodeList* children = node->getChildNodes();
  for (XMLSize_t i = 0; i < children->getLength(); i++) {
    DOMNode* child = children->item(i);
    std::string curr_tag(XMLString::transcode(child->getNodeName()));
    
    if (curr_tag == tag)
      return dynamic_cast<DOMElement*>(child);
  }
  
  return NULL;
}

// Read the dimensions of each of the raw image blocks.
void read_ccd_dims(xercesc::DOMElement * root, 
                   int & ncols, int & nrows) {

  // Iinitialize the output variables
  ncols = -1;
  nrows = -1;
  
  DOMElement* dataset_sources = get_node<DOMElement>(root, "Dataset_Sources");
  
  // There  must be at least one child
  DOMNodeList* children = dataset_sources->getChildNodes();
  if (children->getLength() < 1) 
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting at least one child in the Dataset_Sources node.\n");

  // Iterate over the children   
  for (XMLSize_t i = 0; i < children->getLength(); i++) {
 
    DOMNode* child = children->item(i);
    
    // Get the name if this child
    std::string tag(XMLString::transcode(child->getNodeName()));
    if (tag != "Source_Information")
      continue;
          
    DOMElement* source_info = dynamic_cast<DOMElement*>(child);

    DOMElement* scene_source = get_node<DOMElement>(source_info, "Scene_Source");
    if (scene_source == NULL) 
      continue;
    
    // Get the Image_Interpretation subnode
    DOMElement* image_interpretation = get_node<DOMElement>(scene_source, 
                                                            "Image_Interpretation");
    if (image_interpretation == NULL) 
      continue;
    
    // Get the Spectral_Band_Info subnode
    DOMElement* spectral_band_info = getFirstChildByTagName(image_interpretation, 
                                                          "Spectral_Band_Info");
    if (spectral_band_info == NULL) 
      continue;
    
    std::string cols_str, rows_str;
    cast_xmlch(get_node<DOMElement>(spectral_band_info, "NCOLS")->getTextContent(), cols_str);
    cast_xmlch(get_node<DOMElement>(spectral_band_info, "NROWS")->getTextContent(), rows_str);
    
    ncols = atoi(cols_str.c_str());
    nrows = atoi(rows_str.c_str());
    
    // Found the cols and rows
    break;        
  }
  
  // Check cols and rows are positive. Otherwise they were not read correctly.
  if (ncols <= 0 || nrows <= 0) 
    vw::vw_throw(vw::ArgumentErr() 
                 << "Failed to read the CCD blocl dimensions from the XML file.\n");
}

// Read the camera position and velocities (ephemeris)
void read_ephemeris(xercesc::DOMElement      * data_node,
                    std::vector<vw::Vector3> & positions,
                    std::vector<vw::Vector3> & velocities,
                    std::vector<double>      & position_times) {

  // Wipe the output vectors
  positions.clear(); 
  velocities.clear();
  position_times.clear();

  xercesc::DOMElement* ephemeris = get_node<DOMElement>(data_node, "Ephemeris");
  xercesc::DOMElement* point_list = get_node<DOMElement>(ephemeris, "Points");
  DOMNodeList* children = point_list->getChildNodes();

  for (XMLSize_t i = 0; i < children->getLength(); i++) {
    
    // Check child node type
    DOMNode* child = children->item(i);
    if (child->getNodeType() != DOMNode::ELEMENT_NODE)
      continue;

    // Check the node name
    DOMElement* curr_element = dynamic_cast<DOMElement*>(child);
    std::string tag(XMLString::transcode(curr_element->getTagName()));
    if (tag.find("Point") == std::string::npos)
      continue;
    
    // Get the three sub-nodes
    std::string time_str, xs, ys, zs;
    vw::Vector3 position, velocity;

    cast_xmlch(get_node<DOMElement>(curr_element, "TIME")->getTextContent(), time_str);
    double time = asp::to_epoch(asp::parse_time(time_str));
    
    xercesc::DOMElement* location_node = get_node<DOMElement>(curr_element, "Location");
    cast_xmlch(get_node<DOMElement>(location_node, "X")->getTextContent(), xs);
    cast_xmlch(get_node<DOMElement>(location_node, "Y")->getTextContent(), ys);
    cast_xmlch(get_node<DOMElement>(location_node, "Z")->getTextContent(), zs);
    position = vw::Vector3(atof(xs.c_str()), atof(ys.c_str()), atof(zs.c_str()));
    
    // same for velocity
    xercesc::DOMElement* velocity_node = get_node<DOMElement>(curr_element, "Velocity");
    cast_xmlch(get_node<DOMElement>(velocity_node, "X")->getTextContent(), xs);
    cast_xmlch(get_node<DOMElement>(velocity_node, "Y")->getTextContent(), ys);
    cast_xmlch(get_node<DOMElement>(velocity_node, "Z")->getTextContent(), zs);
    velocity = vw::Vector3(atof(xs.c_str()), atof(ys.c_str()), atof(zs.c_str()));
    
    positions.push_back(position);
    velocities.push_back(velocity);
    position_times.push_back(time);
  } // End loop through points
}

// Read the angles (roll, pitch, yaw) of the camera
void read_rlh(xercesc::DOMElement      * data_node,
              std::vector<vw::Vector3> & rph,
              std::vector<double>      & rph_times) {

  // Wipe the output vectors
  rph.clear(); 
  rph_times.clear();

  xercesc::DOMElement* sat_att = get_node<DOMElement>(data_node, "Satellite_Attitudes");
  xercesc::DOMElement* angles_list = get_node<DOMElement>(sat_att, "Angles_List");

  DOMNodeList* children = angles_list->getChildNodes();
  
  for (XMLSize_t i = 0; i < children->getLength(); i++) {
    
    // Check child node type
    DOMNode* child = children->item(i);
    if (child->getNodeType() != DOMNode::ELEMENT_NODE)
      continue;

    // Check the node name
    DOMElement* curr_element = dynamic_cast<DOMElement*>(child);
    std::string tag(XMLString::transcode(curr_element->getTagName()));
    if (tag.find("Angles") == std::string::npos)
      continue;
    
    // Get the three sub-nodes
    std::string time_str, ys, ps, rs;
    vw::Vector3 vals;

    cast_xmlch(get_node<DOMElement>(curr_element, "TIME")->getTextContent(), time_str);
    double time = asp::to_epoch(asp::parse_time(time_str));

    // Find the angle subnode
    xercesc::DOMElement* angle_node = get_node<DOMElement>(curr_element, "Angle");
    cast_xmlch(get_node<DOMElement>(angle_node, "YAW")->getTextContent(), ys);
    cast_xmlch(get_node<DOMElement>(angle_node, "PITCH")->getTextContent(), ps);
    cast_xmlch(get_node<DOMElement>(angle_node, "ROLL")->getTextContent(), rs);
    vals = vw::Vector3(atof(ys.c_str()), atof(ps.c_str()), atof(rs.c_str()));
    
    rph.push_back(vals);
    rph_times.push_back(time);
  } // End loop through points
}

void parsePrismXml(std::string const& dim_file,
                   int & ncols, int & nrows,
                   double & first_line_time, double & last_line_time,
                   std::vector<vw::Vector3> & positions,
                   std::vector<vw::Vector3> & velocities,
                   std::vector<double> & position_times,
                   std::vector<vw::Vector3> & rph,
                   std::vector<double> & rph_times) {

  // Initalize the output variables
  ncols = -1; 
  nrows = -1;
  first_line_time = -1.0;
  last_line_time = -1.0;
  positions.clear();
  velocities.clear();
  position_times.clear();
  rph.clear();
  rph_times.clear();

  boost::scoped_ptr<XercesDOMParser> parser(new XercesDOMParser());
  parser->setValidationScheme(XercesDOMParser::Val_Always);
  parser->setDoNamespaces(true);
  boost::scoped_ptr<ErrorHandler> errHandler(new HandlerBase());
  parser->setErrorHandler(errHandler.get());

  DOMDocument* xmlDoc = NULL;
  DOMElement* root = NULL;

  try {
    parser->parse(dim_file.c_str());
    xmlDoc = parser->getDocument();
    root = xmlDoc->getDocumentElement();
  } catch(...) {
    vw::vw_throw(vw::ArgumentErr() << "Faile to parse XML file: " << dim_file << "\n");
  }

  // Parse the satellite type
  std::string profile; 
  parse_profile(root, profile);
  if (profile != "ALOS")
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting the value of METADATA_PROFILE to be ALOS.\n");

  // Parse the number of cols and rows in each raw ccd block
  read_ccd_dims(root, ncols, nrows);

  // Parse the first and last line times
  DOMElement* data_node = get_node<DOMElement>(root, "Data_Strip");
  read_first_last_line_times(data_node, first_line_time, last_line_time);
  
  // Parse the camera position and velocities (ephemeris)
  read_ephemeris(data_node, positions, velocities, position_times);

  // TODO(oalexan1): Must fill in an additional position based on the last
  // position and velocity, if not enough samples, as otherwise cannot
  // interpolate in time at the last line.  
  
  // Print each velocity 
  for (size_t i = 0; i < velocities.size(); i++)
    std::cout << "Velocity: " << velocities[i]/vw::math::norm_2(velocities[i]) << "\n";
  
  // Print each position minus the previous position
  for (size_t i = 1; i < positions.size(); i++) 
    std::cout << "Position diff: " << (positions[i] - positions[i-1])/vw::math::norm_2(positions[i] - positions[i-1]) << "\n";
 
  // print the norm of each velocity
  for (size_t i = 0; i < velocities.size(); i++) 
    std::cout << "Velocity norm: " << norm_2(velocities[i]) << "\n";
    
  // Read roll-pitch-yaw angles
  read_rlh(data_node, rph, rph_times);

  return;
}

} // end namespace asp



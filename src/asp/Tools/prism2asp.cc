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

// Parse PRISM data and produce CSM camera files

#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Camera/LinescanUtils.h>
#include <asp/Camera/CsmUtils.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Sessions/CameraUtils.h>
#include <asp/Core/BitChecker.h>
#include <asp/Camera/XMLBase.h>
#include <asp/Camera/TimeProcessing.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// XML parsing
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <limits>
#include <cstring>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace xercesc;
using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;


struct Options: public vw::GdalWriteOptions {
  std::string dim_file, csm_file;
  
  // Constructor
  Options() {}
};

// Return the time in seconds since the epoch, down to the microsecond
double to_epoch(const boost::posix_time::ptime& pt) {
  boost::posix_time::ptime epoch(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::time_duration diff = pt - epoch;
  return diff.total_microseconds() / 1.0e+6;
}

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

  first_line_time = to_epoch(asp::parse_time(first_line_time_str));
  last_line_time  = to_epoch(asp::parse_time(last_line_time_str));
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
    double time = to_epoch(asp::parse_time(time_str));
    
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
    double time = to_epoch(asp::parse_time(time_str));

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

void parseXML(std::string const& dim_file) {

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

  DOMElement* data_node = get_node<DOMElement>(root, "Data_Strip");
  
  // Parse the first and last line times
  double first_line_time = -1.0, last_line_time = -1.0;
  read_first_last_line_times(data_node, first_line_time, last_line_time);
  
  // Parse the camera position and velocities (ephemeris)
  std::vector<vw::Vector3> positions, velocities;
  std::vector<double> position_times;
  read_ephemeris(data_node, positions, velocities, position_times);
  
  // Read roll-pitch-yaw angles
  std::vector<vw::Vector3> rph;
  std::vector<double> rph_times;
  read_rlh(data_node, rph, rph_times);
}

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("");
  general_options.add_options()
    ("dim", po::value(&opt.dim_file)->default_value(""),
     "The input PRISM .DIMA file.")
    ("csm", po::value(&opt.csm_file)->default_value(""),
     "The output CSM camera file.")
  ;
  
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // The dim file is required
  if (opt.dim_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the input .DIMA file.\n" 
                 << usage << general_options);

  // The output file is required
  if (opt.csm_file.empty())
    vw::vw_throw(vw::ArgumentErr() << "Missing the output CSM file.\n");
      
  // TODO(oalexan1): Add logic to log to file     

} // End function handle_arguments

int main(int argc, char * argv[]) {
    
  Options opt;
  try {
    
    // Mandatory initialization for Xerces
    xercesc::XMLPlatformUtils::Initialize();
    
    handle_arguments(argc, argv, opt);
    parseXML(opt.dim_file);
    
    xercesc::XMLPlatformUtils::Terminate();
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

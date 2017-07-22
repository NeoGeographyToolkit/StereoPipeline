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
#include <iomanip>

using namespace vw;
using namespace vw::cartography;
using namespace xercesc;

using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

namespace asp {


DOMElement* SpotXML::open_xml_file(std::string const& xml_path) {

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

void SpotXML::read_xml(std::string const& xml_path) {

  DOMElement * elementRoot = open_xml_file(xml_path);
  parse_xml(elementRoot);
}

BBox2 SpotXML::get_estimated_bounds(std::string const& xml_path) {
  SpotXML xml_reader;
  DOMElement * root = xml_reader.open_xml_file(xml_path);
  // Just get this one node we need to find the four corners
  DOMElement* raster_dims_node = get_node<DOMElement>(root, "Dataset_Frame");
  xml_reader.read_corners(raster_dims_node);
  
  return xml_reader.get_estimated_bounds();
}

std::vector<vw::Vector2> SpotXML::get_lonlat_corners(std::string const& xml_path) {
  SpotXML xml_reader;
  DOMElement * root = xml_reader.open_xml_file(xml_path);
  // Just get this one node we need to find the four corners
  DOMElement* raster_dims_node = get_node<DOMElement>(root, "Dataset_Frame");
  xml_reader.read_corners(raster_dims_node);
  return xml_reader.lonlat_corners;
}

void SpotXML::parse_xml(xercesc::DOMElement* node) {

  //std::cout << "Find dataset\n";
  xercesc::DOMElement* dataset_frame_node       = get_node<DOMElement>(node, "Dataset_Frame"); 
  //xercesc::DOMElement* crs_node                 get_node<DOMElement>(node, "Coordinate_Reference_System");
  //xercesc::DOMElement* image_display_node       get_node<DOMElement>(node, "Image_Display");
  //xercesc::DOMElement* scene_source_node        get_node<DOMElement>(node, "Scene_Source");
  //std::cout << "Find dims\n";
  xercesc::DOMElement* raster_dims_node         = get_node<DOMElement>(node, "Raster_Dimensions");
  xercesc::DOMElement* ephemeris_node           = get_node<DOMElement>(node, "Ephemeris");
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
  read_ephemeris(ephemeris_node);
  read_image_size(raster_dims_node);
  //std::cout << "Parse ephem\n";
  read_attitude(corrected_attitudes_node);
  //std::cout << "Parse angles\n";
  read_look_angles(look_angles_node);
  //std::cout << "Parse line times\n";
  read_line_times(sensor_config_node);
  
  // Set up the base time
  // - The position log starts before the image does, so the first
  //   time there should be a good reference time.
  boost::posix_time::ptime earliest_time = boost::posix_time::time_from_string("2016-05-04 00:00:00.00");
  std::list<std::pair<std::string, vw::Vector3> >::const_iterator iter;
  for (iter=position_logs.begin(); iter!=position_logs.end(); ++iter) {
    std::string s = iter->first;
    boost::replace_all(s, "T", " ");
    boost::posix_time::ptime this_time = boost::posix_time::time_from_string(s);
    if (this_time < earliest_time){
      earliest_time = this_time;
      //std::cout << "Using reference time " << iter->first << std::endl;
    }
  }
  m_time_ref_functor.set_base_time(earliest_time);
  //std::cout << "Done parsing XML.\n";
}


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
  const XMLSize_t num_children = children->getLength();
  for ( XMLSize_t i = 0; i < num_children; ++i ) {
    // Check child node type
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() != DOMNode::ELEMENT_NODE )
      continue;

    // Check the node name
    DOMElement* curr_element = dynamic_cast<DOMElement*>( curr_node );

    if (index >= num_cols)
      vw_throw(ArgumentErr() << "More look angles than rows in SPOT XML file!\n");

    // Look through the three nodes and assign each of them
    // - In this function we do this a little more by hand to try and speed things up
    DOMNodeList* sub_children = curr_element->getChildNodes();
    for ( XMLSize_t j = 0; j < sub_children->getLength(); ++j ) {

      DOMNode* child_node = sub_children->item(j);
      if ( child_node->getNodeType() != DOMNode::ELEMENT_NODE )
        continue;
      DOMElement* child_element = dynamic_cast<DOMElement*>( child_node );
      std::string tag2( XMLString::transcode(child_element->getTagName()) );
      std::string text( XMLString::transcode(child_element->getTextContent()) );

      if (tag2 == "DETECTOR_ID")
        look_angles[index].first = atoi(text.c_str());
      if (tag2 == "PSI_X")
        look_angles[index].second.x() = atof(text.c_str());
      if (tag2 == "PSI_Y")
        look_angles[index].second.y() = atof(text.c_str());
    }

    ++index;

  } // End loop through look angles
  if (index != num_cols)
    vw_throw(ArgumentErr() << "Did not load the correct number of SPOT5 pixel look angles!\n");
}

void SpotXML::read_ephemeris(xercesc::DOMElement* ephemeris_node) {

  position_logs.clear(); // Reset data storage
  velocity_logs.clear();

  // Dig one level down
  xercesc::DOMElement* points_node = get_node<DOMElement>(ephemeris_node, "Points");

  // Pick out the "Point" nodes
  DOMNodeList* children = points_node->getChildNodes();
  for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
    // Check child node type
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() != DOMNode::ELEMENT_NODE )
      continue;

    // Check the node name
    DOMElement* curr_element = dynamic_cast<DOMElement*>( curr_node );
    std::string tag( XMLString::transcode(curr_element->getTagName()) );
    if (tag.find("Point") == std::string::npos)
      continue;

    // Get the three sub-nodes
    xercesc::DOMElement* location_node = get_node<DOMElement>(curr_element, "Location");
    xercesc::DOMElement* velocity_node = get_node<DOMElement>(curr_element, "Velocity");
  
    // Read in both sets of values
    std::string time;
    Vector3 position, velocity;
    
    cast_xmlch( get_node<DOMElement>(curr_element, "TIME" )->getTextContent(), time );
    cast_xmlch( get_node<DOMElement>(location_node, "X")->getTextContent(), position.x() );
    cast_xmlch( get_node<DOMElement>(location_node, "Y")->getTextContent(), position.y() );
    cast_xmlch( get_node<DOMElement>(location_node, "Z")->getTextContent(), position.z() );
    cast_xmlch( get_node<DOMElement>(velocity_node, "X")->getTextContent(), velocity.x() );
    cast_xmlch( get_node<DOMElement>(velocity_node, "Y")->getTextContent(), velocity.y() );
    cast_xmlch( get_node<DOMElement>(velocity_node, "Z")->getTextContent(), velocity.z() );
    
    position_logs.push_back(std::pair<std::string, Vector3>(time, position));
    velocity_logs.push_back(std::pair<std::string, Vector3>(time, velocity));

  } // End loop through corrected attitudes
}



void SpotXML::read_attitude(xercesc::DOMElement* corrected_attitudes_node) {

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
  // - Currently we assume they are always in the same order!
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
  center_line -= 1;
  center_col  -= 1; // Convert from 1-based to 0-based indices.
}





// ----- These functions help convert the input data to a useable format ------

vw::BBox2 SpotXML::get_estimated_bounds() const {
  // Just expand a bounding box to contain all the corners listed in the file.
  vw::BBox2 output;
  for (size_t i=0; i<lonlat_corners.size(); ++i) {
    output.grow(lonlat_corners[i]);
  }
  return output;
}


// Input strings look like this: 2008-03-04T12:31:03.081912
double SpotXML::convert_time(std::string const& s) const {
  try{
    // Replace the T with a space so the default Boost function can parse the time.
    std::string s2 = s;
    boost::replace_all(s2, "T", " ");
    boost::posix_time::ptime time = boost::posix_time::time_from_string(s2);
    return this->m_time_ref_functor(time);
  }catch(...){
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << s << "\n");
  }
  return -1; // Never reached!
}

// This is pretty simple, SPOT5 has a constant time for each line.
vw::camera::LinearTimeInterpolation SpotXML::setup_time_func() const {

  // The metadata tells us the time of the middle line, so find the time for the first line.
  double center_time_d = convert_time(this->center_time);
  double min_line_diff = static_cast<double>(0 - this->center_line);
  double min_line_time = center_time_d + this->line_period*min_line_diff;
  //std::cout << "Setup time functor: " << std::setprecision(12)  << min_line_time << ", " << this->line_period << std::endl;
  //std::cout << std::setprecision(12)  << "Center time: " << center_time_d << std::endl;
  return vw::camera::LinearTimeInterpolation(min_line_time, this->line_period);
}


// Velocities are the sum of inertial velocities and the instantaneous
//  Earth rotation.

// The velocity is already in GCC, so just pack into a function.
vw::camera::LagrangianInterpolation SpotXML::setup_velocity_func() const {

  const int INTERP_RADII = 4; // Reccomended in the docs
  std::vector<double>  time;
  std::vector<Vector3> velocity;

  // Loop through the velocity logs and extract values
  std::list<std::pair<std::string, vw::Vector3> >::const_iterator iter;
  for (iter=velocity_logs.begin(); iter!=velocity_logs.end(); ++iter) {
    time.push_back(convert_time(iter->first));
    velocity.push_back(iter->second);
    //std::cout << "Adding velocity point: " << iter->first 
    //          << " --> " << iter->second << std::endl;
  }
  
  // More generic method for variable time intervals
  //return vw::camera::LagrangianInterpolationVarTime(velocity, time, INTERP_RADII);
  
  // A faster method for when we know the time delta is constant
  double min_time   = time[0];
  double max_time   = time[time.size()-1];
  double time_delta = (max_time - min_time) / (time.size()-1);
  return vw::camera::LagrangianInterpolation(velocity, min_time, time_delta, max_time, INTERP_RADII);
}

// The position is already in GCC, so just pack into a function.
// - Currently this is identical to the velocity function, but this may change later.
vw::camera::LagrangianInterpolation SpotXML::setup_position_func() const {

 const int INTERP_RADII = 4; // Reccomended in the docs
  std::vector<double>  time;
  std::vector<Vector3> position;

  // Loop through the velocity logs and extract values
  std::list<std::pair<std::string, vw::Vector3> >::const_iterator iter;
  for (iter=position_logs.begin(); iter!=position_logs.end(); ++iter) {
    time.push_back(convert_time(iter->first));
    position.push_back(iter->second);
    //std::cout << "Adding position point: " << convert_time(iter->first)
    //          << " --> " << iter->second << std::endl;
  }
  
  // More generic method for variable time intervals
  //return vw::camera::LagrangianInterpolationVarTime(position, time, INTERP_RADII);
  
  // A faster method for when we know the time delta is constant
  double min_time   = time[0];
  double max_time   = time[time.size()-1];
  double time_delta = (max_time - min_time) / (time.size()-1);
  return vw::camera::LagrangianInterpolation(position, min_time, time_delta, max_time, INTERP_RADII);
}

vw::camera::LinearPiecewisePositionInterpolation SpotXML::setup_pose_func(
        vw::camera::LinearTimeInterpolation const& time_func) const {

  // This function returns a functor that returns just the yaw/pitch/roll angles.
  // - The time interval between lines is not constant but it is extremely close.


  // For some reason the corrected pose angles do not start early enough to cover
  // the time span for all of the input lines!
  // - In order to handle this, we repeat the earliest pose value so that it starts
  //   before the first line.
  // - The raw pose angles do start before the lines, but their values differ noticably
  //   from the corrected values.
  
  // Compute how many padded pose entries are needed to cover all of the lines.
  size_t num_lines           = this->image_size[1];
  double num_corrected_poses = static_cast<double>(pose_logs.size());
  double first_line_time     = time_func(0);
  double last_line_time      = time_func(num_lines - 1.0);
  double pose_start_time     = convert_time(pose_logs.front().first);
  double pose_stop_time      = convert_time(pose_logs.back().first);
  double pose_delta_t        = (pose_stop_time - pose_start_time) / (num_corrected_poses - 1.0);
  int    num_prefill_poses   = static_cast<int>(ceil((pose_start_time - first_line_time) / pose_delta_t));
  int    num_postfill_poses  = static_cast<int>(ceil((last_line_time  - pose_stop_time ) / pose_delta_t));
  //std::cout << "First line time: " << first_line_time << std::endl;
  //std::cout << "Last line time:  " << last_line_time  << std::endl;
  //std::cout << "Pose start: " << pose_start_time << std::endl;
  //std::cout << "Pose stop:  " << pose_stop_time  << std::endl;
  num_postfill_poses += 1; // Stick another bit of padding on the back.
                           // This is so our Extrinsics algorithms have enough room to interpolate.
  if (num_prefill_poses < 1)
    num_prefill_poses = 0;
  if (num_postfill_poses < 1)
    num_postfill_poses = 0;
    
  size_t num_total_poses = pose_logs.size() + static_cast<size_t>(num_prefill_poses )
                                            + static_cast<size_t>(num_postfill_poses);

  std::vector<Vector3> pose(num_total_poses);
  std::vector<double>  time(num_total_poses);
  
  // Fill in the pre-padding poses
  size_t index = 0;
  for (int i=0; i<num_prefill_poses; ++i) {
    double time_offset = pose_delta_t*static_cast<double>(num_prefill_poses-i);
    time[index] = convert_time(pose_logs.front().first) - time_offset;
    pose[index] = pose_logs.front().second;
    //std::cout << "PREFILL: " << time[index] << std::endl;
    ++index;
  }

  // Now fill in the real poses
  std::list<std::pair<std::string, vw::Vector3> >::const_iterator iter;
  for (iter=pose_logs.begin(); iter!=pose_logs.end(); ++iter) {
    time[index] = convert_time(iter->first);
    pose[index] = iter->second;
    ++index;
  }

  // Fill in the post-padding poses
  for (int i=0; i<num_postfill_poses; ++i) {
    double time_offset = pose_delta_t*(i+1);
    time[index] = convert_time(pose_logs.back().first) + time_offset;
    pose[index] = pose_logs.back().second;
    //std::cout << "POSTFILL: " << time[index] << std::endl;
    ++index;
  }
  
  //double max_time = time.back();
  double min_time = time.front();
  
  //std::cout << std::setprecision(12) << "Adding pose info: " << min_time << ", " 
  //          << max_time << " -> " << pose_delta_t << std::endl;
  
  return vw::camera::LinearPiecewisePositionInterpolation(pose, min_time, pose_delta_t);

}


} // end namespace asp



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
#include <asp/Camera/PeruSatXML.h>
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

DOMElement* PeruSatXML::open_xml_file(std::string const& xml_path) {

  // Check if the file actually exists and throw a user helpful file.
  if (!boost::filesystem::exists(xml_path))
    vw_throw(ArgumentErr() << "XML file \"" << xml_path << "\" does not exist.");

  std::string error_prefix = "XML file \"" + xml_path + "\" is invalid.\nException message is: \n";
  std::string err_message  = ""; // Filled in later on error

  try{
    // Set up the XML parser if we have not already done so
    if (!m_parser.get()) {
      m_parser.reset(new XercesDOMParser());
      m_errHandler.reset(new HandlerBase());
      m_parser->setValidationScheme(XercesDOMParser::Val_Always);
      m_parser->setDoNamespaces(true);   
      m_parser->setErrorHandler(m_errHandler.get());
    }

    // Load the XML file
    m_parser->parse(xml_path.c_str());
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
  } catch (const std::exception& e) {
    err_message = error_prefix + e.what();
  } catch (...) {
    err_message = "Unrecognized error in XML file \"" + xml_path + "\"\n";
  }
  vw_throw(ArgumentErr() << err_message); // Only get here on error

  return 0;
}

void PeruSatXML::read_xml(std::string const& xml_path) {
  DOMElement * root = open_xml_file(xml_path);
  parse_xml(root);
}

void PeruSatXML::parse_xml(xercesc::DOMElement* root) {

  xercesc::DOMElement* metadata_id = get_node<DOMElement>(root, "Metadata_Identification");

  xercesc::DOMElement* metadata_profile = get_node<DOMElement>(metadata_id, "METADATA_PROFILE");

  std::string sensor_name(XMLString::transcode(metadata_profile->getTextContent()));
  std::string expected_name = "PER1_SENSOR";
  if (sensor_name != expected_name) 
    vw_throw(ArgumentErr() << "Incorrect sensor name. Expected: "
             << expected_name << " but got: " << sensor_name << ".\n");

  xercesc::DOMElement* raster_data = get_node<DOMElement>(root, "Raster_Data");
  read_image_size(raster_data);
  
  // Dig some levels down
  xercesc::DOMElement* geometric_data = get_node<DOMElement>(root, "Geometric_Data");
  xercesc::DOMElement* refined_model = get_node<DOMElement>(geometric_data, "Refined_Model");
  
  xercesc::DOMElement* time = get_node<DOMElement>(refined_model, "Time");
  read_times(time);
  
  xercesc::DOMElement* ephemeris = get_node<DOMElement>(refined_model, "Ephemeris");
  read_ephemeris(ephemeris);
  
  xercesc::DOMElement* attitudes = get_node<DOMElement>(refined_model, "Attitudes");
  read_attitudes(attitudes);
  
  xercesc::DOMElement* geom_calib  = get_node<DOMElement>(refined_model, "Geometric_Calibration");
  xercesc::DOMElement* instr_calib = get_node<DOMElement>(geom_calib,    "Instrument_Calibration");
  xercesc::DOMElement* band_calib  = get_node<DOMElement>(instr_calib,   "Band_Calibration");
  xercesc::DOMElement* look_angles = get_node<DOMElement>(band_calib,    "Polynomial_Look_Angles");
  read_look_angles(look_angles);

  xercesc::DOMElement* instr_biases = get_node<DOMElement>(instr_calib,   "Instrument_Biases");
  read_instr_biases(instr_biases);
  
  xercesc::DOMElement* use_area = get_node<DOMElement>(geometric_data, "Use_Area");
  xercesc::DOMElement* geom_values = get_node<DOMElement>(use_area,
                                                          "Located_Geometric_Values");
  read_center_data(geom_values);
}

void PeruSatXML::read_image_size(xercesc::DOMElement* raster_data_node) {
  xercesc::DOMElement* raster_dims_node = get_node<DOMElement>(raster_data_node,
                                                                 "Raster_Dimensions");

  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NROWS")->getTextContent(), image_size[1]);
  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NCOLS")->getTextContent(), image_size[0]);
}

void PeruSatXML::read_times(xercesc::DOMElement* time) {
  xercesc::DOMElement* time_range = get_node<DOMElement>(time, "Time_Range");

  std::string start_time_str;
  cast_xmlch(get_node<DOMElement>(time_range, "START")->getTextContent(), start_time_str);
  bool is_start_time = true;
  start_time = PeruSatXML::convert_time(start_time_str, is_start_time);

  xercesc::DOMElement* time_stamp = get_node<DOMElement>(time, "Time_Stamp");
  cast_xmlch(get_node<DOMElement>(time_stamp, "LINE_PERIOD")->getTextContent(), line_period);
  std::cout << "--line_period " << line_period << std::endl;
  std::cout << "check if offset is correct in RPC_XML.cc!" << std::endl;
}
  
void PeruSatXML::read_ephemeris(xercesc::DOMElement* ephemeris) {

  // Reset data storage
  position_logs.clear(); 
  velocity_logs.clear();

  xercesc::DOMElement* point_list = get_node<DOMElement>(ephemeris, "Point_List");

  // Pick out the "Point" nodes
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
    std::string time_str, position_str, velocity_str;
    Vector3 position_vec, velocity_vec;
    
    cast_xmlch(get_node<DOMElement>(curr_element, "LOCATION_XYZ")->getTextContent(), position_str);
    cast_xmlch(get_node<DOMElement>(curr_element, "VELOCITY_XYZ")->getTextContent(), velocity_str);
    cast_xmlch(get_node<DOMElement>(curr_element, "TIME")->getTextContent(),         time_str);

    bool is_start_time = false;
    double time = PeruSatXML::convert_time(time_str, is_start_time);
    
    std::string delimiters(",\t ");
    position_vec = str_to_vec<Vector3>(position_str, delimiters);
    std::cout << "--position " << position_vec << std::endl;
    velocity_vec = str_to_vec<Vector3>(velocity_str, delimiters);
    std::cout << "--velocity " << velocity_vec << std::endl;
    
    position_logs.push_back(std::pair<double, Vector3>(time, position_vec));
    velocity_logs.push_back(std::pair<double, Vector3>(time, velocity_vec));
  } // End loop through points
  
}
  
void PeruSatXML::read_attitudes(xercesc::DOMElement* attitudes) {

  // Reset data storage
  pose_logs.clear();

  xercesc::DOMElement* quaternion_list = get_node<DOMElement>(attitudes, "Quaternion_List");

  // Pick out the "Quaternion" nodes
  DOMNodeList* children = quaternion_list->getChildNodes();
  for (XMLSize_t i = 0; i < children->getLength(); i++) {
    
    // Check child node type
    DOMNode* child = children->item(i);
    if (child->getNodeType() != DOMNode::ELEMENT_NODE)
      continue;

    // Check the node time
    DOMElement* curr_element = dynamic_cast<DOMElement*>(child);
    std::string tag(XMLString::transcode(curr_element->getTagName()));
    if (tag.find("Quaternion") == std::string::npos)
      continue;
  
    // Parse the time and 

    std::pair<double, vw::Quaternion<double>> data;
    std::string time_str;
    cast_xmlch(get_node<DOMElement>(curr_element, "TIME")->getTextContent(), time_str);

    bool is_start_time = false;
    data.first = PeruSatXML::convert_time(time_str, is_start_time);
    
    double w, x, y, z;
    cast_xmlch(get_node<DOMElement>(curr_element, "Q0")->getTextContent(), w);
    cast_xmlch(get_node<DOMElement>(curr_element, "Q1")->getTextContent(), x);
    cast_xmlch(get_node<DOMElement>(curr_element, "Q2")->getTextContent(), y);
    cast_xmlch(get_node<DOMElement>(curr_element, "Q3")->getTextContent(), z);
    data.second = vw::Quaternion<double>(w, x, y, z);

    pose_logs.push_back(data);
  } // End loop through attitudes
}

void PeruSatXML::read_look_angles(xercesc::DOMElement* look_angles) {
  std::string delimiters(",\t ");
  std::string tan_psi_x_str;
  cast_xmlch(get_node<DOMElement>(look_angles, "LINE_OF_SIGHT_TANPSIX")->getTextContent(),
             tan_psi_x_str);
  tan_psi_x = str_to_vec<Vector2>(tan_psi_x_str, delimiters);
  std::cout << "--tan_psi_x " << tan_psi_x << std::endl;
  
  std::string tan_psi_y_str;
  cast_xmlch(get_node<DOMElement>(look_angles, "LINE_OF_SIGHT_TANPSIY")->getTextContent(),
             tan_psi_y_str);
  tan_psi_y = str_to_vec<Vector2>(tan_psi_y_str, delimiters);
  std::cout << "--tan_psi_y " << tan_psi_y << std::endl;
}

void PeruSatXML::read_instr_biases(xercesc::DOMElement* instr_biases) {
    
  double w, x, y, z;
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q0")->getTextContent(), w);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q1")->getTextContent(), x);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q2")->getTextContent(), y);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q3")->getTextContent(), z);

  instrument_biases = vw::Quaternion<double>(w, x, y, z);

  std::cout << "--instrument_biases " << instrument_biases << std::endl;
}

void PeruSatXML::read_center_data (xercesc::DOMElement* geom_values) {

  std::string center_time_str;
  cast_xmlch(get_node<DOMElement>(geom_values, "TIME")->getTextContent(), center_time_str);
  cast_xmlch(get_node<DOMElement>(geom_values, "COL")->getTextContent(), center_col);
  cast_xmlch(get_node<DOMElement>(geom_values, "ROW")->getTextContent(), center_row);

  bool is_start_time = false;
  center_time = PeruSatXML::convert_time(center_time_str, is_start_time);

  std::cout << "--center time is " << center_time << std::endl;
  std::cout << "--before substr " << center_col << ' ' << center_row << std::endl;

  // Convert from 1-based to 0-based indices.  
  center_col  -= 1;
  center_row  -= 1;
  
  std::cout << "--after substr " << center_col << ' ' << center_row << std::endl;
}

// Converts a time from string to double precision value measured in seconds
// relative to the start time.
// Input strings look like this: 2008-03-04T12:31:03.08191Z.
double PeruSatXML::convert_time(std::string const& s, bool start_time) {

  if (!start_time && !m_start_time_is_set) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Must set the start time before doing time conversions.\n");

  try{
    // Replace the T with a space so the default Boost function can
    // parse the time.
    std::string s2 = s;
    boost::replace_all(s2, "T", " ");

    // Ensure there are exactly 6 digits for the millisecond or else
    // Boost will complain.
    s2 = fix_millisecond(s2);

    boost::posix_time::ptime time = boost::posix_time::time_from_string(s2);

    if (start_time) {
      m_start_time_is_set = true;
      start_time_stamp = time;
    }

    boost::posix_time::time_duration delta(time - start_time_stamp);
    return delta.total_microseconds() / 1.0e+6;
    
  }catch(...){
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << s << "\n");
  }
  return -1.0; // Never reached
}

// This is pretty simple, PERUSAT5 has a constant time for each line.
vw::camera::LinearTimeInterpolation PeruSatXML::setup_time_func() const {
  // The metadata tells us the time of the middle line, so find the time for the first line.
  double min_line_diff = static_cast<double>(0 - this->center_row);
  double min_line_time = center_time + this->line_period*min_line_diff;
  //std::cout << "Setup time functor: " << std::setprecision(12)  << min_line_time << ", " << this->line_period << std::endl;
  //std::cout << std::setprecision(12)  << "Center time: " << center_time << std::endl;
  return vw::camera::LinearTimeInterpolation(min_line_time, this->line_period);
}

// Velocities are the sum of inertial velocities and the instantaneous
//  Earth rotation.

// The velocity is already in GCC, so just pack into a function.
vw::camera::LagrangianInterpolation PeruSatXML::setup_velocity_func() const {
  const int INTERP_RADII = 4; // Reccomended in the docs
  std::vector<double>  time;
  std::vector<Vector3> velocity;

  // Loop through the velocity logs and extract values
  std::list<std::pair<double, vw::Vector3> >::const_iterator iter;
  for (iter=velocity_logs.begin(); iter!=velocity_logs.end(); iter++) {
    time.push_back(iter->first);
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
vw::camera::LagrangianInterpolation PeruSatXML::setup_position_func() const {

 const int INTERP_RADII = 4; // Reccomended in the docs
  std::vector<double>  time;
  std::vector<Vector3> position;

  // Loop through the velocity logs and extract values
  std::list<std::pair<double, vw::Vector3> >::const_iterator iter;
  for (iter=position_logs.begin(); iter!=position_logs.end(); iter++) {
    time.push_back(iter->first);
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

#if 0
vw::camera::LinearPiecewisePositionInterpolation PeruSatXML::setup_pose_func(
        vw::camera::LinearTimeInterpolation const& time_func) const {

  std::cout << "--fix here!" << std::endl;
  // This function returns a functor that returns just the yaw/pitch/roll angles.
  // - The time interval between lines is not constant but it is extremely close.


  // For some reason the corrected pose angles do not start early enough to cover
  // the time span for all of the input lines!
  // - In order to handle this, we repeat the earliest pose value so that it starts
  //   before the first line.
  // - The raw pose angles do start before the lines, but their values differ noticeably
  //   from the corrected values.
  
  // Compute how many padded pose entries are needed to cover all of the lines.
  size_t num_lines           = this->image_size[1];
  double num_corrected_poses = static_cast<double>(pose_logs.size());
  double first_line_time     = time_func(0);
  double last_line_time      = time_func(num_lines - 1.0);
  double pose_start_time     = pose_logs.front().first;
  double pose_stop_time      = pose_logs.back().first;
  double pose_delta_t        = (pose_stop_time - pose_start_time) / (num_corrected_poses - 1.0);
  int    num_prefill_poses   = static_cast<int>(ceil((pose_start_time - first_line_time) / pose_delta_t));
  int    num_postfill_poses  = static_cast<int>(ceil((last_line_time  - pose_stop_time) / pose_delta_t));
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
    
  size_t num_total_poses = pose_logs.size() + static_cast<size_t>(num_prefill_poses)
                                            + static_cast<size_t>(num_postfill_poses);

  std::vector<Vector3> pose(num_total_poses);
  std::vector<double>  time(num_total_poses);
  
  // Fill in the pre-padding poses
  size_t index = 0;
  for (int i=0; i<num_prefill_poses; i++) {
    double time_offset = pose_delta_t*static_cast<double>(num_prefill_poses-i);
    time[index] = pose_logs.front().first - time_offset;
    //pose[index] = pose_logs.front().second;
    //std::cout << "PREFILL: " << time[index] << std::endl;
    index++;
  }
  std::cout << "--study all this!" << std::endl;

  // Now fill in the real poses
  for (auto iter = pose_logs.begin(); iter != pose_logs.end(); iter++) {
    time[index] = iter->first;
    pose[index] = iter->second;
    index++;
  }

  // Fill in the post-padding poses
  for (int i=0; i<num_postfill_poses; i++) {
    double time_offset = pose_delta_t*(i+1);
    time[index] = pose_logs.back().first + time_offset;
    //pose[index] = pose_logs.back().second;
    //std::cout << "POSTFILL: " << time[index] << std::endl;
    index++;
  }
  
  //double max_time = time.back();
  double min_time = time.front();
  
  //std::cout << std::setprecision(12) << "Adding pose info: " << min_time << ", " 
  //          << max_time << " -> " << pose_delta_t << std::endl;
  
  return vw::camera::LinearPiecewisePositionInterpolation(pose, min_time, pose_delta_t);

}
#endif
  
// Boost does not like a time string such as "2017-12-07 15:36:40.90795Z"
// because it expects precisely 6 digits after the dot (hence for the millisecond).
// Fix that.
std::string PeruSatXML::fix_millisecond(std::string const& in_str) {

  std::string out_str = "";
  bool found_dot = false;
  int num_digits_after_dot = 0;
  for (size_t it = 0; it < in_str.size(); it++) {
    
    if (it + 1 < in_str.size()) {
      // Not yet at the last character
      
      if (in_str[it] == '.') {
        // Found the dot
        found_dot = true;
        out_str += in_str[it];
        continue;
      }

      if (!found_dot) {
        // Not at the dot yet
        out_str += in_str[it];
        continue;
      }

      // After the dot
      if (num_digits_after_dot < 6) {
        out_str += in_str[it];
        num_digits_after_dot++;
      }
      continue;
    }

    // At the last character
    if (in_str[it] >= '0' && in_str[it] <= '9') {
      // The last character is a digit, just append it
      if (num_digits_after_dot < 6) {
        out_str += in_str[it];
        num_digits_after_dot++;
      }

      // See if to append more
      while (num_digits_after_dot < 6) {
        out_str += "0";
        num_digits_after_dot++;
      }
      
    } else {

      // The last character is not a digit, it is likely a "Z"
      while (num_digits_after_dot < 6) {
        // Append zeros
        out_str += "0";
        num_digits_after_dot++;
      }

      // Append the last character, whatever it is
      out_str += in_str[it];
    }
    
  } // End iterating over characters

  return out_str;
}
  
  
} // end namespace asp



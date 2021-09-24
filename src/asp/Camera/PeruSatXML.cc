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
      m_err_handler.reset(new HandlerBase());
      m_parser->setValidationScheme(XercesDOMParser::Val_Always);
      m_parser->setDoNamespaces(true);   
      m_parser->setErrorHandler(m_err_handler.get());
    }

    // Load the XML file
    m_parser->parse(xml_path.c_str());
    DOMDocument* doc  = m_parser->getDocument();
    DOMElement * root = doc->getDocumentElement();
    return root;

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

  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NROWS")->getTextContent(), m_image_size[1]);
  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NCOLS")->getTextContent(), m_image_size[0]);
}

void PeruSatXML::read_times(xercesc::DOMElement* time) {
  xercesc::DOMElement* time_range = get_node<DOMElement>(time, "Time_Range");

  std::string start_time_str;
  cast_xmlch(get_node<DOMElement>(time_range, "START")->getTextContent(), start_time_str);
  bool is_start_time = true;
  m_start_time = PeruSatXML::convert_time(start_time_str, is_start_time);

  xercesc::DOMElement* time_stamp = get_node<DOMElement>(time, "Time_Stamp");
  cast_xmlch(get_node<DOMElement>(time_stamp, "LINE_PERIOD")->getTextContent(), m_line_period);
}
  
void PeruSatXML::read_ephemeris(xercesc::DOMElement* ephemeris) {

  // Reset data storage
  m_positions.clear(); 
  m_velocities.clear();

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
    velocity_vec = str_to_vec<Vector3>(velocity_str, delimiters);

    m_positions.push_back(std::pair<double, Vector3>(time, position_vec));
    m_velocities.push_back(std::pair<double, Vector3>(time, velocity_vec));
  } // End loop through points
  
}
  
void PeruSatXML::read_attitudes(xercesc::DOMElement* attitudes) {

  // Reset data storage
  m_poses.clear();

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

    // Normalize the quaternions to remove any inaccuracy due to the
    // limited precision used to save them on disk.
    data.second = normalize(data.second);
    
    m_poses.push_back(data);
  } // End loop through attitudes
}

void PeruSatXML::read_look_angles(xercesc::DOMElement* look_angles) {
  std::string delimiters(",\t ");
  std::string tan_psi_x_str;
  cast_xmlch(get_node<DOMElement>(look_angles, "LINE_OF_SIGHT_TANPSIX")->getTextContent(),
             tan_psi_x_str);
  m_tan_psi_x = str_to_vec<Vector2>(tan_psi_x_str, delimiters);
  
  std::string tan_psi_y_str;
  cast_xmlch(get_node<DOMElement>(look_angles, "LINE_OF_SIGHT_TANPSIY")->getTextContent(),
             tan_psi_y_str);
  m_tan_psi_y = str_to_vec<Vector2>(tan_psi_y_str, delimiters);
}

void PeruSatXML::read_instr_biases(xercesc::DOMElement* instr_biases) {
    
  double w, x, y, z;
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q0")->getTextContent(), w);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q1")->getTextContent(), x);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q2")->getTextContent(), y);
  cast_xmlch(get_node<DOMElement>(instr_biases, "Q3")->getTextContent(), z);

  m_instrument_biases = vw::Quaternion<double>(w, x, y, z);
}

void PeruSatXML::read_center_data(xercesc::DOMElement* geom_values) {

  std::string center_time_str;
  cast_xmlch(get_node<DOMElement>(geom_values, "TIME")->getTextContent(), center_time_str);
  cast_xmlch(get_node<DOMElement>(geom_values, "COL")->getTextContent(), m_center_col);
  cast_xmlch(get_node<DOMElement>(geom_values, "ROW")->getTextContent(), m_center_row);

  bool is_start_time = false;
  center_time = PeruSatXML::convert_time(center_time_str, is_start_time);

  // Convert from 1-based to 0-based indices.  
  m_center_col -= 1.0;
  m_center_row -= 1.0;
}

// Converts a time from string to double precision value measured in seconds
// relative to the start time.
// Input strings look like this: 2008-03-04T12:31:03.08191Z.
double PeruSatXML::convert_time(std::string const& s, bool is_start_time) {

  if (!is_start_time && !m_start_time_is_set) 
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

    if (is_start_time) {
      m_start_time_is_set = true;
      m_start_time_stamp = time;
    }

    boost::posix_time::time_duration delta(time - m_start_time_stamp);
    return delta.total_microseconds() / 1.0e+6;
    
  }catch(...){
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << s << "\n");
  }
  return -1.0; // Never reached
}

// Find the time at each line (line starts from 0) by multiplying by
// the period.  All times are relative to the starting time (see the
// convert_time() function).
  
// Note: PeruSat also has the center time, under Located_Geometric_Values,
// and that corresponds to line (num_lines - 1)/2.0 as expected. Yet PeruSat
// also provides there a center row, but that one is wrong and not equal
// to (num_lines - 1)/2.0.
vw::camera::LinearTimeInterpolation PeruSatXML::setup_time_func() const {
  return vw::camera::LinearTimeInterpolation(m_start_time, m_line_period);
}

// The position is already in GCC, so just pack into a function.
// - Currently this is identical to the velocity function, but this may change later.
vw::camera::LagrangianInterpolation PeruSatXML::setup_position_func
(vw::camera::LinearTimeInterpolation const& time_func) const {

  // Sanity check, we should be able to find the position for each image line
  size_t num_lines           = m_image_size[1];
  double first_line_time     = time_func(0);
  double last_line_time      = time_func(num_lines - 1.0);
  double num_positions       = m_positions.size();
  double position_start_time = m_positions.front().first;
  double position_stop_time  = m_positions.back().first;
  double position_delta_t    = (position_stop_time - position_start_time) / (num_positions - 1.0);
  
  if (position_start_time > first_line_time || position_stop_time < last_line_time)
    vw_throw(ArgumentErr() << "The position timestamps do not fully span the "
             << "range of times for the image lines.");

  // Use Lagrange interpolation with degree 3 polynomials with 4
  // points in piecewise manner out of 5 provided in the xml
  // file. This is what the doc recommends.
  // TODO(oalexan1): Implement Lagrange interpolation of even degree
  // using an odd number of points, and see if using all 5 points at
  // once is any better.
  const int INTERP_RADII = 2; 
  std::vector<double>  time_vec;
  std::vector<Vector3> position_vec;

  // Loop through the positions and extract values
  int index = 0;
  for (auto iter = m_positions.begin(); iter != m_positions.end(); iter++) {
    time_vec.push_back(iter->first);
    position_vec.push_back(iter->second);

    // Sanity check. The times at which the positions are given must
    // be uniformly distributed.
    if (index > 0) {
      double err = std::abs(time_vec[index] - time_vec[index - 1] - position_delta_t)
        / position_delta_t;
      if (err > 1.0e-6) 
        vw_throw(ArgumentErr() << "The position timestamps are not uniformly distributed.");
    }
    
    index++;
  }
  
  // More generic method for variable time intervals
  // return vw::camera::LagrangianInterpolationVarTime(position_vec, time_vec, INTERP_RADII);
  
  // A faster method for when we know the time delta is constant
  return vw::camera::LagrangianInterpolation(position_vec, position_start_time,
                                             position_delta_t, position_stop_time, INTERP_RADII);
}
  
// Velocities are the sum of inertial velocities and the instantaneous
//  Earth rotation.

// The velocity is already in GCC, so just pack into a function.
vw::camera::LagrangianInterpolation PeruSatXML::setup_velocity_func
(vw::camera::LinearTimeInterpolation const& time_func) const {

  // Sanity check, we should be able to find the velocity for each image line
  size_t num_lines           = m_image_size[1];
  double first_line_time     = time_func(0);
  double last_line_time      = time_func(num_lines - 1.0);
  double num_velocities      = m_velocities.size();
  double velocity_start_time = m_velocities.front().first;
  double velocity_stop_time  = m_velocities.back().first;
  double velocity_delta_t    = (velocity_stop_time - velocity_start_time) / (num_velocities - 1.0);

  if (velocity_start_time > first_line_time || velocity_stop_time < last_line_time)
    vw_throw(ArgumentErr() << "The velocity timestamps do not fully span the "
             << "range of times for the image lines.");

  if (velocity_start_time > first_line_time || velocity_stop_time < last_line_time)
    vw_throw(ArgumentErr() << "The velocity timestamps do not fully span the "
             << "range of times for the image lines.");

  // See note when the position function was set up earlier.
  const int INTERP_RADII = 2;
  std::vector<double>  time_vec;
  std::vector<Vector3> velocity_vec;

  // Loop through the velocities and extract values
  int index = 0;
  for (auto iter = m_velocities.begin(); iter != m_velocities.end(); iter++) {
    time_vec.push_back(iter->first);
    velocity_vec.push_back(iter->second);

    // Sanity check. The times at which the velocitys are given must
    // be uniformly distributed.
    if (index > 0) {
      double err = std::abs(time_vec[index] - time_vec[index - 1] - velocity_delta_t)
        / velocity_delta_t;
      if (err > 1.0e-6) 
        vw_throw(ArgumentErr() << "The velocity timestamps are not uniformly distributed.");
    }

    index++;
  }

  // More generic method for variable time intervals
  //return vw::camera::LagrangianInterpolationVarTime(velocity_vec, time_vec, INTERP_RADII);

  // A faster method for when we know the time delta is constant
  return vw::camera::LagrangianInterpolation(velocity_vec, velocity_start_time,
                                             velocity_delta_t, velocity_stop_time, INTERP_RADII);
}

// Put the timestamps and poses in vectors and form the pose
// interpolation object.

// TODO(oalexan1): See if using bicubic pose interpolation (as the
// SPOT-6 manual suggests) is better than the bilinear interpolation
// used now.
vw::camera::SLERPPoseInterpolation PeruSatXML::setup_pose_func
  (vw::camera::LinearTimeInterpolation const& time_func) const {

  size_t num_lines           = m_image_size[1];
  double first_line_time     = time_func(0);
  double last_line_time      = time_func(num_lines - 1.0);

  double num_poses           = m_poses.size();
  double pose_start_time     = m_poses.front().first;
  double pose_stop_time      = m_poses.back().first;
  double pose_delta_t        = (pose_stop_time - pose_start_time) / (num_poses - 1.0);

  if (pose_start_time > first_line_time || pose_stop_time < last_line_time)
    vw_throw(ArgumentErr() << "The quaternion timestamps do not fully span the "
             << "range of times for the image lines.");
  
  std::vector<vw::Quaternion<double>> pose_vec(num_poses);
  std::vector<double>  time_vec(num_poses);
  int index = 0;
  for (auto iter = m_poses.begin(); iter != m_poses.end(); iter++) {
    time_vec[index] = iter->first;
    pose_vec[index] = iter->second;

    // Sanity check. The times at which the quaternions are given must
    // be uniformly distributed. The quaternions are sampled more
    // densely than the positions and velocities, so we tolerate
    // a bit more divergence from uniform sampling.
    if (index > 0) {
      double err = std::abs(time_vec[index] - time_vec[index - 1] - pose_delta_t) / pose_delta_t;
      if (err > 0.01) 
        vw_throw(ArgumentErr() << "The quaternion timestamps are not uniformly distributed.");
    }
    
    index++;
  }

  double min_time = time_vec.front();
  return vw::camera::SLERPPoseInterpolation(pose_vec, min_time, pose_delta_t);
}
  
// Boost does not like a time string such as "2017-12-07 15:36:40.90795Z"
// because it expects precisely 6 digits after the dot. Fix that.
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



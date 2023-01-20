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

// Pleiades camera model. The documentation used is airbus-pleiades-imagery-user-guide-15042021.pdf.

#include <vw/Core/Exception.h>          // for ArgumentErr, vw_throw, etc
#include <vw/Math/Vector.h>             // for Vector, Vector3, Vector4, etc
#include <vw/Cartography/Datum.h>       // for Datum
#include <vw/FileIO/DiskImageResourceGDAL.h>

#include <asp/Camera/XMLBase.h>
#include <asp/Camera/PleiadesXML.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/XMLBase.h>
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

using namespace vw;
using namespace vw::cartography;
using namespace xercesc;

using asp::XmlUtils::get_node;
using asp::XmlUtils::cast_xmlch;

namespace asp {

DOMElement* PleiadesXML::open_xml_file(std::string const& xml_path) {

  // Check if the file actually exists and throw a user helpful file.
  if (!boost::filesystem::exists(xml_path))
    vw_throw(ArgumentErr() << "XML file: " << xml_path << " does not exist.");

  std::string error_prefix = "XML file: " + xml_path + " is invalid.\nException message is: \n";
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

void PleiadesXML::read_xml(std::string const& xml_path) {
  DOMElement * root = open_xml_file(xml_path);
  parse_xml(root);
}

void PleiadesXML::parse_xml(xercesc::DOMElement* root) {

  xercesc::DOMElement* metadata_id = get_node<DOMElement>(root, "Metadata_Identification");

  xercesc::DOMElement* metadata_profile = get_node<DOMElement>(metadata_id, "METADATA_PROFILE");

  std::string sensor_name(XMLString::transcode(metadata_profile->getTextContent()));
  
  std::string expected_name = "PHR_SENSOR";
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
  xercesc::DOMElement* instr_calib = get_node<DOMElement>(geom_calib, "Instrument_Calibration");

  xercesc::DOMElement* swath_range = get_node<DOMElement>(instr_calib, "Swath_Range");
  read_ref_col_row(swath_range);

  xercesc::DOMElement* look_angles = get_node<DOMElement>(instr_calib, "Polynomial_Look_Angles");
  read_look_angles(look_angles);

  PleiadesXML::parse_accuracy_stdv(root);

  return;
}

void PleiadesXML::read_image_size(xercesc::DOMElement* raster_data_node) {
  xercesc::DOMElement* raster_dims_node = get_node<DOMElement>(raster_data_node,
                                                                 "Raster_Dimensions");

  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NROWS")->getTextContent(), m_image_size[1]);
  cast_xmlch(get_node<DOMElement>(raster_dims_node, "NCOLS")->getTextContent(), m_image_size[0]);
}

void PleiadesXML::read_times(xercesc::DOMElement* time) {
  xercesc::DOMElement* time_range = get_node<DOMElement>(time, "Time_Range");

  // In addition to the relative start time stored in m_start_time, we will store
  // the start time string as well, in m_start_time_str, to be used later
  cast_xmlch(get_node<DOMElement>(time_range, "START")->getTextContent(), m_start_time_str);
  bool is_start_time = true;
  m_start_time = PleiadesXML::convert_time(m_start_time_str, is_start_time);
  
  std::string end_time_str;
  cast_xmlch(get_node<DOMElement>(time_range, "END")->getTextContent(), end_time_str);
  is_start_time = false;
  m_end_time = PleiadesXML::convert_time(end_time_str, is_start_time);

  xercesc::DOMElement* time_stamp = get_node<DOMElement>(time, "Time_Stamp");
  cast_xmlch(get_node<DOMElement>(time_stamp, "LINE_PERIOD")->getTextContent(), m_line_period);
  // Convert from milliseconds to seconds
  m_line_period /= 1000.0; 
}
  
void PleiadesXML::read_ephemeris(xercesc::DOMElement* ephemeris) {

  // Reset data storage
  m_positions.clear(); 
  m_velocities.clear();

  xercesc::DOMElement* ephemeris_used = get_node<DOMElement>(ephemeris, "EPHEMERIS_USED");

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
    double time = PleiadesXML::convert_time(time_str, is_start_time);
    std::string delimiters(",\t ");
    position_vec = str_to_vec<Vector3>(position_str, delimiters);
    velocity_vec = str_to_vec<Vector3>(velocity_str, delimiters);

    m_positions.push_back(std::pair<double, Vector3>(time, position_vec));
    m_velocities.push_back(std::pair<double, Vector3>(time, velocity_vec));
  } // End loop through points

  // Sanity check
  if (m_positions.size() < 2)
  vw_throw(ArgumentErr() << "Expecting to read at least two positions from the xml .\n");  
}

// Given a calendar time, find the midnight time. Just put zeros for hours, minutes, and seconds.
// An input time looks like: 2022-04-13T22:46:31.4540000
void calc_midnight_time(std::string const& start_time, std::string& midnight_time) {

  size_t hour_pos = start_time.find("T");
  if (hour_pos == std::string::npos)
    vw::vw_throw(vw::ArgumentErr()
                 << "Could not parse time string: " << start_time << ".\n");
  hour_pos += 1; // move past the "T"

  midnight_time = start_time;
  for (size_t it = hour_pos; it < midnight_time.size(); it++) {
    if (midnight_time[it] >= '0' && midnight_time[it] <= '9') {
      midnight_time.replace(it, 1, "0");
    }
  }

  return;
}
  
void PleiadesXML::read_attitudes(xercesc::DOMElement* attitudes) {

  xercesc::DOMElement* quaternion_root = get_node<DOMElement>(attitudes, "Polynomial_Quaternions");

  // Read the quaternion offset field
  std::string offset_str;
  cast_xmlch(get_node<DOMElement>(quaternion_root, "OFFSET")->getTextContent(), offset_str);

  // Per the documentation, this offset is from midnight. Need to transform it to be relative
  // to the start time.
  std::string midnight_time_str; 
  calc_midnight_time(m_start_time_str, midnight_time_str);

  bool is_start_time = false;
  double midnight_time = PleiadesXML::convert_time(midnight_time_str, is_start_time);
  m_quat_offset_time = midnight_time + atof(offset_str.c_str());

#if 0
  // Turning this off as it is fails on a real test case.
  // Untested adjustments for the case when the midnight is computed
  // for the wrong day. Not sure if this will ever happen. Try to
  // ensure that m_start_time <= m_quat_offset_time <= m_end_time.
  double full_day = 3600.0 * 24.0;
  if (m_quat_offset_time < m_start_time)
    m_quat_offset_time += full_day;
  if (m_quat_offset_time > m_end_time)
    m_quat_offset_time -= full_day;
  
  if (m_quat_offset_time < m_start_time || m_quat_offset_time > m_end_time)
    vw_throw(ArgumentErr() << "Failed to compute the quaternion offset. "
             << "Check the start time, end time, and the quaternion OFFSET field, "
             << "which is meant to be in seconds since midnight.\n");
#endif
  
  if (m_quat_offset_time < m_start_time)  // This never happened
    vw_out(WarningMessage) << "Quaternion offset time is less than start time (difference: "
                           << m_quat_offset_time - m_start_time << " seconds). "
                           << "Some caution may be advised.\n";
  if (m_quat_offset_time > m_end_time) // This was observed to happen
    vw_out(WarningMessage) << "Quaternion offset time in seconds ("
                           << m_quat_offset_time - m_start_time << ") "
                           << "is greater than end time ("
                           << m_end_time - m_start_time
                           << "). A discrepancy of a few seconds is likely acceptable.\n";
  
  // Read the quaternion scale field
  std::string scale_str;
  cast_xmlch(get_node<DOMElement>(quaternion_root, "SCALE")->getTextContent(), scale_str);
  m_quat_scale = atof(scale_str.c_str());

  // Read the quaternion coefficients that will be used with the quaternion
  // polynomial to find the quaternions at any time (page 77)
  m_quaternion_coeffs.clear();
  std::vector<std::string> tags = {"Q0", "Q1", "Q2", "Q3"};
  for (size_t it = 0; it < tags.size(); it++) {
    xercesc::DOMElement* qi = get_node<DOMElement>(quaternion_root, tags[it]);

    int deg = 0;
    cast_xmlch(get_node<DOMElement>(qi, "DEGREE")->getTextContent(), deg);
    if (deg != 3)
      vw_throw(ArgumentErr() << "Expecting the degree of the quaternion polynomial to be 3.\n");

    std::string quat_str;
    cast_xmlch(get_node<DOMElement>(qi, "COEFFICIENTS")->getTextContent(), quat_str);

    vw::Vector<double, 4> v = vw::str_to_vec<vw::Vector<double, 4>>(quat_str);
    m_quaternion_coeffs.push_back(v);
  }

  return;
}

void PleiadesXML::read_ref_col_row(xercesc::DOMElement* swath_range) {
  m_ref_row = 1; // page 76 in the doc
  
  std::string ref_col;
  cast_xmlch(get_node<DOMElement>(swath_range, "FIRST_COL")->getTextContent(),
             ref_col);
  m_ref_col = atoi(ref_col.c_str());

  // subtract 1, as we prefer to start rows and columns from 0
  m_ref_row -= 1;
  m_ref_col -= 1;
}
  
void PleiadesXML::read_look_angles(xercesc::DOMElement* look_angles) {

  // Pages 75 and 100 in the doc
  m_coeff_psi_x = vw::Vector2(0, 0);
  std::string xlos_0;
  cast_xmlch(get_node<DOMElement>(look_angles, "XLOS_0")->getTextContent(),
             xlos_0);
  m_coeff_psi_x[0] = atof(xlos_0.c_str());
  
  std::string xlos_1;
  cast_xmlch(get_node<DOMElement>(look_angles, "XLOS_1")->getTextContent(),
             xlos_1);
  m_coeff_psi_x[1] = atof(xlos_1.c_str());

  // Unlike for PeruSat, there's only one coeff_psi_y value. Keep the same
  // interface though.
  m_coeff_psi_y = vw::Vector2(0, 0);
  std::string ylos_0;
  cast_xmlch(get_node<DOMElement>(look_angles, "YLOS_0")->getTextContent(),
             ylos_0);
  m_coeff_psi_y[0] = atof(ylos_0.c_str());
}

// Converts a time from string to double precision value measured in seconds
// relative to the start time. First time it is called for the start time.
// Input strings look like this: 2022-04-13T22:46:31.4540000Z
double PleiadesXML::convert_time(std::string const& s, bool is_start_time) {

  if (!is_start_time && !m_start_time_is_set) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Must set the start time before doing time conversions.\n");

  try{
    boost::posix_time::ptime time = asp::parse_time(s);
    
    // If this is the first invocation, find the start time first
    if (is_start_time) {
      m_start_time_is_set = true;
      m_start_time_stamp = time;
    }

    // Now find the relative start time
    boost::posix_time::time_duration delta(time - m_start_time_stamp);

    // Go from microseconds to seconds
    return delta.total_microseconds() / 1.0e+6;
    
  }catch(...){
    vw::vw_throw(vw::ArgumentErr() << "Failed to parse time from string: " << s << "\n");
  }
  return -1.0; // Never reached
}

// This is an optional field, used only for error propagation  
void PleiadesXML::parse_accuracy_stdv(xercesc::DOMElement* root) {
  m_accuracy_stdv = 0.0;
  try {
    xercesc::DOMElement* quality_assessment = get_node<DOMElement>(root, "Quality_Assessment");
    xercesc::DOMElement* planimetric_accuracy_measurement
      = get_node<DOMElement>(quality_assessment, "Planimetric_Accuracy_Measurement");
    xercesc::DOMElement* quality_values = get_node<DOMElement>(planimetric_accuracy_measurement,
                                                               "Quality_Values");
    std::string accuracy_stdv;
    cast_xmlch(get_node<DOMElement>(quality_values, "ACCURACY_STDV")->getTextContent(),
               accuracy_stdv);
    m_accuracy_stdv = atof(accuracy_stdv.c_str());
  } catch (...) {
    m_accuracy_stdv = 0.0; 
  }

  return;
}

// Find the time at each line. According to the doc (page 76),
// reference line is the first line. In ASP, line count starts from 0.
// We will use the convention that all times are relative to time
// at line 0, so time at line 0 is 0. There can be negative times,
// as the positions and velocities are tabulated at times both
// before the first line and after the last time.
vw::camera::LinearTimeInterpolation PleiadesXML::setup_time_func() const {
  return vw::camera::LinearTimeInterpolation(m_start_time, m_line_period);
}

// The position is already in GCC, so just pack into a function.
// Currently this is identical to the velocity function, but this may change later.
vw::camera::LagrangianInterpolation PleiadesXML::setup_position_func
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

  // Use Lagrange interpolation with degree 7 polynomials with 8
  // samples, per the doc (page 77).
  const int INTERP_RADIUS = 4;  // interpolation order = 2 * INTERP_RADIUS
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
  
  // We know the time delta is constant, so the data is uniformly distributed.
  return vw::camera::LagrangianInterpolation(position_vec, position_start_time,
                                             position_delta_t, position_stop_time, INTERP_RADIUS);
}
  
// Velocities are the sum of inertial velocities and the instantaneous
//  Earth rotation.

// The velocity is already in GCC, so just pack into a function.
vw::camera::LagrangianInterpolation PleiadesXML::setup_velocity_func
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
  const int INTERP_RADIUS = 4; // Interpolation order = 2 * INTERP_RADIUS
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
  //return vw::camera::LagrangianInterpolationVarTime(velocity_vec, time_vec, INTERP_RADIUS);

  // A faster method for when we know the time delta is constant
  return vw::camera::LagrangianInterpolation(velocity_vec, velocity_start_time,
                                             velocity_delta_t, velocity_stop_time, INTERP_RADIUS);
}
  
} // end namespace asp

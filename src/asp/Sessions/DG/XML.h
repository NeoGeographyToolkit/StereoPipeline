// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// These are objects that relate directly to block in XML that we need
// to read. They only read and then store the raw values. Other
// objects will interpret the results.

#ifndef __STEREO_SESSION_DG_XML_H__
#define __STEREO_SESSION_DG_XML_H__

#include <asp/Sessions/DG/XMLBase.h>
#include <vector>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

namespace asp {

  // Objects that represent read data from XML. These also provide a
  // storage structure for modification later on.
  class ImageXML : public XMLBase {

    void parse_meta( xercesc::DOMElement* node );
    void parse_tlc_list( xercesc::DOMElement* node );
    void parse_image_size( xercesc::DOMElement* node );

  public:
    ImageXML();

    void parse( xercesc::DOMElement* node );

    std::string tlc_start_time;
    std::string first_line_start_time;
    std::vector<std::pair<double,double> > tlc_vec; // Line -> time offset pairings
    vw::Vector2i image_size;
  };

  class GeometricXML : public XMLBase {

    void parse_principal_distance( xercesc::DOMElement* node );
    void parse_optical_distortion( xercesc::DOMElement* node );
    void parse_perspective_center( xercesc::DOMElement* node );
    void parse_camera_attitude( xercesc::DOMElement* node );
    void parse_detector_mounting( xercesc::DOMElement* node );

  public:
    GeometricXML();

    void parse( xercesc::DOMElement* node );

    double principal_distance;   // mm
    vw::int32 optical_polyorder;
    vw::Vector<double> optical_a, optical_b; // Don't currently support these
    vw::Vector3 perspective_center;  // meters in spacecraft frame
    vw::Quat camera_attitude;
    vw::Vector2 detector_origin;     // mm
    double detector_rotation;    // degrees about Z+ in camera frame
    double detector_pixel_pitch; // mm
  };

  class EphemerisXML : public XMLBase {

    void parse_meta( xercesc::DOMElement* node );
    void parse_eph_list( xercesc::DOMElement* node );

  public:
    EphemerisXML();

    void parse( xercesc::DOMElement* node );

    std::string start_time;      // UTC
    double time_interval;        // seconds
    std::vector<vw::Vector3> position_vec, velocity_vec; // ECEF
    std::vector<vw::Vector<double,6> > covariance_vec;   // Tri-diagonal
  };

  class AttitudeXML : public XMLBase {

    void parse_meta( xercesc::DOMElement* node );
    void parse_att_list( xercesc::DOMElement* node );

  public:
    AttitudeXML();

    void parse( xercesc::DOMElement* node );

    std::string start_time;
    double time_interval;
    std::vector<vw::Quat> quat_vec;
    std::vector<vw::Vector<double,10> > covariance_vec;
  };

  // Helper functions to allow us to fill the objects. This doesn't
  // really help with code reuse but I think it makes it easer to
  // read.
  void read_xml( std::string const& filename,
                 GeometricXML& geo,
                 AttitudeXML& att,
                 EphemerisXML& eph,
                 ImageXML& img );

} //end namespace asp

#endif//__STEREO_SESSION_DG_XML_H__

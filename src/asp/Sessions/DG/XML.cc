// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <asp/Sessions/DG/XML.h>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace xercesc;

void asp::ImageXML::parse_meta( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "TLCTIME" )->getTextContent(),
              tlc_start_time );
  cast_xmlch( get_node<DOMElement>( node, "FIRSTLINETIME" )->getTextContent(),
              first_line_start_time );
  size_t num_tlc;
  cast_xmlch( get_node<DOMElement>( node, "NUMTLC" )->getTextContent(),
              num_tlc );
  tlc_vec.resize( num_tlc );
}

void asp::ImageXML::parse_tlc_list( xercesc::DOMElement* node ) {
  DOMNodeList* children = node->getChildNodes();
  size_t count = 0;

  for ( XMLSize_t i = 0; i < children->getLength(); i++ ) {
    if ( children->item(i)->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* element = dynamic_cast<DOMElement*>( children->item(i) );
      std::string buffer;
      cast_xmlch( element->getTextContent(), buffer );

      std::istringstream istr( buffer );
      istr >> tlc_vec[count].first >> tlc_vec[count].second;

      count++;
    }
  }

  VW_ASSERT( count == tlc_vec.size(),
             IOErr() << "Read incorrect number of TLC." );
}

void asp::ImageXML::parse_image_size( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "NUMROWS" )->getTextContent(),
              image_size[1] );
  cast_xmlch( get_node<DOMElement>( node, "NUMCOLUMNS" )->getTextContent(),
              image_size[0] );
}

asp::ImageXML::ImageXML() : XMLBase(3) {}

void asp::ImageXML::parse( xercesc::DOMElement* node ) {
  DOMElement* image = get_node<DOMElement>( node, "IMAGE" );
  parse_meta( image );
  check_argument(0);

  parse_tlc_list( get_node<DOMElement>( image, "TLCLISTList" ) );
  check_argument(1);

  parse_image_size( node );
  check_argument(2);
}

void asp::GeometricXML::parse_principal_distance( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "PD" )->getTextContent(),
              principal_distance );
}

void asp::GeometricXML::parse_optical_distortion( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "POLYORDER" )->getTextContent(),
              optical_polyorder );
  if ( optical_polyorder > 0 )
    vw_throw( NoImplErr() << "We don't support optical distortion.\n" );
}

void asp::GeometricXML::parse_perspective_center( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "CX" )->getTextContent(),
              perspective_center[0] );
  cast_xmlch( get_node<DOMElement>( node, "CY" )->getTextContent(),
              perspective_center[1] );
  cast_xmlch( get_node<DOMElement>( node, "CZ" )->getTextContent(),
              perspective_center[2] );
}

void asp::GeometricXML::parse_camera_attitude( xercesc::DOMElement* node ) {
  Vector4 buffer; // We buffer since Q will want to constantly
  // renormalize itself.
  cast_xmlch( get_node<DOMElement>( node, "QCS4" )->getTextContent(),
              buffer[0] );
  cast_xmlch( get_node<DOMElement>( node, "QCS1" )->getTextContent(),
              buffer[1] );
  cast_xmlch( get_node<DOMElement>( node, "QCS2" )->getTextContent(),
              buffer[2] );
  cast_xmlch( get_node<DOMElement>( node, "QCS3" )->getTextContent(),
              buffer[3] );
  camera_attitude = Quat(buffer[0],buffer[1],buffer[2],buffer[3]);
}

void asp::GeometricXML::parse_detector_mounting( xercesc::DOMElement* node ) {
  // Assumes only one band
  DOMNodeList* children = node->getChildNodes();

  // Do one pass counting the number of bands
  XMLSize_t band_count = 0;
  XMLSize_t band_index = 0;
  for ( XMLSize_t i = 0; i < children->getLength(); i++ ) {
    DOMNode* current = children->item(i);
    if ( current->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* element =
        dynamic_cast< DOMElement* > (current);

      std::string tag( XMLString::transcode(element->getTagName()) );
      if (boost::starts_with( tag, "BAND_" )) {
        band_index = i;
        band_count++;
      }
    }
  }

  // Sanity check that there is just one band
  VW_ASSERT( band_count == 1,
             NoImplErr() << "We only expect to see one band data.\n" );

  // Load up the single band
  DOMElement* band = dynamic_cast<DOMElement*>(children->item(band_index));
  DOMElement* detector = get_node<DOMElement>( band, "DETECTOR_ARRAY" );
  cast_xmlch( get_node<DOMElement>(detector,"DETORIGINX")->getTextContent(),
              detector_origin[0] );
  cast_xmlch( get_node<DOMElement>(detector,"DETORIGINY")->getTextContent(),
              detector_origin[1] );
  cast_xmlch( get_node<DOMElement>(detector,"DETROTANGLE")->getTextContent(),
              detector_rotation );
  cast_xmlch( get_node<DOMElement>(detector,"DETPITCH")->getTextContent(),
              detector_pixel_pitch );
}

asp::GeometricXML::GeometricXML() : XMLBase(5) {}

void asp::GeometricXML::parse( xercesc::DOMElement* node ) {
  DOMNodeList* children = node->getChildNodes();
  const XMLSize_t nodeCount = children->getLength();

  for ( XMLSize_t i = 0; i < nodeCount; ++i ) {
    DOMNode* current = children->item(i);
    if ( current->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* element =
        dynamic_cast< xercesc::DOMElement* >( current );

      std::string tag( XMLString::transcode(element->getTagName()) );
      if ( tag == "PRINCIPAL_DISTANCE" ) {
        parse_principal_distance( element );
        check_argument(0);
      } else if ( tag == "OPTICAL_DISTORTION" ) {
        parse_optical_distortion( element );
        check_argument(1);
      } else if ( tag == "PERSPECTIVE_CENTER" ) {
        parse_perspective_center( element );
        check_argument(2);
      } else if ( tag == "CAMERA_ATTITUDE" ) {
        parse_camera_attitude( element );
        check_argument(3);
      } else if ( tag == "DETECTOR_MOUNTING" ) {
        parse_detector_mounting( element );
        check_argument(4);
      }
    }
  }
}

void asp::EphemerisXML::parse_meta( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "STARTTIME" )->getTextContent(),
              start_time );
  cast_xmlch( get_node<DOMElement>( node, "TIMEINTERVAL" )->getTextContent(),
              time_interval );
  size_t num_points;
  cast_xmlch( get_node<DOMElement>( node, "NUMPOINTS" )->getTextContent(),
              num_points );
  position_vec.resize( num_points );
  velocity_vec.resize( num_points );
  covariance_vec.resize( num_points );
}

void asp::EphemerisXML::parse_eph_list( xercesc::DOMElement* node ) {
  DOMNodeList* children = node->getChildNodes();
  size_t count = 0;

  for ( XMLSize_t i = 0; i < children->getLength(); i++ ) {
    if ( children->item(i)->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* element = dynamic_cast<DOMElement*>( children->item(i) );
      std::string buffer;
      cast_xmlch( element->getTextContent(), buffer );

      std::istringstream istr( buffer );
      std::string index_b;
      istr >> index_b;
      size_t index = size_t(boost::lexical_cast<float>(index_b) + 0.5) - 1;
      istr >> position_vec[index][0] >> position_vec[index][1]
           >> position_vec[index][2] >> velocity_vec[index][0]
           >> velocity_vec[index][1] >> velocity_vec[index][2];
      istr >> covariance_vec[index][0] >> covariance_vec[index][1]
           >> covariance_vec[index][2] >> covariance_vec[index][3]
           >> covariance_vec[index][4] >> covariance_vec[index][5];

      count++;
    }
  }

  VW_ASSERT( count == position_vec.size(),
             IOErr() << "Read incorrect number of points." );
}

asp::EphemerisXML::EphemerisXML() : XMLBase(2) {}

void asp::EphemerisXML::parse( xercesc::DOMElement* node ) {
  parse_meta( node );
  check_argument(0);

  parse_eph_list( get_node<DOMElement>( node, "EPHEMLISTList" ) );
  check_argument(1);
}

void asp::AttitudeXML::parse_meta( xercesc::DOMElement* node ) {
  cast_xmlch( get_node<DOMElement>( node, "STARTTIME" )->getTextContent(),
              start_time );
  cast_xmlch( get_node<DOMElement>( node, "TIMEINTERVAL" )->getTextContent(),
              time_interval );
  size_t num_points;
  cast_xmlch( get_node<DOMElement>( node, "NUMPOINTS" )->getTextContent(),
              num_points );
  quat_vec.resize( num_points );
  covariance_vec.resize( num_points );
}

void asp::AttitudeXML::parse_att_list( xercesc::DOMElement* node ) {
  DOMNodeList* children = node->getChildNodes();
  size_t count = 0;
  Vector4 qbuf;

  for ( XMLSize_t i = 0; i < children->getLength(); i++ ) {
    if ( children->item(i)->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* element = dynamic_cast<DOMElement*>( children->item(i) );
      std::string buffer;
      cast_xmlch( element->getTextContent(), buffer );

      std::istringstream istr( buffer );
      std::string index_b;
      istr >> index_b;
      size_t index = size_t(boost::lexical_cast<float>(index_b) + 0.5) - 1;
      istr >> qbuf[0] >> qbuf[1] >> qbuf[2] >> qbuf[3];
      quat_vec[index] = Quat(qbuf[3], qbuf[0], qbuf[1], qbuf[2] );
      istr >> covariance_vec[index][0] >> covariance_vec[index][1]
           >> covariance_vec[index][2] >> covariance_vec[index][3]
           >> covariance_vec[index][4] >> covariance_vec[index][5]
           >> covariance_vec[index][6] >> covariance_vec[index][7]
           >> covariance_vec[index][8] >> covariance_vec[index][9];

      count++;
    }
  }

  VW_ASSERT( count == quat_vec.size(),
             IOErr() << "Read incorrect number of points." );
}

asp::AttitudeXML::AttitudeXML() : XMLBase(2) {}

void asp::AttitudeXML::parse( xercesc::DOMElement* node ) {
  parse_meta( node );
  check_argument(0);

  parse_att_list( get_node<DOMElement>( node, "ATTLISTList" ) );
  check_argument(1);
}

// Helper functions to allow us to fill the objects
void asp::read_xml( std::string const& filename,
                    GeometricXML& geo,
                    AttitudeXML& att,
                    EphemerisXML& eph,
                    ImageXML& img ) {

  boost::scoped_ptr<XercesDOMParser> parser( new XercesDOMParser() );
  parser->setValidationScheme(XercesDOMParser::Val_Always);
  parser->setDoNamespaces(true);
  boost::scoped_ptr<ErrorHandler> errHandler( new HandlerBase() );
  parser->setErrorHandler(errHandler.get());

  parser->parse( filename.c_str() );
  DOMDocument* xmlDoc = parser->getDocument();
  DOMElement* elementRoot = xmlDoc->getDocumentElement();

  DOMNodeList* children = elementRoot->getChildNodes();
  for ( XMLSize_t i = 0; i < children->getLength(); ++i ) {
    DOMNode* curr_node = children->item(i);
    if ( curr_node->getNodeType() == DOMNode::ELEMENT_NODE ) {
      DOMElement* curr_element =
        dynamic_cast<DOMElement*>( curr_node );

      std::string tag( XMLString::transcode(curr_element->getTagName()) );

      if ( tag == "GEO" )
        geo.parse( curr_element );
      else if ( tag == "EPH" )
        eph.parse( curr_element );
      else if ( tag == "ATT" )
        att.parse( curr_element );
      else if ( tag == "IMD" )
        img.parse( curr_element );
    }
  }
}

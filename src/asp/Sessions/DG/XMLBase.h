// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// This is header shouldn't be included by any other headers since it
// brings in a bunch of Xerces Headers. This way I can limit the
// spread of those headers and objects.

#ifndef __STEREO_SESSION_DG_XMLBASE_H__
#define __STEREO_SESSION_DG_XMLBASE_H__

#include <vw/Core.h>
#include <boost/lexical_cast.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/XMLString.hpp>

namespace asp {

  // XML parsing base that provides tools for verifying that we read
  // everything we were expecting.
  class XMLBase {
    vw::uint8 m_num_arguments;
    vw::int32 m_checksum;
    vw::int32 m_good;

  protected:
    // Used to check off that one of the arguments has been read.
    void check_argument( vw::uint8 arg );

    // Helper function to convert XML text to binary value we want.
    template <class T>
    void cast_xmlch( const XMLCh* ch, T& dst) {
      char* text = xercesc::XMLString::transcode(ch);
      dst = boost::lexical_cast<T>( text );
      xercesc::XMLString::release( &text );
    }

    // Helper function to retreive a node via string and verify that
    // only one exists.
    template <class T>
    T* get_node( xercesc::DOMElement* element, std::string const& tag ) {
      XMLCh* tag_c =
        xercesc::XMLString::transcode(tag.c_str());
      xercesc::DOMNodeList* list = element->getElementsByTagName( tag_c );
      VW_ASSERT( list->getLength() != 0,
                 vw::IOErr() << "Couldn't find \"" << tag << "\" tag." );
      VW_ASSERT( list->getLength() == 1,
                 vw::IOErr() << "Found multiple \"" << tag << "\" tags." );
      xercesc::XMLString::release(&tag_c);
      return dynamic_cast<T*>(list->item(0));
    }

  public:
    XMLBase( vw::uint8 num_arguments );

    bool is_good() const;
  };

} // end namespace asp

#endif//__STEREO_SESSION_DG_XMLBASE_H__

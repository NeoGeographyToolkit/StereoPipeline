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


// This header shouldn't be included by any other headers since it
// brings in a bunch of Xerces Headers. This way I can limit the
// spread of those headers and objects.

#ifndef __STEREO_SESSION_DG_BitChecker_H__
#define __STEREO_SESSION_DG_BitChecker_H__

#include <vw/Core/Exception.h>
#include <vw/Core/FundamentalTypes.h>

#include <string>

#include <boost/lexical_cast.hpp>

#include <bitset>


#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/Xerces_autoconf_config.hpp>


// TODO: Rename this file to something like XML_utils

namespace asp {

namespace XmlUtils {

/// Helper function to convert XML text to binary value we want.
template <class T>
void cast_xmlch( const XMLCh* ch, T& dst) {
  char* text = xercesc::XMLString::transcode(ch);
  try {
    dst = boost::lexical_cast<T>( text );
  } catch (boost::bad_lexical_cast const& e) {
    vw_throw(vw::ArgumentErr() << "Failed to parse string: " << text << "\n");
  }

  xercesc::XMLString::release( &text );
}


/// Helper function to retreive a node via string and verify that only one exists.
template <class T>
T* get_node( xercesc::DOMElement* element, std::string const& tag ) {
  XMLCh* tag_c = xercesc::XMLString::transcode(tag.c_str());
  xercesc::DOMNodeList* list = element->getElementsByTagName( tag_c );
  VW_ASSERT( list->getLength() != 0,
             vw::IOErr() << "Couldn't find \"" << tag << "\" tag." );
  VW_ASSERT( list->getLength() == 1,
             vw::IOErr() << "Found multiple \"" << tag << "\" tags." );
  xercesc::XMLString::release(&tag_c);
  return dynamic_cast<T*>(list->item(0));
}

} // End namespace XmlUtils 

} // end namespace asp

#endif//__STEREO_SESSION_DG_BitChecker_H__

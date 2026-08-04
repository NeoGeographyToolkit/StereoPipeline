// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

// This header should not be included by other headers since it
// brings in xercesc headers. This limits the spread of those
// headers and objects.

#ifndef __STEREO_SESSION_XMLBASE_H__
#define __STEREO_SESSION_XMLBASE_H__

#include <vw/Core/Exception.h>
#include <vw/Core/FundamentalTypes.h>

#include <string>
#include <vector>

// Can't do much about warnings in boost except to hide them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <boost/lexical_cast.hpp>
#pragma GCC diagnostic pop

#include <bitset>

#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/Xerces_autoconf_config.hpp>

namespace asp {

namespace XmlUtils {

/// Helper function to convert XML text to a binary value.
template <class T>
void cast_xmlch(const XMLCh* ch, T& dst) {
  char* text = xercesc::XMLString::transcode(ch);
  try {
    dst = boost::lexical_cast<T>(text);
  } catch (boost::bad_lexical_cast const& e) {
    vw_throw(vw::ArgumentErr() << "Failed to parse string: " << text << "\n");
  }
  xercesc::XMLString::release(&text);
}

/// Return an element's local name, with any namespace prefix stripped.
/// Newer Vantor/Maxar ISD XML namespaces every tag (lv1b:IMD,
/// isdc:SCANDIRECTION), so getTagName() returns the full "prefix:name"
/// string. getLocalName() returns just the local part. For non-namespaced
/// XML getLocalName() can be null, so fall back to getTagName().
std::string element_local_name(xercesc::DOMElement* element);

/// Helper function to retrieve a node via string and verify that only one exists.
template <class T>
T* get_node(xercesc::DOMElement* element, std::string const& tag) {
  XMLCh* tag_c  = xercesc::XMLString::transcode(tag.c_str());
  XMLCh* star_c = xercesc::XMLString::transcode("*");
  // Match by local name in any (or no) namespace. Newer Vantor/Maxar ISD XML
  // namespaces every element (e.g. isdc:STARTTIME), so getElementsByTagName,
  // which matches the full "prefix:name", would find nothing.
  xercesc::DOMNodeList* list = element->getElementsByTagNameNS(star_c, tag_c);
  VW_ASSERT(list->getLength() != 0,
            vw::IOErr() << "Could not find \"" << tag << "\" tag.");
  VW_ASSERT(list->getLength() == 1,
            vw::IOErr() << "Found multiple \"" << tag << "\" tags.");
  xercesc::XMLString::release(&tag_c);
  xercesc::XMLString::release(&star_c);
  return dynamic_cast<T*>(list->item(0));
}

/// Parse all doubles from a named XML block in raw text using strtod.
/// Finds the block whose local name is blockTag (ignoring any namespace
/// prefix on the opening and closing tags), skips any XML tags inside, and
/// extracts all floating-point numbers. Much faster than xercesc DOM
/// traversal for large blocks of numeric data.
void parseDoublesFromXmlBlock(std::string const& rawXml,
                              std::string const& blockTag,
                              std::vector<double>& values);

} // End namespace XmlUtils

} // end namespace asp

#endif//__STEREO_SESSION_XMLBASE_H__

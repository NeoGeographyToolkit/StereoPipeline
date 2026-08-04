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


#include <asp/Camera/XMLBase.h>

#include <cstdlib> // strtod
#include <vector>

using namespace vw;

std::string asp::XmlUtils::element_local_name(xercesc::DOMElement* element) {
  const XMLCh* local = element->getLocalName();
  // getLocalName() is null for DOM Level 1 nodes with no namespace info.
  const XMLCh* name = (local != NULL) ? local : element->getTagName();
  char* text = xercesc::XMLString::transcode(name);
  std::string result(text);
  xercesc::XMLString::release(&text);
  return result;
}

namespace {

// Find a start tag <blockTag> or <prefix:blockTag> (when start is true), or an
// end tag </blockTag> or </prefix:blockTag> (when start is false), ignoring any
// namespace prefix. Returns the byte just past the start tag's '>', or the byte
// at the end tag's '<'. Returns npos if not found. This mirrors how the newer
// Vantor/Maxar ISD XML namespaces the list containers (e.g. isdc:EPHEMLISTList).
size_t findBlockTag(std::string const& xml, std::string const& blockTag,
                    bool start, size_t from) {
  size_t n = xml.size();
  size_t i = from;
  while (i < n) {
    size_t lt = xml.find('<', i);
    if (lt == std::string::npos)
      return std::string::npos;
    size_t j = lt + 1;
    bool isEnd = (j < n && xml[j] == '/');
    if (isEnd)
      j++;
    // Skip declarations, comments, and processing instructions.
    if (j < n && (xml[j] == '?' || xml[j] == '!')) {
      i = lt + 1;
      continue;
    }
    // Read the qualified tag name up to whitespace, '/', or '>'.
    size_t nameStart = j;
    while (j < n && xml[j] != '>' && xml[j] != '/' && xml[j] != ' ' &&
           xml[j] != '\t' && xml[j] != '\r' && xml[j] != '\n')
      j++;
    std::string qname = xml.substr(nameStart, j - nameStart);
    size_t colon = qname.find(':');
    std::string local = (colon == std::string::npos) ? qname : qname.substr(colon + 1);
    if (local == blockTag && isEnd == !start) {
      if (!start)
        return lt; // position of '<' of the end tag
      size_t gt = xml.find('>', j);
      if (gt == std::string::npos)
        return std::string::npos;
      return gt + 1; // just past the '>' of the start tag
    }
    i = lt + 1;
  }
  return std::string::npos;
}

} // end anonymous namespace

void asp::XmlUtils::parseDoublesFromXmlBlock(
    std::string const& rawXml,
    std::string const& blockTag,
    std::vector<double>& values) {

  values.clear();

  // Find the block boundaries by local name, ignoring any namespace prefix.
  size_t blockStart = findBlockTag(rawXml, blockTag, true, 0);
  if (blockStart == std::string::npos)
    vw_throw(ArgumentErr() << "Tag not found: " << blockTag << "\n");

  size_t blockEnd = findBlockTag(rawXml, blockTag, false, blockStart);
  if (blockEnd == std::string::npos)
    vw_throw(ArgumentErr() << "Closing tag not found: " << blockTag << "\n");

  const char* p = rawXml.c_str() + blockStart;
  const char* end = rawXml.c_str() + blockEnd;

  while (p < end) {
    // Skip XML tags
    if (*p == '<') {
      while (p < end && *p != '>')
        p++;
      if (p < end)
        p++;
      continue;
    }
    // Skip whitespace
    if (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
      p++;
      continue;
    }
    // Parse a number
    char* next = nullptr;
    double val = strtod(p, &next);
    if (next > p) {
      values.push_back(val);
      p = next;
    } else {
      p++; // skip unexpected char
    }
  }
}

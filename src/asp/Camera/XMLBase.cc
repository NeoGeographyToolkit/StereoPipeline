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

void asp::XmlUtils::parseDoublesFromXmlBlock(
    std::string const& rawXml,
    std::string const& openTag,
    std::string const& closeTag,
    std::vector<double>& values) {

  values.clear();

  // Find the block boundaries
  size_t blockStart = rawXml.find(openTag);
  if (blockStart == std::string::npos)
    vw_throw(ArgumentErr() << "Tag not found: " << openTag << "\n");
  blockStart += openTag.size();

  size_t blockEnd = rawXml.find(closeTag, blockStart);
  if (blockEnd == std::string::npos)
    vw_throw(ArgumentErr() << "Closing tag not found: " << closeTag << "\n");

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

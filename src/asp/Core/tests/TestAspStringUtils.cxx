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
//  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
//  License for the specific language governing permissions and limitations
//  under the License.
// __END_LICENSE__

#include <test/Helpers.h>
#include <asp/Core/AspStringUtils.h>

using namespace asp;

TEST(AspStringUtils, ParseAppendMetadataPreservesEqualsInValues) {
  std::map<std::string, std::string> keywords;
  keywords["EXISTING"] = "keep";

  parse_append_metadata("A=1 B=a=b C=x==y", keywords);

  EXPECT_EQ("keep", keywords["EXISTING"]);
  EXPECT_EQ("1", keywords["A"]);
  EXPECT_EQ("a=b", keywords["B"]);
  EXPECT_EQ("x==y", keywords["C"]);
}

TEST(AspStringUtils, ParseAppendMetadataRejectsMissingValue) {
  std::map<std::string, std::string> keywords;
  EXPECT_THROW(parse_append_metadata("A=", keywords), vw::ArgumentErr);
}

// As with GDAL -mo, a value keeps everything after the first equal sign,
// including spaces. A token without an equal sign extends the current value.
TEST(AspStringUtils, ParseAppendMetadataPreservesSpacesInValues) {
  std::map<std::string, std::string> keywords;

  parse_append_metadata("VAR1=value with spaces VAR2=plain", keywords);
  EXPECT_EQ("value with spaces", keywords["VAR1"]);
  EXPECT_EQ("plain", keywords["VAR2"]);

  // Spaces and equal signs together in one value.
  keywords.clear();
  parse_append_metadata("VAR3=a = b c", keywords);
  EXPECT_EQ("a = b c", keywords["VAR3"]);

  // A single pair with spaces, as from a repeated --mo option.
  keywords.clear();
  parse_append_metadata("VAR4=value with spaces", keywords);
  EXPECT_EQ("value with spaces", keywords["VAR4"]);
}

// A leading token with no equal sign cannot start a value.
TEST(AspStringUtils, ParseAppendMetadataRejectsLeadingValueWord) {
  std::map<std::string, std::string> keywords;
  EXPECT_THROW(parse_append_metadata("word A=1", keywords), vw::ArgumentErr);
}

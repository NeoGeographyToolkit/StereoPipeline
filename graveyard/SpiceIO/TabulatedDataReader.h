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


/// \file TabulatedDataReader.h
///

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

namespace asp {
namespace spice {

  class TabulatedDataReader {
  public:
    /* Constructor / Destructor */
    TabulatedDataReader( const std::string &filename, const std::string &delimeters);
    ~TabulatedDataReader() { close(); }

    void close() {
      if (m_file.is_open()) {
        m_file.close();
      }
    }

    /* Accessors */
    int find_line_with_text(std::string query,
                            std::vector<std::string> &result);

  private:
    std::string m_delimeters;

    std::ifstream m_file;
  };

}} // end namespace asp::spice

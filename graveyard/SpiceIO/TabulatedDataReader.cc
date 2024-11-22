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


/// \file TabulatedDataReader.cc
///

#include <asp/SpiceIO/TabulatedDataReader.h>
#include <boost/algorithm/string.hpp>
#include <vw/Core/Exception.h>

using namespace std;

#define LINE_MAXSIZE 4096

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
/*               TabulatedDataReader Class Methods               */
/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */


namespace asp {
namespace spice {

  TabulatedDataReader::TabulatedDataReader( const std::string &filename,
                                            const std::string &delimeters = ",") {
    m_delimeters = delimeters;

    m_file.open(filename.c_str());
    if ( !m_file.is_open() ) {
      throw vw::IOErr() << "Failed to open tabulated data record: " << filename << ".";
    }
  }


  // Returns 1 on success, 0 on failure
  int TabulatedDataReader::find_line_with_text(std::string query,
                                               std::vector<std::string> &result) {
    // Reset file pointer to the top of the file
    m_file.seekg(0);

    int found = 0;
    char line[LINE_MAXSIZE];

    // Read through the lines until the search returns a match
    while ( !found && !m_file.eof() ) {
      m_file.getline(line, LINE_MAXSIZE);

      // If the text is found, cut up this line using the delimeters
      // and return true.
      if(boost::find_first(line, query)) {
        string str_line(line);
        cout << str_line << endl;
        boost::split( result, str_line, boost::is_any_of(m_delimeters) );
        vector<string>::iterator iter = result.begin();

        //      unsigned int c = 0;
        for (iter = result.begin(); iter != result.end(); iter++) {
          boost::trim(*iter);
          //      cout << c << ":   " << *iter << "\n";
          //      c++;
        }
        found = 1;
      }
    }

    return found;
  }

}} //end namespace asp::spice



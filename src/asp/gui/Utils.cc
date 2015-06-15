// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NASA Vision Workbench is licensed under the Apache License,
//  Version 2.0 (the "License"); you may not use this file except in
//  compliance with the License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


/// \file Utils.cc
///
///

#include <string>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <vw/Core/Exception.h>

using namespace std;

namespace vw {

  void extractWindowDims(// inputs
                         std::string const& geom,
                         // outputs
                         int & windowWidX, int & windowWidY
                         ){
  
    // Parse a string in the format '500x600'.
  
    windowWidX = 1200; windowWidY = 800;

    char * str = (char*) geom.c_str();
    char * pch;
    char delimiter[] = "x";
  
    pch = strtok (str, delimiter);
    if (pch == NULL) return;
    windowWidX = atoi(pch);
  
    pch = strtok (NULL, delimiter);
    if (pch == NULL) return;
    windowWidY = atoi(pch);

    if (windowWidX <= 0 || windowWidY <= 0){
      vw_throw( ArgumentErr() << "The window dimensions must be positive. "
                << "Error parsing: " << geom << "\n");
    }
  
  }

}

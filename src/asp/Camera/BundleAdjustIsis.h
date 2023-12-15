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

/// \file BundleAdjustIsis.h

/// Utilities for handling ISIS's jigsaw control network format.

#ifndef __BUNDLE_ADJUST_ISIS_H__
#define __BUNDLE_ADJUST_ISIS_H__

#include <isis/ControlNet.h>
#include <isis/SerialNumberList.h>

#include <boost/shared_ptr.hpp>

#include <string>
#include <map>

namespace vw {
  namespace ba {
    class ControlNetwork;
  }
  namespace cartography {
    class Datum;
  }
}

namespace asp {

struct BAParams;

// Use this struct to collect all the data needed to handle an ISIS cnet.
struct IsisCnetData {
  Isis::ControlNetQsp isisCnet;
  boost::shared_ptr<Isis::SerialNumberList> isisImgData;
  std::map<int, int> isisToAspControlPointId;
  
  IsisCnetData() {
    isisCnet = Isis::ControlNetQsp(NULL);
    isisImgData = boost::shared_ptr<Isis::SerialNumberList>(NULL);
    isisToAspControlPointId.clear();
  }
};

// Load an ISIS cnet file and copy it to an ASP control network. The ISIS cnet
// will be used when saving the updated cnet.  
void loadIsisCnet(std::string const& isisCnetFile, 
                  std::string const& outputPrefix, 
                  std::vector<std::string> const& image_files,
                  // Outputs
                  vw::ba::ControlNetwork& cnet,
                  IsisCnetData & isisCnetData);

// Update an ISIS cnet with the latest info on triangulated points and outliers,
// and write it to disk at <outputPrefix>.net.
void saveUpdatedIsisCnet(std::string const& outputPrefix, 
                  asp::BAParams const& param_storage,
                  IsisCnetData & isisCnetData);

// Create and save an ISIS cnet from a given control network and latest param
// values.
void saveIsisCnet(std::string const& outputPrefix, 
                  vw::ba::ControlNetwork const& cnet,
                  vw::cartography::Datum const& datum,
                  asp::BAParams const& param_storage);

} // end namespace asp

#endif // __BUNDLE_ADJUST_ISIS_H__

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

namespace asp {
  
  struct IsisCnetData {
    Isis::ControlNetQsp m_controlNet;
    boost::shared_ptr<Isis::SerialNumberList> m_serialNumberList;
  };
  
  void bundle_adjust_isis();
} // end namespace asp

#endif // __BUNDLE_ADJUST_ISIS_H__

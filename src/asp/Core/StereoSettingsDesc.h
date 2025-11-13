// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file StereoOptionsDesc.h

/// Logic for stereo settings descriptions. This uses heavily boost program
/// options.

#ifndef __ASP_CORE_STEREO_OPTIONS_DESC_H__
#define __ASP_CORE_STEREO_OPTIONS_DESC_H__

#include <boost/program_options.hpp>
#include <vw/FileIO/GdalWriteOptions.h>

namespace asp {

  // Program options for each executable/step
  struct PreProcessingDescription: public boost::program_options::options_description {
    PreProcessingDescription();
  };
  struct CorrelationDescription: public boost::program_options::options_description {
    CorrelationDescription();
  };
  struct SubpixelDescription: public boost::program_options::options_description {
    SubpixelDescription();
  };
  struct FilteringDescription: public boost::program_options::options_description {
    FilteringDescription();
  };
  struct TriangulationDescription: public boost::program_options::options_description {
    TriangulationDescription();
  };
  struct GUIDescription: public boost::program_options::options_description {
    GUIDescription();
  };
  struct ParseDescription: public boost::program_options::options_description {
    ParseDescription();
  };
  struct ParallelDescription: public boost::program_options::options_description {
    ParallelDescription();
  };
  struct UndocOptsDescription: public boost::program_options::options_description {
    UndocOptsDescription();
  };

  boost::program_options::options_description
  generate_config_file_options(vw::GdalWriteOptions& opt);

  // This handles options which are not in stereo_settings(), but
  // rather in 'opt'. So they are not config options set in
  // stereo.default but only command-line options.
  void addAspGlobalOptions(boost::program_options::options_description & description,
                           ASPGlobalOptions & opt);


}

#endif//__ASP_CORE_STEREO_OPTIONS_DESC_H__

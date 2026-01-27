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

/// \file AsterProc.h
///
/// Utilities for processing ASTER Level 1A data, including HDF extraction
/// and radiometric corrections. See aster2asp for usage and references.

#ifndef __ASP_CORE_ASTERPROC_H__
#define __ASP_CORE_ASTERPROC_H__

#include <string>

// Forward declarations
namespace vw {
  class GdalWriteOptions;
}

namespace asp {

// Generate a temporary directory for HDF extraction
std::string genTmpDir(std::string const& output_prefix);

// Extract data from HDF file to temporary directory
void extractHdfData(std::string const& hdf_file, std::string const& hdfOutDir);

// Apply radiometric corrections to ASTER imagery
void applyRadiometricCorrections(std::string const& input_image,
                                 std::string const& corr_table,
                                 std::string const& out_image,
                                 vw::GdalWriteOptions const& opt);

} // namespace asp

#endif // __ASP_CORE_ASTERPROC_H__

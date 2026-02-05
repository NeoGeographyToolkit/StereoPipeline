/* Copyright (c) 2021, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
 * platform" software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <asp/Rig/SystemUtils.h>

#include <boost/filesystem.hpp>
#include <glog/logging.h>

namespace rig {

// Create a directory recursively, unless it exists already. This works like mkdir -p.
void createDir(std::string const& dir) {
  if (dir == "")
    return;  // This can be useful if dir was created with parent_path().

  if (!boost::filesystem::exists(dir)) {
    if (!boost::filesystem::create_directories(dir) || !boost::filesystem::is_directory(dir))
      LOG(FATAL) << "Failed to create directory: " << dir << "\n";
  }
}

}  // end namespace rig

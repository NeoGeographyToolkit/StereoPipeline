/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <Rig/sparse_map.h>
#include <Rig/sparse_mapping.h>
#include <Rig/tensor.h>

namespace sparse_mapping {
  
void SparseMap::InitializeCidFidToPid() {
  sparse_mapping::InitializeCidFidToPid(cid_to_filename_.size(),
                        pid_to_cid_fid_,
                        &cid_fid_to_pid_);
}

} // end namespace sparse_mapping

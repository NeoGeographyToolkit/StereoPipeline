#ifndef ASP_RIG_MERGE_MAPS_H
#define ASP_RIG_MERGE_MAPS_H

#include <string>

namespace rig {
  class CameraParameters;
}

namespace asp {
  class nvmData;
}

namespace rig {

class RigSet;
  
// Merge two maps
void MergeMaps(asp::nvmData const& A_in,
               asp::nvmData const& B_in,
               rig::RigSet const& R,
               int num_image_overlaps_at_endpoints,
               bool fast_merge,
               bool no_transform,
               double close_dist,
               std::string const& image_sensor_list,
               int num_threads, 
               asp::nvmData & C_out);

}  // namespace rig

#endif  // ASP_RIG_MERGE_MAPS_H

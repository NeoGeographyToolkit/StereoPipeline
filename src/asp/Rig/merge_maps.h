#ifndef RIG_CALIBRATOR_MERGE_MAPS_H_
#define RIG_CALIBRATOR_MERGE_MAPS_H_

namespace camera {
  class CameraParameters;
}

namespace rig {

class nvmData;
  
// Merge two maps
void MergeMaps(rig::nvmData const& A_in,
               rig::nvmData const& B_in,
               rig::RigSet const& R,
               int num_image_overlaps_at_endpoints,
               bool fast_merge,
               bool no_transform,
               double close_dist,
               std::string const& image_sensor_list, 
               rig::nvmData & C_out);

}  // namespace rig

#endif  // RIG_CALIBRATOR_MERGE_MAPS_H_

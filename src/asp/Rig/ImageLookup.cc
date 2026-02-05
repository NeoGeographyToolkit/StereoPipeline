/* Copyright (c) 2021-2026, United States Government, as represented by the
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

#include <asp/Rig/image_lookup.h>
#include <asp/Rig/camera_image.h>
#include <asp/Rig/rig_config.h>
#include <asp/Rig/basic_algs.h>
#include <asp/Rig/RigCameraParams.h>
#include <asp/Rig/interpolation_utils.h>
#include <asp/Rig/transform_utils.h>
#include <asp/Core/Nvm.h>
#include <asp/Rig/RigImageIO.h>

#include <vw/Core/Log.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

namespace fs = boost::filesystem;

namespace rig {
  
// Sort by timestamps adjusted to be relative to the ref camera clock.
// Additionally sort by image name, so that the order is deterministic.
bool timestampLess(cameraImage i, cameraImage j) {
  return (i.ref_timestamp < j.ref_timestamp) || 
    (i.ref_timestamp == j.ref_timestamp && i.image_name < j.image_name);
}

// Find an image at the given timestamp or right after it. We assume
// that during repeated calls to this function we always travel
// forward in time, and we keep track of where we are in the bag using
// the variable start_pos that we update as we go.
bool lookupImage(// Inputs
                 double desired_time, MsgMap const& msgs,
                 // Outputs
                 cv::Mat& image, std::string & image_name,
                 MsgMapIter& start_pos, double& found_time) {
  // Initialize the outputs. Note that start_pos is passed in from outside.
  image = cv::Mat();
  image_name = "";
  found_time = -1.0;

   int num_msgs = msgs.size();
   double prev_image_time = -1.0;

   for (auto local_pos = start_pos; local_pos != msgs.end(); local_pos++) {
     start_pos = local_pos;  // save this for exporting

     rig::ImageMessage const& imgMsg = local_pos->second; // alias
     found_time = imgMsg.timestamp;

     // Sanity check: We must always travel forward in time
     if (found_time < prev_image_time) {
       LOG(FATAL) << "Found images not in chronological order.\n"
                  << std::fixed << std::setprecision(17)
                  << "Times in wrong order: " << prev_image_time << ' ' << found_time << ".\n";
       continue;
     }

     prev_image_time = found_time;

     if (found_time >= desired_time) {
       // Found the desired data. Do a deep copy, to not depend on the
       // original structure.
       imgMsg.image.copyTo(image);
       image_name = imgMsg.name;
       return true;
     }
   }
  return false;
}

// A function to extract poses, filenames, and timestamps from read data.
void lookupFilesPoses(// Inputs
                      rig::RigSet const& R,
                      std::vector<std::map<double, rig::ImageMessage>> const& image_maps,
                      std::vector<std::map<double, rig::ImageMessage>> const& depth_maps,
                      // Outputs
                      std::vector<double>& ref_timestamps,
                      std::vector<Eigen::Affine3d> & world_to_ref) {

  // Sanity checks
  if (image_maps.size() != depth_maps.size() || image_maps.size() != R.cam_names.size())
    LOG(FATAL) << "Bookkeeping failure in lookupFilesPoses()\n";
  
  // Wipe the outputs
  ref_timestamps.clear();
  world_to_ref.clear();

  int num_cams = R.cam_names.size();
  for (size_t cam_type = 0; cam_type < image_maps.size(); cam_type++) {

    auto const& image_map = image_maps[cam_type];

    for (auto it = image_map.begin(); it != image_map.end(); it++) {
      // Collect the ref cam timestamps, world_to_ref, in chronological order
      if (R.isRefSensor(R.cam_names[cam_type])) {
        world_to_ref.push_back(it->second.world_to_cam);
        ref_timestamps.push_back(it->second.timestamp);
      }
    }
  }
}

// Look up each ref cam image by timestamp, with the rig assumption. In between
// any two ref cam timestamps, which are no further from each other than the
// bracket length, look up one or more images of each of the other camera types
// in the rig. If more than one choice, and bracket_single_image is true, choose
// the one closest to the midpoint of the two bracketing ref cam timestamps.
// This way there's more wiggle room later if one attempts to modify the
// timestamp offset.
// TODO(oalexan1): This is messy code but was developed with much testing,
// and it works. If it is to be cleaned up, it should be done with care.
// Especially the cases when the other cameras have exactly the same timestamp
// as the ref cam are tricky, and it gets even trickier at the last timestamp.
// Also need to consider the case when timestamp_offsets_max_change is true.
void lookupImagesAndBrackets(// Inputs
                             double bracket_len,
                             double timestamp_offsets_max_change,
                             bool bracket_single_image,
                             rig::RigSet   const& R,
                             std::vector<double> const& ref_timestamps,
                             std::vector<MsgMap> const& image_data,
                             std::vector<MsgMap> const& depth_data,
                             // Outputs
                             std::vector<rig::cameraImage>& cams,
                             std::vector<double>& min_timestamp_offset,
                             std::vector<double>& max_timestamp_offset) {

  vw::vw_out() << "Looking up the images and bracketing the timestamps." << std::endl;

  int num_ref_cams = ref_timestamps.size();
  int num_cam_types = R.cam_names.size();

  // Sanity checks
  if (R.cam_names.size() != image_data.size()) 
    LOG(FATAL) << "Expecting as many sensors as image datasets for them.\n";
  if (R.cam_names.size() != depth_data.size()) 
    LOG(FATAL) << "Expecting as many sensors as depth datasets for them.\n";
    
  // Initialize the outputs
  cams.clear();
  min_timestamp_offset.resize(num_cam_types, -1.0e+100);
  max_timestamp_offset.resize(num_cam_types,  1.0e+100);

  // A lot of care is needed with positions. This remembers how we travel in time
  // for each camera type so we have fewer messages to search.
  // But if a mistake is done below it will mess up this bookkeeping.
  std::vector<MsgMapIter> image_start_positions(num_cam_types);
  std::vector<MsgMapIter> depth_start_positions(num_cam_types);
  for (int cam_it = 0; cam_it < num_cam_types; cam_it++) {
    image_start_positions[cam_it] = image_data[cam_it].begin();
    depth_start_positions[cam_it] = depth_data[cam_it].begin();
  }

  double big = std::numeric_limits<double>::max();
  
  // Populate the data for each camera image
  for (int beg_ref_it = 0; beg_ref_it < num_ref_cams; beg_ref_it++) {

    // For when we have last ref timestamp and last other cam timestamp and they are equal
    int end_ref_it = beg_ref_it + 1;
    bool last_timestamp = (end_ref_it == num_ref_cams);
    if (last_timestamp) end_ref_it = beg_ref_it;

    for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
      std::vector<rig::cameraImage> local_cams;
      bool success = false;

      // The ref cam does not need bracketing, but the others need to be bracketed
      // by ref cam, so there are two cases to consider.

      if (R.isRefSensor(R.cam_names[cam_type])) {
        // Case of ref sensor
        rig::cameraImage cam;
        cam.camera_type   = cam_type;
        cam.timestamp     = ref_timestamps[beg_ref_it];
        cam.ref_timestamp = cam.timestamp;  // the time offset is 0 between ref and itself
        cam.beg_ref_index = beg_ref_it;
        cam.end_ref_index = beg_ref_it;  // same index for beg and end

        // Start looking up the image timestamp from this position. Some care
        // is needed here as we advance in time in image_start_positions[cam_type].
        double found_time = -1.0;
        // This has to succeed since this timestamp came from an existing image
        bool have_lookup =  
          rig::lookupImage(cam.timestamp, image_data[cam_type],
                           // Outputs
                           cam.image, cam.image_name, 
                           image_start_positions[cam_type], // this will move forward
                           found_time);
        
        if (!have_lookup)
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";

        // The exact time is expected
        if (found_time != cam.timestamp)
          LOG(FATAL) << std::fixed << std::setprecision(17)
                     << "Cannot look up camera at time " << cam.timestamp << ".\n";

        success = true;
        local_cams.push_back(cam);

      } else {
        // Case of not ref sensor
        // Need care here since sometimes ref_cam and current cam can have
        // exactly the same timestamp, so then bracketing should succeed.

        // Convert the bracketing timestamps to current cam's time
        double ref_to_cam_offset = R.ref_to_cam_timestamp_offsets[cam_type];
        double beg_timestamp     = ref_timestamps[beg_ref_it] + ref_to_cam_offset;
        double end_timestamp     = ref_timestamps[end_ref_it] + ref_to_cam_offset;
        if (end_timestamp == beg_timestamp && last_timestamp)  // necessary adjustment
          end_timestamp = std::nextafter(end_timestamp, big); 

        if (end_timestamp <= beg_timestamp)
          LOG(FATAL) << "Ref timestamps must be in strictly increasing order.\n";

        // Find the image timestamp closest to the midpoint of the brackets. This will give
        // more room to vary the timestamp later.
        double mid_timestamp = (beg_timestamp + end_timestamp)/2.0;

        // Search forward in time from image_start_positions[cam_type].
        // We will update that too later. One has to be very careful
        // with it so it does not go too far forward in time
        // so that at the next iteration we are passed what we
        // search for.
        MsgMapIter start_pos = image_start_positions[cam_type]; // care here
        double curr_timestamp = beg_timestamp;                  // start here
        double found_time = -1.0;
        std::vector<double> found_times;
        std::vector<cv::Mat> found_images;
        std::vector<std::string> found_image_names;
        while (1) {
          
          // Stop when we are past the end of the bracket
          if (found_time > end_timestamp) break;

          cv::Mat image;
          std::string image_name;
          bool have_lookup =
            rig::lookupImage(curr_timestamp, image_data[cam_type],
                             // Outputs
                             image, image_name,
                             // care here, start_pos moves forward
                             start_pos,
                             // found_time will be updated now
                             found_time);

          // Need not succeed, but then there's no need to go on as we
          // are at the end
          if (!have_lookup)
            break; 

          // Check if the found time is in the bracket. Note how we allow
          // found_time == beg_timestamp if there's no other choice.
          bool is_in_bracket = (beg_timestamp <= found_time && found_time < end_timestamp);
          double curr_dist = std::abs(found_time - mid_timestamp);

          // Must respect the bracket length, unless best time equals beg time,
          // as then the bracketing is not going to be used.
          bool fail_bracket_len = ((found_time > beg_timestamp && 
                                    end_timestamp - beg_timestamp > bracket_len));

          if (is_in_bracket && !fail_bracket_len) {
            // Update the start position for the future only if this is a good
            // solution. Otherwise we may have moved too far.
            image_start_positions[cam_type] = start_pos;
            
            // Record the found image
            found_images.push_back(cv::Mat());
            image.copyTo(found_images.back());
            found_times.push_back(found_time);
            found_image_names.push_back(image_name);
          }

          // Go forward in time. We count on the fact that
          // lookupImage() looks forward from given guess.
          // Careful here with the api of std::nextafter().
          curr_timestamp = std::nextafter(found_time, big);
        } // end while loop
        
        if (bracket_single_image) {
          // Must pick only one image, which is closest to the midpoint 
          cv::Mat best_image;
          std::string best_image_name;
          double best_dist = 1.0e+100;
          double best_time = -1.0;
          best_dist = 1.0e+100;
          // loop over found images to find the closest one to the midpoint. Save 
          // that as best image, and update best_time and best_image_name, and best_dist.
          for (size_t i = 0; i < found_times.size(); i++) {
            double curr_dist = std::abs(found_times[i] - mid_timestamp);
            if (curr_dist < best_dist) {
              best_dist = curr_dist;
              best_time = found_times[i];
              found_images[i].copyTo(best_image);
              best_image_name = found_image_names[i];
            }
          }

          if (best_time < 0.0) continue;  // bracketing failed
          
          // Make the vector of found images contain only the best image
          found_images.resize(1);
          found_times.resize(1);
          found_image_names.resize(1);
          best_image.copyTo(found_images[0]);
          found_times[0] = best_time;
          found_image_names[0] = best_image_name;
        }

        // Iterate ove found_times and add to local_cams
        for (size_t i = 0; i < found_times.size(); i++) {
          rig::cameraImage cam;
          cam.camera_type   = cam_type;
          cam.timestamp     = found_times[i];
          cam.ref_timestamp = found_times[i] - ref_to_cam_offset;
          cam.beg_ref_index = beg_ref_it;
          cam.end_ref_index = end_ref_it;
          found_images[i].copyTo(cam.image);
          cam.image_name = found_image_names[i];
          local_cams.push_back(cam);
        }
          
        success = (local_cams.size() > 0);
      } // end case of not ref sensor

      if (!success) continue;
      
      // Iterate over the local_cams and update the timestamp offset bounds
      for (size_t i = 0; i < local_cams.size(); i++) {
        auto & cam = local_cams[i];

        if (!R.isRefSensor(R.cam_names[cam_type])) { // Not a ref sensor
          double ref_to_cam_offset = R.ref_to_cam_timestamp_offsets[cam_type];

          // Assuming the option --bracket_single_image is used, 
          // cam.timestamp was chosen as centrally as possible so that
          // ref_timestamps[beg_ref_it] + ref_to_cam_offset <= cam.timestamp
          // and
          // cam.timestamp <= ref_timestamps[end_ref_it] + ref_to_cam_offset
          // Find the range of potential future values of ref_to_cam_offset so that
          // cam.timestamp still respects these bounds.
          min_timestamp_offset[cam_type]
            = std::max(min_timestamp_offset[cam_type],
                      cam.timestamp - ref_timestamps[end_ref_it]);
          max_timestamp_offset[cam_type]
            = std::min(max_timestamp_offset[cam_type],
                      cam.timestamp - ref_timestamps[beg_ref_it]);
        }

        // Look up the closest depth in time (either before or after cam.timestamp)
        // This need not succeed.
        cam.cloud_timestamp = -1.0;  // will change
        if (!depth_data.empty()) 
          rig::lookupImage(cam.timestamp,  // start looking from this time forward
                           depth_data[cam_type],
                           // Outputs
                           cam.depth_cloud, cam.depth_name, 
                           depth_start_positions[cam_type],  // this will move forward
                           cam.cloud_timestamp);             // found time
        
        cams.push_back(cam);
      } // end loop over local_cams
    }  // end loop over camera types
  }    // end loop over ref images

  // Adjust for timestamp_offsets_max_change. Printing the bounds is useful if
  // the timestamps can be allowed to change. Turn off printing this for now, as
  // it is rather confusing. 
  // vw::vw_out() << "If optimizing the timestamp offset, its bounds must be, per sensor:\n";
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    if (R.isRefSensor(R.cam_names[cam_type]))
      continue;  // bounds don't make sense here
    min_timestamp_offset[cam_type] = std::max(min_timestamp_offset[cam_type],
                                              R.ref_to_cam_timestamp_offsets[cam_type]
                                              - timestamp_offsets_max_change);
    max_timestamp_offset[cam_type] = std::min(max_timestamp_offset[cam_type],
                                              R.ref_to_cam_timestamp_offsets[cam_type]
                                              + timestamp_offsets_max_change);
    // Tighten the range a bit to ensure we don't exceed things when we add and
    // subtract timestamps later. Note that timestamps are measured in seconds
    // and fractions of a second since epoch and can be quite large so precision
    // loss can easily happen.
    double delta = (max_timestamp_offset[cam_type] - min_timestamp_offset[cam_type]);
    delta = std::max(std::min(delta/10.0, 1.0e-5), 0.0);
    min_timestamp_offset[cam_type] += delta;
    max_timestamp_offset[cam_type] -= delta;
    // Print the timestamp offset allowed ranges
    // vw::vw_out() << std::setprecision(8) << R.cam_names[cam_type]
    //           << ": [" << min_timestamp_offset[cam_type]
    //           << ", " << max_timestamp_offset[cam_type] << "]\n";
  }

}

// Assuming that the rig constraint is not used, initialize the 'cams' structure
// by copying each image and its other data in that structure as expected
// by later code. See also lookupImagesAndBrackets() when some selection based
// on bracketing takes place.
void lookupImagesNoBrackets(// Inputs
                            rig::RigSet const& R,
                            std::vector<MsgMap> const& image_data,
                            std::vector<MsgMap> const& depth_data,
                            // Outputs
                            std::vector<rig::cameraImage>& cams,
                            std::vector<double>& min_timestamp_offset,
                            std::vector<double>& max_timestamp_offset) {

  vw::vw_out() << "Looking up the images." << std::endl;
  int num_cam_types = R.cam_names.size();
  
  // Initialize the outputs
  cams.clear();
  min_timestamp_offset.resize(num_cam_types, -1.0e+100);
  max_timestamp_offset.resize(num_cam_types,  1.0e+100);

  // A lot of care is needed with positions. This remembers how we travel in time
  // for each camera type so we have fewer messages to search.
  // But if a mistake is done below it will mess up this bookkeeping.
  std::vector<MsgMapIter> image_start_positions(num_cam_types);
  std::vector<MsgMapIter> depth_start_positions(num_cam_types);
  for (int cam_it = 0; cam_it < num_cam_types; cam_it++) {
    image_start_positions[cam_it] = image_data[cam_it].begin();
    depth_start_positions[cam_it] = depth_data[cam_it].begin();
  }

  // Populate the data for each camera image
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {

    int cam_it = -1;
    for (auto map_it = image_data[cam_type].begin(); map_it != image_data[cam_type].end();
         map_it++) {
      cam_it++;
      
      rig::cameraImage cam;
      cam.camera_type   = cam_type;
      cam.timestamp     = (map_it->second).timestamp;
      cam.ref_timestamp = cam.timestamp; // no rig, so no timestamp offset
      // These two values below should not be needed with no rig
      cam.beg_ref_index = cam_it;
      cam.end_ref_index = cam_it;

      // Start looking up the image timestamp from this position. Some care
      // is needed here as we advance in time in image_start_positions[cam_type].
      double found_time = -1.0;
      // This has to succeed since this timestamp originally came from an existing image
      bool have_lookup =  
        rig::lookupImage(cam.timestamp, image_data[cam_type],
                         // Outputs
                         cam.image, cam.image_name, 
                         image_start_positions[cam_type],  // this will move forward
                         found_time);
      if (!have_lookup)
        LOG(FATAL) << std::fixed << std::setprecision(17)
                   << "Cannot look up camera at time " << cam.timestamp << ".\n";

      // The exact time is expected
      if (found_time != cam.timestamp)
        LOG(FATAL) << std::fixed << std::setprecision(17)
                   << "Cannot look up camera at time " << cam.timestamp << ".\n";
      
      // Look up the closest cloud in time (either before or after cam.timestamp)
      // This need not succeed.
      cam.cloud_timestamp = -1.0;  // will change
      if (!depth_data.empty()) 
        rig::lookupImage(cam.timestamp,  // start looking from this time forward
                               depth_data[cam_type],
                               // Outputs
                               cam.depth_cloud, cam.depth_name, 
                               depth_start_positions[cam_type],  // this will move forward
                               cam.cloud_timestamp);             // found time

      // Accept this camera
      cams.push_back(cam);
    }  // end loop over camera types
  }    // end loop over ref images

  return;
}

// Look up images, with or without the rig constraint. See individual functions
// below for more details.
void lookupImagesOneRig(// Inputs
                        bool no_rig, double bracket_len,
                        double timestamp_offsets_max_change,
                        bool bracket_single_image,
                        rig::RigSet const& R,
                        std::vector<MsgMap> const& image_maps,
                        std::vector<MsgMap> const& depth_maps,
                        // Outputs
                        std::vector<double>                 & ref_timestamps,
                        std::vector<Eigen::Affine3d>        & world_to_ref,
                        std::vector<rig::cameraImage> & cams,
                        std::vector<Eigen::Affine3d>        & world_to_cam,
                        std::vector<double>                 & min_timestamp_offset,
                        std::vector<double>                 & max_timestamp_offset) {
  
  rig::lookupFilesPoses(// Inputs
                        R, image_maps, depth_maps,
                        // Outputs
                        ref_timestamps, world_to_ref);
  
  if (!no_rig) 
    lookupImagesAndBrackets(// Inputs
                            bracket_len,  
                            timestamp_offsets_max_change,  
                            bracket_single_image,
                            R, ref_timestamps,  image_maps, depth_maps,  
                            // Outputs
                            cams, min_timestamp_offset, max_timestamp_offset);
  else
    lookupImagesNoBrackets(// Inputs
                           R, image_maps, depth_maps,  
                           // Outputs
                           cams, min_timestamp_offset, max_timestamp_offset);
  
  // See how many timestamps we have for each camera
  std::map<int, int> num_images;
  int num_cam_types = R.cam_names.size();
  for (int cam_type_it = 0; cam_type_it < num_cam_types; cam_type_it++)
    num_images[cam_type_it] = 0;
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++)
    num_images[cams[cam_it].camera_type]++;
  bool is_good = true;
  for (int cam_type_it = 0; cam_type_it < num_cam_types; cam_type_it++) {
    vw::vw_out() << "Number of images for sensor: " << R.cam_names[cam_type_it] << ": "
              << num_images[cam_type_it] << std::endl;

    if (num_images[cam_type_it] == 0)
      is_good = false;
  }
  if (!is_good)
    vw::vw_out(vw::WarningMessage) << "Could not find images for all sensors.\n";

  // The images may need to be resized to be the same
  // size as in the calibration file. Sometimes the full-res images
  // can be so blurry that interest point matching fails, hence the
  // resizing.
  for (size_t it = 0; it < cams.size(); it++)
    rig::adjustImageSize(R.cam_params[cams[it].camera_type], cams[it].image);

  // Sort by the timestamp in reference camera time. This is essential
  // for matching each image to other images close in time. Note
  // that this does not affect the book-keeping of beg_ref_index
  // and end_ref_it in this vector because those indices point to
  // world_to_ref and ref_timestamp, which do not change.
  // Note that this happens for each rig. So, different rigs
  // have the ref timestamps individually sorted, and later
  // these will be concatenated.
  std::sort(cams.begin(), cams.end(), rig::timestampLess);

  // Parse the transform from the world to each cam, which were known
  // on input. Later, if use_initial_rig_transform is specified,
  // these will be computed based on the rig.  Since
  // image_maps[cam_type] is sorted chronologically, travel in time
  // along at as we move along the cams array. Use the two arrays
  // below to remember where we left off.
  // TODO(oalexan1): This is fragile. It relies on cams being sorted by time.
  // Make the cams array have a world_to_cam entry and remove the loop below.
  world_to_cam.resize(cams.size());
  std::vector<MsgMapIter> beg_pos(num_cam_types); 
  std::vector<MsgMapIter> end_pos(num_cam_types); 
  for (int cam_type = 0; cam_type < num_cam_types; cam_type++) {
    beg_pos[cam_type] = image_maps[cam_type].begin();
    end_pos[cam_type] = image_maps[cam_type].end();
  }
  for (size_t cam_it = 0; cam_it < cams.size(); cam_it++) {
    int cam_type = cams[cam_it].camera_type;
    for (auto pos = beg_pos[cam_type]; pos != end_pos[cam_type]; pos++) {
      if (cams[cam_it].timestamp == pos->first) {
        world_to_cam[cam_it] = (pos->second).world_to_cam;
        beg_pos[cam_type] = pos;  // save for next time
        break;
      }
    }
  }  
  return; 
}

// Look up images for a set of rigs. This requires looking up images for individual rigs,
// then concatenating the results and adjusting the book-keeping.
void lookupImages(// Inputs
                  bool no_rig, double bracket_len,
                  double timestamp_offsets_max_change,
                  bool bracket_single_image,
                  rig::RigSet const& R,
                  std::vector<MsgMap> const& image_maps,
                  std::vector<MsgMap> const& depth_maps,
                  // Outputs
                  std::vector<double>                 & ref_timestamps,
                  std::vector<Eigen::Affine3d>        & world_to_ref,
                  std::vector<rig::cameraImage> & cams,
                  std::vector<Eigen::Affine3d>        & world_to_cam,
                  std::vector<double>                 & min_timestamp_offset,
                  std::vector<double>                 & max_timestamp_offset) {

  // Wipe the outputs
  ref_timestamps.clear();
  world_to_ref.clear();
  cams.clear();
  world_to_cam.clear();
  min_timestamp_offset.clear();
  max_timestamp_offset.clear();

  for (size_t rig_id = 0; rig_id < R.cam_set.size(); rig_id++) {

    // Create a single rig
    rig::RigSet sub_rig = R.subRig(rig_id);

    // Prepare the inputs for the subrig
    std::vector<MsgMap> sub_image_maps;
    std::vector<MsgMap> sub_depth_maps;
    for (size_t sub_it = 0; sub_it < sub_rig.cam_names.size(); sub_it++) {
      std::string sensor_name = sub_rig.cam_names[sub_it];
      int rig_set_it = R.sensorIndex(sensor_name); // index in the larger rig
      sub_image_maps.push_back(image_maps[rig_set_it]);
      sub_depth_maps.push_back(depth_maps[rig_set_it]);
    }

    std::vector<double>                 sub_ref_timestamps;
    std::vector<Eigen::Affine3d>        sub_world_to_ref;
    std::vector<rig::cameraImage> sub_cams;
    std::vector<Eigen::Affine3d>        sub_world_to_cam;
    std::vector<double>                 sub_min_timestamp_offset;
    std::vector<double>                 sub_max_timestamp_offset;

    // Do the work for the subrig
    lookupImagesOneRig(// Inputs
                       no_rig, bracket_len, timestamp_offsets_max_change, 
                       bracket_single_image,
                       sub_rig,  
                       sub_image_maps, sub_depth_maps,  
                       // Outputs
                       sub_ref_timestamps, sub_world_to_ref, sub_cams,  
                       sub_world_to_cam, sub_min_timestamp_offset, sub_max_timestamp_offset);

    // Save the endpoints for ref timestamps and all cams, before concatenation
    size_t prev_ref_end = ref_timestamps.size();
    size_t prev_end = cams.size();

    // Append the answers
    ref_timestamps.insert(ref_timestamps.end(), sub_ref_timestamps.begin(),
                          sub_ref_timestamps.end());
    world_to_ref.insert(world_to_ref.end(), sub_world_to_ref.begin(), sub_world_to_ref.end());
    cams.insert(cams.end(), sub_cams.begin(), sub_cams.end());
    world_to_cam.insert(world_to_cam.end(), sub_world_to_cam.begin(), sub_world_to_cam.end());
    min_timestamp_offset.insert(min_timestamp_offset.end(), sub_min_timestamp_offset.begin(),
                                sub_min_timestamp_offset.end());
    max_timestamp_offset.insert(max_timestamp_offset.end(), sub_max_timestamp_offset.begin(),
                                sub_max_timestamp_offset.end());

    // Update the bookkeeping in 'cams'
    for (size_t cam_it = prev_end; cam_it < cams.size(); cam_it++) {

      // Find the current sensor index in the larger rig set
      int subrig_sensor_index = cams[cam_it].camera_type;
      std::string subrig_sensor = sub_rig.cam_names[subrig_sensor_index];
      int rig_sensor_index = R.sensorIndex(subrig_sensor);
      cams[cam_it].camera_type = rig_sensor_index;

      // Update the pointers to indices in ref_timestamps
      cams[cam_it].beg_ref_index += prev_ref_end;     
      cams[cam_it].end_ref_index += prev_ref_end;
    }
  }

  return;
}

// Find pointers to the camera and reference images that bracket the
// camera image. Great care is needed here. Two cases are considered,
// if there is a rig or not. If no_rig is true, then the reference images are
// the same as the camera images. 
void calcBracketing(// Inputs
                  bool no_rig, int cid, int cam_type,
                  std::vector<rig::cameraImage> const& cams,
                  std::vector<double> const& ref_timestamps,
                  rig::RigSet   const& R,
                  // Will not be changed but need access
                  std::vector<double> & world_to_cam_vec,
                  std::vector<double> & world_to_ref_vec,
                  std::vector<double> & ref_to_cam_vec,
                  std::vector<double> & ref_identity_vec,
                  std::vector<double> & right_identity_vec,
                  // Outputs
                  double* & beg_cam_ptr, 
                  double* & end_cam_ptr, 
                  double* & ref_to_cam_ptr,
                  double  & beg_ref_timestamp, 
                  double  & end_ref_timestamp,
                  double  & cam_timestamp) {

  if (!no_rig) {
    // Model the rig, use timestamps
    int beg_ref_index = cams[cid].beg_ref_index;
    int end_ref_index = cams[cid].end_ref_index;

    // Left bracketing ref cam for a given cam. For a ref cam, this is itself.
    beg_cam_ptr = &world_to_ref_vec[rig::NUM_RIGID_PARAMS * beg_ref_index];

    // Right bracketing camera. When the cam is the ref type,
    // or when this cam is the last one and has exactly
    // same timestamp as the ref cam, this is not used.
    if (R.isRefSensor(R.cam_names[cam_type]) || beg_ref_index == end_ref_index)
      end_cam_ptr = &right_identity_vec[0];
    else
      end_cam_ptr = &world_to_ref_vec[rig::NUM_RIGID_PARAMS * end_ref_index];

    // The beg and end timestamps will be the same only for the
    // ref cam or for last non-ref cam whose timestamp is same
    // as ref cam timestamp which is also last.
    beg_ref_timestamp = ref_timestamps[beg_ref_index];
    end_ref_timestamp = ref_timestamps[end_ref_index];
    cam_timestamp = cams[cid].timestamp;  // uses current camera's clock

  } else {
    // No rig. Then, beg_cam_ptr is just current camera, not the
    // ref bracketing cam, end_cam_ptr is the identity. The timestamps
    // will be the same, so the camera brackets itself.
    cam_timestamp     = cams[cid].timestamp;
    beg_ref_timestamp = cam_timestamp;
    end_ref_timestamp = cam_timestamp;

    // Note how we use world_to_cam_vec and not world_to_ref_vec for 
    // the beg cam. The end cam is unused.
    beg_cam_ptr = &world_to_cam_vec[rig::NUM_RIGID_PARAMS * cid];
    end_cam_ptr = &right_identity_vec[0];
  }

  // Transform from reference camera to given camera. Won't be used when
  // FLAGS_no_rig is true or when the cam is of ref type.
  if (no_rig || R.isRefSensor(R.cam_names[cam_type]))
    ref_to_cam_ptr = &ref_identity_vec[0];
  else
    ref_to_cam_ptr = &ref_to_cam_vec[rig::NUM_RIGID_PARAMS * cam_type];

  return;
}
  
}  // end namespace rig

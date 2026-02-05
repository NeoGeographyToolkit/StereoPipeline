/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
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
#include <asp/Rig/RigThread.h>
#include <asp/Rig/RigConfig.h>
#include <asp/Rig/BasicAlgs.h>
#include <asp/Core/Nvm.h>

#include <opencv2/features2d/features2d.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Given a map, extract a submap with only specified images. Works
// with nvm files where features are either shifted relative
// to the optical center or not, and saves the submap in the same format.

// Can be useful if the map failed to build properly, but parts of it
// are still salvageable. Those can be extracted, new small maps can
// be created of the region that failed, then all maps can be merged
// together.

// It is suggested that each extracted submap consist only of consecutive
// images (each image overlaps with the one before and after it). Such
// submaps are easier to merge.

// Usage:
// sfm_submap -input_map <input map> -output_map <output map> <images to keep>
//
// sfm_submap -input_map <input map> -output_map <output map> -image_list <file>

DEFINE_string(input_map, "",
              "The input map, in .nvm format.");

DEFINE_string(output_map, "",
              "The output map, in .nvm format.");

DEFINE_string(image_list, "",
              "A file having the names of the images to be included in "
              "the submap, one per line.");

void parameterValidation() {
  if (FLAGS_input_map == "")
    LOG(FATAL) << "Must specify the input map.\n";
  
  if (FLAGS_output_map == "")
    LOG(FATAL) << "Must specify the output map.\n";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  parameterValidation();

  std::vector<std::string> images_to_keep;
  if (FLAGS_image_list == "") {
    // Get the images from the command line
    for (int i = 1; i < argc; i++)
      images_to_keep.push_back(argv[i]);
  } else {
    // Get the images from a file
    std::string image;
    std::ifstream image_handle(FLAGS_image_list);
    while (image_handle >> image)
      images_to_keep.push_back(image);
  }

  asp::nvmData nvm;
  asp::readNvm(FLAGS_input_map,
               nvm.cid_to_keypoint_map,
               nvm.cid_to_filename,
               nvm.pid_to_cid_fid,
               nvm.pid_to_xyz,
               nvm.world_to_cam,
               nvm.focal_lengths);
 
  std::string offsets_file = asp::offsetsFilename(FLAGS_input_map);
  if (!fs::exists(offsets_file))
    std::cout << "WARNING: No offsets file found. Will not write offsets for the submap.\n";
  else 
    asp::readNvmOffsets(offsets_file, nvm.optical_centers);

  // Extract the submap. Will also extract a subset of the optical centers.
  asp::ExtractSubmap(images_to_keep, nvm);

  asp::writeNvm(nvm.cid_to_keypoint_map,
                nvm.cid_to_filename,
                nvm.pid_to_cid_fid,
                nvm.pid_to_xyz,
                nvm.world_to_cam,
                FLAGS_output_map);

  if (std::ifstream(offsets_file)) {
    std::string output_offsets_file = asp::offsetsFilename(FLAGS_output_map);
    asp::writeNvmOffsets(output_offsets_file, nvm.optical_centers);
  }

  return 0;
}


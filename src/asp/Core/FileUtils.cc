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

#include <asp/Core/FileUtils.h>

#include <vw/FileIO/FileTypes.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace fs = boost::filesystem;

namespace asp {

  // Return true if the first file exists and is newer than all of the other files.
  bool first_is_newer(std::string              const& test_file, 
                      std::vector<std::string> const& other_files) {
    if (!boost::filesystem::exists(test_file))
      return false;
    std::time_t test_time = boost::filesystem::last_write_time(test_file);
    for (size_t i = 0; i < other_files.size(); i++) {
      if (other_files[i] == "") // Ignore blank files that were passed in.
        continue;
      if (!boost::filesystem::exists(other_files[i]))
        return false;
      std::time_t t = boost::filesystem::last_write_time(other_files[i]);
      if (test_time < t)
        return false;
    }
    return true;
  }

  bool first_is_newer(std::string const& test_file, std::string const& f1) {
    std::vector<std::string> vec(1);
    vec[0] = f1;
    return first_is_newer(test_file, vec);
  }
  bool first_is_newer(std::string const& test_file, 
                      std::string const& f1, std::string const& f2) {
    std::vector<std::string> vec(2);
    vec[0] = f1;  vec[1] = f2;
    return first_is_newer(test_file, vec);
  }
  bool first_is_newer(std::string const& test_file, 
                      std::string const& f1, std::string const& f2,
                      std::string const& f3, std::string const& f4) {
    std::vector<std::string> vec(4);
    vec[0] = f1;  vec[1] = f2;
    vec[2] = f2;  vec[3] = f4;
    return first_is_newer(test_file, vec);
  }

  // Read a vector of doubles from a file
  void read_vec(std::string const& filename, std::vector<double> & vals) {
    vals.clear();
    std::ifstream ifs(filename.c_str());
    if (!ifs.good()) 
      vw::vw_throw(vw::ArgumentErr() << "Could not open file: " << filename);
    
    double val;
    while (ifs >> val)
      vals.push_back(val);
    ifs.close();
  }

  void read_2d_points(std::string const& file, std::vector<vw::Vector2> & points) {

    std::ifstream ifs(file.c_str());
    if (!ifs.good())
      vw::vw_throw(vw::ArgumentErr() << "Could not open file for reading: " << file << "\n");
  
    double a, b;
    points.clear();
    while (ifs >> a >> b){
      vw::Vector2 p(a, b);
      points.push_back(p);
    }
  }

  void read_3d_points(std::string const& file,
                      std::vector<vw::Vector3> & points) {

    std::ifstream ifs(file.c_str());
    if (!ifs.good())
      vw::vw_throw(vw::ArgumentErr() << "Could not open file for reading: " << file << "\n");

    double a, b, c;
    points.clear();
    while (ifs >> a >> b >> c) {
      vw::Vector3 p(a, b, c);
      points.push_back(p);
    }
  }

// Write a vector of strings from a file, one per line.
void write_list(std::string const& file, std::vector<std::string> const & list) {
  std::ofstream fh(file.c_str());
  for (size_t i = 0; i < list.size(); i++)
    fh << list[i] << std::endl;
  fh.close();
}

// Read a vector of strings from a file, with spaces and newlines acting as separators.
// Throw an exception if the list is empty.
void read_list(std::string const& file, std::vector<std::string> & list) {
  list.clear();
  std::ifstream fh(file);
  std::string val;
  while (fh >> val)
    list.push_back(val);

  if (list.empty())
    vw::vw_throw(vw::ArgumentErr() << "Could not read any entries from: " << file << ".\n");

  fh.close();
}

// Read the target name (planet name) from the plain text portion of an ISIS cub
// file or from a CSM file. Will return "UNKNOWN" if the target name cannot be
// found.
std::string read_target_name(std::string const& filename) {

  std::string target = "UNKNOWN";
  
  std::ifstream handle;
  handle.open(filename.c_str());
  if (handle.fail())
    return target; // No luck
  
  std::string line;
  int count = 0;
  while (getline(handle, line, '\n')) {
    if (line == "End") 
      return target; // No luck, reached the end of the text part of the cub

    // If the input file is a .tif rather than a .cub, it could have several
    // GB and not have this info anyway, so exist early.
    count++;
    if (count > 1000)
      break;
    
    // Find the line having TargetName
    boost::to_lower(line);
    if (line.find("targetname") == std::string::npos) 
      continue;

    // Replace the equal sign with a space and read the second
    // non-space entry from this line
    boost::replace_all(line, "=", " ");
    std::istringstream iss(line);
    std::string val;
    if (! (iss >> val >> target)) 
      continue;

    // Wipe any commas, quotes, or spaces (this is for CSM files)
    boost::replace_all(target, ",", "");
    boost::replace_all(target, "\"", "");
    boost::replace_all(target, " ", "");
    
    boost::to_upper(target);
    
    // If empty, return UNKNOWN
    if (target == "")
       target = "UNKNOWN";
    
    // Found a target, no need to go on   
    return target;
  }
  
  return target;
}

// Given a vector of files, with each file being an image, camera,
// or a text file having images or cameras, return the list of
// all found images and cameras. This is a local auxiliary 
// function not exposed in the header file.
void readImagesCamsOrLists(std::vector<std::string> const & in,
                           std::vector<std::string>       & out) {

  // Wipe the output
  out.clear();

  for (size_t i = 0; i < in.size(); i++) {

    if (vw::has_image_extension(in[i]) || vw::has_cam_extension(in[i])) {
    
      // Simply append the image or camera to the list
      out.push_back(in[i]);
      
    } else {

      // Read the list, append all entries from it
      std::string ext = vw::get_extension(in[i]);
      if (ext == ".txt") {

        std::vector<std::string> list;
        asp::read_list(in[i], list);
        for (size_t j = 0; j < list.size(); j++) 
            out.push_back(list[j]);
            
      } else if (boost::iends_with(in[i], ".adjust")) {
        vw::vw_throw(vw::ArgumentErr() << "The file " << in[i] << " is an adjustment. "
                  << "Use the original cameras and the option "
                  << "--bundle-adjust-prefix.\n");
      } else {
        vw::vw_throw(vw::ArgumentErr() << "Unknown file type passed on input: "
          << in[i] << ".\n");
      }
    }
  }

  return;
} 

// Given a list of images/cameras and/or lists of such things, put the images
// and the cameras in separate vectors.
void separate_images_from_cameras(std::vector<std::string> const& inputs,
                                  std::vector<std::string>      & images,
                                  std::vector<std::string>      & cameras,
                                  bool ensure_equal_sizes) {

  // There are N images and possibly N camera paths.
  // There are several situations:
  // 1. img1.cub ... imgN.cub for ISIS with non-proj images
  // 2. img1.tif ... imgN.tif img1.cub ... imgN.cub for ISIS with proj images
  // 3. img1.tif ... imgN.tif for RPC with embedded RPC in the tif files 
  // 4. img1.tif ... imgN.tif cam1 .... camN for all other cases.

  // Consider the case when some of the inputs are lists of images/cameras
  std::vector<std::string> inputs2;
  readImagesCamsOrLists(inputs, inputs2);

  // Check that all files exist
  for (size_t i = 0; i < inputs2.size(); i++) {
    if (!fs::exists(inputs2[i])) {
      vw::vw_throw(vw::ArgumentErr() << "Cannot find the file: " << inputs2[i] << ".\n");
      return;
    }
  }
  
  // Images and cameras may be interleaved. Separate them.
  images.clear();
  cameras.clear();
  for (size_t i = 0; i < inputs2.size(); i++) {
    if (vw::has_image_extension(inputs2[i]))
      images.push_back(inputs2[i]);
    else if (vw::has_cam_extension(inputs2[i]))
      cameras.push_back(inputs2[i]);
    else if (boost::iends_with(inputs2[i], ".adjust"))
      vw::vw_throw(vw::ArgumentErr() << "The file " << inputs2[i] << " is an adjustment. "
                << "Use the original cameras and the option "
                << "--bundle-adjust-prefix.\n");  
    else 
      vw::vw_throw(vw::ArgumentErr() << "Unknown file type passed on input: "
                << inputs2[i] << ".\n");
  }

  // Then concatenate them again, but with the images first and the cameras
  // second.
  inputs2.clear();
  for (size_t i = 0; i < images.size(); i++)  
    inputs2.push_back(images[i]);
  for (size_t i = 0; i < cameras.size(); i++) 
    inputs2.push_back(cameras[i]);
  images.clear();
  cameras.clear();

  // See if we have cub files and/or camera files (.cub files are also cameras)  
  bool has_cub    = false;
  bool has_nocub  = false;
  bool has_cam    = false;
  for (size_t i = 0; i < inputs2.size(); i++) {
    std::string ext = vw::get_extension(inputs2[i]);
    if (ext == ".cub")
      has_cub   = true;
    if (ext != ".cub")
      has_nocub = true;
    if (vw::has_cam_extension(inputs2[i]))
      has_cam   = true;
  }
  
  // Let the first half of the data be images, and the second half be cameras.
  // Unless we have only .cub files, when all the data are images.
  if ((has_cub && !has_nocub) || (!has_cam)) {
    // Only cubes, or only non-cameras, cases 1 and 3 above
    for (size_t i=0; i < inputs2.size(); ++i) 
      images.push_back(inputs2[i]);
  } else {
    // Images and cameras (cameras could be cubes)
    if (inputs2.size() % 2 != 0)
      vw::vw_throw(vw::ArgumentErr() << "Expecting as many images as cameras.\n");
    
    int half = inputs2.size()/2;
    for (int i = 0;    i < half;   i++) images.push_back(inputs2[i]);
    for (int i = half; i < 2*half; i++) cameras.push_back(inputs2[i]);
  }
  
  // Verification for images
  for (size_t i = 0; i < images.size(); i++) {
    if (!vw::has_image_extension(images[i])) {
      vw::vw_throw(vw::ArgumentErr() 
                   << "Expecting an image, got: " << images[i] << ".\n");
    }
  }

  // Verification for cameras
  for (size_t i = 0; i < cameras.size(); i++) {
    if (!vw::has_cam_extension(cameras[i])) {
      vw::vw_throw(vw::ArgumentErr() 
                   << "Expecting a camera, got: " << cameras[i] << ".\n");
    }
  }

  if (images.size() != cameras.size() && !cameras.empty()) {
    vw::vw_throw(vw::ArgumentErr() 
                 << "Expecting the number of images and cameras to agree.\n");
  }

  if (ensure_equal_sizes) {
    while (cameras.size() < images.size())
      cameras.push_back("");
  }
  
  return;
}

// Return the path to the left stats file
std::string leftStatsFile(std::string const& out_prefix) {
  return out_prefix + "-lStats.tif";
}

// Return the path to the right stats file
std::string rightStatsFile(std::string const& out_prefix) {
  return out_prefix + "-rStats.tif";
}

} // end namespace asp

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

#ifndef __CORE_FILE_UTILS_H__
#define __CORE_FILE_UTILS_H__

#include <string>
#include <vector>

#include <vw/Math/Vector.h>

namespace asp {

  /// Return true if the first file exists and is newer than all of the other files.
  /// - Also returns false if any files are missing.
  /// - If a blank "" file is passed in it is ignored.
  bool first_is_newer(std::string              const& test_file, 
                      std::vector<std::string> const& other_files);
  // Convenience wrappers.
  bool first_is_newer(std::string const& test_file, std::string const& f1);
  bool first_is_newer(std::string const& test_file, 
                      std::string const& f1, std::string const& f2);
  bool first_is_newer(std::string const& test_file, 
                      std::string const& f1, std::string const& f2,
                      std::string const& f3, std::string const& f4);

  /// Read a vector of doubles from a file  
  void read_vec(std::string const& filename, std::vector<double> & vals);

  void read_2d_points(std::string const& file, std::vector<vw::Vector2> & points);
  void read_3d_points(std::string const& file, std::vector<vw::Vector3> & points);

  /// Write a vector of strings from a file, one per line.
  void write_list(std::string const& file, std::vector<std::string> const & list);

  /// Read a vector of strings from a file, with spaces and newlines acting as separators.
  /// Throw an exception if the list is empty.
  void read_list(std::string const& file, std::vector<std::string> & list);
  
  /// Read the target name (planet name) from the plain text portion of an ISIS cub file
  std::string read_target_name(std::string const& filename);

  // Given a vector of files, with each file being an image, camera,
  // or a text file having images or cameras, return the list of
  // all found images and cameras. This is a local auxiliary 
  // function not exposed in the header file.
  void readImagesCamsOrLists(std::vector<std::string> const & in,
                            std::vector<std::string>       & out);

  /// Given a list of images/cameras and/or lists of such things, put the images
  /// and the cameras in separate vectors.
void separate_images_from_cameras(std::vector<std::string> const& inputs,
                                  std::vector<std::string>      & images,
                                  std::vector<std::string>      & cameras,
                                  bool ensure_equal_sizes);

  // Read a matrix of vectors from a file or string. Each line has n elements
  // (2 for Vector2, 3 for Vector3). Empty lines separate matrix rows.
  void read_matrix_from_file(std::string const& file,
                             std::vector<std::vector<vw::Vector2>> & mat);
  void read_matrix_from_file(std::string const& file,
                             std::vector<std::vector<vw::Vector3>> & mat);
  void read_matrix_from_string(std::string const& str,
                               std::vector<std::vector<vw::Vector2>> & mat);
  void read_matrix_from_string(std::string const& str,
                               std::vector<std::vector<vw::Vector3>> & mat);

// Create symlinks to the input images for skip_image_normalization mode.
// The symlinks are relative to the output directory.
void createSymLinks(std::string const& left_input_file,
                    std::string const& right_input_file,
                    std::string const& out_prefix,
                    std::string      & left_output_file,
                    std::string      & right_output_file);

} //end namespace asp

#endif//__CORE_FILE_UTILS_H__

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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

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

  // Consider a stream, like a text file. Each line has n elements,
  // to be read in a vector. Every now and then there is an empty
  // line. Put all vectors between two empty lines as the row in a
  // matrix. Hence we get a matrix of vectors.
  template<class StreamT, class VectorT>
  void read_matrix_from_stream(std::string const & file,
                               StreamT & str, std::vector<std::vector<VectorT>> & mat) {
    mat.clear();
    bool first_row = true;
    int num_cols = 0;
    std::vector<VectorT> row;
    char line[2048];
    while (str.getline(line, 2048)) {

      // An empty line or one starting with a space is a separator
      if ( (line[0] == '\0' || line[0] == ' ') && !row.empty()) {

        if (first_row) {
          num_cols = row.size();
          first_row = false;

        }

        if (num_cols != int(row.size())) {
          vw::vw_throw( vw::ArgumentErr()
                        << "Failed parsing a matrix from: " << file
                        << ". Not all rows have the same size.\n" );
        }

        mat.push_back(row);

        row.clear();       // reset
        continue;
      }

      if (line[0] == '\0' || line[0] == ' ') continue;

      // Read elements
      std::istringstream is(line);
      VectorT v;
      for (size_t p = 0; p < v.size(); p++) {
        if (! (is >> v[p]) ){
          vw::vw_throw( vw::ArgumentErr() << "Failed parsing " << v.size()
                        << " elements from line " << std::string(line)
                        << " in file " << file << "\n");
        }
      }

      row.push_back(v);
    }

    // last row
    if (!row.empty()) 
      mat.push_back(row);

  }

  template<class VectorT>
  void read_matrix_from_file(std::string const & file,
                             std::vector< std::vector<VectorT> > & mat){
    std::ifstream ifs(file.c_str());
    read_matrix_from_stream(file, ifs, mat);
  }
  
  template<class VectorT>
  void read_matrix_from_string(std::string const & str,
                               std::vector< std::vector<VectorT> > & mat){
    std::istringstream ifs(str);
    read_matrix_from_stream(str, ifs, mat);
  }

} //end namespace asp

#endif//__CORE_FILE_UTILS_H__

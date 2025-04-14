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

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <asp/Core/FileUtils.h>

namespace asp{

  using namespace vw;

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

  void read_1d_points(std::string const& file, std::vector<double> & points){

    std::ifstream ifs(file.c_str());
    if (!ifs.good())
      vw_throw( ArgumentErr() << "Could not open file for reading: " << file << "\n" );

    double a;
    points.clear();
    while (ifs >> a)
      points.push_back(a);
  }

  void read_2d_points(std::string const& file, std::vector<Vector2> & points){

    std::ifstream ifs(file.c_str());
    if (!ifs.good())
      vw_throw( ArgumentErr() << "Could not open file for reading: " << file << "\n" );
  
    double a, b;
    points.clear();
    while (ifs >> a >> b){
      Vector2 p(a, b);
      points.push_back(p);
    }
  }

  void read_3d_points(std::string const& file,
                      std::vector<Vector3> & points){

    std::ifstream ifs(file.c_str());
    if (!ifs.good())
      vw_throw( ArgumentErr() << "Could not open file for reading: " << file << "\n" );

    double a, b, c;
    points.clear();
    while (ifs >> a >> b >> c){
      Vector3 p(a, b, c);
      points.push_back(p);
    }
  }
  
}

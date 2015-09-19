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


/// \file results.cc
///

/************************************************************************
 *     File: results.cc
 *     Date: December 2008
 *       By: Ara Nefian and Michael Broxton
 *      For: NASA Ames Research Center, Intelligent Mechanisms Group
 * Function: Main program for the measuring the performance and accuracy of the stereo pipeline
 ************************************************************************/
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/FileIO.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>

using namespace vw;
using namespace vw::math;
using namespace vw::camera;
using namespace vw::stereo;

#include <asp/Core/StereoSettings.h>
#include <asp/Sessions.h>

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1
#include <asp/IsisIO/DiskImageResourceIsis.h>
#endif

// Boost headers
#include <boost/thread/xtime.hpp>
// Posix time is not fully supported in the version of Boost for RHEL
// Workstation 4
#ifdef __APPLE__
#include <boost/date_time/posix_time/posix_time.hpp>
#else
#include <ctime>
#endif


using namespace std;

//***********************************************************************
// MAIN
//***********************************************************************
int main(int argc, char* argv[]) {


  string true_prefix = argv[1];
  string pred_prefix = argv[2];

  //string true_prefix = "/Users/anefian/projects/stereopipeline/data/apollo/apollo-crop-small/results/apollo-crop-small";
  //string pred_prefix = "/Users/anefian/projects/stereopipeline/data/apollo/apollo-crop-small/results/apollo-crop-small";

  string true_disp_map_h_filename;
  string true_disp_map_v_filename;
  string pred_disp_map_h_filename;
  string pred_disp_map_v_filename;
  string good_pixels_map_filename;

  true_disp_map_h_filename = true_prefix+"-R-H.tif";
  true_disp_map_v_filename = true_prefix+"-R-V.tif";
  pred_disp_map_h_filename = pred_prefix+"-R-H.tif";
  pred_disp_map_v_filename = pred_prefix+"-R-V.tif";
  good_pixels_map_filename = pred_prefix+"-GoodPixelMap.tif";

  DiskImageView<PixelGray<float> > true_disp_map_h (true_disp_map_h_filename);
  DiskImageView<PixelGray<float> > true_disp_map_v (true_disp_map_v_filename);
  DiskImageView<PixelGray<float> > pred_disp_map_h (pred_disp_map_h_filename);
  DiskImageView<PixelGray<float> > pred_disp_map_v (pred_disp_map_v_filename);
  DiskImageView<PixelRGB<uint8> >  good_pixels_map (good_pixels_map_filename);

  //compute the average coverage measure
  int img_size = true_disp_map_h.rows() * true_disp_map_h.cols();
  int num_good_pixels = 0;
  int i, j;

  for (i = 0; i < good_pixels_map.rows(); i++){
    for (j = 0; j < good_pixels_map.cols(); j++){

      if (good_pixels_map(i,j).r()==200){
          num_good_pixels++;
      }
    }
  }

  float avg_cover = num_good_pixels/(float) img_size;
  printf("Average Coverage = %f\n", avg_cover);

  //compute the average error measure
  float avg_error_h = 0;

  for (i = 0; i < pred_disp_map_h.rows(); i++){
    for (j = 0; j < pred_disp_map_h.cols(); j++){
      float tmp = pred_disp_map_h(i,j).v()-true_disp_map_h(i,j).v();
       avg_error_h = avg_error_h + tmp*tmp;
    }
  }

  avg_error_h = avg_error_h/num_good_pixels;
  printf("avg_error_h = %f\n", avg_error_h);

  float avg_error_v = 0;

  for (i = 0; i < pred_disp_map_h.rows(); i++){
    for (j = 0; j < pred_disp_map_h.cols(); j++){
      float tmp = pred_disp_map_v(i,j).v()-true_disp_map_v(i,j).v();
       avg_error_v = avg_error_v + tmp*tmp;
    }
  }

  avg_error_v = avg_error_v/num_good_pixels;
  printf("avg_error_v = %f\n", avg_error_v);
  float avg_error = 0.5*(avg_error_v + avg_error_h);
  printf("avg_error = %f\n", avg_error);

  return(EXIT_SUCCESS);
}

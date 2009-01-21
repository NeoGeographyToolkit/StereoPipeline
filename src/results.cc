// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
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

#include "StereoSettings.h"
#include "stereo.h"
#include "StereoSession.h"

// Support for ISIS image files
#if defined(ASP_HAVE_PKG_ISIS) && ASP_HAVE_PKG_ISIS == 1 
#include "Isis/DiskImageResourceIsis.h"
#include "Isis/StereoSessionIsis.h"
#endif

#include "HRSC/StereoSessionHRSC.h"
#if defined(ASP_HAVE_PKG_SPICE) && ASP_HAVE_PKG_SPICE == 1
#include "MOC/StereoSessionMOC.h"
#include "MRO/StereoSessionCTX.h"
#endif
#include "RMAX/StereoSessionRmax.h"

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

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<PixelDisparity<float> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

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

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

/// \file TestNURBS.h
///

#include <cxxtest/TestSuite.h>
#include "stereo.h"
#include "SurfaceNURBS.h"
#include "vw/vw.h"
#include "vw/FileIO.h"
#include "vw/stereo/DisparityMap.h"

using namespace std;
using namespace vw;
using namespace vw::stereo;

class TestNURBS : public CxxTest::TestSuite {
 public:

  void testNURBS() {

    printf("\n\n");
    
    std::string out_prefix = "/irg/home/mbroxton/arabia/out";

    

    vcl_cout << "\nLoading image " << out_prefix + "-H-preNURBS.exr" << ":\n";
    vil_image_view<double> Limg = read_hdr_image(out_prefix + "-H-preNURBS.exr");
    vcl_cout << "\nLoading image " << out_prefix + "-V-preNURBS.exr" << ":\n";
    vil_image_view<double> Rimg = read_hdr_image(out_prefix + "-V-preNURBS.exr");

    write_image(Limg, out_prefix + "-H-before.pgm");
    write_image(Rimg, out_prefix + "-V-before.pgm");

    //    DisparityMap disparities(Limg, Rimg);

    vil_image_view<double> Lafter;
    //    vil_image_view<double> Rafter;

    FitSurfaceNURBS(Limg, Lafter, 10, vw_TRUE); 
    //    FitNURBSurface(Rimg, Rafter, 3, vw_TRUE); 
      
    write_image(Lafter, out_prefix + "-H-after.pgm");
    //    write_image(Rafter, out_prefix + "-V-after.pgm");


  }
  
};


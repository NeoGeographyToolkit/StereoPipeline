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

/// \file BundleAdjustUtils.cc
///

#include "BundleAdjustUtils.h"

#include <vw/Camera/BundleAdjust.h>
#include <vw/Camera.h>
#include <vw/Stereo.h>
#include <vw/Cartography.h>

using namespace vw;
using namespace vw::camera;
using namespace vw::stereo;
using namespace vw::ip;

void read_adjustments(std::string const& filename, Vector3& position_correction, Quaternion<double>& pose_correction) {
  std::ifstream istr(filename.c_str());
  istr >> position_correction[0] >> position_correction[1] >> position_correction[2];
  istr >> pose_correction.w() >> pose_correction.x() >> pose_correction.y() >> pose_correction.z();
}

void write_adjustments(std::string const& filename, Vector3 const& position_correction, Quaternion<double> const& pose_correction) {
  std::ofstream ostr(filename.c_str());
  ostr << position_correction[0] << " " << position_correction[1] << " " << position_correction[2] << "\n";
  ostr << pose_correction.w() << " " << pose_correction.x() << " " << pose_correction.y() << " " << pose_correction.z() << " " << "\n";
}

void compute_stereo_residuals(std::vector<boost::shared_ptr<CameraModel> > const& camera_models,
                              ControlNetwork const& cnet) {

  // Compute pre-adjustment residuals and convert to bundles
  int n = 0;
  double error_sum = 0; 
  double min_error = ScalarTypeLimits<double>::highest();
  double max_error = ScalarTypeLimits<double>::lowest();
  for (unsigned i = 0; i < cnet.size(); ++i) {
    for (unsigned j = 0; j+1 < cnet[i].size(); ++j) {
      ++n;
      int cam1 = cnet[i][j].image_id();
      int cam2 = cnet[i][j+1].image_id();
      Vector2 pix1 = cnet[i][j].position();
      Vector2 pix2 = cnet[i][j+1].position();

      StereoModel sm(*(camera_models[cam1]), *(camera_models[cam2]));
      double error;
      Vector3 pos = sm(pix1,pix2,error);
      error_sum += error;
      min_error = std::min(min_error, error);
      max_error = std::max(max_error, error);
    }
  }  
  std::cout << "\nStereo Intersection Residuals -- Min: " << min_error << "  Max: " << max_error << "  Average: " << (error_sum/n) << "\n";
}

void add_matched_points(ControlNetwork& cnet,
                        std::vector<InterestPoint> const& ip1,
                        std::vector<InterestPoint> const& ip2,
                        int camera_id1, int camera_id2,
                        std::vector<boost::shared_ptr<CameraModel> > const& camera_models) {

  double min_convergence_angle = 5.0 * M_PI/180.0; // 5 degrees

  for (unsigned i=0; i < ip1.size(); ++i) {
    ControlMeasure m1(ip1[i].x, ip1[i].y, ip1[i].scale, ip1[i].scale, camera_id1);
    ControlMeasure m2(ip2[i].x, ip2[i].y, ip2[i].scale, ip2[i].scale, camera_id2);
    
    unsigned pos1 = cnet.find_measure(m1);
    unsigned pos2 = cnet.find_measure(m2);
    
    if ( pos1 != cnet.size() && pos2 == cnet.size() ) {        // Contains m1 aready
      cnet[pos1].add_measure(m2);
    } else if ( pos1 == cnet.size() && pos2 != cnet.size() ) { // Contains m2 aready
      cnet[pos2].add_measure(m1);
    } else if ( pos1 == cnet.size() && pos2 == cnet.size() ) { // Contains neither
      // ... create a stereo model for this image pair...
      StereoModel sm(*(camera_models[camera_id1]), *(camera_models[camera_id2]));
      if ( sm.convergence_angle(m1.position(), m2.position()) > min_convergence_angle) {

        ControlPoint cpoint(ControlPoint::TiePoint);
        double error;
      
	cpoint.set_position(sm(m1.position(), m2.position(), error));

        // The stereo model returns a null point, in some cases where
        // the rays are very close to parallel, or if the point would
        // have appeared behind the cameras.  We skip null points.
        if (cpoint.position() != Vector3()) {
          cpoint.set_sigma(error,error,error);
          cpoint.add_measure(m1);
          cpoint.add_measure(m2);
          cnet.add_control_point(cpoint);
        }
      }
    } else if (pos1 != pos2) {                                 // Contains both, but in seperate control points
      ControlPoint& p1 = cnet[pos1];
      ControlPoint& p2 = cnet[pos2];
      
      // Merge the twe control points into one.
      for (unsigned m=0; m < p2.size(); ++m) 
        p1.add_measure(p2[m]);
      p1.set_position((p1.position() + p2.position())/2);
      p1.set_sigma((p1.sigma() + p2.sigma())/2);
      cnet.delete_control_point(pos2);
    }
  }
}


int add_ground_control_points(vw::camera::ControlNetwork& cnet,
                              std::string filename, int camera_id) {

  std::ifstream istr(filename.c_str());
  int count = 0;
  while (!istr.eof()) {
    Vector2 pix;
    Vector3 loc;
    Vector3 sigma;

    istr >> pix[0] >> pix[1] >> loc[0] >> loc[1] >> loc[2] >> sigma[0] >> sigma[1] >> sigma[2];
    if (loc[0] !=0) {  // ignore blank lines
      Vector3 xyz = cartography::lon_lat_radius_to_xyz(loc);
      std::cout << "GCP: " << xyz << "\n";
      
      ControlMeasure m(pix[0], pix[1], 1.0, 1.0, camera_id);
      ControlPoint cpoint(ControlPoint::GroundControlPoint);
      cpoint.set_position(xyz[0],xyz[1],xyz[2]);
      cpoint.set_sigma(sigma[0],sigma[1],sigma[2]);
      cpoint.add_measure(m);
      cnet.add_control_point(cpoint);
      ++count;
    }
  }
  istr.close();
  return count;
}

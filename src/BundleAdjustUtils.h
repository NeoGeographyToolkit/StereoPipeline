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

/// \file BundleAdjustUtils.h
///

#ifndef __BUNDLE_ADJUST_UTILS_H__
#define __BUNDLE_ADJUST_UTILS_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/ControlNetwork.h>
#include <vw/Math.h>
#include <vw/InterestPoint.h>

void read_adjustments(std::string const& filename, vw::Vector3& position_correction, vw::math::Quaternion<double>& pose_correction);
void write_adjustments(std::string const& filename, vw::Vector3 const& position_correction, vw::math::Quaternion<double> const& pose_correction);

void compute_stereo_residuals(std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
                              vw::camera::ControlNetwork const& cnet);

void add_matched_points(vw::camera::ControlNetwork& cnet,
                        std::vector<vw::ip::InterestPoint> const& ip1,
                        std::vector<vw::ip::InterestPoint> const& ip2,
                        int camera_id1, int camera_id2,
                        std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models);

int add_ground_control_points(vw::camera::ControlNetwork& cnet, std::string filename, int camera_id);

#endif // __BUNDLE_ADJUST_UTILS_H__

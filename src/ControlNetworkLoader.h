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

/// \file ControlNetworkLoader.h
///

#ifndef __CONTROL_NETWORK_LOADER_H__
#define __CONTROL_NETWORK_LOADER_H__

#include <vw/Camera/CameraModel.h>
#include <vw/Camera/ControlNetwork.h>

void build_control_network( boost::shared_ptr<vw::camera::ControlNetwork> cnet,
			    std::vector<boost::shared_ptr<vw::camera::CameraModel> > const& camera_models,
			    std::vector<std::string> image_files,
			    int min_matches = 30);

// Legacy
void add_ground_control_points_past( boost::shared_ptr<vw::camera::ControlNetwork> cnet,
				     std::vector<std::string> image_files );

std::vector<std::string> sort_out_gcps( std::vector<std::string>& image_files );

void add_ground_control_points( boost::shared_ptr<vw::camera::ControlNetwork> cnet,
				std::vector<std::string> const& image_files,
				std::vector<std::string> const& gcp_files );

#endif // __CONTROL_NETWORK_LOADER_H__

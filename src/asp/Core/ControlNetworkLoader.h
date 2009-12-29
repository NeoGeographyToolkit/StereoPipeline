// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file ControlNetworkLoader.h
///
/// An optimized loader that builds an intermediate data product
/// before converting to a control network. Contains additional error
/// checking looking for 'spiral errors', or control points that develop
/// measures to the same image in multiple places due to poor interest
/// point performance.

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

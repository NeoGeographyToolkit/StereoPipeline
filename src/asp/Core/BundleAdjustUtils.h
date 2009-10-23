// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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

#endif // __BUNDLE_ADJUST_UTILS_H__

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


/// \file StereoSessionSpot.h
///
/// This a session to support SPOT5 satellite images.

#ifndef __STEREO_SESSION_SPOT_H__
#define __STEREO_SESSION_SPOT_H__

#include <asp/Sessions/StereoSessionConcrete.h>
#include <vw/Stereo/StereoModel.h>

#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/AffineEpipolar.h>

namespace asp {

// TODO: Refactor the session classes again?

//TODO: Add STEREOSESSION_DISKTRANSFORM_TYPE template so we can support map projected SPOT5 images. 

  /// Generic stereoSession implementation for images which we can read/write with GDAL.
  /// - This class adds a "preprocessing hook" which aligns and normalizes the images using the specified methods.
  class StereoSessionSpot : public StereoSessionConcrete<DISKTRANSFORM_TYPE_MATRIX,   
                                                         STEREOMODEL_TYPE_SPOT5> {

  public:
    StereoSessionSpot(){}
    virtual ~StereoSessionSpot(){}

    virtual std::string name() const { return "spot5"; }

    /// Stage 1: Preprocessing
    ///
    /// Pre file is a pair of images.            ( ImageView<PixelT> )
    /// Post file is a pair of grayscale images. ( ImageView<PixelGray<float> > )
    virtual void pre_preprocessing_hook(bool adjust_left_image_size,
					std::string const& left_input_file,
					std::string const& right_input_file,
					std::string      & left_output_file,
					std::string      & right_output_file);

    /// Override the base class implementation, SPOT5 images never have georef.
    virtual vw::cartography::GeoReference get_georef();

    /// Specialization of shared_preprocessing_hook currently required for this class.
    /// - TODO: Refactor things so we don't need all this duplicate code!!!
    bool unshared_preprocessing_hook(asp::BaseOptions              & options,
			                               std::string const             & left_input_file,
			                               std::string const             & right_input_file,
			                               std::string                   & left_output_file,
			                               std::string                   & right_output_file,
			                               std::string                   & left_cropped_file,
			                               std::string                   & right_cropped_file,
			                               float                         & left_nodata_value,
			                               float                         & right_nodata_value,
			                               bool                          & has_left_georef,
			                               bool                          & has_right_georef,
			                               vw::cartography::GeoReference & left_georef,
			                               vw::cartography::GeoReference & right_georef);

    /// Simple factory function
    static StereoSession* construct() { return new StereoSessionSpot; }
   
   private:
    // Cache these so we don't have to keep reloading them.
    vw::ImageFormat m_left_input_format,   m_right_input_format;
  };


} // End namespace asp

#endif//__STEREO_SESSION_SPOT_H__

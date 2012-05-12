// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


/// \file StereoSession.cc
///

#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>
#include <asp/Sessions/DG/StereoSessionDG.h>
#include <asp/Sessions/RPC/StereoSessionRPC.h>

#include <vw/Core/Exception.h>

#include <map>

using namespace vw;

// This creates an anonymous namespace where the lookup table for
// stereo sessions lives.
namespace {
  typedef std::map<std::string,asp::StereoSession::construct_func> ConstructMapType;
  ConstructMapType *stereo_session_construct_map = 0;
}

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector2f> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

namespace asp {

  // Pass over all the string variables we use
  void StereoSession::initialize( BaseOptions const& options,
                                  std::string const& left_image_file,
                                  std::string const& right_image_file,
                                  std::string const& left_camera_file,
                                  std::string const& right_camera_file,
                                  std::string const& out_prefix,
                                  std::string const& extra_argument1,
                                  std::string const& extra_argument2,
                                  std::string const& extra_argument3,
                                  std::string const& extra_argument4) {
    m_options = options;
    m_left_image_file = left_image_file;
    m_right_image_file = right_image_file;
    m_left_camera_file = left_camera_file;
    m_right_camera_file = right_camera_file;
    m_out_prefix = out_prefix;
    m_extra_argument1 = extra_argument1;
    m_extra_argument2 = extra_argument2;
    m_extra_argument3 = extra_argument3;
    m_extra_argument4 = extra_argument4;
  }

  static void register_default_session_types() {
    static bool already = false;
    if ( already ) return;
    already = true;
    StereoSession::register_session_type( "pinhole",
                                          &StereoSessionPinhole::construct );
    StereoSession::register_session_type( "dg",
                                          &StereoSessionDG::construct );
    StereoSession::register_session_type( "rpc",
                                          &StereoSessionRPC::construct );
  }

  StereoSession* StereoSession::create( std::string const& session_type ) {
    register_default_session_types();
    if( stereo_session_construct_map ) {
      ConstructMapType::const_iterator i =
        stereo_session_construct_map->find( session_type );
      if( i != stereo_session_construct_map->end() )
        return i->second();
    }
    vw_throw( vw::NoImplErr() << "Unsuppported stereo session type: "
              << session_type );
    return 0; // never reached
  }

  void StereoSession::register_session_type( std::string const& id,
                                             StereoSession::construct_func func) {
    if( ! stereo_session_construct_map )
      stereo_session_construct_map = new ConstructMapType();
    stereo_session_construct_map->insert( std::make_pair( id, func ) );
  }

  // Base class implementation of processing steps. All of these don't
  // perform any thing, they're just place holders.
  void StereoSession::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                    boost::shared_ptr<vw::camera::CameraModel> &cam2) {
    cam1 = camera_model(m_left_image_file, m_left_camera_file);
    cam2 = camera_model(m_right_image_file, m_right_camera_file);
  }


  // LUT image access methods
  bool StereoSession::has_lut_images() const { return false; }

  ImageViewRef<Vector2f> StereoSession::lut_image_left() const {
    vw_throw(NoImplErr() << "This stereo session doesn't utilize LUT images.");
    return DiskImageView<Vector2f>("");
  }

  ImageViewRef<Vector2f> StereoSession::lut_image_right() const {
    vw_throw(NoImplErr() << "This stereo session doesn't utilize LUT images.");
    return DiskImageView<Vector2f>("");
  }

  // Processing Hooks. The default is to do nothing.
  void StereoSession::pre_preprocessing_hook(std::string const& input_file1,
                                             std::string const& input_file2,
                                             std::string &output_file1,
                                             std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::post_preprocessing_hook(std::string const& input_file1,
                                              std::string const& input_file2,
                                              std::string &output_file1,
                                              std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::pre_correlation_hook(std::string const& input_file1,
                                           std::string const& input_file2,
                                           std::string &output_file1,
                                           std::string &output_file2) {
    output_file1 = input_file1;
    output_file2 = input_file2;
  }

  void StereoSession::post_correlation_hook(std::string const& input_file,
                                            std::string & output_file) {
    output_file = input_file;
  }

  void StereoSession::pre_filtering_hook(std::string const& input_file,
                                         std::string & output_file) {
    output_file = input_file;
  }

  void StereoSession::post_filtering_hook(std::string const& input_file,
                                          std::string & output_file) {
    output_file = input_file;
  }

  ImageViewRef<PixelMask<Vector2f> >
  StereoSession::pre_pointcloud_hook(std::string const& input_file) {
    return DiskImageView<PixelMask<Vector2f> >( input_file );
  }

  void StereoSession::post_pointcloud_hook(std::string const& input_file,
                                           std::string & output_file) {
    output_file = input_file;
  }

}

// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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

// Pass over all the string variables we use
void asp::StereoSession::initialize( asp::BaseOptions const& options,
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
  asp::StereoSession::register_session_type( "pinhole",
                                             &asp::StereoSessionPinhole::construct );
  asp::StereoSession::register_session_type( "dg",
                                             &asp::StereoSessionDG::construct );
  asp::StereoSession::register_session_type( "rpc",
                                             &asp::StereoSessionRPC::construct );
}

asp::StereoSession* asp::StereoSession::create( std::string const& session_type ) {
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

void asp::StereoSession::register_session_type( std::string const& id,
                                                asp::StereoSession::construct_func func) {
  if( ! stereo_session_construct_map )
    stereo_session_construct_map = new ConstructMapType();
  stereo_session_construct_map->insert( std::make_pair( id, func ) );
}

// Base class implementation of processing steps. All of these don't
// perform any thing, they're just place holders.
void asp::StereoSession::camera_models(boost::shared_ptr<vw::camera::CameraModel> &cam1,
                                       boost::shared_ptr<vw::camera::CameraModel> &cam2) {
  cam1 = camera_model(m_left_image_file, m_left_camera_file);
  cam2 = camera_model(m_right_image_file, m_right_camera_file);
}

void asp::StereoSession::pre_preprocessing_hook(std::string const& input_file1,
                                                std::string const& input_file2,
                                                std::string &output_file1,
                                                std::string &output_file2) {
  output_file1 = input_file1;
  output_file2 = input_file2;
}

void asp::StereoSession::post_preprocessing_hook(std::string const& input_file1,
                                                 std::string const& input_file2,
                                                 std::string &output_file1,
                                                 std::string &output_file2) {
  output_file1 = input_file1;
  output_file2 = input_file2;
}

void asp::StereoSession::pre_correlation_hook(std::string const& input_file1,
                                              std::string const& input_file2,
                                              std::string &output_file1,
                                              std::string &output_file2) {
  output_file1 = input_file1;
  output_file2 = input_file2;
}

void asp::StereoSession::post_correlation_hook(std::string const& input_file,
                                               std::string & output_file) {
  output_file = input_file;
}

void asp::StereoSession::pre_filtering_hook(std::string const& input_file,
                                            std::string & output_file) {
  output_file = input_file;
}

void asp::StereoSession::post_filtering_hook(std::string const& input_file,
                                             std::string & output_file) {
  output_file = input_file;
}

ImageViewRef<PixelMask<Vector2f> >
asp::StereoSession::pre_pointcloud_hook(std::string const& input_file) {
  return DiskImageView<PixelMask<Vector2f> >( input_file );
}

void asp::StereoSession::post_pointcloud_hook(std::string const& input_file,
                                              std::string & output_file) {
  output_file = input_file;
}

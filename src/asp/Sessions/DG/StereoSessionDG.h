// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file StereoSessionDG.h
///
/// This a session to hopefully support Digital Globe images from
/// Quickbird and World View.

#ifndef __STEREO_SESSION_DG_H__
#define __STEREO_SESSION_DG_H__

#include <asp/Sessions/Pinhole/StereoSessionPinhole.h>

namespace asp {

  class StereoSessionDG : public StereoSessionPinhole {
    bool m_rpc_map_projected;
    std::string m_lut_image_left;
    std::string m_lut_image_right;

  public:
    StereoSessionDG();
    virtual ~StereoSessionDG();

    // Initializer that determines if our input images are map
    // projected or if this is just a straight Digitial Globe session.
    void initialize(BaseOptions const& options,
                    std::string const& left_image_file,
                    std::string const& right_image_file,
                    std::string const& left_camera_file,
                    std::string const& right_camera_file,
                    std::string const& out_prefix,
                    std::string const& extra_argument1,
                    std::string const& extra_argument2,
                    std::string const& extra_argument3,
                    std::string const& extra_argument4);

    // Produces a camera model from the images
    virtual boost::shared_ptr<vw::camera::CameraModel>
    camera_model( std::string const& image_file,
                  std::string const& camera_file = "" );

    // LUT access (enabled only when working with images map projected by RPC)
    virtual bool has_lut_images() const;
    virtual vw::DiskImageView<vw::Vector2f> lut_image_left() const;
    virtual vw::DiskImageView<vw::Vector2f> lut_image_right() const;

    // Stage 1: Preprocessing
    //
    // Pre file is a pair of images.            ( ImageView<PixelT> )
    // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
    virtual void pre_preprocessing_hook(std::string const& input_file1,
                                        std::string const& input_file2,
                                        std::string &output_file1,
                                        std::string &output_file2);

    static StereoSession* construct() { return new StereoSessionDG; }
  };

}

#endif//__STEREO_SESSION_DG_H__

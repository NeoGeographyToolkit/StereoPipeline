// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <asp/Tools/stereo_refinement.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // user safety check
    //---------------------------------------------------------
    {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1,camera_model2);

      // Do the camera's appear to be in the same location?
      if ( norm_2(camera_model1->camera_center(Vector2()) -
                  camera_model2->camera_center(Vector2())) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should be double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";
    }

    // Internal Processes
    //---------------------------------------------------------
    stereo_refinement( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : REFINEMENT FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

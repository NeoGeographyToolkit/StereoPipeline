// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// \file stereo_triangulation

#ifndef __ASP_STEREO_TRIANGULATION_H__
#define __ASP_STEREO_TRIANGULATION_H__

#include <asp/Tools/stereo.h>
#include <vw/Cartography.h>
#include <vw/Camera/CameraModel.h>

namespace vw {

  void stereo_triangulation( Options const& opt ) {
    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 4 --> TRIANGULATION \n";

    try {
      std::string prehook_filename;
      opt.session->pre_pointcloud_hook(opt.out_prefix+"-F.tif",
                                       prehook_filename);
      DiskImageView<PixelMask<Vector2f> > disparity_map(prehook_filename);

      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1, camera_model2);

#if HAVE_PKG_VW_BUNDLEADJUSTMENT
      // If the user has generated a set of position and pose
      // corrections using the bundle_adjust program, we read them in
      // here and incorporate them into our camera model.
      Vector3 position_correction;
      Quaternion<double> pose_correction;
      if (fs::exists(fs::path(in_file1).replace_extension("adjust"))) {
        read_adjustments(fs::path(in_file1).replace_extension("adjust").string(),
                         position_correction, pose_correction);
        camera_model1 =
          boost::shared_ptr<CameraModel>(new AdjustedCameraModel(camera_model1,
                                                                 position_correction,
                                                                 pose_correction));
      }
      if (fs::exists(fs::path(in_file2).replace_extension("adjust"))) {
        read_adjustments(fs::path(in_file2).replace_extension("adjust").string(),
                         position_correction, pose_correction);
        camera_model2 =
          boost::shared_ptr<CameraModel>(new AdjustedCameraModel(camera_model2,
                                                                 position_correction,
                                                                 pose_correction));
      }
#endif

      // If the distance from the left camera center to a point is
      // greater than the universe radius, we remove that pixel and
      // replace it with a zero vector, which is the missing pixel value
      // in the point_image.
      //
      // We apply the universe radius here and then write the result
      // directly to a file on disk.
      stereo::UniverseRadiusFunc universe_radius_func(Vector3(),0,0);
      if ( stereo_settings().universe_center == "CAMERA" ) {
        universe_radius_func =
          stereo::UniverseRadiusFunc(camera_model1->camera_center(Vector2()),
                                     stereo_settings().near_universe_radius,
                                     stereo_settings().far_universe_radius);
      } else if ( stereo_settings().universe_center == "ZERO" ) {
        universe_radius_func =
          stereo::UniverseRadiusFunc(Vector3(),
                                     stereo_settings().near_universe_radius,
                                     stereo_settings().far_universe_radius);
      }

      // Apply radius function and stereo model in one go
      vw_out() << "\t--> Generating a 3D point cloud.   " << std::endl;
      ImageViewRef<Vector3> point_cloud =
        per_pixel_filter(stereo::stereo_triangulate( disparity_map,
                                                     camera_model1.get(),
                                                     camera_model2.get() ),
                         universe_radius_func);

      DiskImageResourceGDAL point_cloud_rsrc(opt.out_prefix + "-PC.tif",
                                             point_cloud.format(),
                                             opt.raster_tile_size,
                                             opt.gdal_options );

      if ( opt.stereo_session_string == "isis" ) {
        write_image(point_cloud_rsrc, point_cloud,
                    TerminalProgressCallback("asp", "\t--> Triangulating: "));
      } else
        block_write_image(point_cloud_rsrc, point_cloud,
                          TerminalProgressCallback("asp", "\t--> Triangulating: "));
      vw_out() << "\t--> " << universe_radius_func;

    } catch (IOErr &e) {
      vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage -- could not read input files.\n"
                << e.what() << "\nExiting.\n\n" );
    }
  }

} // end namespace vw

#endif//__ASP_STEREO_TRIANGULATION_H__

// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file stereo.cc
///
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/Cartography.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>

using namespace vw;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<float, 2> > { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}

// Class definition
template <class DisparityImageT>
class StereoAndErrorView : public ImageViewBase<StereoAndErrorView<DisparityImageT> >
{
  DisparityImageT m_disparity_map;
  stereo::StereoModel m_stereo_model;
  typedef typename DisparityImageT::pixel_type dpixel_type;

  template <class PixelT>
  struct NotSingleChannel {
    static const bool value = (1 != CompoundNumChannels<typename UnmaskedPixelType<PixelT>::type>::value);
  };

  template <class T>
  inline typename boost::enable_if<IsScalar<T>,Vector3>::type
  StereoModelHelper( stereo::StereoModel const& model, Vector2 const& index,
                     T const& disparity, double& error ) const {
    return model( index, Vector2( index[0] + disparity, index[1] ), error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value == 1),Vector3>::type
  StereoModelHelper( stereo::StereoModel const& model, Vector2 const& index,
                     T const& disparity, double& error ) const {
    return model( index, Vector2( index[0] + disparity, index[1] ), error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value != 1),Vector3>::type
  StereoModelHelper( stereo::StereoModel const& model, Vector2 const& index,
                     T const& disparity, double& error ) const {
    return model( index, Vector2( index[0] + disparity[0],
                                  index[1] + disparity[1] ), error );
  }

public:

  typedef Vector4 pixel_type;
  typedef const Vector4 result_type;
  typedef ProceduralPixelAccessor<StereoAndErrorView> pixel_accessor;

  StereoAndErrorView( DisparityImageT const& disparity_map,
                      vw::camera::CameraModel const* camera_model1,
                      vw::camera::CameraModel const* camera_model2,
                      bool least_squares_refine = false) :
    m_disparity_map(disparity_map),
    m_stereo_model(camera_model1, camera_model2, least_squares_refine) {}

  StereoAndErrorView( DisparityImageT const& disparity_map,
                      stereo::StereoModel const& stereo_model) :
    m_disparity_map(disparity_map),
    m_stereo_model(stereo_model) {}

  inline int32 cols() const { return m_disparity_map.cols(); }
  inline int32 rows() const { return m_disparity_map.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
    if ( is_valid(m_disparity_map(i,j,p)) ) {
      pixel_type result;
      subvector(result,0,3) = StereoModelHelper( m_stereo_model, Vector2(i,j),
                                                 m_disparity_map(i,j,p), result[3] );
      return result;
    }
    // For missing pixels in the disparity map, we return a null 3D position.
    return pixel_type();
  }

  /// \cond INTERNAL
  typedef StereoAndErrorView<typename DisparityImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const { return prerasterize_type( m_disparity_map.prerasterize(bbox), m_stereo_model ); }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
  /// \endcond
};

// Variant that uses LUT tables
template <class DisparityImageT, class LUTImage1T, class LUTImage2T>
class StereoLUTAndErrorView : public ImageViewBase<StereoLUTAndErrorView<DisparityImageT, LUTImage1T, LUTImage2T> >
{
  DisparityImageT m_disparity_map;
  LUTImage1T m_lut_image1;
  InterpolationView< EdgeExtensionView<LUTImage2T, ConstantEdgeExtension>, BilinearInterpolation>  m_lut_image2;
  LUTImage2T m_lut_image2_org;
  stereo::StereoModel m_stereo_model;
  typedef typename DisparityImageT::pixel_type dpixel_type;

  template <class PixelT>
  struct NotSingleChannel {
    static const bool value = (1 != CompoundNumChannels<typename UnmaskedPixelType<PixelT>::type>::value);
  };

  template <class T>
  inline typename boost::enable_if<IsScalar<T>,Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, double& error ) const {
    return m_stereo_model( m_lut_image1(i,j),
                           m_lut_image2( T(i) + disparity, j ), error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value == 1),Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, double& error ) const {
    return m_stereo_model( m_lut_image1(i,j),
                           m_lut_image2( float(i) + disparity, j ),  error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value != 1),Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, double& error ) const {
    return m_stereo_model( m_lut_image1(i,j),
                           m_lut_image2( float(i) + disparity[0],
                                         float(j) + disparity[1] ), error );
  }

public:

  typedef Vector4 pixel_type;
  typedef const Vector4 result_type;
  typedef ProceduralPixelAccessor<StereoLUTAndErrorView> pixel_accessor;

  StereoLUTAndErrorView( ImageViewBase<DisparityImageT> const& disparity_map,
                         ImageViewBase<LUTImage1T> const& lut_image1,
                         ImageViewBase<LUTImage2T> const& lut_image2,
                         vw::camera::CameraModel const* camera_model1,
                         vw::camera::CameraModel const* camera_model2,
                         bool least_squares_refine = false) :
    m_disparity_map(disparity_map.impl()), m_lut_image1( lut_image1.impl() ),
    m_lut_image2(interpolate(lut_image2.impl())),
    m_lut_image2_org( lut_image2.impl() ),
    m_stereo_model(camera_model1, camera_model2, least_squares_refine) {}

  StereoLUTAndErrorView( ImageViewBase<DisparityImageT> const& disparity_map,
                         ImageViewBase<LUTImage1T> const& lut_image1,
                         ImageViewBase<LUTImage2T> const& lut_image2,
                         stereo::StereoModel const& stereo_model) :
    m_disparity_map(disparity_map.impl()), m_lut_image1(lut_image1.impl()),
    m_lut_image2(interpolate(lut_image2.impl())),
    m_lut_image2_org( lut_image2.impl() ),
    m_stereo_model(stereo_model) {}

  inline int32 cols() const { return m_disparity_map.cols(); }
  inline int32 rows() const { return m_disparity_map.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
    if ( is_valid(m_disparity_map(i,j,p)) ) {
      pixel_type result;
      subvector(result,0,3) = StereoModelHelper( i, j, m_disparity_map(i,j,p), result[3] );
      return result;
    }
    // For missing pixels in the disparity map, we return a null 3D position.
    return pixel_type();
  }

  /// \cond INTERNAL
  typedef StereoLUTAndErrorView<typename DisparityImageT::prerasterize_type,
                                typename LUTImage1T::prerasterize_type,
                                LUTImage2T > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const { return prerasterize_type( m_disparity_map.prerasterize(bbox), m_lut_image1.prerasterize(bbox), m_lut_image2_org, m_stereo_model ); }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
  /// \endcond
};

template <class ImageT>
StereoAndErrorView<ImageT>
stereo_error_triangulate( ImageViewBase<ImageT> const& v,
                          vw::camera::CameraModel const* camera1,
                          vw::camera::CameraModel const* camera2 ) {
  return StereoAndErrorView<ImageT>( v.impl(), camera1, camera2 );
}

template <class ImageT>
StereoAndErrorView<ImageT>
lsq_stereo_error_triangulate( ImageViewBase<ImageT> const& v,
                              vw::camera::CameraModel const* camera1,
                              vw::camera::CameraModel const* camera2 ) {
  return StereoAndErrorView<ImageT>( v.impl(), camera1, camera2, true );
}

template <class DisparityT, class LUT1T, class LUT2T>
StereoLUTAndErrorView<DisparityT, LUT1T, LUT2T>
stereo_error_triangulate( ImageViewBase<DisparityT> const& disparity,
                          ImageViewBase<LUT1T> const& lut1,
                          ImageViewBase<LUT2T> const& lut2,
                          vw::camera::CameraModel const* camera1,
                          vw::camera::CameraModel const* camera2 ) {
  typedef StereoLUTAndErrorView<DisparityT,LUT1T,LUT2T> result_type;
  return result_type( disparity.impl(), lut1.impl(), lut2.impl(),
                      camera1, camera2 );
}

template <class DisparityT, class LUT1T, class LUT2T>
StereoLUTAndErrorView<DisparityT, LUT1T, LUT2T>
lsq_stereo_error_triangulate( ImageViewBase<DisparityT> const& disparity,
                              ImageViewBase<LUT1T> const& lut1,
                              ImageViewBase<LUT2T> const& lut2,
                              vw::camera::CameraModel const* camera1,
                              vw::camera::CameraModel const* camera2 ) {
  typedef StereoLUTAndErrorView<DisparityT,LUT1T,LUT2T> result_type;
  return result_type( disparity.impl(), lut1.impl(), lut2.impl(),
                      camera1, camera2, true );
}


void stereo_triangulation( Options const& opt ) {
  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 4 --> TRIANGULATION \n";

  try {
    ImageViewRef<PixelMask<Vector2f> > disparity_map =
      opt.session->pre_pointcloud_hook(opt.out_prefix+"-F.tif");

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
    if ( stereo_settings().universe_center == "camera" ) {
      universe_radius_func =
        stereo::UniverseRadiusFunc(camera_model1->camera_center(Vector2()),
                                   stereo_settings().near_universe_radius,
                                   stereo_settings().far_universe_radius);
    } else if ( stereo_settings().universe_center == "zero" ) {
      universe_radius_func =
        stereo::UniverseRadiusFunc(Vector3(),
                                   stereo_settings().near_universe_radius,
                                   stereo_settings().far_universe_radius);
    }

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud.   " << std::endl;
    ImageViewRef<Vector4> point_cloud;
    if ( opt.session->has_lut_images() ) {
      if ( stereo_settings().use_least_squares )
        point_cloud =
          per_pixel_filter(lsq_stereo_error_triangulate( disparity_map,
                                                         opt.session->lut_image_left(),
                                                         opt.session->lut_image_right(),
                                                         camera_model1.get(),
                                                         camera_model2.get() ),
                           universe_radius_func);
      else
        point_cloud =
          per_pixel_filter(stereo_error_triangulate( disparity_map,
                                                     opt.session->lut_image_left(),
                                                     opt.session->lut_image_right(),
                                                     camera_model1.get(),
                                                     camera_model2.get() ),
                           universe_radius_func);
    } else {
      if ( stereo_settings().use_least_squares )
        point_cloud =
          per_pixel_filter(lsq_stereo_error_triangulate( disparity_map,
                                                         camera_model1.get(),
                                                         camera_model2.get() ),
                           universe_radius_func);
      else
        point_cloud =
          per_pixel_filter(stereo_error_triangulate( disparity_map,
                                                     camera_model1.get(),
                                                     camera_model2.get() ),
                           universe_radius_func);
    }

    vw_out(VerboseDebugMessage,"asp") << "Writing Point Cloud: "
                                      << opt.out_prefix + "-PC.tif\n";

    DiskImageResource* rsrc =
      asp::build_gdal_rsrc( opt.out_prefix + "-PC.tif",
                            point_cloud, opt );
    if ( opt.stereo_session_string == "isis" )
      write_image(*rsrc, point_cloud,
                  TerminalProgressCallback("asp", "\t--> Triangulating: "));
    else
      block_write_image(*rsrc, point_cloud,
                        TerminalProgressCallback("asp", "\t--> Triangulating: "));
    delete rsrc;
    vw_out() << "\t--> " << universe_radius_func;

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main( int argc, char* argv[] ) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // user safety check
    //---------------------------------------------------------
    try {
      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt.session->camera_models(camera_model1,camera_model2);

      // Do the cameras appear to be in the same location?
      if ( norm_2(camera_model1->camera_center(Vector2()) -
                  camera_model2->camera_center(Vector2())) < 1e-3 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear to be in the same location!\n"
          << "\tYou should double check your given camera\n"
          << "\tmodels as most likely stereo won't be able\n"
          << "\tto triangulate or perform epipolar rectification.\n";

      // Developer friendly help
      VW_OUT(DebugMessage,"asp") << "Camera 1 location: " << camera_model1->camera_center(Vector2()) << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(camera_model1->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 location: " << camera_model2->camera_center(Vector2()) << "\n"
                                 << "   in Lon Lat Rad: " << cartography::xyz_to_lon_lat_radius(camera_model2->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 1 Pointing Dir: " << camera_model1->pixel_to_vector(Vector2()) << "\n"
                                 << "      dot against pos: " << dot_prod(camera_model1->pixel_to_vector(Vector2()),
                                                                          camera_model1->camera_center(Vector2())) << "\n";
      VW_OUT(DebugMessage,"asp") << "Camera 2 Pointing Dir: " << camera_model2->pixel_to_vector(Vector2()) << "\n"
                                 << "      dot against pos: " << dot_prod(camera_model2->pixel_to_vector(Vector2()),
                                                                          camera_model2->camera_center(Vector2())) << "\n";

      // Can cameras triangulate to point at something in front of them?
      stereo::StereoModel model( camera_model1.get(), camera_model2.get() );
      double error;
      Vector3 point = model( Vector2(), Vector2(), error );
      if ( dot_prod( camera_model1->pixel_to_vector(Vector2()),
                     point - camera_model1->camera_center(Vector2()) ) < 0 )
        vw_out(WarningMessage,"console")
          << "Your cameras appear not to be pointing at the same location!\n"
          << "\tA test vector triangulated backwards through\n"
          << "\tthe camera models. You should double check\n"
          << "\tyour input models as most likely stereo won't\n"
          << "\tbe able to triangulate.\n";
    } catch ( camera::PixelToRayErr const& e ) {
    } catch ( camera::PointToPixelErr const& e ) {
      // Silent. Top Left pixel might not be valid on a map
      // projected image.
    }

    // Internal Processes
    //---------------------------------------------------------
    stereo_triangulation( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : TRIANGULATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

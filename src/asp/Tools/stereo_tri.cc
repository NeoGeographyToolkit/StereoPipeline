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


/// \file stereo_tri.cc
///

#include <asp/Tools/stereo.h>
#include <asp/Sessions/RPC/RPCModel.h>
#include <asp/Sessions/RPC/RPCStereoModel.h>
#include <vw/Cartography.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>
#include <ctime>

using namespace vw;
using namespace asp;

namespace vw {
  typedef Vector<double, 6> Vector6;
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 6> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<float, 2> > { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}

namespace asp{

  // ImageView operator that takes the last three elements of a vector
  // (the error part) and replaces them with the norm of that 3-vector.
  struct PointAndErrorNorm : public ReturnFixedType<Vector4> {
    Vector4 operator() (Vector6 const& pt) const {
      Vector4 result;
      subvector(result,0,3) = subvector(pt,0,3);
      result[3] = norm_2(subvector(pt,3,3));
      return result;
    }
  };
  template <class ImageT>
  UnaryPerPixelView<ImageT, PointAndErrorNorm>
  inline point_and_error_norm( ImageViewBase<ImageT> const& image ) {
    return UnaryPerPixelView<ImageT, PointAndErrorNorm>( image.impl(),
                                                         PointAndErrorNorm() );
  }

  template <class ImageT>
  void save_point_cloud(ImageT const& point_cloud, Options const& opt){

    std::string point_cloud_file = opt.out_prefix + "-PC.tif";
    vw_out() << "Writing Point Cloud: " << point_cloud_file << "\n";

    if ( opt.stereo_session_string == "isis" ){
      // ISIS does not support multi-threading
      asp::write_gdal_image( point_cloud_file, point_cloud, opt,
                             TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }else{
      asp::block_write_gdal_image( point_cloud_file, point_cloud, opt,
                                   TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }

  }

}

template <class DisparityImageT, class TX1T, class TX2T, class StereoModelT>
class StereoTXAndErrorView : public ImageViewBase<StereoTXAndErrorView<DisparityImageT, TX1T, TX2T, StereoModelT> >
{
  DisparityImageT m_disparity_map;
  TX1T m_tx1;
  TX2T m_tx2;
  StereoModelT m_stereo_model;
  typedef typename DisparityImageT::pixel_type dpixel_type;

  template <class PixelT>
  struct NotSingleChannel {
    static const bool value = (1 != CompoundNumChannels<typename UnmaskedPixelType<PixelT>::type>::value);
  };

  template <class T>
  inline typename boost::enable_if<IsScalar<T>,Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, Vector3& error ) const {
    return m_stereo_model( m_tx1.reverse( Vector2(i,j) ),
                           m_tx2.reverse( Vector2(T(i) + disparity, j) ), error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value == 1),Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, Vector3& error ) const {
    return m_stereo_model( m_tx1.reverse( Vector2(i,j) ),
                           m_tx2.reverse( Vector2(float(i)+disparity, j) ), error );
  }

  template <class T>
  inline typename boost::enable_if_c<IsCompound<T>::value && (CompoundNumChannels<typename UnmaskedPixelType<T>::type>::value != 1),Vector3>::type
  StereoModelHelper( size_t i, size_t j, T const& disparity, Vector3& error ) const {
    if ( !is_valid( disparity ) ){
      return Vector3(); // out of bounds
    }

    return m_stereo_model( m_tx1.reverse( Vector2(i,j) ),
                           m_tx2.reverse( Vector2( double(i) + disparity[0],
                                                   double(j) + disparity[1] ) ),
                           error );
  }

public:

  typedef Vector6 pixel_type;
  typedef Vector6 result_type;
  typedef ProceduralPixelAccessor<StereoTXAndErrorView> pixel_accessor;

  StereoTXAndErrorView( ImageViewBase<DisparityImageT> const& disparity_map,
                        TX1T const& tx1, TX2T const& tx2,
                        StereoModelT const& stereo_model) :
    m_disparity_map(disparity_map.impl()), m_tx1(tx1), m_tx2(tx2),
    m_stereo_model(stereo_model) {}

  inline int32 cols() const { return m_disparity_map.cols(); }
  inline int32 rows() const { return m_disparity_map.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {
    if ( is_valid(m_disparity_map(i,j,p)) ) {
      Vector3 error;
      pixel_type result;
      subvector(result,0,3) = StereoModelHelper( i, j, m_disparity_map(i,j,p), error );
      subvector(result,3,3) = error;
      return result;
    }
    // For missing pixels in the disparity map, we return a null 3D position.
    return pixel_type();
  }

  /// \cond INTERNAL
  typedef StereoTXAndErrorView<CropView<ImageView<typename DisparityImageT::pixel_type> >,
                               TX1T, TX2T, StereoModelT> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    typedef typename DisparityImageT::pixel_type DPixelT;
    ImageView<DPixelT> disparity_preraster( crop( m_disparity_map, bbox ) );

    // Work out what spots in the right image we'll be touching.
    BBox2i disparity_range = vw::stereo::get_disparity_range( disparity_preraster );
    disparity_range.max() += Vector2i(1,1);
    BBox2i right_bbox = bbox + disparity_range.min();
    right_bbox.max() += disparity_range.size();

    // This is to help any transforms (right now just RPCMapTransform)
    // that must cache their side data. Normally this would happen if
    // we were using a TransformView. Copies are made of the
    // transforms so we are not having a race condition with setting
    // the cache in both transforms while the other threads want to do
    // the same.
    TX1T left_tx = m_tx1; left_tx.reverse_bbox(bbox);
    TX2T right_tx = m_tx2; right_tx.reverse_bbox(right_bbox);

    return prerasterize_type( crop(disparity_preraster,-bbox.min().x(),-bbox.min().y(),cols(),rows()),
                              left_tx, right_tx, m_stereo_model );
  }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
  /// \endcond
};

template <class DisparityT, class TX1T, class TX2T, class StereoModelT>
StereoTXAndErrorView<DisparityT, TX1T, TX2T, StereoModelT>
stereo_error_triangulate( ImageViewBase<DisparityT> const& disparity,
                          TX1T const& tx1, TX2T const& tx2,
                          StereoModelT const& model ) {
  typedef StereoTXAndErrorView<DisparityT, TX1T, TX2T, StereoModelT> result_type;
  return result_type( disparity.impl(), tx1, tx2, model );
}

template <class SessionT>
void stereo_triangulation( Options const& opt ) {
  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 4 --> TRIANGULATION \n";

  typedef ImageViewRef<PixelMask<Vector2f> > PVImageT;
  typedef typename SessionT::stereo_model_type StereoModelT;
  try {
    PVImageT disparity_map =
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
      if (opt.stereo_session_string == "rpc")
        vw_throw(InputErr() << "Stereo with RPC cameras cannot have the camera as the universe center.\n");

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
    vw_out() << "\t--> " << universe_radius_func;

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud.   " << std::endl;
    ImageViewRef<Vector6> point_cloud;
    if ( stereo_settings().use_least_squares ) {
      StereoModelT stereo_model( camera_model1.get(), camera_model2.get(), true );
      point_cloud =
        per_pixel_filter(
          stereo_error_triangulate( disparity_map,
                                    boost::dynamic_pointer_cast<SessionT>(opt.session)->tx_left(),
                                    boost::dynamic_pointer_cast<SessionT>(opt.session)->tx_right(),
                                    stereo_model ), universe_radius_func );
    } else {
      StereoModelT stereo_model( camera_model1.get(), camera_model2.get(), false );
      point_cloud =
        per_pixel_filter(
          stereo_error_triangulate( disparity_map,
                                    boost::dynamic_pointer_cast<SessionT>(opt.session)->tx_left(),
                                    boost::dynamic_pointer_cast<SessionT>(opt.session)->tx_right(),
                                    stereo_model ), universe_radius_func );
    }

    if (stereo_settings().compute_error_vector)
      save_point_cloud(point_cloud, opt);
    else
      save_point_cloud(point_and_error_norm(point_cloud), opt);

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main( int argc, char* argv[] ) {

  stereo_register_sessions();

  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      TriangulationDescription() );

    // Internal Processes
    //---------------------------------------------------------
    if (        opt.stereo_session_string == "pinhole" ) {
      stereo_triangulation<StereoSessionPinhole>( opt );
    } else if ( opt.stereo_session_string == "nadirpinhole" ) {
      stereo_triangulation<StereoSessionNadirPinhole>( opt );
    } else if ( opt.stereo_session_string == "isis"   ) {
      stereo_triangulation<StereoSessionIsis>( opt );
    } else if ( opt.stereo_session_string == "rpc"    ) {
      stereo_triangulation<StereoSessionRPC>( opt );
    } else if ( opt.stereo_session_string == "dg"     ) {
      stereo_triangulation<StereoSessionDG>( opt );
    } else if ( opt.stereo_session_string == "dgmaprpc" ) {
      stereo_triangulation<StereoSessionDGMapRPC>( opt );
    }

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : TRIANGULATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

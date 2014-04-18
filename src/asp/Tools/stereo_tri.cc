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
  template<> struct PixelFormatID<Vector<float, 6> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<float, 4> >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
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
  void save_point_cloud(Vector3 const& shift,
                        ImageT const& point_cloud, Options const& opt){

    std::string point_cloud_file = opt.out_prefix + "-PC.tif";
    vw_out() << "Writing point cloud: " << point_cloud_file << "\n";

    if ( opt.session->name() == "isis" ){
      // ISIS does not support multi-threading
      asp::write_approx_gdal_image
        ( point_cloud_file, shift,
          stereo_settings().point_cloud_rounding_error,
          point_cloud, opt,
          TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }else{
      asp::block_write_approx_gdal_image
        ( point_cloud_file, shift,
          stereo_settings().point_cloud_rounding_error,
          point_cloud, opt,
          TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }

  }

}

// class LargeTriErrorFilter
// Input: A vector, representing a point in a point cloud. First three
// elements of the vector are the xyz coordinates of the
// point. Subsequent elements are the triangulation error, either as a
// scalar (the norm of the error) or as a vector (the error itself).
// Output: The same point if the triangulation error is <=
// max_valid_triangulation_error, and the zero vector otherwise.

class LargeTriErrorFilter: public vw::UnaryReturnSameType {
private:

  double m_max_valid_triangulation_error;

public:
  LargeTriErrorFilter(double max_valid_triangulation_error): m_max_valid_triangulation_error(max_valid_triangulation_error){
    VW_ASSERT(m_max_valid_triangulation_error > 0,
              vw::ArgumentErr() << "LargeTriErrorFilter: max-valid-triangulation error must be positive.");
  }

  // A version without triangulation error
  template <class ElemT>
  Vector<ElemT,3> operator() (Vector<ElemT,3> const& pix) const {
    return pix;
  }

  // A version with norm of triangulation error
  template <class ElemT>
  Vector<ElemT,4> operator() (Vector<ElemT,4> const& pix) const {
    if (subvector(pix,0,3) != Vector<ElemT,3>() ) {
      double err = norm_2(subvector(pix,3,pix.size() - 3));
      if (err > m_max_valid_triangulation_error) {
        return Vector<ElemT,4>();
      } else {
        return pix;
      }
    }
    return Vector<ElemT,4>();
  }
  
  // A version with 3D triangulation error
  template <class ElemT>
  Vector<ElemT,6> operator() (Vector<ElemT,6> const& pix) const {
    if (subvector(pix,0,3) != Vector<ElemT,3>() ) {
      double err = norm_2(subvector(pix,3,pix.size() - 3));
      if (err > m_max_valid_triangulation_error) {
        return Vector<ElemT,6>();
      } else {
        return pix;
      }
    }
    return Vector<ElemT,6>();
  }
    
};

template <class DisparityImageT, class TX1T, class TX2T, class StereoModelT>
class StereoTXAndErrorView : public ImageViewBase<StereoTXAndErrorView<DisparityImageT, TX1T, TX2T, StereoModelT> >
{
  DisparityImageT m_disparity_map;
  TX1T m_tx1;
  TX2T m_tx2;
  StereoModelT m_stereo_model;

  typedef typename DisparityImageT::pixel_type DPixelT;

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

  typedef StereoTXAndErrorView<CropView<ImageView<DPixelT> >,
                               TX1T, TX2T, StereoModelT> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return PreRasterHelper( bbox, m_tx1, m_tx2 );
  }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }

private:

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

  template <class T1, class T2>
  typename boost::disable_if< boost::mpl::and_<boost::is_same<T1,StereoSessionDGMapRPC::left_tx_type>,
                                               boost::is_same<T2,StereoSessionDGMapRPC::right_tx_type> >,
                              prerasterize_type>::type
  PreRasterHelper( BBox2i const& bbox, T1 const& tx1, T2 const& tx2 ) const {
    // General Case
    ImageView<DPixelT> disparity_preraster( crop( m_disparity_map, bbox ) );

    return prerasterize_type( crop( disparity_preraster, -bbox.min().x(), -bbox.min().y(), cols(), rows() ),
                              tx1, tx2, m_stereo_model );
  }

  template <class T1, class T2>
  typename boost::enable_if< boost::mpl::and_<boost::is_same<T1,StereoSessionDGMapRPC::left_tx_type>,
                                              boost::is_same<T2,StereoSessionDGMapRPC::right_tx_type> >,
                             prerasterize_type>::type
  PreRasterHelper( BBox2i const& bbox, T1 const& tx1, T2 const& tx2 ) const {
    // RPC Map Transform needs to be explicitly copied and told to
    // cache for performance.
    ImageView<DPixelT> disparity_preraster( crop( m_disparity_map, bbox ) );

    // Work out what spots in the right image we'll be touching.
    BBox2i disparity_range = vw::stereo::get_disparity_range( disparity_preraster );
    disparity_range.max() += Vector2i(1,1);
    BBox2i right_bbox = bbox + disparity_range.min();
    right_bbox.max() += disparity_range.size();

    // This is to help any transforms (right now just Map2CamTrans)
    // that must cache their side data. Normally this would happen if
    // we were using a TransformView. Copies are made of the
    // transforms so we are not having a race condition with setting
    // the cache in both transforms while the other threads want to do
    // the same.
    T1 tx1_copy = tx1;
    T2 tx2_copy = tx2;
    tx1_copy.tx1.reverse_bbox( bbox );
    tx2_copy.tx1.reverse_bbox( right_bbox );

    return prerasterize_type( crop(disparity_preraster,-bbox.min().x(),-bbox.min().y(),cols(),rows()),
                              tx1_copy, tx2_copy, m_stereo_model );
  }

};

Vector3 find_approx_points_median(std::vector<Vector3> const& points){

  // Find the median of the x coordinates of points, then of y, then of
  // z. Perturb the median a bit to ensure it is never exactly on top
  // of a real point, as in such a case after subtraction of that
  // point from median we'd get the zero vector which by convention
  // is invalid.

  if (points.empty()) return Vector3();

  Vector3 median;
  std::vector<double> V(points.size());
  for (int i = 0; i < (int)median.size(); i++){
    for (int p = 0; p < (int)points.size(); p++) V[p] = points[p][i];
    std::sort(V.begin(), V.end());
    median[i] = V[points.size()/2];

    median[i] += median[i]*1e-10*rand()/double(RAND_MAX);
  }

  return median;
}

Vector3 find_point_cloud_center(Vector2i const& tile_size,
                                ImageViewRef<Vector6> const& point_cloud){

  // Compute the point cloud in a tile around the center of the
  // cloud. Find the median of all the points in that cloud.  That
  // will be the cloud center. If the tile is too small, spiral away
  // from the center adding other tiles.  Keep the tiles aligned to a
  // multiple of tile_size, for consistency with how the point cloud
  // is written to disk later on.

  int numx = (int)ceil(point_cloud.cols()/double(tile_size[0]));
  int numy = (int)ceil(point_cloud.rows()/double(tile_size[1]));

  std::vector<Vector3> points;
  for (int r = 0; r <= std::max(numx/2, numy/2); r++){
    // We are now on the boundary of the square of size 2*r with
    // center at (numx/2, numy/2). Iterate over that boundary.
    for (int x = numx/2-r; x <= numx/2+r; x++){
      for (int y = numy/2-r; y <= numy/2+r; y++){
        if ( x != numx/2-r && x != numx/2+r &&
             y != numy/2-r && y != numy/2+r) continue; // skip inner points
        if (x < 0 || y < 0 || x >= numx || y >= numy) continue; // out of bounds

        BBox2i box(x*tile_size[0], y*tile_size[1], tile_size[0], tile_size[1]);
        box.crop(bounding_box(point_cloud));

        // Crop to the cloud area actually having points
        box.crop(stereo_settings().trans_crop_win);
        
        // Triangulate in the existing box
        ImageView<Vector6> cropped_cloud = crop(point_cloud, box);
        for (int px = 0; px < cropped_cloud.cols(); px++){
          for (int py = 0; py < cropped_cloud.rows(); py++){
            Vector3 xyz = subvector(cropped_cloud(px, py), 0, 3);
            if (xyz == Vector3()) continue;
            points.push_back(xyz);
          }
        }
        
        // Stop if we have enough points to do a reliable mean estimation
        if (points.size() > 100)
          return find_approx_points_median(points);
        
      }
      
    }
  }

  // Have to use what we've got
  return find_approx_points_median(points);
}


template <class DisparityT, class TX1T, class TX2T, class StereoModelT>
StereoTXAndErrorView<DisparityT, TX1T, TX2T, StereoModelT>
stereo_error_triangulate( ImageViewBase<DisparityT> const& disparity,
                          TX1T const& tx1, TX2T const& tx2,
                          StereoModelT const& model ) {
  typedef StereoTXAndErrorView<DisparityT, TX1T, TX2T, StereoModelT> result_type;
  return result_type( disparity.impl(), tx1, tx2, model );
}

bool read_point(std::string const& file, Vector3 & point){

  point = Vector3();
  
  std::ifstream fh(file.c_str());
  if (!fh.good()) return false;

  for (int c = 0; c < (int)point.size(); c++)
    if (! (fh >> point[c]) ) return false;

  return true;
}

void write_point(std::string const& file, Vector3 const& point){

  std::ofstream fh(file.c_str());
  fh.precision(18); // precision(16) is not enough
  for (int c = 0; c < (int)point.size(); c++)
    fh << point[c] << " ";
  fh << std::endl;

}

template <class SessionT>
void stereo_triangulation( Options const& opt ) {

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
    try{
      if ( stereo_settings().universe_center == "camera" ) {
        if (opt.session->name() == "rpc")
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
    }catch(...){
      vw_out(WarningMessage) << "Could not find the camera center. Will not be able to filter triangulated points by radius.\n";
    }
    
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

    // Compute the point cloud center, unless done by now
    Vector3 cloud_center;
    if (!stereo_settings().save_double_precision_point_cloud){
      std::string cloud_center_file = opt.out_prefix + "-PC-center.txt";
      if (!read_point(cloud_center_file, cloud_center)){
        cloud_center = find_point_cloud_center(opt.raster_tile_size,
                                        point_cloud);
        write_point(cloud_center_file, cloud_center);
      }
    }
    if (stereo_settings().compute_point_cloud_center_only){
      vw_out() << "Computed the point cloud center. Will stop here." << std::endl;
      return;
    }

    double min_tri_err = stereo_settings().max_valid_triangulation_error;
    if (min_tri_err > 0)
      point_cloud = per_pixel_filter(point_cloud,
                                     LargeTriErrorFilter(min_tri_err));
    
    // We are supposed to do the triangulation in trans_crop_win only.
    // So force rasterization in that box only using crop(), then pad
    // with zeros, as we want to have the point cloud to have the same
    // dimensions as L.tif, for the sake of point2dem.
    BBox2i cbox = stereo_settings().trans_crop_win;

    if (stereo_settings().compute_error_vector){
      ImageViewRef<Vector6> crop_pc = crop(point_cloud, cbox);
      save_point_cloud(cloud_center,
                       crop(edge_extend(crop_pc, ZeroEdgeExtension()),
                            bounding_box(point_cloud) - cbox.min()),
                       opt);
    }else{
      ImageViewRef<Vector4> crop_pc
        = crop(point_and_error_norm(point_cloud), cbox);
      save_point_cloud(cloud_center,
                       crop(edge_extend(crop_pc, ZeroEdgeExtension()),
                            bounding_box(point_cloud) - cbox.min()),
                       opt);
    }

    // Must print this at the end, as it contains statistics on the number of
    // rejected points.
    vw_out() << "\t--> " << universe_radius_func;
    
  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage -- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  }
}

int main( int argc, char* argv[] ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 4 --> TRIANGULATION \n";

  stereo_register_sessions();

  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      TriangulationDescription() );

    // Triangulation uses small tiles.
    //---------------------------------------------------------
    int ts = Options::tri_tile_size();
    opt.raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------
#define INSTANTIATE(T,NAME) if ( opt.session->name() == NAME ) { stereo_triangulation<T>(opt); }

    INSTANTIATE(StereoSessionPinhole,"pinhole");
    INSTANTIATE(StereoSessionNadirPinhole,"nadirpinhole");
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    INSTANTIATE(StereoSessionIsis,"isis");
#endif
    INSTANTIATE(StereoSessionRPC,"rpc");
    INSTANTIATE(StereoSessionDG,"dg");
    INSTANTIATE(StereoSessionDGMapRPC,"dgmaprpc");

#undef INSTANTIATE

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : TRIANGULATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

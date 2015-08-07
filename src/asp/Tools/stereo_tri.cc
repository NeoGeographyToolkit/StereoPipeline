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
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/RPCStereoModel.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <vw/Cartography.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>
#include <ctime>

using namespace vw;
using namespace asp;
using namespace std;

// These are used to read and write tif images with vector pixels
namespace vw {
  typedef Vector<double, 6> Vector6;
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 6> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<double, 4> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<float,  6> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
  template<> struct PixelFormatID<Vector<float,  4> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
  template<> struct PixelFormatID<Vector<float,  2> >  { static const PixelFormatEnum value = VW_PIXEL_GENERIC_2_CHANNEL; };
}

/// The main class for taking in a set of disparities and returning a point cloud via joint triangulation.
template <class DisparityImageT, class TXT, class StereoModelT>
class StereoTXAndErrorView : public ImageViewBase<StereoTXAndErrorView<DisparityImageT, TXT, StereoModelT> >
{
  vector<DisparityImageT> m_disparity_maps;
  vector<TXT>  m_transforms; // e.g., map-projection or homography to undo
  StereoModelT m_stereo_model;
  bool         m_is_map_projected;
  typedef typename DisparityImageT::pixel_type DPixelT;

public:

  typedef Vector6 pixel_type;
  typedef Vector6 result_type;
  typedef ProceduralPixelAccessor<StereoTXAndErrorView> pixel_accessor;

  /// Constructor
  StereoTXAndErrorView( vector<DisparityImageT> const& disparity_maps,
                        vector<TXT>             const& transforms,
                        StereoModelT            const& stereo_model,
                        bool is_map_projected) :
    m_disparity_maps(disparity_maps),
    m_transforms(transforms),
    m_stereo_model(stereo_model),
    m_is_map_projected(is_map_projected) {

    // Sanity check
    for (int p = 1; p < (int)m_disparity_maps.size(); p++){
      if (m_disparity_maps[0].cols() != m_disparity_maps[p].cols() ||
          m_disparity_maps[0].rows() != m_disparity_maps[p].rows()   )
        vw_throw( ArgumentErr() << "In multi-view triangulation, all disparities must have the same dimensions.\n" );
    }
  }

  inline int32 cols  () const { return m_disparity_maps[0].cols(); }
  inline int32 rows  () const { return m_disparity_maps[0].rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  /// Compute the 3D coordinate corresponding to a pixel location.
  /// - p is not actually used here, it should always be zero!
  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {

    // For each input image, de-warp the pixel in to the native camera coordinates
    int num_disp = m_disparity_maps.size();
    vector<Vector2> pixVec(num_disp + 1);
    pixVec[0] = m_transforms[0].reverse(Vector2(i,j)); // De-warp "left" pixel
    for (int c = 0; c < num_disp; c++){
      Vector2 pix;
      DPixelT disp = m_disparity_maps[c](i,j,p); // Disparity value at this pixel
      if (is_valid(disp)) // De-warp the "right" pixel
        pix = m_transforms[c+1].reverse( Vector2(i,j) + stereo::DispHelper(disp) );
      else // Insert flag values
        pix = Vector2(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());
      pixVec[c+1] = pix;
    }

    // Compute the location of the 3D point observed by each input pixel
    Vector3 errorVec;
    pixel_type result;
    subvector(result,0,3) = m_stereo_model(pixVec, errorVec);
    subvector(result,3,3) = errorVec;
    return result; // Contains location and error vector
  }

  typedef StereoTXAndErrorView<ImageViewRef<DPixelT>, TXT, StereoModelT> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return PreRasterHelper( bbox, m_transforms );
  }
  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }

private:

  /// RPC Map Transform needs to be explicitly copied and told to cache for performance.
  template <class T>
  prerasterize_type PreRasterHelper( BBox2i const& bbox, vector<T> const& transforms) const {

    // Code for NON-MAP-PROJECTED session types.
    if (m_is_map_projected == false) {
      // We explicitly bring in-memory the disparities for the current box
      // to speed up processing later, and then we pretend this is the entire
      // image by virtually enlarging it using a CropView.
      vector< ImageViewRef<DPixelT> > disparity_cropviews;
      for (int p = 0; p < (int)m_disparity_maps.size(); p++){
        ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ) );
        ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
        disparity_cropviews.push_back(cropview_clip);
      }

      return prerasterize_type(disparity_cropviews, transforms, m_stereo_model, m_is_map_projected);
    }

    // Code for MAP-PROJECTED session types.

    // This is to help any transforms (right now just Map2CamTrans)
    // that must cache their side data. Normally this would happen if
    // we were using a TransformView. Copies are made of the
    // transforms so we are not having a race condition with setting
    // the cache in both transforms while the other threads want to do the same.
    vector<T> transforms_copy = transforms;
    transforms_copy[0].reverse_bbox(bbox); // As a side effect this call makes transforms_copy create a local cache we want later

    if (transforms_copy.size() != m_disparity_maps.size() + 1){
      vw_throw( ArgumentErr() << "In multi-view triangulation, "
                << "the number of disparities must be one less "
                << "than the number of images." );
    }

    vector< ImageViewRef<DPixelT> > disparity_cropviews;
    for (int p = 0; p < (int)m_disparity_maps.size(); p++){

      // We explicitly bring in-memory the disparities for the current
      // box to speed up processing later, and then we pretend this is
      // the entire image by virtually enlarging it using a CropView.
      ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ) );
      ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
      disparity_cropviews.push_back(cropview_clip);

      // Work out what spots in the right image we'll be touching.
      BBox2i disparity_range = stereo::get_disparity_range(clip);
      disparity_range.max() += Vector2i(1,1);
      BBox2i right_bbox = bbox + disparity_range.min();
      right_bbox.max() += disparity_range.size();

      // Also cache the data for subsequent transforms
      transforms_copy[p+1].reverse_bbox(right_bbox); // As a side effect this call makes transforms_copy create a local cache we want later
    }

    return prerasterize_type(disparity_cropviews, transforms_copy, m_stereo_model, m_is_map_projected);
  } // End function PreRasterHelper() DGMapRPC version

}; // End class StereoTXAndErrorView

/// Just a wrapper function for StereoTXAndErrorView view construction
template <class DisparityT, class TXT, class StereoModelT>
StereoTXAndErrorView<DisparityT, TXT, StereoModelT>
stereo_error_triangulate( vector<DisparityT> const& disparities,
                          vector<TXT>        const& transforms,
                          StereoModelT       const& model,
                          bool is_map_projected ) {

  typedef StereoTXAndErrorView<DisparityT, TXT, StereoModelT> result_type;
  return result_type( disparities, transforms, model, is_map_projected );
}



namespace asp{

  // TODO: Move some of these functions to a class or something!

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
  void save_point_cloud(Vector3 const& shift, ImageT const& point_cloud,
                        string const& point_cloud_file,
                        Options const& opt){

    vw_out() << "Writing point cloud: " << point_cloud_file << "\n";

    // TODO: Replace this with with a function call!
    if ( (opt.session->name() == "isis") || (opt.session->name() == "isismapisis")){
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

  Vector3 find_approx_points_median(vector<Vector3> const& points){

    // Find the median of the x coordinates of points, then of y, then of
    // z. Perturb the median a bit to ensure it is never exactly on top
    // of a real point, as in such a case after subtraction of that
    // point from median we'd get the zero vector which by convention
    // is invalid.

    if (points.empty()) return Vector3();

    Vector3 median;
    vector<double> V(points.size());
    for (int i = 0; i < (int)median.size(); i++){
      for (int p = 0; p < (int)points.size(); p++) V[p] = points[p][i];
      sort(V.begin(), V.end());
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

    vector<Vector3> points;
    for (int r = 0; r <= max(numx/2, numy/2); r++){
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

  bool read_point(string const& file, Vector3 & point){

    point = Vector3();

    ifstream fh(file.c_str());
    if (!fh.good()) return false;

    for (int c = 0; c < (int)point.size(); c++)
      if (! (fh >> point[c]) ) return false;

    return true;
  }

  void write_point(string const& file, Vector3 const& point){

    ofstream fh(file.c_str());
    fh.precision(18); // precision(16) is not enough
    for (int c = 0; c < (int)point.size(); c++)
      fh << point[c] << " ";
    fh << endl;

  }

}

/// Main triangulation function
template <class SessionT>
void stereo_triangulation( string          const& output_prefix,
                           vector<Options> const& opt_vec ) {

  typedef          ImageViewRef<PixelMask<Vector2f> >  PVImageT;
  typedef typename SessionT::stereo_model_type         StereoModelT;

  const bool is_map_projected = SessionT::isMapProjected();

  try { // Outer try/catch

    // Collect the images, cameras, and transforms. The left image is
    // the same in all n-1 stereo pairs forming the n images multiview
    // system. Same for cameras and transforms.
    vector<string> images;
    vector< boost::shared_ptr<camera::CameraModel> > cameras;
    vector<typename SessionT::tx_type> transforms;
    for (int p = 0; p < (int)opt_vec.size(); p++){

      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt_vec[p].session->camera_models(camera_model1, camera_model2);

      boost::shared_ptr<SessionT> sPtr = boost::dynamic_pointer_cast<SessionT>(opt_vec[p].session);

      if (p == 0){ // The first image is the "left" image for all pairs.
        images.push_back(opt_vec[p].in_file1);
        cameras.push_back(camera_model1);
        transforms.push_back(sPtr->tx_left());
      }
      images.push_back(opt_vec[p].in_file2);
      cameras.push_back(camera_model2);
      transforms.push_back(sPtr->tx_right());
    }

    // If the distance from the left camera center to a point is
    // greater than the universe radius, we remove that pixel and
    // replace it with a zero vector, which is the missing pixel value in the point_image.
    //
    // We apply the universe radius here and then write the result directly to a file on disk.
    stereo::UniverseRadiusFunc universe_radius_func(Vector3(),0,0);
    try{
      if ( stereo_settings().universe_center == "camera" ) {
        if (opt_vec[0].session->name() == "rpc")
          vw_throw(InputErr() << "Stereo with RPC cameras cannot "
                   << "have the camera as the universe center.\n");

        universe_radius_func = stereo::UniverseRadiusFunc(cameras[0]->camera_center(Vector2()),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      } else if ( stereo_settings().universe_center == "zero" ) {
        universe_radius_func = stereo::UniverseRadiusFunc(Vector3(),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      }
    } catch (std::exception &e) {
      std::cout << e.what() << std::endl;
      vw_out(WarningMessage) << "Could not find the camera center. "
                             << "Will not be able to filter triangulated points by radius.\n";
    } // End try/catch

    // Strip the smart pointers and form the stereo model
    std::vector<const vw::camera::CameraModel *> camera_ptrs;
    int num_cams = cameras.size();
    for (int c = 0; c < num_cams; c++)
      camera_ptrs.push_back(cameras[c].get());
    StereoModelT stereo_model( camera_ptrs, stereo_settings().use_least_squares );

    vector<PVImageT> disparity_maps;
    for (int p = 0; p < (int)opt_vec.size(); p++){
      disparity_maps.push_back(opt_vec[p].session->pre_pointcloud_hook(opt_vec[p].out_prefix+"-F.tif"));
    }

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud." << endl;
    ImageViewRef<Vector6> point_cloud = per_pixel_filter
                                            (stereo_error_triangulate(disparity_maps, transforms, stereo_model, is_map_projected),
                                             universe_radius_func);

    // If we crop the left and right images, at each run we must
    // recompute the cloud center, as the cropping windows may
    // have changed.
    bool crop_left_and_right =
      ( stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0)) &&
      ( stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0) );

    // Compute the point cloud center, unless done by now
    Vector3 cloud_center = Vector3();
    if (!stereo_settings().save_double_precision_point_cloud){
      string cloud_center_file = output_prefix + "-PC-center.txt";
      if (!read_point(cloud_center_file, cloud_center) || crop_left_and_right){
        if (!stereo_settings().skip_point_cloud_center_comp) {
          cloud_center
            = find_point_cloud_center(opt_vec[0].raster_tile_size, point_cloud);
          write_point(cloud_center_file, cloud_center);
        }
      }
    }
    if (stereo_settings().compute_point_cloud_center_only){
      vw_out() << "Computed the point cloud center. Will stop here." << endl;
      return;
    }

    // We are supposed to do the triangulation in trans_crop_win only.
    // So force rasterization in that box only using crop(), then pad
    // with zeros, as we want to have the point cloud to have the same
    // dimensions as L.tif, for the sake of point2dem.
    BBox2i cbox = stereo_settings().trans_crop_win;
    string point_cloud_file = output_prefix + "-PC.tif";
    if (stereo_settings().compute_error_vector){

      if (num_cams > 2)
        vw_out(WarningMessage) << "For more than two cameras, the error "
                               << "vector between rays is not meaningful. "
                               << "Setting it to (err_len, 0, 0)." << endl;

      ImageViewRef<Vector6> crop_pc = crop(point_cloud, cbox);
      save_point_cloud(cloud_center,
                       crop(edge_extend(crop_pc, ZeroEdgeExtension()),
                            bounding_box(point_cloud) - cbox.min()),
                       point_cloud_file, opt_vec[0]);
    }else{
      ImageViewRef<Vector4> crop_pc = crop(point_and_error_norm(point_cloud), cbox);
      save_point_cloud(cloud_center,
                       crop(edge_extend(crop_pc, ZeroEdgeExtension()),
                            bounding_box(point_cloud) - cbox.min()),
                       point_cloud_file, opt_vec[0]);
    } // End if/else

    // Must print this at the end, as it contains statistics on the number of rejected points.
    vw_out() << "\t--> " << universe_radius_func;

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage "
              << "-- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  } // End outer try/catch
} // End function stereo_triangulation()


int main( int argc, char* argv[] ) {

  try {

    vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 4 --> TRIANGULATION \n";

    stereo_register_sessions();

    // Unlike other stereo executables, triangulation can handle multiple images and cameras.
    bool verbose = false;
    vector<Options> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, TriangulationDescription(),
                         verbose, output_prefix, opt_vec);

    if (opt_vec.size() > 1){
      // For multiview, turn on logging to file in the run directory
      // in output_prefix, not just in individual subdirectories.
      asp::log_to_file(argc, argv, opt_vec[0].stereo_default_filename,
                       output_prefix);
    }

    // Keep only those stereo pairs for which filtered disparity exists
    vector<Options> opt_vec_new;
    for (int p = 0; p < (int)opt_vec.size(); p++){
      if (fs::exists(opt_vec[p].out_prefix+"-F.tif"))
        opt_vec_new.push_back(opt_vec[p]);
    }
    opt_vec = opt_vec_new;
    if (opt_vec.empty())
      vw_throw( ArgumentErr() << "No valid F.tif files found.\n" );

    // Triangulation uses small tiles.
    //---------------------------------------------------------
    int ts = Options::tri_tile_size();
    for (int s = 0; s < (int)opt_vec.size(); s++)
      opt_vec[s].raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------
#define INSTANTIATE(T,NAME) if ( opt_vec[0].session->name() == NAME ) { \
      stereo_triangulation<T>(output_prefix, opt_vec); }

    INSTANTIATE(StereoSessionPinhole,           "pinhole"     );
    INSTANTIATE(StereoSessionNadirPinhole,      "nadirpinhole");
    INSTANTIATE(StereoSessionRPC,               "rpc"         );
    INSTANTIATE(StereoSessionDG,                "dg"          );
    INSTANTIATE(StereoSessionDGMapRPC,          "dgmaprpc"    );
    INSTANTIATE(StereoSessionRPCMapRPC,         "rpcmaprpc"   );
    INSTANTIATE(StereoSessionPinholeMapPinhole, "pinholemappinhole"   );
#if defined(ASP_HAVE_PKG_ISISIO) && ASP_HAVE_PKG_ISISIO == 1
    INSTANTIATE(StereoSessionIsis,         "isis"        );
    INSTANTIATE(StereoSessionIsisMapIsis,  "isismapisis" );
#endif

#undef INSTANTIATE

    vw_out() << "\n[ " << current_posix_time_string() << " ] : TRIANGULATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

// We must have the implementations of all sessions for triangulation
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/StereoSessionMapProj.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/Covariance.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/DisparityProcessing.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspLog.h>
#include <asp/Core/IpMatchingAlgs.h>
#include <asp/Core/GdalUtils.h>
#include <asp/Tools/stereo.h>

#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Cartography/BathyStereoModel.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Image/Filter.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Cartography/Map2CamTrans.h>
#include <vw/Core/Stopwatch.h>

#include <xercesc/util/PlatformUtils.hpp>
#include <ctime>

using namespace vw;

namespace asp {

enum OUTPUT_CLOUD_TYPE {FULL_CLOUD, BATHY_CLOUD, TOPO_CLOUD}; // all, below water, above water

/// The main class for taking in a set of disparities and returning a
/// point cloud using triangulation. Will compute the triangulation
/// error, and perhaps propagate covariances (creating stddev).
class StereoTriangulation:
  public ImageViewBase<StereoTriangulation> {
  std::vector<DispImageType>    m_disparity_maps;
  std::vector<const vw::camera::CameraModel*> m_camera_ptrs;
  std::vector<vw::TransformPtr> m_transforms; // e.g., map-projection or homography to undo
  vw::cartography::Datum        m_datum;
  vw::stereo::StereoModel       m_stereo_model;
  vw::BathyStereoModel          m_bathy_model;
  bool                          m_is_map_projected;
  bool                          m_bathy_correct;
  OUTPUT_CLOUD_TYPE             m_cloud_type;
  ImageViewRef<PixelMask<float>> m_left_aligned_bathy_mask;
  ImageViewRef<PixelMask<float>> m_right_aligned_bathy_mask;

  typedef typename DispImageType::pixel_type DPixelT;

public:

  typedef Vector6 pixel_type;
  typedef Vector6 result_type;
  typedef ProceduralPixelAccessor<StereoTriangulation> pixel_accessor;

  /// Constructor
  StereoTriangulation(std::vector<DispImageType>    const& disparity_maps,
                      std::vector<const vw::camera::CameraModel*> const& camera_ptrs,
                      std::vector<vw::TransformPtr> const& transforms,
                      vw::cartography::Datum        const& datum,
                      vw::stereo::StereoModel       const& stereo_model,
                      vw::BathyStereoModel          const& bathy_model,
                      bool is_map_projected,
                      bool bathy_correct, OUTPUT_CLOUD_TYPE cloud_type,
                      ImageViewRef<PixelMask<float>> left_aligned_bathy_mask,
                      ImageViewRef<PixelMask<float>> right_aligned_bathy_mask):
    m_disparity_maps(disparity_maps), m_camera_ptrs(camera_ptrs),
    m_transforms(transforms), m_datum(datum),
    m_stereo_model(stereo_model),
    m_bathy_model(bathy_model),
    m_is_map_projected(is_map_projected),
    m_bathy_correct(bathy_correct),
    m_cloud_type(cloud_type),
    m_left_aligned_bathy_mask(left_aligned_bathy_mask),
    m_right_aligned_bathy_mask(right_aligned_bathy_mask) {

    // Sanity check
    for (int p = 1; p < (int)m_disparity_maps.size(); p++) {
      if (m_disparity_maps[0].cols() != m_disparity_maps[p].cols() ||
          m_disparity_maps[0].rows() != m_disparity_maps[p].rows())
        vw_throw(ArgumentErr() << "In multi-view triangulation, all disparities "
                  << "must have the same dimensions.\n");
    }
  }

  inline int32 cols  () const { return m_disparity_maps[0].cols(); }
  inline int32 rows  () const { return m_disparity_maps[0].rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  /// Compute the 3D coordinate corresponding to a pixel location.
  /// - p is not actually used here, it should always be zero!
  inline result_type operator()(size_t i, size_t j, size_t p = 0) const {

    // For each input image, de-warp the pixel in to the native camera coordinates
    int num_disp = m_disparity_maps.size();
    std::vector<Vector2> pixVec(num_disp + 1);
    pixVec[0] = m_transforms[0]->reverse(Vector2(i,j)); // De-warp "left" pixel
    for (int c = 0; c < num_disp; c++) {
      Vector2 pix;
      DPixelT disp = m_disparity_maps[c](i,j,p); // Disparity value at this pixel
      if (is_valid(disp)) // De-warp the "right" pixel
        pix = m_transforms[c+1]->reverse(Vector2(i,j) + stereo::DispHelper(disp));
      else // Insert flag values
        pix = Vector2(std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::quiet_NaN());
      pixVec[c+1] = pix;
    }

    // Compute the location of the 3D point observed by each input pixel
    // when no bathymetry correction is needed.
    Vector3 errorVec;
    pixel_type result;
    if (!m_bathy_correct) {
      try {
        subvector(result, 0, 3) = m_stereo_model(pixVec, errorVec);
        double errLen = norm_2(errorVec);
        if (!stereo_settings().propagate_errors) {
          subvector(result, 3, 3) = errorVec;
        } else {
          // Store intersection error norm in band 3, horizontal
          // stddev in band 4, and vertical stddev in band 5 (if band
          // index starts from 0).
          result[3] = errLen;
          auto const& v = asp::stereo_settings().horizontal_stddev; // alias
          subvector(result, 4, 2)
            = asp::propagateCovariance(subvector(result, 0, 3),
                                       m_datum, v[0], v[1],
                                       m_camera_ptrs[0], m_camera_ptrs[1],
                                       pixVec[0], pixVec[1]);
        }

        // Filter by triangulation error, if desired
        if (stereo_settings().max_valid_triangulation_error > 0.0 &&
            errLen > stereo_settings().max_valid_triangulation_error) {
          result = pixel_type();
          errorVec = Vector3();
        }
      } catch(...) {
        return pixel_type(); // The zero vector, it means that there is no valid data
      }

      return result; // Contains location and error vector
    }

    // Continue with bathymetry correction. Note how we assume no
    // multi-view stereo happens.
    Vector2 lpix(i, j);
    DPixelT disp = m_disparity_maps[0](i, j, p);
    if (!is_valid(disp)) {
      subvector(result, 0, 3) = Vector3(0, 0, 0);
      subvector(result, 3, 3) = Vector3(0, 0, 0);
      return result;
    }

    // See if both the left and right aligned matching pixels are in the aligned
    // bathymetry masks which means bathymetry correction should happen.
    // Note that pixVec has the unwarped left and right pixels.
    Vector2 rpix = lpix + stereo::DispHelper(disp);

    // Do bathy only when the mask is invalid (under water)
    bool do_bathy = vw::areMasked(m_left_aligned_bathy_mask, m_right_aligned_bathy_mask,
                                  lpix, rpix);

    if (m_cloud_type == TOPO_CLOUD) {
      if (!do_bathy) {
        // We are on dry land. Triangulate as before. In this mode
        // the bathy plane may not even exist.
        subvector(result, 0, 3) = m_stereo_model(pixVec, errorVec);
        subvector(result, 3, 3) = errorVec;
        return result;
      } else {
        // We are under water, but we are not interested in the bathy
        // point cloud. Return no-data.
        subvector(result, 0, 3) = Vector3(0, 0, 0);
        subvector(result, 3, 3) = Vector3(0, 0, 0);
        return result;
      }
    }

    if ((m_cloud_type == BATHY_CLOUD && !do_bathy)) {
      // There is no point in continuing, as we won't get what is asked
      subvector(result, 0, 3) = Vector3(0, 0, 0);
      subvector(result, 3, 3) = Vector3(0, 0, 0);
      return result;
    }

    // Do the triangulation. The did_bathy variable may change. 
    bool did_bathy = false;
    subvector(result, 0, 3) = m_bathy_model(pixVec, errorVec, do_bathy, did_bathy);
    subvector(result, 3, 3) = errorVec;

    // If we wanted to do bathy correction and it did not happen, or the opposite,
    // don't return the computed answer
    if ((m_cloud_type == BATHY_CLOUD && !did_bathy) ||
        (m_cloud_type == TOPO_CLOUD && did_bathy)) {
      subvector(result, 0, 3) = Vector3(0, 0, 0);
      subvector(result, 3, 3) = Vector3(0, 0, 0);
      return result;
    }

    // Filter by triangulation error, if desired
    if (stereo_settings().max_valid_triangulation_error > 0.0 &&
        norm_2(errorVec) > stereo_settings().max_valid_triangulation_error) {
      result = pixel_type();
      errorVec = Vector3();
    }

    return result; // Contains location and error vector
  }

  typedef StereoTriangulation prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    return PreRasterHelper(bbox, m_transforms);
  }
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }

private:

  // Find the region associated with the right image that we need to bring in memory
  // based on the disparity 
  BBox2i calc_right_bbox(BBox2i const& left_bbox, ImageView<DPixelT> const& disparity) const {
      BBox2i disparity_range = stereo::get_disparity_range(disparity);
      disparity_range.max() += Vector2i(1,1);
      BBox2i right_bbox = left_bbox + disparity_range.min();
      right_bbox.max() += disparity_range.size();
      return right_bbox;
  }

  /// RPC Map Transform needs to be explicitly copied and told to cache for performance.
  template <class T>
  prerasterize_type PreRasterHelper(BBox2i const& bbox, std::vector<T> const& transforms) const {

    ImageViewRef<PixelMask<float>> in_memory_left_aligned_bathy_mask;
    ImageViewRef<PixelMask<float>> in_memory_right_aligned_bathy_mask;

    // Code for NON-MAP-PROJECTED session types.
    if (m_is_map_projected == false) {
      // We explicitly bring in-memory the disparities for the current box
      // to speed up processing later, and then we pretend this is the entire
      // image by virtually enlarging it using a CropView.
      std::vector<ImageViewRef<DPixelT>> disparity_cropviews;
      for (int p = 0; p < (int)m_disparity_maps.size(); p++) {
        ImageView<DPixelT> clip = crop(m_disparity_maps[p], bbox);
        ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(),
                                                   cols(), rows());
        disparity_cropviews.push_back(cropview_clip);

        if (m_bathy_correct) {
          // Bring the needed parts of the bathy masks in memory as well.
          // We assume no multiview for stereo with bathy correction.
          BBox2i right_bbox = calc_right_bbox(bbox, clip);

          // Bring the needed parts of the bathy masks in memory as well
          BBox2i cropped_right_bbox = right_bbox;
          cropped_right_bbox.expand(1); // will be needed later during triangulation
          cropped_right_bbox.crop(bounding_box(m_right_aligned_bathy_mask));
          ImageView<PixelMask<float>> l_mask_clip = crop(m_left_aligned_bathy_mask, bbox);
          ImageView<PixelMask<float>> r_mask_clip = crop(m_right_aligned_bathy_mask,
                                                         cropped_right_bbox);
          in_memory_left_aligned_bathy_mask
            = crop(l_mask_clip, -bbox.min().x(), -bbox.min().y(), cols(), rows());
          in_memory_right_aligned_bathy_mask
            = crop(r_mask_clip, -cropped_right_bbox.min().x(),
                   -cropped_right_bbox.min().y(), cols(), rows());
        }
      }

      return prerasterize_type(disparity_cropviews, m_camera_ptrs, transforms, m_datum,
                               m_stereo_model, m_bathy_model,
                               m_is_map_projected, m_bathy_correct, m_cloud_type,
                               in_memory_left_aligned_bathy_mask,
                               in_memory_right_aligned_bathy_mask);
    }

    // Code for MAP-PROJECTED session types.

    // This is to help any transforms (right now just Map2CamTrans)
    // that must cache their side data. Normally this would happen if
    // we were using a TransformView. Copies are made of the
    // transforms so we are not having a race condition with setting
    // the cache in both transforms while the other threads want to do the same.
    // Without some sort of duplication function in the transform base class we need
    // to manually copy the Map2CamTrans type which is pretty hacky.
    std::vector<T> transforms_copy(transforms.size());
    for (size_t i = 0; i < transforms.size(); ++i)
      transforms_copy[i] = vw::cartography::mapproj_trans_copy(transforms[i]);

    // As a side effect, this call makes transforms_copy create a local cache we
    // want later. Caching is fast for dense rasterizing purposes, but slow for
    // sparse pixels.
    transforms_copy[0]->reverse_bbox(bbox);
    if (transforms_copy.size() != m_disparity_maps.size() + 1) {
      vw_throw(ArgumentErr() << "In multi-view triangulation, "
                << "the number of disparities must be one less "
                << "than the number of images.");
    }

    std::vector<ImageViewRef<DPixelT>> disparity_cropviews;
    for (int p = 0; p < (int)m_disparity_maps.size(); p++) {

      // We explicitly bring in-memory the disparities for the current
      // box to speed up processing later, and then we pretend this is
      // the entire image by virtually enlarging it using a CropView.

      ImageView<DPixelT> clip(crop(m_disparity_maps[p], bbox));
      ImageViewRef<DPixelT> cropview_clip = crop(clip, -bbox.min().x(), -bbox.min().y(),
                                                 cols(), rows());
      disparity_cropviews.push_back(cropview_clip);

      // Calculate the bbox necessary to bring things into memory
      BBox2i right_bbox = calc_right_bbox(bbox, clip);

      if (m_bathy_correct) {
        // Bring the needed parts of the bathy masks in memory as well
        BBox2i cropped_right_bbox = right_bbox;
        cropped_right_bbox.expand(1); // will be needed later during triangulation
        cropped_right_bbox.crop(bounding_box(m_right_aligned_bathy_mask));
        ImageView<PixelMask<float>> l_mask_clip = crop(m_left_aligned_bathy_mask, bbox);
        ImageView<PixelMask<float>> r_mask_clip = crop(m_right_aligned_bathy_mask,
                                                       cropped_right_bbox);

        in_memory_left_aligned_bathy_mask
          = crop(l_mask_clip, -bbox.min().x(), -bbox.min().y(), cols(), rows());
        in_memory_right_aligned_bathy_mask
          = crop(r_mask_clip, -cropped_right_bbox.min().x(),
                 -cropped_right_bbox.min().y(), cols(), rows());
      }

      // Also cache the data for subsequent transforms
      // As a side effect this call makes transforms_copy create a local cache we want later
      transforms_copy[p+1]->reverse_bbox(right_bbox);
    }

    return prerasterize_type(disparity_cropviews, m_camera_ptrs, transforms_copy, m_datum,
                             m_stereo_model, m_bathy_model, m_is_map_projected,
                             m_bathy_correct, m_cloud_type,
                             in_memory_left_aligned_bathy_mask,
                             in_memory_right_aligned_bathy_mask);
  } // End function PreRasterHelper() mapprojected version
}; // End class StereoTriangulation

/// A wrapper function for StereoTriangulation view construction
StereoTriangulation
stereo_triangulation(std::vector<DispImageType> const& disparities,
                     std::vector<const vw::camera::CameraModel*> const& camera_ptrs,
                     std::vector<vw::TransformPtr>  const& transforms,
                     vw::cartography::Datum         const& datum,
                     vw::stereo::StereoModel        const& stereo_model,
                     vw::BathyStereoModel           const& bathy_model,
                     bool is_map_projected,
                     bool bathy_correct,
                     OUTPUT_CLOUD_TYPE cloud_type,
                     ImageViewRef<PixelMask<float>> left_aligned_bathy_mask,
                     ImageViewRef<PixelMask<float>> right_aligned_bathy_mask) {

  typedef StereoTriangulation result_type;
  return result_type(disparities, camera_ptrs, transforms, datum, stereo_model, bathy_model,
                     is_map_projected, bathy_correct, cloud_type,
                     left_aligned_bathy_mask, right_aligned_bathy_mask);
}

// TODO(oalexan1): Move some of these functions to a class or something!

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
inline point_and_error_norm(ImageViewBase<ImageT> const& image) {
  return UnaryPerPixelView<ImageT, PointAndErrorNorm>(image.impl(), PointAndErrorNorm());
}

template <class ImageT>
void save_point_cloud(Vector3 const& shift, ImageT const& point_cloud,
                      std::string const& point_cloud_file,
                      vw::cartography::GeoReference const& georef,
                      ASPGlobalOptions const& opt) {

  vw_out() << "Writing point cloud: " << point_cloud_file << "\n";
  bool has_georef = true;
  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float

  std::map<std::string, std::string> keywords; // will go to the geoheader
  if (stereo_settings().propagate_errors) {
    keywords["BAND1"] = "ECEF_X";
    keywords["BAND2"] = "ECEF_Y";
    keywords["BAND3"] = "ECEF_Z";
    keywords["BAND4"] = "IntersectionErr";
    keywords["BAND5"] = "HorizontalStdDev";
    keywords["BAND6"] = "VerticalStdDev";
  }

  if (opt.session->supports_multi_threading()) {
    asp::block_write_approx_gdal_image
      (point_cloud_file, shift,
       stereo_settings().point_cloud_rounding_error,
       point_cloud,
       has_georef, georef, has_nodata, nodata,
       opt, TerminalProgressCallback("asp", "\t--> Triangulating: "),
       keywords);
  } else {
    // ISIS does not support multi-threading
    asp::write_approx_gdal_image
      (point_cloud_file, shift,
       stereo_settings().point_cloud_rounding_error,
       point_cloud,
       has_georef, georef, has_nodata, nodata,
       opt, TerminalProgressCallback("asp", "\t--> Triangulating: "),
       keywords);
  }
}

// TODO(oalexan1): Move this to some low-level utils file  
Vector3 find_approx_points_median(std::vector<Vector3> const& points) {

  // Find the median of the x coordinates of points, then of y, then of
  // z. Perturb the median a bit to ensure it is never exactly on top
  // of a real point, as in such a case after subtraction of that
  // point from median we'd get the zero vector which by convention
  // is invalid.

  if (points.empty())
    return Vector3();

  Vector3 median;
  std::vector<double> V(points.size());
  for (int i = 0; i < (int)median.size(); i++) {
    for (int p = 0; p < (int)points.size(); p++) V[p] = points[p][i];
    sort(V.begin(), V.end());
    median[i] = V[points.size()/2];

    median[i] += median[i]*1e-10*rand()/double(RAND_MAX);
  }

  return median;
}

// Find the point cloud center
// TODO(oalexan1): Move this to some low-level point cloud utils file
Vector3 find_point_cloud_center(Vector2i const& tile_size,
                                ImageViewRef<Vector6> const& point_cloud) {

  // Estimate the cloud center with coarse sampling
  std::vector<Vector3> points;
  for (int attempt = 1; attempt <= 4; attempt++) {
    double numSamples = 25 * attempt;
    int dcol = round(point_cloud.cols()/numSamples);
    int drow = round(point_cloud.rows()/numSamples);
    if (dcol < 1)
      dcol = 1;
    if (drow < 1)
      drow = 1;

    // Iterate over the cloud with this sampling rate
    for (int col = 0; col < point_cloud.cols(); col += dcol) {
      for (int row = 0; row < point_cloud.rows(); row += drow) {
        Vector3 xyz = subvector(point_cloud(col, row), 0, 3);
        if (xyz == Vector3())
          continue; // skip invalid points
        points.push_back(xyz);
      }
    }
    if (points.size() > 1)
      return find_approx_points_median(points);

    vw::vw_out() << "Failed to estimate the point cloud center in attempt: "
                 << attempt << ". Will try again with denser sampling.\n";
  }

  // If sampling fails, do a more thorough algorithm. This can be very slow
  // if the point cloud has a big hole in the middle.

  // Compute the point cloud in a tile around the center of the
  // cloud. Find the median of all the points in that cloud.  That
  // will be the cloud center. If the tile is too small, spiral away
  // from the center adding other tiles.  Keep the tiles aligned to a
  // multiple of tile_size, for consistency with how the point cloud
  // is written to disk later on.
  // The logic is done on tiles of pixels, rather than per individual
  // pixel, as that's the only way we can access the point cloud structure.
  int numx = (int)ceil(point_cloud.cols()/double(tile_size[0]));
  int numy = (int)ceil(point_cloud.rows()/double(tile_size[1]));

  // Trace an ever growing square "ring"
  for (int r = 0; r <= std::max(numx/2, numy/2); r++) {

    // We are now on the boundary of the square of size 2*r with
    // center at (numx/2, numy/2). Iterate over that boundary.
    for (int x = numx/2-r; x <= numx/2+r; x++) {
      for (int y = numy/2-r; y <= numy/2+r; y++) {

        if (x != numx/2-r && x != numx/2+r &&
             y != numy/2-r && y != numy/2+r)
          continue; // Skip inner points; we must keep on the "ring" for given r.

        if (x < 0 || y < 0 || x >= numx || y >= numy)
          continue; // out of bounds

        BBox2i tile(x*tile_size[0], y*tile_size[1], tile_size[0], tile_size[1]);
        tile.crop(bounding_box(point_cloud));

        // Crop to the cloud area actually having points
        tile.crop(stereo_settings().trans_crop_win);

        // Triangulate in the existing tile. That will trigger
        // many complicated calculations and the result will be
        // in memory in cropped_cloud.
        ImageView<Vector6> cropped_cloud = crop(point_cloud, tile);
        for (int px = 0; px < cropped_cloud.cols(); px++) {
          for (int py = 0; py < cropped_cloud.rows(); py++) {
            Vector3 xyz = subvector(cropped_cloud(px, py), 0, 3);
            if (xyz == Vector3())
              continue;
            points.push_back(xyz);
          }
        }

        // Stop if we have enough points to do a reliable mean estimation
        if (points.size() > 100)
          return find_approx_points_median(points);

      }// end y loop
    }// end x loop
  }// end r loop

  // Have to use what we've got
  return find_approx_points_median(points);
}

// TODO(oalexan1): Move this to some low-level new util file
bool read_point(std::string const& file, Vector3 & point) {
  point = Vector3();

  std::ifstream fh(file.c_str());
  if (!fh.good()) return false;

  for (int c = 0; c < (int)point.size(); c++)
    if (! (fh >> point[c])) return false;

  return true;
}
void write_point(std::string const& file, Vector3 const& point) {
  std::ofstream fh(file.c_str());
  fh.precision(18); // precision(16) is not enough
  for (int c = 0; c < (int)point.size(); c++)
    fh << point[c] << " ";
  fh << "\n";
}

// This is some logic unrelated to triangulation, but there seems to be no
// good place to put it. Unalign the disparity, and/or create match points
// from disparity, and/or solve for jitter.
void disp_or_matches_work(std::string const& output_prefix,
                          std::vector<ASPGlobalOptions> const& opt_vec,
                          std::vector<vw::TransformPtr> const& transforms,
                          std::vector<DispImageType> const& disparity_maps,
                          std::vector<std::string> const& image_files,
                          std::vector<std::string> const& camera_files,
                          // Cameras can change
                          std::vector<boost::shared_ptr<camera::CameraModel>> & cameras) {
  // Sanity check for some of the operations below
  VW_ASSERT(disparity_maps.size() == 1 && transforms.size() == 2,
            vw::ArgumentErr() << "Expecting two images and one disparity.\n");

  ASPGlobalOptions opt = opt_vec[0];
  bool is_map_projected = opt.session->isMapProjected();

  // Transforms to compensate for alignment
  vw::TransformPtr left_trans  = transforms[0];
  vw::TransformPtr right_trans = transforms[1];

  // Create a disparity map with between the original unaligned images
  if (stereo_settings().unalign_disparity) {
    bool matches_as_txt = stereo_settings().matches_as_txt;
    std::string unaligned_disp_file = asp::unwarped_disp_file(output_prefix,
                                                              opt.in_file1,
                                                              opt.in_file2,
                                                              matches_as_txt);
    unalign_disparity(is_map_projected, disparity_maps[0], left_trans, right_trans,
                      opt, unaligned_disp_file);
  }

  // If the images are mapprojected and we know the original image names,
  // use those for the match file. That because the matches are between
  // the original images, not the map-projected ones.
  std::string img_file_key = "INPUT_IMAGE_FILE";
  std::string left_raw_image = opt.in_file1, right_raw_image = opt.in_file2;
  {
    std::string img_file;
    boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(opt.in_file1));
    vw::cartography::read_header_string(*rsrc.get(), img_file_key, img_file);
    if (!img_file.empty())
      left_raw_image = img_file;
  }
  {
    std::string img_file;
    boost::shared_ptr<vw::DiskImageResource> rsrc(new vw::DiskImageResourceGDAL(opt.in_file2));
    vw::cartography::read_header_string(*rsrc.get(), img_file_key, img_file);
    if (!img_file.empty())
      right_raw_image = img_file;
  }
  bool matches_as_txt = stereo_settings().matches_as_txt;
  std::string match_file = ip::match_filename(output_prefix + "-disp",
                                              left_raw_image, right_raw_image,
                                              matches_as_txt);

  // Pull matches from disparity.
  if (stereo_settings().num_matches_from_disparity > 0 &&
      stereo_settings().num_matches_from_disp_triplets > 0) {
    vw_throw(ArgumentErr() << "Cannot have both --num-matches-from-disparity and  "
              << "--num-matches-from-disp-triplets.\n");
  }

  if (stereo_settings().num_matches_from_disparity > 0) {
    bool gen_triplets = false;
    matchesFromDisp(opt, disparity_maps[0],
                    left_raw_image, right_raw_image,
                    left_trans, right_trans, match_file,
                    stereo_settings().num_matches_from_disparity,
                    gen_triplets, is_map_projected,
                    stereo_settings().matches_as_txt);
  }
  if (stereo_settings().num_matches_from_disp_triplets > 0) {
    bool gen_triplets = true;
    matchesFromDisp(opt, disparity_maps[0],
                    left_raw_image, right_raw_image,
                    left_trans, right_trans, match_file,
                    stereo_settings().num_matches_from_disp_triplets,
                    gen_triplets, is_map_projected,
                    stereo_settings().matches_as_txt);
  }

  return;
}

/// Main triangulation function
void stereo_triangulation(std::string const& output_prefix,
                          std::vector<ASPGlobalOptions> const& opt_vec) {

  try { // Outer try/catch

    bool is_map_projected = opt_vec[0].session->isMapProjected();

    // Collect the images, cameras, and transforms. The left image is
    // the same in all n-1 stereo pairs forming the n images multiview
    // system. Same for cameras and transforms.
    std::vector<std::string> image_files, camera_files;
    std::vector<boost::shared_ptr<camera::CameraModel>> cameras;
    std::vector<vw::TransformPtr> transforms;
    for (int p = 0; p < (int)opt_vec.size(); p++) {

      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt_vec[p].session->camera_models(camera_model1, camera_model2);

      boost::shared_ptr<StereoSession> sPtr = opt_vec[p].session;

      if (p == 0) { // The first image is the "left" image for all pairs.
        image_files.push_back(opt_vec[p].in_file1);
        camera_files.push_back(opt_vec[p].cam_file1);
        cameras.push_back(camera_model1);
        transforms.push_back(sPtr->tx_left());
      }

      image_files.push_back(opt_vec[p].in_file2);
      camera_files.push_back(opt_vec[p].cam_file2);
      cameras.push_back(camera_model2);
      transforms.push_back(sPtr->tx_right());
    }

    // If the distance from the left camera center to a point is greater than
    // the universe radius, we remove that pixel and replace it with a zero
    // vector, which is the missing pixel value in the point_image.
    //
    // We apply the universe radius here and then write the result directly to a
    // file on disk.
    stereo::UniverseRadiusFunc universe_radius_func(Vector3(), 0, 0);
    try{
      if (stereo_settings().universe_center == "camera") {
        if (opt_vec[0].session->name() == "rpc") {
          vw_throw(InputErr() << "Stereo with RPC cameras cannot "
                              << "have the camera as the universe center.\n");
        }

        universe_radius_func
          = stereo::UniverseRadiusFunc(cameras[0]->camera_center(Vector2()),
                                       stereo_settings().near_universe_radius,
                                       stereo_settings().far_universe_radius);
      } else if (stereo_settings().universe_center == "zero") {
        universe_radius_func
        = stereo::UniverseRadiusFunc(Vector3(),
                                     stereo_settings().near_universe_radius,
                                     stereo_settings().far_universe_radius);
      }
    } catch (std::exception &e) {
      vw_out() << e.what() << "\n";
      vw_out(WarningMessage) << "Could not find the camera center. "
                             << "Will not be able to filter triangulated points by radius.\n";
    } // End try/catch

    std::vector<DispImageType> disparity_maps;
    for (int p = 0; p < (int)opt_vec.size(); p++)
      disparity_maps.push_back
        (opt_vec[p].session->pre_pointcloud_hook(opt_vec[p].out_prefix+"-F.tif"));

    bool do_disp_or_matches_work
      = (stereo_settings().unalign_disparity                        ||
         stereo_settings().num_matches_from_disparity > 0           ||
         stereo_settings().num_matches_from_disp_triplets > 0);
    if (do_disp_or_matches_work) {
      disp_or_matches_work(output_prefix, opt_vec, transforms,
                           disparity_maps, image_files, camera_files,
                           // Cameras can change
                           cameras);
    }

    // In correlator mode, can go no further
    if (asp::stereo_settings().correlator_mode) {
      vw_out() << "\t--> Skipping triangulation in correlator mode.\n";
      return;
    }

    if (is_map_projected)
      vw_out() << "\t--> Inputs are map projected." << "\n";

    // Strip the smart pointers and form the stereo model
    std::vector<const vw::camera::CameraModel*> camera_ptrs;
    int num_cams = cameras.size();
    for (int c = 0; c < num_cams; c++)
      camera_ptrs.push_back(cameras[c].get());

    // Convert the angle tol to be in terms of dot product and pass it
    // to the stereo model.
    double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos
      (stereo_settings().min_triangulation_angle*M_PI/180);

    // For bathymetry, apply alignment to masks, if it was not done already in preprocessing.
    // This situation occurs when parallel_stereo is called with --prev-run-prefix,
    // and the previous run was not a bathy run. Note that we do not do this
    // when we skip the point cloud center computation. In that mode we are processing
    // individual parallel_stereo tiles, and the stereo_tri call which aligns the bathy
    // masks and computes the cloud center just finished. This is fragile logic.
    bool bathy_correct = asp::doBathy(asp::stereo_settings());
    if (bathy_correct && !stereo_settings().skip_point_cloud_center_comp)
      opt_vec[0].session->align_bathy_masks(opt_vec[0]);

    // Create both a regular stereo model and a bathy stereo
    // model. Will use the latter only if we do bathymetry. This way
    // the regular stereo model and bathy stereo model can have
    // different interfaces and the former need not know about the
    // latter. Templates are avoided too.
    vw::stereo::StereoModel stereo_model(camera_ptrs, angle_tol);
    vw::BathyStereoModel bathy_stereo_model(camera_ptrs, angle_tol);

    // See if to return all triangulated points, the ones where bathy correction took
    // place, or the ones were it did not take place. Switch to an enum
    // as that is faster to check for later than a string.
    OUTPUT_CLOUD_TYPE cloud_type;
    if (stereo_settings().output_cloud_type == "all")
      cloud_type = FULL_CLOUD;
    else if (stereo_settings().output_cloud_type == "bathy")
      cloud_type = BATHY_CLOUD;
    else if (stereo_settings().output_cloud_type == "topo")
      cloud_type = TOPO_CLOUD;
    else
      vw_throw(ArgumentErr() << "Unknown value for --output-cloud-type.\n");

    // Load the bathy plane and masks
    std::vector<vw::BathyPlane> bathy_plane_vec;
    ImageViewRef<PixelMask<float>> left_aligned_bathy_mask, right_aligned_bathy_mask;
    if (bathy_correct) {

      if (disparity_maps.size() != 1)
        vw_throw(ArgumentErr()
                 << "Bathymetry correction does not work with multiview stereo.\n");

      opt_vec[0].session->read_aligned_bathy_masks(left_aligned_bathy_mask,
                                                   right_aligned_bathy_mask);

      if (left_aligned_bathy_mask.cols() != disparity_maps[0].cols() ||
          left_aligned_bathy_mask.rows() != disparity_maps[0].rows())
        vw_throw(ArgumentErr() << "The dimensions of disparity and left "
                  << "aligned bathymetry mask must agree.\n");

      // The bathy plane is needed only for the underwater component
      if (asp::stereo_settings().output_cloud_type != "topo") {
        int num_images = 2;
        std::string planes_to_load 
          = asp::readBathyPlanesStrOrList(stereo_settings().bathy_plane, 
                                          stereo_settings().bathy_plane_list);
        vw::readBathyPlanes(planes_to_load, num_images, bathy_plane_vec);
        bathy_stereo_model.set_bathy(stereo_settings().refraction_index, bathy_plane_vec);
      }
    }

    // Used to find the datum for the given planet
    vw::cartography::GeoReference georef = opt_vec[0].session->get_georef();

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud." << "\n";
    ImageViewRef<Vector6> point_cloud = per_pixel_filter
      (stereo_triangulation(disparity_maps, camera_ptrs, transforms, georef.datum(),
                            stereo_model, bathy_stereo_model,
                            is_map_projected, bathy_correct, cloud_type,
                            left_aligned_bathy_mask, right_aligned_bathy_mask),
         universe_radius_func);

    // In correlator mode, can go no further
    if (asp::stereo_settings().correlator_mode)
      return;

    // If we crop the left and right images, at each run we must
    // recompute the cloud center, as the cropping windows may have changed.
    bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

    // Compute the point cloud center, unless done by now
    Vector3 cloud_center = Vector3();
    if (!stereo_settings().save_double_precision_point_cloud) {
      std::string cloud_center_file = output_prefix + "-PC-center.txt";
      if (!read_point(cloud_center_file, cloud_center) || crop_left || crop_right) {
        if (!stereo_settings().skip_point_cloud_center_comp) {
          vw::Stopwatch sw;
          sw.start();
          cloud_center = find_point_cloud_center(opt_vec[0].raster_tile_size, point_cloud);
          vw_out() << "Writing point cloud center: " << cloud_center_file << "\n";
          write_point(cloud_center_file, cloud_center);
          sw.stop();
          vw::vw_out() << "Elapsed time in point cloud center estimation: "
            << sw.elapsed_seconds() << " seconds.\n";
        }
      } else {
        vw_out() << "Reading existing point cloud center: " << cloud_center_file << "\n";
      }
    }
    if (stereo_settings().compute_point_cloud_center_only) {
      vw_out() << "Computed the point cloud center.\n";
      return;
    }

    // We are supposed to do the triangulation in trans_crop_win only
    // so force rasterization in that box only using crop().
    // The cloud has 4 bands unless computing the error vector or stddev,
    // when it has 6.
    BBox2i cbox = stereo_settings().trans_crop_win;
    std::string point_cloud_file = output_prefix + "-PC.tif";
    if (stereo_settings().compute_error_vector ||
        stereo_settings().propagate_errors) {
      // The case num_cams > 2 && stereo_settings().propagate_errors
      // will throw an exception, so we won't get here.
      if (num_cams > 2 && stereo_settings().compute_error_vector)
        vw_out(WarningMessage) << "For more than two cameras, the error "
                               << "vector between rays is not meaningful. "
                               << "Setting it to (err_len, 0, 0)." << "\n";

      ImageViewRef<Vector6> crop_pc = crop(point_cloud, cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, georef, opt_vec[0]);
    } else {
      ImageViewRef<Vector4> crop_pc = crop(point_and_error_norm(point_cloud), cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, georef, opt_vec[0]);
    } // End if/else

    // Must print this at the end, as it contains statistics on the number of rejected points.
    vw_out() << "\t--> " << universe_radius_func;

  } catch (IOErr const& e) {
    vw_throw(ArgumentErr() << "\nUnable to start at the triangulation stage. "
              << "Could not read input files.\n" << e.what() << "\n");
  } // End outer try/catch
} // End function stereo_triangulation()

} // End namespace asp

int main(int argc, char* argv[]) {

  if (asp::stereo_settings().correlator_mode &&
      asp::stereo_settings().num_matches_from_disparity <= 0 &&
      asp::stereo_settings().num_matches_from_disp_triplets <= 0) {
    vw_out() << "The triangulation step is skipped with --correlator-mode.\n";
    return 0;
  }

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << asp::current_posix_time_string()
             << " ]: Stage 5 --> TRIANGULATION\n";

    asp::stereo_register_sessions();

    // Unlike other stereo executables, triangulation can handle multiple images and cameras.
    bool verbose = false;
    std::vector<asp::ASPGlobalOptions> opt_vec;
    std::string output_prefix;
    asp::parseStereoArgs(argc, argv, asp::TriangulationDescription(),
                         verbose, output_prefix, opt_vec);

    if (opt_vec.size() > 1) {
      // For multiview, turn on logging to file in the run directory
      // in output_prefix, not just in individual subdirectories.
      asp::log_to_file(argc, argv, opt_vec[0].stereo_default_filename,
                       output_prefix);
    }

    // Keep only those stereo pairs for which filtered disparity exists
    std::vector<asp::ASPGlobalOptions> opt_vec_new;
    for (int p = 0; p < (int)opt_vec.size(); p++) {
      if (fs::exists(opt_vec[p].out_prefix+"-F.tif"))
        opt_vec_new.push_back(opt_vec[p]);
    }
    opt_vec = opt_vec_new;
    if (opt_vec.empty())
      vw_throw(ArgumentErr() << "No valid F.tif files found.\n");

    // Triangulation uses small tiles.
    //---------------------------------------------------------
    int ts = asp::ASPGlobalOptions::tri_tile_size();
    for (int s = 0; s < (int)opt_vec.size(); s++)
      opt_vec[s].raster_tile_size = Vector2i(ts, ts);

    // This is good info to have at triangulation. Also prints a warning regarding
    // small triangulation angle and potentially an empty point cloud.
    if (opt_vec.size() == 1 && !asp::stereo_settings().stereo_dist_mode)
      asp::estimate_convergence_angle(opt_vec[0]);

    asp::stereo_triangulation(output_prefix, opt_vec);

    vw_out() << "\n[ " << asp::current_posix_time_string() << " ]: TRIANGULATION FINISHED\n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}

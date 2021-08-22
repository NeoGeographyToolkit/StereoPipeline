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


/// \file LocalAlignment.cc
///

#include <vw/Math/Transform.h>
#include <vw/Image/PixelMask.h>
#include <vw/Image/Interpolation.h>

#include <asp/Core/LocalAlignment.h>
#include <asp/Core/ImageNormalization.h>
#include <asp/Core/AffineEpipolar.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/OpenCVUtils.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/filesystem.hpp>
#include <boost/dll.hpp>
#include <limits>
#include <cctype>

using namespace vw;
namespace fs = boost::filesystem;

// Debug logic
#define DEBUG_IP 1

namespace asp {

  // Algorithm to perform local alignment. Approach:
  //  - Given the global interest points and the left crop window, find
  //    the right crop window.
  //  - Crop the globally aligned images to these crop windows and find
  //    the interest points for the crops
  //  - Use the interest points to find the local alignment
  //  - Apply the composition of the global and local alignment to the
  //    original unaligned images to find the locally aligned images
  //  - Save the locally aligned images to disk
  //  - Estimate the search range for the locally aligned images

  void local_alignment(// Inputs
                       ASPGlobalOptions   & opt,
                       std::string const  & session_name,
                       int                  max_tile_size,
                       vw::BBox2i    const& tile_crop_win,
                       bool                 write_nodata,
                       vw::camera::CameraModel const * left_camera_model,
                       vw::camera::CameraModel const * right_camera_model,
                       vw::cartography::Datum  const & datum,
                       // Outputs
                       vw::BBox2i         & left_trans_crop_win,
                       vw::BBox2i         & right_trans_crop_win,
                       vw::Matrix<double> & left_local_mat,
                       vw::Matrix<double> & right_local_mat,
                       std::string        & left_aligned_file,
                       std::string        & right_aligned_file,
                       int                & min_disp,
                       int                & max_disp) {
  
  
    // Read the global unaligned images and interest points
  
    std::string left_unaligned_file = opt.in_file1;
    std::string right_unaligned_file = opt.in_file2;

    // TODO(oalexan1): Not sure if parallel_stereo won't strip the
    // crop win information.
    bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));
    if (crop_left) 
      left_unaligned_file = opt.out_prefix + "-L-cropped.tif";
    if (crop_right) 
      right_unaligned_file = opt.out_prefix + "-R-cropped.tif";

    vw_out() << "\t--> Reading global interest points.\n";
    std::vector<vw::ip::InterestPoint> left_ip, right_ip;
    std::string match_filename = vw::ip::match_filename(opt.out_prefix,
                                                        left_unaligned_file,
                                                        right_unaligned_file);
    if (!fs::exists(match_filename))
      vw_throw(ArgumentErr() << "Missing IP file: " << match_filename);

    vw_out() << "\t    * Loading match file: " << match_filename << "\n";
    vw::ip::read_binary_match_file(match_filename, left_ip, right_ip);

    // Read the globally aligned images and alignment transforms
  
    std::string left_globally_aligned_file = opt.out_prefix + "-L.tif";
    std::string right_globally_aligned_file = opt.out_prefix + "-R.tif";
    boost::shared_ptr<DiskImageResource>
      left_rsrc (vw::DiskImageResourcePtr(left_globally_aligned_file)),
      right_rsrc(vw::DiskImageResourcePtr(right_globally_aligned_file));

    float left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
    float right_nodata_value = std::numeric_limits<float>::quiet_NaN();
    if ( left_rsrc->has_nodata_read() )
      left_nodata_value  = left_rsrc->nodata_read();
    if ( right_rsrc->has_nodata_read() )
      right_nodata_value = right_rsrc->nodata_read();

    // Note that we do not create masked images using nodata values.
    DiskImageView<PixelGray<float> > left_globally_aligned_image(left_rsrc),
      right_globally_aligned_image(right_rsrc);

    // At image edges, the tile we work with can be a sliver which can
    // cause issues. Grow it to a square tile, then crop it to the
    // image bounding box (which may make it non-square again, so
    // repeat this a few times during which the box grows
    // bigger).
    left_trans_crop_win = tile_crop_win;
    for (int attempt = 0; attempt < 10; attempt++) {
      int width  = left_trans_crop_win.width();
      int height = left_trans_crop_win.height();
      left_trans_crop_win = grow_box_to_square(left_trans_crop_win, max_tile_size);

      // Ensure we do not exceed the image bounds
      left_trans_crop_win.crop(vw::bounding_box(left_globally_aligned_image));

      // TODO(oalexan1): Here need find the bounding box of the valid data
      // and if necessary grow left_trans_crop_win to make the bounding box
      // of the valid data be as wide and tall as a full tile.

      if (left_trans_crop_win.width() == width && left_trans_crop_win.height() == height) 
        break;
    }
    
    Matrix<double> left_global_mat  = math::identity_matrix<3>();
    Matrix<double> right_global_mat = math::identity_matrix<3>();
    read_matrix(left_global_mat, opt.out_prefix + "-align-L.exr");
    read_matrix(right_global_mat, opt.out_prefix + "-align-R.exr");
    vw::HomographyTransform left_trans(left_global_mat);
    vw::HomographyTransform right_trans(right_global_mat);

    // Estimate the search range based on ip in the current box. If
    // cannot find enough such ip, expand the box a little. Don't try
    // too hard though, as then we end up with too many outliers which
    // can't be filtered easily.
    size_t min_num_ip = 20;
    BBox2i ip_crop_win = left_trans_crop_win;
    int win_size = std::max(left_trans_crop_win.width(), left_trans_crop_win.height());
    
    for (int pass = 0; pass < 2; pass++) {
      
      // Apply the transforms to all the IP we found
      std::vector<vw::ip::InterestPoint> left_trans_ip, right_trans_ip;
      
      for (size_t i = 0; i < left_ip.size(); i++) {
            Vector2 left_pt (left_ip [i].x, left_ip [i].y);
        Vector2 right_pt(right_ip[i].x, right_ip[i].y);
        
        left_pt  = left_trans.forward(left_pt);
        right_pt = right_trans.forward(right_pt);
        
        if (!ip_crop_win.contains(left_pt)) 
          continue;
        
        right_trans_crop_win.grow(right_pt);
        
        // First copy all the data from the input ip, then apply the transform
        left_trans_ip.push_back(left_ip[i]);
        right_trans_ip.push_back(right_ip[i]);
        left_trans_ip.back().x  = left_pt.x();
        left_trans_ip.back().y  = left_pt.y();
        right_trans_ip.back().x = right_pt.x();
        right_trans_ip.back().y = right_pt.y();
      }

#if DEBUG_IP
      std::string out_match_filename = vw::ip::match_filename(opt.out_prefix + "-tile",
                                                              "L.tif", "R.tif");
      vw_out() << "Writing match file: " << out_match_filename << "\n";
      vw::ip::write_binary_match_file(out_match_filename, left_trans_ip, right_trans_ip);
#endif
      
      if (left_trans_ip.size() >= min_num_ip) 
        break;
      
      ip_crop_win.expand(win_size/10);
    }
    
    // Round up
    right_trans_crop_win.expand(1);
    right_trans_crop_win.crop(bounding_box(right_globally_aligned_image));
  
    // TODO(oalexan1): Should we find the search range based on IP or based on D_sub?
    vw_out() << "IP-based search range: " << right_trans_crop_win << std::endl;

    // Redo ip matching in the current tile. It should be more accurate after alignment
    // and cropping

    std::vector<vw::ip::InterestPoint> left_local_ip, right_local_ip;
    detect_match_ip(left_local_ip, right_local_ip,
                    crop(left_globally_aligned_image, left_trans_crop_win),
                    crop(right_globally_aligned_image, right_trans_crop_win), 
                    stereo_settings().ip_per_tile,  
                    "", "", // do not save any results to disk  
                    left_nodata_value, right_nodata_value,
                    "" // do not save any match file to disk
                    );

#if DEBUG_IP 
    {
      vw::cartography::GeoReference georef;
      bool has_georef = false, has_nodata = true;
      float nan_nodata = std::numeric_limits<float>::quiet_NaN();

      std::string left_crop = opt.out_prefix + "-" + "left-crop.tif";
      vw_out() << "\t--> Writing: " << left_crop << "\n";
      block_write_gdal_image(left_crop, crop(left_globally_aligned_image,
                                             left_trans_crop_win),
                             has_georef, georef,
                             has_nodata, left_nodata_value, opt,
                             TerminalProgressCallback("asp","\t  Left:  "));
      
      std::string right_crop = opt.out_prefix + "-" + "right-crop.tif";
      vw_out() << "\t--> Writing: " << right_crop << "\n";
      block_write_gdal_image(right_crop, crop(right_globally_aligned_image,
                                             right_trans_crop_win),
                             has_georef, georef,
                             has_nodata, right_nodata_value, opt,
                             TerminalProgressCallback("asp","\t  Right:  "));

      std::string local_match_filename = vw::ip::match_filename(opt.out_prefix,
                                                                left_crop, right_crop);
      vw_out() << "Writing match file: " << local_match_filename << "\n";
      vw::ip::write_binary_match_file(local_match_filename, left_local_ip, right_local_ip);
    }
#endif
    
    // The matrices which take care of the crop to the current tile
    Matrix<double> left_crop_mat  = math::identity_matrix<3>();
    Matrix<double> right_crop_mat = math::identity_matrix<3>();

    left_crop_mat (0, 2) = -left_trans_crop_win.min().x();
    left_crop_mat (1, 2) = -left_trans_crop_win.min().y();
    right_crop_mat(0, 2) = -right_trans_crop_win.min().x();
    right_crop_mat(1, 2) = -right_trans_crop_win.min().y();

    // Find the local alignment
    std::vector<size_t> ip_inlier_indices;
    bool crop_to_shared_area = false;
    Vector2i local_trans_aligned_size =
      affine_epipolar_rectification(left_trans_crop_win.size(), right_trans_crop_win.size(),
                                    stereo_settings().local_alignment_threshold,
                                    stereo_settings().alignment_num_ransac_iterations,
                                    left_local_ip, right_local_ip,
                                    crop_to_shared_area,
                                    left_local_mat, right_local_mat, &ip_inlier_indices);

    Vector2 params = stereo_settings().local_alignment_outlier_removal_params;
#if 0
    // TODO(oalexan1): Make this into a function!
    // Convert ip to original unaligned and uncropped coordinates
    std::vector<vw::ip::InterestPoint> left_global_ip;
    std::vector<vw::ip::InterestPoint> right_global_ip;

    std::cout << "---total ip " << left_local_ip.size() << std::endl;
    
    for (size_t it = 0; it < ip_inlier_indices.size(); it++) {
      int i = ip_inlier_indices[it];

      Vector2 left_pt (left_local_ip [i].x, left_local_ip [i].y);
      Vector2 right_pt(right_local_ip[i].x, right_local_ip[i].y);

      left_pt  = left_trans.reverse (left_pt  + left_trans_crop_win.min());
      right_pt = right_trans.reverse(right_pt + right_trans_crop_win.min());

      // First copy all the data from the input ip, then apply the transform
      left_global_ip.push_back(left_local_ip[i]);
      right_global_ip.push_back(right_local_ip[i]);
      left_global_ip.back().x  = left_pt.x();
      left_global_ip.back().y  = left_pt.y();
      right_global_ip.back().x = right_pt.x();
      right_global_ip.back().y = right_pt.y();
    }
    
    std::string unaligned_match_filename = vw::ip::match_filename(opt.out_prefix + "-tile",
                                                                  opt.in_file1,
                                                                  opt.in_file2);
    vw_out() << "Writing match file: " << unaligned_match_filename << "\n";
    vw::ip::write_binary_match_file(unaligned_match_filename, left_global_ip, right_global_ip);
    

    // Filter out IPs which fall outside the specified elevation range
    std::cout << "--pct is " << params[0] << std::endl;
    std::cout << "factor is " << params[1] << std::endl;
    // The default percentage value is 95. For filtering by triangulation error
    // that is on the high side, so adjust it.
    // TODO(oalexan1): Think more of this.
    if (params[0] < 100.0)
      filter_ip_using_cameras(left_global_ip, right_global_ip,  
                              left_camera_model,
                              right_camera_model,
                              (75.0/95.0)*params[0], params[1]);

    // Transform back and do again affine epipolar rectification

    left_local_ip.clear();
    right_local_ip.clear();
    for (size_t i = 0; i < left_global_ip.size(); i++) {

      Vector2 left_pt (left_global_ip [i].x, left_global_ip [i].y);
      Vector2 right_pt(right_global_ip[i].x, right_global_ip[i].y);

      left_pt  = left_trans.forward(left_pt)   - left_trans_crop_win.min();
      right_pt = right_trans.forward(right_pt) - right_trans_crop_win.min();

      // First copy all the data from the input ip, then apply the transform
      left_local_ip.push_back(left_global_ip[i]);
      right_local_ip.push_back(right_global_ip[i]);
      left_local_ip.back().x  = left_pt.x();
      left_local_ip.back().y  = left_pt.y();
      right_local_ip.back().x = right_pt.x();
      right_local_ip.back().y = right_pt.y();
    }

    // Redo affine epipolar alignment
    local_trans_aligned_size =
      affine_epipolar_rectification(left_trans_crop_win.size(), right_trans_crop_win.size(),
                                    stereo_settings().local_alignment_threshold,
                                    stereo_settings().alignment_num_ransac_iterations,
                                    left_local_ip, right_local_ip,
                                    crop_to_shared_area,
                                    left_local_mat, right_local_mat, &ip_inlier_indices);
    
#endif
    
    // Combination of global alignment, crop to current tile, and local alignment
    Matrix<double> combined_left_mat  = left_local_mat * left_crop_mat * left_global_mat;
    Matrix<double> combined_right_mat = right_local_mat * right_crop_mat * right_global_mat;

    // Apply the combined transforms to the original unscaled and
    // unaligned images, then scale them too.
    Vector<float32> left_stats, right_stats;
    std::string left_stats_file  = opt.out_prefix + "-lStats.tif";
    std::string right_stats_file = opt.out_prefix + "-rStats.tif";
    vw_out() << "Reading: " << left_stats_file << ' ' << right_stats_file << std::endl;
    read_vector(left_stats,  left_stats_file);
    read_vector(right_stats, right_stats_file);

    float left_unaligned_nodata_value = -32768.0;
    float right_unaligned_nodata_value = -32768.0;
    {
      // Retrieve nodata values and let the handles go out of scope
      // right away.  For this to work, the ISIS type must be
      // registered with the DiskImageResource class. This happens in
      // "stereo.cc", so these calls will create DiskImageResourceIsis
      // objects.
      boost::shared_ptr<DiskImageResource>
        left_unaligned_rsrc (DiskImageResourcePtr(left_unaligned_file)),
        right_unaligned_rsrc(DiskImageResourcePtr(right_unaligned_file));
      asp::get_nodata_values(left_unaligned_rsrc, right_unaligned_rsrc,
                             left_unaligned_nodata_value, right_unaligned_nodata_value);
    }

    // Set up image masks
    DiskImageView<float> left_unaligned_image (left_unaligned_file);
    DiskImageView<float> right_unaligned_image (right_unaligned_file);
    ImageViewRef<PixelMask<float>> left_masked_image
      = create_mask_less_or_equal(left_unaligned_image,  left_unaligned_nodata_value);
    ImageViewRef<PixelMask<float>> right_masked_image
      = create_mask_less_or_equal(right_unaligned_image, right_unaligned_nodata_value);

    PixelMask<float> nodata_mask = PixelMask<float>(); // invalid value for a PixelMask
    
    // Apply the combined transform to original left and right images,
    // rather than to already transformed images. This way we avoid
    // double interpolation.
    ImageViewRef<PixelMask<float>> left_aligned_image  
      = vw::transform(left_masked_image,
                  HomographyTransform(combined_left_mat),
                  local_trans_aligned_size.x(), local_trans_aligned_size.y(),
                  ValueEdgeExtension<PixelMask<float>>(nodata_mask),
                  BilinearInterpolation());

    ImageViewRef<PixelMask<float>> right_aligned_image  
      = vw::transform(right_masked_image,
                  HomographyTransform(combined_right_mat),
                  local_trans_aligned_size.x(), local_trans_aligned_size.y(),
                  ValueEdgeExtension<PixelMask<float>>(nodata_mask),
                  BilinearInterpolation());

    // Normalize the locally aligned images
    bool use_percentile_stretch = false;
    bool do_not_exceed_min_max = (session_name == "isis" ||
                                  session_name == "isismapisis");
    asp::normalize_images(stereo_settings().force_use_entire_range,
                          stereo_settings().individually_normalize,
                          use_percentile_stretch, 
                          do_not_exceed_min_max,
                          left_stats, right_stats,
                          left_aligned_image, right_aligned_image);

    // The S2P MGM algorithm needs NaN data
    float nan_nodata = std::numeric_limits<float>::quiet_NaN();

    // Fully form the image clips in memory. Otherwise, if something
    // goes wrong, there's an abort in block_write_gdal_image
    // somewhere, instead of exceptions being propagated
    // gracefully. It could not be found in reasonable time where the
    // abort was happening.
    
    ImageView<float> left_trans_clip
      = apply_mask(left_aligned_image, nan_nodata); 
    ImageView<float> right_trans_clip
      = apply_mask(crop
                   (edge_extend(right_aligned_image,
                                ValueEdgeExtension<PixelMask<float>>(nodata_mask)),
                    bounding_box(left_aligned_image)), // note the left bounding box
                   nan_nodata);
    
    // Write the locally aligned images to disk
    vw::cartography::GeoReference georef;
    bool has_georef = false, has_aligned_nodata = write_nodata;
    std::string left_tile = "left-aligned-tile.tif";
    std::string right_tile = "right-aligned-tile.tif";
    left_aligned_file = opt.out_prefix + "-" + left_tile; 
    vw_out() << "\t--> Writing: " << left_aligned_file << "\n";
    block_write_gdal_image(left_aligned_file, left_trans_clip,
                           has_georef, georef,
                           has_aligned_nodata, nan_nodata, opt,
                           TerminalProgressCallback("asp","\t  Left:  "));
    right_aligned_file = opt.out_prefix + "-" + right_tile;
    vw_out() << "\t--> Writing: " << right_aligned_file << "\n";
    block_write_gdal_image(right_aligned_file,
                           right_trans_clip,
                           has_georef, georef,
                           has_aligned_nodata, nan_nodata, opt,
                           TerminalProgressCallback("asp","\t  Right:  "));
    
    // TODO(oalexan1): Make the code below a function, called
    // aligned_disp_search_range().
    
    // Apply local alignment to inlier ip and estimate the search range
    vw::HomographyTransform left_local_trans (left_local_mat);
    vw::HomographyTransform right_local_trans(right_local_mat);

    // Find the transformed IP
    std::vector<vw::ip::InterestPoint> left_trans_local_ip;
    std::vector<vw::ip::InterestPoint> right_trans_local_ip;

    for (size_t it = 0; it < ip_inlier_indices.size(); it++) {
      int i = ip_inlier_indices[it];
      Vector2 left_pt (left_local_ip [i].x, left_local_ip [i].y);
      Vector2 right_pt(right_local_ip[i].x, right_local_ip[i].y);

      left_pt  = left_local_trans.forward(left_pt);
      right_pt = right_local_trans.forward(right_pt);

      // First copy all the data from the input ip, then apply the transform
      left_trans_local_ip.push_back(left_local_ip[i]);
      right_trans_local_ip.push_back(right_local_ip[i]);
      left_trans_local_ip.back().x  = left_pt.x();
      left_trans_local_ip.back().y  = left_pt.y();
      right_trans_local_ip.back().x = right_pt.x();
      right_trans_local_ip.back().y = right_pt.y();

#if DEBUG_IP
      // TODO(oalexan1): Some of these IP can still have outliers which
      // can result in an unreasonably large disparity range.
      // std::cout << "Diff is " << right_pt - left_pt << std::endl;
#endif
    }

    // Filter outliers
    if (params[0] < 100.0)
      asp::filter_ip_by_disparity(params[0], params[1],
                                  left_trans_local_ip, right_trans_local_ip); 

    //  Find the disparity search range
    BBox2 disp_range;
    for (size_t it = 0; it < left_trans_local_ip.size(); it++) {
      Vector2 left_pt (left_trans_local_ip [it].x, left_trans_local_ip [it].y);
      Vector2 right_pt(right_trans_local_ip[it].x, right_trans_local_ip[it].y);
      disp_range.grow(right_pt - left_pt);
    }
    
#if DEBUG_IP
    std::string local_aligned_match_filename
      = vw::ip::match_filename(opt.out_prefix, left_tile, right_tile);
    vw_out() << "Writing match file: " << local_aligned_match_filename << "\n";
    vw::ip::write_binary_match_file(local_aligned_match_filename, left_trans_local_ip,
                                    right_trans_local_ip);
#endif
    
    // Expand the disparity search range a bit
    double disp_width = disp_range.width();
    double disp_extra = disp_width * stereo_settings().disparity_range_expansion_percent / 100.0;
    
    min_disp = floor(disp_range.min().x() - disp_extra/2.0);
    max_disp = ceil(disp_range.max().x() + disp_extra/2.0);

    return;
  }

  // Grow a box to a square size. If one of the input dimensions is
  // even and another odd, we'll never get there but what we get will
  // be good enough.
  vw::BBox2i grow_box_to_square(vw::BBox2i const& box, int max_size) {
    int width = box.width();
    int height = box.height();

    if (width > max_size || height > max_size) 
      vw_throw(vw::ArgumentErr() << "Expecting tiles to have at most dimensions of "
               << max_size << ", but got the tile " << box << ".\n");

    int diffx = max_size - width;
    int diffy = max_size - height;

    BBox2i out_box = box;
    out_box.min() -= Vector2(diffx/2, diffy/2);
    out_box.max() += Vector2(diffx/2, diffy/2);

    return out_box;
  }

  // TODO(oalexan1): if left pix or right pix is invalid in the image,
  // the disparity must be invalid! Test with OpenCV SGBM, libelas, and mgm!
  // Also implement for unalign_2d_disparity.
  
  // DO the same for unalign_2d_disparity.
  // Go from 1D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_1d_disparity(// Inputs
                            vw::ImageViewRef<float> aligned_disp_1d, 
                            vw::BBox2i const& left_crop_win, 
                            vw::BBox2i const& right_crop_win,
                            vw::math::Matrix<double> const& left_align_mat,
                            vw::math::Matrix<double> const& right_align_mat,
                            // Output
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> & unaligned_disp_2d) {

    vw::HomographyTransform left_align_trans (left_align_mat);
    vw::HomographyTransform right_align_trans(right_align_mat);

    float nan_nodata = std::numeric_limits<float>::quiet_NaN(); // NaN value
    PixelMask<float> nodata_mask = PixelMask<float>(); // invalid value for a PixelMask

    ImageViewRef<PixelMask<float>> masked_aligned_disp_1d
      = create_mask(aligned_disp_1d, nan_nodata);

    // TODO(oalexan1): Here bilinear interpolation is used. This will
    // make the holes a little bigger where there is no data. Need
    // to figure out if it is desired to fill holes.
    ImageViewRef<PixelMask<float>> interp_aligned_disp_1d
      = interpolate(masked_aligned_disp_1d, BilinearInterpolation(),
                    ValueEdgeExtension<PixelMask<float>>(nodata_mask));
    
    unaligned_disp_2d.set_size(left_crop_win.width(), left_crop_win.height());

    for (int col = 0; col < unaligned_disp_2d.cols(); col++) {
      for (int row = 0; row < unaligned_disp_2d.rows(); row++) {
        Vector2 left_pix(col, row);
        Vector2 left_trans_pix = left_align_trans.forward(left_pix);
        PixelMask<float> interp_disp
          = interp_aligned_disp_1d(left_trans_pix.x(), left_trans_pix.y());

        if (!is_valid(interp_disp)) {
          unaligned_disp_2d(col, row) = PixelMask<Vector2f>();
          unaligned_disp_2d(col, row).invalidate();
          continue;
        }

        // Since the disparity is 1D, the y value (row) is the same
        // as for the input.
        Vector2 right_trans_pix(left_trans_pix.x() + interp_disp.child(), left_trans_pix.y());

        // Undo the transform
        Vector2 right_pix = right_align_trans.reverse(right_trans_pix);

        // Un-transformed disparity
        Vector2 disp_pix = right_pix - left_pix;

        // Adjust for the fact that the two tiles before alignment
        // were crops from larger images
        disp_pix += (right_crop_win.min() - left_crop_win.min());
      
        unaligned_disp_2d(col, row).child() = Vector2f(disp_pix.x(), disp_pix.y());
        unaligned_disp_2d(col, row).validate();
      }   
    }
  
  }

  // Go from 2D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_2d_disparity(// Inputs
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> const& aligned_disp_2d,
                            vw::BBox2i const& left_crop_win, 
                            vw::BBox2i const& right_crop_win,
                            vw::math::Matrix<double> const& left_align_mat,
                            vw::math::Matrix<double> const& right_align_mat,
                            // Output
                            vw::ImageView<vw::PixelMask<vw::Vector2f>> & unaligned_disp_2d) {
    
    vw::HomographyTransform left_align_trans (left_align_mat);
    vw::HomographyTransform right_align_trans(right_align_mat);

    PixelMask<vw::Vector2f> nodata_pix;
    nodata_pix.invalidate();
    
    // TODO(oalexan1): Here bilinear interpolation is used. This will
    // make the holes a little bigger where there is no data. Need
    // to figure out if it is desired to fill holes.
    ImageViewRef<vw::PixelMask<vw::Vector2f>> interp_aligned_disp_2d
      = interpolate(aligned_disp_2d, BilinearInterpolation(),
                    ValueEdgeExtension<PixelMask<vw::Vector2f>>(nodata_pix));
    
    unaligned_disp_2d.set_size(left_crop_win.width(), left_crop_win.height());

    for (int col = 0; col < unaligned_disp_2d.cols(); col++) {
      for (int row = 0; row < unaligned_disp_2d.rows(); row++) {
        Vector2 left_pix(col, row);
        Vector2 left_trans_pix = left_align_trans.forward(left_pix);
        PixelMask<vw::Vector2f> interp_disp
          = interp_aligned_disp_2d(left_trans_pix.x(), left_trans_pix.y());
        
        if (!is_valid(interp_disp)) {
          unaligned_disp_2d(col, row) = PixelMask<Vector2f>();
          unaligned_disp_2d(col, row).invalidate();
          continue;
        }

        // Do the math with doubles rather than with floats, so cast
        // Vector2f to Vector2.
        Vector2 right_trans_pix = left_trans_pix + Vector2(interp_disp.child());

        // Undo the transform
        Vector2 right_pix = right_align_trans.reverse(right_trans_pix);

        // Un-transformed disparity
        Vector2 disp_pix = right_pix - left_pix;

        // Adjust for the fact that the two tiles before alignment
        // were crops from larger images
        disp_pix += (right_crop_win.min() - left_crop_win.min());
      
        unaligned_disp_2d(col, row).child() = Vector2f(disp_pix.x(), disp_pix.y());
        unaligned_disp_2d(col, row).validate();
      }   
    }
    
  }
  
  // Read the list of external stereo programs (plugins) and extract
  // the path to each such plugin and its library dependencies.
  void parse_plugins_list(std::map<std::string, std::string> & plugins,
                          std::map<std::string, std::string> & plugin_libs) {

    // Wipe the outputs
    plugins.clear();
    plugin_libs.clear();
    
    // The plugins are stored in ISISROOT as they are installed with
    // conda. By now the variable ISISROOT should point out to where
    // those are (see asp::set_asp_env_vars()).

    // But note that the plugin list is in the ASP install dir, and
    // not in ISISROOT. This only makes a difference in dev mode.
    
    char * isis_root = getenv("ISISROOT");
    if (isis_root == NULL)
      vw_throw(vw::ArgumentErr() << "The variable ISISROOT was not set.\n");
    
    // Get the path to the plugins from the path of the ASP stereo_corr
    // executable asking for it.
    std::string base_path
      = boost::dll::program_location().parent_path().parent_path().string();
    std::string plugin_list = base_path + "/plugins/stereo" + "/plugin_list.txt";
    
    std::ifstream handle;
    handle.open(plugin_list.c_str());
    if (handle.fail()) 
      vw_throw( vw::IOErr() << "Unable to open file \"" << plugin_list << "\"" );
    
    std::string line;
    while ( getline(handle, line, '\n') ){
      
      if (line.size() == 0 || line[0] == '#')
        continue; // skip comment and empty line
      
      std::string plugin_name, plugin_path, plugin_lib;
      std::istringstream is(line);
      
      // Extract the plugin name and path
      if (!(is >> plugin_name >> plugin_path)) 
        continue;
      
      // Make the plugin name lower-case, but not the rest of the values
      boost::to_lower(plugin_name);
      
      // The plugin lib is optional
      is >> plugin_lib;

      plugin_path = std::string(isis_root) + "/" + plugin_path;

      if (plugin_lib != "") {
        plugin_lib  = std::string(isis_root) + "/" + plugin_lib;
        plugin_lib += ":";
      }

      plugin_lib += std::string(isis_root) + "/lib";
      
      plugins[plugin_name]     = plugin_path;
      plugin_libs[plugin_name] = plugin_lib;
    }

    return;
  }

  // Given a string like "mgm -O 8 -s vfit", separate the name,
  // which is the first word, from the options, which is the rest.
  void parse_stereo_alg_name_and_opts(std::string const& stereo_alg,
                                      std::string      & alg_name,
                                      std::string      & alg_opts) {

    std::istringstream iss(stereo_alg);
    if (!(iss >> alg_name)) 
      vw_throw(vw::ArgumentErr() << "Cannot parse the stereo algorithm from string: "
               << stereo_alg << ".\n");

    alg_opts = "";
    std::string val;
    while (iss >> val)
      alg_opts += val + " ";

    // Make the algorithm name lower-case, but not the options
    boost::to_lower(alg_name);
  }

  // Return true for an option name, which is a dash followed by a non-integer
  bool is_option_name(std::string const& val) {
    return ( val.size() >= 2 && val[0] == '-' && (val[1] < '0' || val[1] > '9') );
  }

  // Return true if looking at a string having an equal sign
  bool is_env_var_and_val(std::string const& val) {
    for (size_t it = 0; it < val.size(); it++) 
      if (val[it] == '=') 
        return true;
  
    return false;
  }

  // Remove spaces at the end
  void rm_trailing_whitespace(std::string & val) {
    while (val.size() > 0 && (val.back() == ' ' || val.back() == '\t'))
      val.pop_back();
  }
  
  // From A=b extract A as the name, and b as he value
  void get_env_var_name_and_val(std::string const& in,
                                std::string & name,
                                std::string & val) {

    name = "";
    val = "";

    bool found_equal = false;

    for (size_t it = 0; it < in.size(); it++) {
      if (in[it] == '=') {
        found_equal = true;
        continue;
      }

      if (!found_equal) 
        name += in[it];
      else
        val += in[it];
    }

    if (name == "" || val == "")
      vw_throw(vw::ArgumentErr() << "Could not extract name and value from string: "
               << in << ".\n");
  }

  // Put option-value pairs in a string. Respect the order given by pos_to_opt.
  std::string concatenate_optons(std::map<std::string, std::string> const& opt_map,
                                 std::map<int, std::string> const& pos_to_opt,
                                 std::string const& sep) {

    std::string out;
    for (auto it = pos_to_opt.begin(); it != pos_to_opt.end(); it++) {
      
      std::string const& opt = it->second; // alias

      auto it2 = opt_map.find(opt);
      if (it2 == opt_map.end()) 
        continue;
      
      out += it2->first + sep + it2->second + " ";
    }

    rm_trailing_whitespace(out);
    return out;
  }
  
  // Given an input string having algorithm options, like "-v 4", and
  // environmental variables, like "VAL=5", possibly with repetitions,
  // so VAL=5 and VAL=6 can both be present, separate the two kinds
  // and remove the repetitions by keeping the values later in the
  // string. Do not allow any input character except letters, numbers,
  // space, period, underscore, plus, minus, and equal signs, for
  // security purposes.
  void extract_opts_and_env_vars(std::string const& input_str,
                                 std::string & options,
                                 std::map<std::string, std::string> & option_map,
                                 std::string & env_vars,
                                 std::map<std::string, std::string> & env_vars_map) {

    options = "";
    env_vars = "";
    option_map.clear();
    env_vars_map.clear();
    
    // Input validation
    for (size_t it = 0; it < input_str.size(); it++) {
      
      if (std::isalnum(input_str[it]) || input_str[it] == ' ' || input_str[it] == '\t' ||
          input_str[it] == '_' || input_str[it] == '+' || input_str[it] == '-' ||
          input_str[it] == '=' || input_str[it] == '.')
        continue;

      vw_throw(vw::ArgumentErr() << "Only the alphanumeric and ' ', '_', '-', '+', '=', '.' "
               << "characters are allowed as part of the stereo options. Got: "
               << input_str[it] << ".\n");
    }

    // Tokenize the inputs
    std::vector<std::string> tokens;
    std::istringstream iss(input_str);
    std::string val;
    while (iss >> val) 
      tokens.push_back(val);

    // Ensure that the order is the same as in the input If
    // an item shows up twice in the input, keep the position of the
    // first occurrence.
    std::map<std::string, int> opt_to_pos;
    std::map<int, std::string> pos_to_opt;

    for (size_t it = 0; it < tokens.size(); it++) {

      std::string const& token = tokens[it];

      if (is_env_var_and_val(token)) {

        // Is an env variable, A=b, like for MGM
        std::string name, val;
        get_env_var_name_and_val(token, name, val);
        env_vars_map[name] = val;
        
        if (opt_to_pos.find(name) == opt_to_pos.end()) {
          // First time it is encountered
          opt_to_pos[name] = it;
          pos_to_opt[it] = name;
        }

      } else {

        // Must be an option, like -v 3.
        if (!is_option_name(token)) 
          vw_throw(vw::ArgumentErr() << "Expecting an option, so something starting "
                   << "with a dash. Got: " << token << ".\n");
        
        if (opt_to_pos.find(token) == opt_to_pos.end()) {
          // First time it is encountered
          opt_to_pos[token] = it;
          pos_to_opt[it] = token;
        }
        
        // All the tokens that follow and which are not options or env
        // vars must be values for this option
        std::string val = "";
        for (size_t it2 = it + 1; it2 < tokens.size(); it2++) {

          if (is_option_name(tokens[it2]) || is_env_var_and_val(tokens[it2])) {

            // This token is not a value, so have to stop this inner
            // loop. Note how we modify the counter 'it' of the outer
            // loop so we revisit tokens[it2] in that loop.
            it = it2 - 1;
            break;
          }

          // Concatenate all the values
          val += tokens[it2] + " ";

          // Note how we modify the outer loop counter 'it'. It will
          // be incremented further as soon as we go to the outer
          // loop, so that loop will not examine tokens[it2] with which
          // we just dealt. Without this line one runs into a bug if
          // it2 is tokens.size() - 1 as the modification of 'it'
          // further up does not kick in.
          it = it2;
        }
        
        rm_trailing_whitespace(val);
        
        if (!val.empty())
          option_map[token] = val;
        else
          option_map[token] = "1"; // if an option has no value treat it as a boolean
      } 
    }
    
    // Now that the repeated options have been collapsed, put these back in strings
    options  = concatenate_optons(option_map, pos_to_opt, " ");
    env_vars = concatenate_optons(env_vars_map, pos_to_opt, "=");
  }

  // Call the OpenCV BM or SGBM algorithm
  void call_opencv_bm_or_sgbm(std::string const& left_file,
                              std::string const& right_file,
                              std::string const& mode, // bm or a flavor of sgbm 
                              int block_size,
                              int min_disp,
                              int max_disp,
                              int prefilter_cap, 
                              int uniqueness_ratio, 
                              int speckle_size, 
                              int speckle_range, 
                              int disp12_diff, 
                              int texture_thresh,      // only for BM
                              int P1,                  // only for SGBM
                              int P2,                  // only for SGBM
                              ASPGlobalOptions const& opt,
                              std::string const& disparity_file,
                              // Output
                              vw::ImageView<float> & out_disp) {
    
    DiskImageView<float> left(left_file);
    DiskImageView<float> right(right_file);

    cv::Mat left_cv, right_cv;
    asp::formScaledByteCVImage(left, left_cv);
    asp::formScaledByteCVImage(right, right_cv);

    // Create the bm and sgbm objects. Their parameters will be overwritten later.
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
  
    if (block_size < 1 || block_size % 2 != 1)
      vw_throw(ArgumentErr() << "The block size must be positive and odd.\n");
    
    int num_disp = (int) 16 * ceil((max_disp - min_disp) / 16.0);

    // Set BM parameters
    bm->setBlockSize(block_size);
    bm->setMinDisparity(min_disp);
    bm->setNumDisparities(num_disp);
    bm->setTextureThreshold(texture_thresh);
    bm->setPreFilterCap(prefilter_cap);
    bm->setUniquenessRatio(uniqueness_ratio);
    bm->setSpeckleWindowSize(speckle_size);
    bm->setSpeckleRange(speckle_range);
    bm->setDisp12MaxDiff(disp12_diff);
    
    // Set SGBM parameters
    
    int cn = 1; // Number of channels
    sgbm->setBlockSize(block_size);
    sgbm->setMinDisparity(min_disp);
    sgbm->setNumDisparities(num_disp);
    sgbm->setP1(P1*cn*block_size*block_size);
    sgbm->setP2(P2*cn*block_size*block_size);
    sgbm->setPreFilterCap(prefilter_cap);
    sgbm->setUniquenessRatio(uniqueness_ratio);
    sgbm->setSpeckleWindowSize(speckle_size);
    sgbm->setSpeckleRange(speckle_range);
    sgbm->setDisp12MaxDiff(disp12_diff);
  
    vw_out() << "Running OpenCV correlation with "
             << "-mode " << mode << " "
             << "-block_size " << block_size << " "
             << "-P1 " << P1 << " "
             << "-P2 " << P2 << " "
             << "-prefilter_cap " << prefilter_cap << " "
             << "-uniqueness_ratio " << uniqueness_ratio << " "
             << "-speckle_size " << speckle_size << " "
             << "-speckle_range " << speckle_range << " "
             << "-disp12_diff " << disp12_diff << " "
             << "--texture_thresh " << texture_thresh << std::endl;

    if (mode == "bm") {
      // Nothing to do here
    } else if (mode == "hh") {
      sgbm->setMode(cv::StereoSGBM::MODE_HH);
    } else if (mode == "sgbm") {
      sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    } else if (mode == "3way") {
      sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    } else {
      vw_throw(ArgumentErr() << "Unknown OpenCV stereo mode: " << mode << ".\n");
    }

    // Expand the image with padding, or else OpenCV crops the output
    // disparity, oddly enough. Idea copied from S2P in sgbm.cpp.
    // Copyright (c) 2012-2013, Gabriele Facciolo
    // This program is free software: you can use, modify and/or redistribute it
    // under the terms of the simplified BSD License. You should have received a
    // copy of this license along this program. If not, see
    // <http://www.opensource.org/licenses/bsd-license.html>.
    float nan = std::numeric_limits<float>::quiet_NaN();
    int pad1  = std::max(max_disp, 0), pad2 = std::max(-min_disp, 0);
    cv::Mat left_cv_extra(left_cv.rows, left_cv.cols + pad1 + pad2, left_cv.type(),
                          cv::Scalar(nan));
    cv::Mat right_cv_extra(right_cv.rows, right_cv.cols + pad1 + pad2, right_cv.type(),
                           cv::Scalar(nan));
    asp::cvInsertBlock(left_cv, pad1, 0, left_cv_extra);
    asp::cvInsertBlock(right_cv, pad1, 0, right_cv_extra);

    cv::Mat disp, disp_extra;

    // Do the correlation
    if (mode == "bm") {
      bm->compute(left_cv_extra, right_cv_extra, disp_extra);
    } else if (mode == "hh" || mode == "sgbm" || mode == "3way") {
      sgbm->compute(left_cv_extra, right_cv_extra, disp_extra);
    }

    // Crop the disparity to remove the padding
    disp = disp_extra(cv::Range::all(), cv::Range(pad1, pad1 + left_cv.cols));

    // Copy to ASP's format and handle no-data
    vw::ImageView<float> asp_disp;
    asp_disp.set_size(disp.cols, disp.rows);
    for (int row = 0; row < disp.rows; row++) {
      for (int col = 0; col < disp.cols; col++) {
        if (disp.at<int16_t>(row, col) == -16*(-min_disp+1)) {
          // Convert from the OpenCV no-data value to NaN
          asp_disp(col, row) = nan;
        } else {
          // sgbm output disparity map is a 16-bit signed single-channel
          // image of the same size as the input image. It contains
          // disparity values scaled by 16. So, to get the floating-point
          // disparity map, you need to divide each disp element by 16.
          asp_disp(col, row) = -((float) disp.at<int16_t>(row, col)) / 16.0;
        }
      
        // Where the input image is nan, make the disparity nan too,
        // removing some sgbm artifacts
        if (std::isnan(left(col, row)))
          asp_disp(col, row) = nan;
      }
    }

    // Write the disparity to disk
    vw::cartography::GeoReference georef;
    bool   has_georef = false;
    bool   has_nodata = true;
    vw_out() << "Writing: " << disparity_file << "\n";
    vw::cartography::block_write_gdal_image(disparity_file, asp_disp,
                                            has_georef, georef,
                                            has_nodata, nan, opt,
                                            TerminalProgressCallback
                                            ("asp", "\t--> Disparity :"));


    // Assign the disparity to the output variable (this should not do a copy).
    out_disp = asp_disp;
  }
  
} // namespace asp

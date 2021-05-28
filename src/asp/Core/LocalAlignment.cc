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

#include <boost/filesystem.hpp>
#include <limits>

using namespace vw;
namespace fs = boost::filesystem;

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

  void local_alignment(ASPGlobalOptions  & opt,
                       std::string const & session_name,
                       BBox2i const      & left_trans_crop_win,
                       BBox2i            & right_trans_crop_win,
                       Matrix<double>    & left_local_mat,
                       Matrix<double>    & right_local_mat,
                       std::string       & left_aligned_file,
                       std::string       & right_aligned_file,
                       int               & min_disp,
                       int               & max_disp) {
  
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

    DiskImageView<PixelGray<float> > left_globally_aligned_image (left_rsrc ),
      right_globally_aligned_image(right_rsrc);

    float left_nodata_value  = std::numeric_limits<float>::quiet_NaN();
    float right_nodata_value = std::numeric_limits<float>::quiet_NaN();
    if ( left_rsrc->has_nodata_read() )
      left_nodata_value  = left_rsrc->nodata_read();
    if ( right_rsrc->has_nodata_read() )
      right_nodata_value = right_rsrc->nodata_read();
  
    Matrix<double> left_global_mat  = math::identity_matrix<3>();
    Matrix<double> right_global_mat = math::identity_matrix<3>();
    read_matrix(left_global_mat, opt.out_prefix + "-align-L.exr");
    read_matrix(right_global_mat, opt.out_prefix + "-align-R.exr");
    vw::HomographyTransform left_trans(left_global_mat);
    vw::HomographyTransform right_trans(right_global_mat);

    // Apply the transforms to all the IP we found
    std::vector<vw::ip::InterestPoint> left_trans_ip, right_trans_ip;

    // Estimate the search range based on ip

    for (size_t i = 0; i < left_ip.size(); i++) {

      Vector2 left_pt (left_ip [i].x, left_ip [i].y);
      Vector2 right_pt(right_ip[i].x, right_ip[i].y);

      left_pt  = left_trans.forward(left_pt);
      right_pt = right_trans.forward(right_pt);

      if (!left_trans_crop_win.contains(left_pt)) 
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

    // Round up
    right_trans_crop_win.expand(1);
    right_trans_crop_win.crop(bounding_box(right_globally_aligned_image));
  
    //std::string out_match_filename = vw::ip::match_filename(opt.out_prefix + "-aligned",
    //                                                        opt.in_file1, opt.in_file2);
  
    //vw_out() << "Writing match file: " << out_match_filename << ".\n";
    //vw::ip::write_binary_match_file(out_match_filename, left_trans_ip, right_trans_ip);

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

    //std::string local_match_filename = vw::ip::match_filename(opt.out_prefix + "-local",
    //                                                          "L.tif", "R.tif");
    //vw_out() << "Writing match file: " << local_match_filename << ".\n";
    //vw::ip::write_binary_match_file(local_match_filename, left_local_ip, right_local_ip);

    // The matrices which take care of the crop to the current tile
    Matrix<double> left_crop_mat  = math::identity_matrix<3>();
    Matrix<double> right_crop_mat = math::identity_matrix<3>();

    left_crop_mat (0, 2) = -left_trans_crop_win.min().x();
    left_crop_mat (1, 2) = -left_trans_crop_win.min().y();
    right_crop_mat(0, 2) = -right_trans_crop_win.min().x();
    right_crop_mat(1, 2) = -right_trans_crop_win.min().y();

    // Find the local alignment
    std::vector<size_t> ip_inlier_indices;
    Vector2i local_trans_aligned_size =
      affine_epipolar_rectification(left_trans_crop_win.size(), right_trans_crop_win.size(),
                                    stereo_settings().local_alignment_threshold,
                                    stereo_settings().alignment_num_ransac_iterations,
                                    left_local_ip, right_local_ip,
                                    left_local_mat, right_local_mat, &ip_inlier_indices);

    // Combination of global alignment, crop to current tile, and local alignment
    Matrix<double> combined_left_mat  = left_local_mat * left_crop_mat * left_global_mat;
    Matrix<double> combined_right_mat = right_local_mat * right_crop_mat * right_global_mat;

    // Apply the combined transforms to the original unscaled and unaligned images,
    // then scale them too.
    Vector<float32> left_stats, right_stats;
    std::string left_stats_file  = opt.out_prefix + "-lStats.tif";
    std::string right_stats_file = opt.out_prefix + "-rStats.tif";
    vw_out() << "Reading: " << left_stats_file << ' ' << right_stats_file << std::endl;
    read_vector(left_stats,  left_stats_file);
    read_vector(right_stats, right_stats_file);

    float left_unaligned_nodata_value = -32768.0;
    float right_unaligned_nodata_value = -32768.0;
    {
      // Retrieve nodata values and let the handles go out of scope right away.
      // For this to work, the ISIS type must be registered with the
      // DiskImageResource class.  This happens in "stereo.cc", so
      // these calls will create DiskImageResourceIsis objects.
      boost::shared_ptr<DiskImageResource>
        left_unaligned_rsrc (DiskImageResourcePtr(left_unaligned_file)),
        right_unaligned_rsrc(DiskImageResourcePtr(right_unaligned_file));
      asp::get_nodata_values(left_unaligned_rsrc, right_unaligned_rsrc,
                             left_unaligned_nodata_value, right_unaligned_nodata_value);
    }

    // Set up image masks
    DiskImageView<float> left_unaligned_image (left_unaligned_file);
    DiskImageView<float> right_unaligned_image (right_unaligned_file);
    ImageViewRef< PixelMask<float> > left_masked_image
      = create_mask_less_or_equal(left_unaligned_image,  left_unaligned_nodata_value);
    ImageViewRef< PixelMask<float> > right_masked_image
      = create_mask_less_or_equal(right_unaligned_image, right_unaligned_nodata_value);

    // Apply the combined transform to original left and right images,
    // rather than to already transformed images. This way we avoid
    // double interpolation.
    ImageViewRef< PixelMask<float> > left_aligned_image  
      = transform(left_masked_image,
                  HomographyTransform(combined_left_mat),
                  local_trans_aligned_size.x(), local_trans_aligned_size.y());
    ImageViewRef< PixelMask<float> > right_aligned_image  
      = transform(right_masked_image,
                  HomographyTransform(combined_right_mat),
                  local_trans_aligned_size.x(), local_trans_aligned_size.y());

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

    PixelMask<float> nodata_mask = PixelMask<float>(); // invalid value for a PixelMask
  
    // The S2P MGM algorithm needs NaN data
    float nan_nodata = std::numeric_limits<float>::quiet_NaN();

    // Write the locally aligned images to disk
    vw::cartography::GeoReference georef;
    bool has_georef = false, has_aligned_nodata = true;
    std::string left_tile = "left-aligned-tile.tif";
    std::string right_tile = "right-aligned-tile.tif";
    left_aligned_file = opt.out_prefix + "-" + left_tile; 
    vw_out() << "\t--> Writing: " << left_aligned_file << ".\n";
    block_write_gdal_image(left_aligned_file, apply_mask(left_aligned_image, nan_nodata),
                           has_georef, georef,
                           has_aligned_nodata, nan_nodata, opt,
                           TerminalProgressCallback("asp","\t  Left:  "));
  
    right_aligned_file = opt.out_prefix + "-" + right_tile;
    vw_out() << "\t--> Writing: " << right_aligned_file << ".\n";
    block_write_gdal_image(right_aligned_file,
                           apply_mask
                           (crop
                            (edge_extend(right_aligned_image,
                                         ValueEdgeExtension<PixelMask<float>>(nodata_mask)),
                             bounding_box(left_aligned_image)), // note the left bounding box
                            nan_nodata),
                           has_georef, georef,
                           has_aligned_nodata, nan_nodata, opt,
                           TerminalProgressCallback("asp","\t  Right:  "));
  
    // Apply local alignment to inlier ip and estimate the search range
    std::vector<vw::ip::InterestPoint> left_trans_local_ip;
    std::vector<vw::ip::InterestPoint> right_trans_local_ip;
    vw::HomographyTransform left_local_trans (left_local_mat);
    vw::HomographyTransform right_local_trans(right_local_mat);
    BBox2 disp_range;
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

      disp_range.grow(right_pt - left_pt);
    }

    // std::string local_aligned_match_filename
    //  = vw::ip::match_filename(opt.out_prefix, left_tile, right_tile);
    // vw_out() << "Writing match file: " << local_aligned_match_filename << ".\n";
    // vw::ip::write_binary_match_file(local_aligned_match_filename, left_trans_local_ip,
    //                                   right_trans_local_ip);

    double disp_width = disp_range.width();
    double disp_extra = disp_width * stereo_settings().disparity_range_expansion_percent / 100.0;
    min_disp = floor(disp_range.min().x() - disp_extra/2.0);
    max_disp = ceil(disp_range.max().x() + disp_extra/2.0);

    return;
  }

  
  // Go from 1D disparity of images with affine epipolar alignment to the 2D
  // disparity by undoing the transforms that applied this alignment.
  void unalign_disparity(// Inputs
                         vw::ImageViewRef<float> disp_1d, 
                         vw::BBox2i const& left_crop_win, 
                         vw::BBox2i const& right_crop_win,
                         vw::math::Matrix<double> const& left_align_mat,
                         vw::math::Matrix<double> const& right_align_mat,
                         // Output
                         vw::ImageView<vw::PixelMask<vw::Vector2f>> & disp_2d) {

    vw::HomographyTransform left_align_trans (left_align_mat);
    vw::HomographyTransform right_align_trans(right_align_mat);

    float nan_nodata = std::numeric_limits<float>::quiet_NaN(); // NaN value
    PixelMask<float> nodata_mask = PixelMask<float>(); // invalid value for a PixelMask

    ImageViewRef<PixelMask<float>> masked_disp_1d = create_mask(disp_1d, nan_nodata);

    // TODO(oalexan1): Here bilinear interpolation is used. This will
    // make the holes a little bigger where there is no data. Need
    // to figure out if it is desired to fill holes.
    ImageViewRef<PixelMask<float>> interp_disp_1d
      = interpolate(masked_disp_1d, BilinearInterpolation(),
                    ValueEdgeExtension<PixelMask<float>>(nodata_mask));
    
    disp_2d.set_size(left_crop_win.width(), left_crop_win.height());

    for (int col = 0; col < disp_2d.cols(); col++) {
      for (int row = 0; row < disp_2d.rows(); row++) {
        Vector2 left_pix(col, row);
        Vector2 left_trans_pix = left_align_trans.forward(left_pix);
        PixelMask<float> interp_disp = interp_disp_1d(left_trans_pix.x(), left_trans_pix.y());

        if (!is_valid(interp_disp)) {
          disp_2d(col, row) = PixelMask<Vector2f>();
          disp_2d(col, row).invalidate();
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
      
        disp_2d(col, row).child() = Vector2f(disp_pix.x(), disp_pix.y());
        disp_2d(col, row).validate();
      }   
    }
  
  }

} // namespace asp

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

// \file image_mosaic.cc
//
// Tool for creating mosaics of images on disk. Currently supports one line of
// images.

#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/AspLog.h>
#include <asp/Core/Macros.h>
#include <asp/Core/InterestPointMatching.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/GdalUtils.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Image/Algorithms2.h>
#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/Interpolation.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/InterestPoint/MatcherIO.h>
#include <vw/Math/Geometry.h>
#include <vw/Math/RANSAC.h>
#include <vw/Image/Filter.h>
#include <vw/Image/Transform.h>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <limits>

using namespace vw;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/// GDAL block write sizes must be a multiple to 16 so if the input value is
///  not a multiple of 16 increase it until it is.
void fix_tile_multiple(int &size) {
  const int TILE_MULTIPLE = 16;
  if (size % TILE_MULTIPLE != 0)
    size = ((size / TILE_MULTIPLE) + 1) * TILE_MULTIPLE;
}

struct Options: vw::GdalWriteOptions {
  std::vector<std::string> image_files;
  std::string orientation, output_image, output_type, out_prefix;
  int    overlap_width, band, blend_radius, ip_per_tile, num_ransac_iterations;
  bool   has_input_nodata_value, has_output_nodata_value, reverse, rotate,
         use_affine_transform, rotate90, rotate90ccw;
  double input_nodata_value, output_nodata_value, inlier_threshold;
  Vector2 big_tile_size;
  Options(): has_input_nodata_value(false), has_output_nodata_value(false),
             input_nodata_value (std::numeric_limits<double>::quiet_NaN()),
             output_nodata_value(std::numeric_limits<double>::quiet_NaN()){}
};

/// Load an input image, respecting the user parameters.
void get_input_image(std::string const& path,
                     Options const& opt,
                     ImageViewRef<float> &image,
                     double &nodata) {

  // Extract the desired band. // In VW, bands start from 0, not 1.
  image = vw::read_channel<float>(path, opt.band - 1);
  
  // Read the nodata-value from disk
  DiskImageResourceGDAL in_rsrc(path);
  bool has_nodata = in_rsrc.has_nodata_read();
  if (has_nodata)
    nodata = in_rsrc.nodata_read();
  else
    nodata = std::numeric_limits<double>::quiet_NaN();
}

/// Get a list of matched IP, looking in certain image regions.
void match_ip_in_regions(std::string const& image_file1,
                         std::string const& image_file2,
                         BBox2i const& roi1,
                         BBox2i const& roi2,
                         std::vector<ip::InterestPoint> &matched_ip1,
                         std::vector<ip::InterestPoint> &matched_ip2,
                         Options const& opt) {

  // Load the input images
  ImageViewRef<float> image1,  image2;
  double              nodata1, nodata2;
  get_input_image(image_file1, opt, image1, nodata1);
  get_input_image(image_file2, opt, image2, nodata2);

  vw_out() << "Matching interest points between: " << image_file1 << " and "
           << image_file2 << std::endl;
  
  std::string match_file;
  bool matches_as_txt = false;
  if (opt.out_prefix != "") {
    // Write a match file for debugging
    match_file = ip::match_filename(opt.out_prefix, image_file1, image_file2, matches_as_txt);

    // If the match file already exists, load it instead of finding new points.
    if (fs::exists(match_file)) {
      vw_out() << "Reading matched interest points from file: " << match_file << std::endl;
      ip::read_match_file(match_file, matched_ip1, matched_ip2, matches_as_txt);
      vw_out() << "Read in " << matched_ip1.size() << " matched IP.\n";
    }
  }

  if (matched_ip1.empty()) {
    // Now find and match interest points in the selected regions
    size_t number_of_jobs = 1;
    bool use_cached_ip = false;    
    asp::detect_match_ip(matched_ip1, matched_ip2,
                        crop(image1, roi1),
                        crop(image2, roi2), opt.ip_per_tile, number_of_jobs,
                        "", "", use_cached_ip,
                        nodata1, nodata2, match_file);
  }

  // TODO: This should be a function!
  // Adjust the IP to account for the search ROIs.
  for (size_t i=0; i<matched_ip1.size(); ++i) {
    matched_ip1[i].x  += roi1.min()[0];
    matched_ip1[i].ix += roi1.min()[0];
    matched_ip1[i].y  += roi1.min()[1];
    matched_ip1[i].iy += roi1.min()[1];
    
    matched_ip2[i].x  += roi2.min()[0];
    matched_ip2[i].ix += roi2.min()[0];
    matched_ip2[i].y  += roi2.min()[1];
    matched_ip2[i].iy += roi2.min()[1];
  }
} // End function match_ip_in_regions

/// Compute a matrix transform between images, searching for IP in
///  the specified regions.
Matrix<double> compute_ip_matching(std::string const& image_file1,
                                   std::string const& image_file2,
                                   BBox2i const& roi1,
                                   BBox2i const& roi2,
                                   Options & opt) {

  // Find IP, looking in only the specified regions.
  std::vector<ip::InterestPoint> matched_ip1, matched_ip2;
  match_ip_in_regions(image_file1, image_file2, roi1, roi2,
                      matched_ip1,  matched_ip2, opt);

  // Clean up lists.
  std::vector<Vector3> ransac_ip1 = iplist_to_vectorlist(matched_ip1);
  std::vector<Vector3> ransac_ip2 = iplist_to_vectorlist(matched_ip2);

  // RANSAC parameters
  int min_num_output_inliers = ransac_ip1.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;
  vw_out() << "Number of RANSAC iterations: " << opt.num_ransac_iterations << "\n";
  vw_out() << "Inlier threshold in pixels: " << opt.inlier_threshold << "\n";

  Matrix<double> tf;
  std::vector<size_t> indices;
  try {
    if (opt.use_affine_transform) {
      vw::math::RandomSampleConsensus<vw::math::AffineFittingFunctor,
                                      vw::math::InterestPointErrorMetric>
                                        ransac(vw::math::AffineFittingFunctor(),
                                              vw::math::InterestPointErrorMetric(),
                                              opt.num_ransac_iterations,
                                              opt.inlier_threshold,
                                              min_num_output_inliers,
                                              reduce_min_num_output_inliers_if_no_fit);
      tf      = ransac(ransac_ip2, ransac_ip1);
      indices = ransac.inlier_indices(tf, ransac_ip2, ransac_ip1);
    } else { // More restricted transform
      vw::math::RandomSampleConsensus<vw::math::TranslationRotationFittingFunctorN<2>,
                                      vw::math::InterestPointErrorMetric>
                                      ransac(vw::math::TranslationRotationFittingFunctorN<2>(),
                                             vw::math::InterestPointErrorMetric(),
                                             opt.num_ransac_iterations,
                                             opt.inlier_threshold,
                                             min_num_output_inliers,
                                             reduce_min_num_output_inliers_if_no_fit);
      tf      = ransac(ransac_ip2, ransac_ip1);
      indices = ransac.inlier_indices(tf, ransac_ip2, ransac_ip1);
    }
  } catch (...) {
    vw_throw( ArgumentErr() << "Automatic alignment failed in the RANSAC fit.");
  }

  // Keeping only inliers
  std::vector<ip::InterestPoint> inlier_ip1, inlier_ip2;
  for ( size_t i = 0; i < indices.size(); i++ ) {
    inlier_ip1.push_back( matched_ip1[indices[i]] );
    inlier_ip2.push_back( matched_ip2[indices[i]] );
  } 

  std::string match_file;
  bool matches_as_txt = false;
  if (opt.out_prefix != "") {
    // Write a match file for debugging
    match_file = ip::clean_match_filename(opt.out_prefix, image_file1, image_file2,
                                          matches_as_txt);
    vw_out() << "Writing inlier matches after RANSAC to: " << match_file << std::endl;
    ip::write_match_file(match_file, inlier_ip1, inlier_ip2, matches_as_txt);
  }
  
  return tf;
}

// TODO: Pass in image ref instead of paths?
/// Compute the transform from image1 to image2
///  (the top left corner of image1 is (0,0))
Matrix<double> compute_relative_transform(std::string const& image1,
                                          std::string const& image2,
                                          Options& opt) {

  Vector2i size1 = file_image_size(image1);
  Vector2i size2 = file_image_size(image2);

  // Set up the ROIs for the two images based on the selected orientation.
  // - Currently only horizontal orientation is supported.
  BBox2i roi1, roi2;
  if (opt.orientation == "horizontal") {
    roi1.min() = Vector2(std::max(size1[0] - opt.overlap_width, 0), 0);
    roi1.max() = size1; // Bottom right corner
    roi2.min() = Vector2(0, 0); // Top left corner
    roi2.max() = Vector2(std::min(opt.overlap_width, size2[0]), size2[1]); // Bottom right
  }

  if (roi1.empty() || roi2.empty())
    vw_throw( ArgumentErr() << "Unrecognized image orientation!");

  Matrix<double> tf;
  
  // Try several times, with ever-larger value of ip-per-tile. Normally this value 
  // is computed internally, or passed in by the user via --ip-per-tile. In 
  // either case, it will be recorded in asp::stereo_settings().ip_per_tile,
  // which we will increase in subsequent attempts.
  int num_attempts = 3;
  for (int attempt = 1; attempt <= num_attempts; attempt++) {
    try {
      tf = compute_ip_matching(image1, image2, roi1, roi2, opt);
      break;
    } catch (std::exception const& e) {
      vw_out() << "Failed with error: " << e.what() << "\n";
      if (attempt == num_attempts)
        vw_throw(ArgumentErr()
                 << "Failed to compute the relative transform. Inspect your images.\n");
    }

    // Wipe any old match file to force it to be regenerated
    std::string match_file;
    bool matches_as_txt = false;
    if (opt.out_prefix != "")
      match_file = ip::match_filename(opt.out_prefix, image1, image2, matches_as_txt);
    if (!match_file.empty() && fs::exists(match_file)) {
      vw::vw_out() << "Removing old match file: " << match_file << "\n";
      fs::remove(match_file);
    }
      
    // Multiply the number of IP per tile by 4 and try again.
    asp::stereo_settings().ip_per_tile *= 4;
    opt.ip_per_tile = asp::stereo_settings().ip_per_tile;
    vw_out() << "Increasing --ip-per-tile to: " << asp::stereo_settings().ip_per_tile 
             << "\n";
  }
  
  return tf;

} // End function compute_relative_transform

/// Compute the positions of each image relative to the first image.
/// - The top left corner of the first image is coordinate 0,0 in the output image.
void compute_all_image_positions(Options & opt,
                                 std::vector<boost::shared_ptr<vw::Transform>> & transforms,
                                 std::vector<BBox2i>          & bboxes,
                                 Vector2i                     & output_image_size) {

  const size_t num_images = opt.image_files.size();

  transforms.resize(num_images);
  bboxes.resize(num_images);

  // Init the bounding box to contain the first image
  BBox2i output_bbox;
  Vector2i image_size = file_image_size(opt.image_files[0]);
  output_bbox.grow(Vector2i(0,0));
  output_bbox.grow(image_size);

  // Init values for the first image
  
  Vector2   to(0, 0);
  Matrix2x2 mo;
  mo(0,0) = 1;
  mo(0,1) = 0;
  mo(1,0) = 0;
  mo(1,1) = 1;
  transforms[0] = boost::shared_ptr<vw::Transform>(new AffineTransform(mo, to));
  bboxes    [0] = output_bbox;
  
  // This approach only works for serial pairs, if we add another type of
  //  orientation it will need to be changed.
  Matrix<double> last_transform = identity_matrix(3);
  
  for (size_t i=1; i<num_images; ++i) {

    Matrix<double> relative_transform = 
        compute_relative_transform(opt.image_files[i-1], opt.image_files[i], opt);

    image_size = file_image_size(opt.image_files[i]);

    Matrix<double> absolute_transform;
    if (i == 0) { // First transform
      absolute_transform = relative_transform;
    } else { // Chain from the last transform
      absolute_transform = last_transform * relative_transform;
    }
    last_transform = absolute_transform;
    
    Vector2   t(absolute_transform(0,2), absolute_transform(1,2));
    Matrix2x2 m;
    m(0,0) = absolute_transform(0,0);
    m(0,1) = absolute_transform(0,1);
    m(1,0) = absolute_transform(1,0);
    m(1,1) = absolute_transform(1,1);
    boost::shared_ptr<vw::Transform> tf_ptr(new AffineTransform(m, t));

    Matrix<double> inv_absolute_transform = inverse(absolute_transform);
    Vector2   ti(inv_absolute_transform(0,2), inv_absolute_transform(1,2));
    Matrix2x2 mi;
    mi(0,0) = inv_absolute_transform(0,0);
    mi(0,1) = inv_absolute_transform(0,1);
    mi(1,0) = inv_absolute_transform(1,0);
    mi(1,1) = inv_absolute_transform(1,1);
    boost::shared_ptr<vw::Transform> inv_tf_ptr(new AffineTransform(mi, ti));
    
    transforms[i] = tf_ptr; // Record transfrom from output to input

    vw_out() << "relative_transform: " << relative_transform << std::endl;
    vw_out() << "absolute_transform: " << absolute_transform << std::endl;

    // Update the overall output bbox with the new image added
    // TODO: Add other corners!
    Vector2 new_bot_right_corner = tf_ptr->forward(image_size);
    output_bbox.grow(new_bot_right_corner);
    
    //std::cout << "image_size: " << image_size << std::endl;
    //std::cout << "new_bot_right_corner: " << new_bot_right_corner << std::endl;
    //std::cout << "Overall bbox: " << output_bbox << std::endl;

    // Update this image's bbox in output image
    BBox2f this_bbox = compute_transformed_bbox_fast(
                                BBox2i(0,0,image_size[0],image_size[1]),
                                *tf_ptr);
    this_bbox.expand(1);
    this_bbox.crop(output_bbox); // TODO: Should not be needed!
    bboxes[i] = this_bbox;
    /*
    // DEBUG Write out the entire transformed image
    write_image( "input.tif",transform(DiskImageView<float>(opt.image_files[i]), 
                                      AffineTransform(m, t),
                                      output_bbox.size()[0], output_bbox.size()[1]) );
    */
    //std::cout << "This bbox: "    << bboxes[i]   << std::endl;

  } // End loop through images
  
  output_image_size = output_bbox.size();
}


/// A class to mosaic and rescale images using bilinear interpolation.
template <class T>
class ImageMosaicView: public ImageViewBase<ImageMosaicView<T> >{
private:
  std::vector<ImageViewRef<T> > const& m_images;
  std::vector<boost::shared_ptr<vw::Transform> > const& m_transforms;
  std::vector<BBox2i>          const& m_bboxes;
  int            m_blend_radius;
  Vector2i const m_output_image_size;
  double         m_output_nodata_value;

public:
  ImageMosaicView(std::vector<ImageViewRef<T> > const& images,
                  std::vector<boost::shared_ptr<vw::Transform> > const& transforms,
                  std::vector<BBox2i>          const& bboxes,
                  int      blend_radius,
                  Vector2i output_image_size, 
                  double   output_nodata_value):
    m_images(images), m_transforms(transforms),
    m_bboxes(bboxes), m_blend_radius(blend_radius),
    m_output_image_size(output_image_size),
    m_output_nodata_value(output_nodata_value){}

  typedef float pixel_type;
  typedef float result_type;
  typedef ProceduralPixelAccessor<ImageMosaicView> pixel_accessor;

  inline int32 cols  () const { return m_output_image_size[0]; }
  inline int32 rows  () const { return m_output_image_size[1]; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline result_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "ImageMosaicView::operator()(...) is not implemented");
    return result_type();
  }

  typedef CropView<ImageView<result_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Initialize the output tile
    ImageView<result_type> tile   (bbox.width(), bbox.height());
    ImageView<float      > weights(bbox.width(), bbox.height());
    fill(tile,    m_output_nodata_value);
    fill(weights, 0);

    // Loop through the intersecting input images and paste them in
    //  to the output image.
    for (size_t i=0; i<m_images.size(); ++i) {

      //std::cout << "i = " << i << std::endl;
      //std::cout << "bbox = " << bbox << std::endl;
      //std::cout << "m_bboxes[i] = " << m_bboxes[i] << std::endl;

      // Get the intersection (if any) of this image with the current bbox.
      if (!m_bboxes[i].intersects(bbox)) {
        //std::cout << "Skipping\n";
        continue;
      }
      BBox2i intersect = m_bboxes[i];
      intersect.crop(bbox);

      typedef ImageView<T> ImageT;
      typedef InterpolationView<ImageT, BilinearInterpolation> InterpT;
      
      // Find the required section of the input image.
      //BBox2f temp_bbox = compute_transformed_bbox_fast(intersect, inv_trans);
      BBox2i temp_bbox = m_transforms[i]->reverse_bbox(intersect);
      temp_bbox.expand(BilinearInterpolation::pixel_buffer);
      BBox2i input_bbox = temp_bbox;
      input_bbox.crop(bounding_box(m_images[i]));
      
      BBox2i tile_bbox = intersect - bbox.min(); // ROI of this input in the output tile

      //std::cout << "intersect = " << intersect << std::endl;
      //std::cout << "intersect.max() = " << intersect.max() << std::endl;
      //std::cout << "input_bbox = " << input_bbox << std::endl;
      //std::cout << "tile_bbox = " << tile_bbox << std::endl;

      // TODO: Clean up
      AffineTransform* temp = dynamic_cast<AffineTransform*>(m_transforms[i].get());

      // --> Requires loading a larger section of the input image,
      //     calling grassfire on it, and then extracting out the section we need.

      /// This sets the extra area we work with to improve the blending weights.
      BBox2i expanded_intersect = intersect;
      expanded_intersect.expand(m_blend_radius);
      
      // Get the cropped piece of the transformed input image that we need
      ImageView<T> trans_input = crop(transform(m_images[i], *temp,
                                                ZeroEdgeExtension(),
                                                BilinearInterpolation()),
                                      expanded_intersect);
      ImageView<double> input_weights;
      //  = grassfire(notnodata(apply_mask(trans_input,0), 0));
      bool fill_holes  = false; // Don't fill holes
      bool min_weights = true;  // This option works better with the applied cutoffs
      centerline_weights(trans_input, input_weights, BBox2i(), fill_holes, min_weights);
      
      double dist = std::min(intersect.height(), intersect.width()) / 2.0;
      double denom = dist + m_blend_radius;
      
      double cutoff = (m_blend_radius/denom);//*(dist/denom);
      //std::cout << "dist = " << dist << std::endl;
      //std::cout << "cutoff = " << cutoff << std::endl;
      
      for (int r=0; r<input_weights.rows(); ++r) {
        for (int c=0; c<input_weights.cols(); ++c) {
          if (input_weights(c,r) > cutoff)
            input_weights(c,r) = cutoff;
        }
      }
      //std::string fix = "_"+num2str(i)+"_"+num2str(bbox.min()[0])+"_"+num2str(bbox.min()[1])+".tif";
      //write_image("input"+fix, apply_mask(trans_input,0));
      //write_image("weights"+fix, input_weights);
      //write_image("weights_crop"+fix, crop(input_weights, BBox2i(m_blend_radius,m_blend_radius, intersect.width(), intersect.height())));

      // Copy that piece to the output tile, applying the mask.
      for (int r=0; r<intersect.height(); ++r) {
        for (int c=0; c<intersect.width(); ++c) {

          double weight = input_weights(c+m_blend_radius,r+m_blend_radius);
          //if (weight > cutoff)
          //  weight = cutoff;
          
          T pixel = trans_input(c+m_blend_radius,r+m_blend_radius);
          if (is_valid(pixel)) {
            float value = remove_mask(pixel);
            int o_c = c+tile_bbox.min()[0];
            int o_r = r+tile_bbox.min()[1];
            if (weights(o_c, o_r) == 0)
              tile(o_c, o_r) = value * weight;
            else
              tile(o_c, o_r) += value * weight;
            weights(o_c, o_r) += weight;
          }
        }
      } // End loop through tile intersection

      //std::cout << "input finished\n";
    } // End loop through input images

    //std::string fix = "_"+num2str(bbox.min()[0])+"_"+num2str(bbox.min()[1])+".tif";
    //write_image("tile"+fix, tile);
    //write_image("tile_weights"+fix, weights);
    
    // Normalize output by the weight.
    for (int c = 0; c < bbox.width(); c++){
      for (int r = 0; r < bbox.height(); r++){
        if ( weights(c, r) > 0 )
          tile(c, r) /= weights(c, r);
      } // End row loop
    } // End col loop
    
    //write_image("tile_norm"+fix, tile);
    
    //std::cout << "Tile finished\n";
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  } // End function prerasterize

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
}; // End class ImageMosaicView


// Write the image out, converting to the specified data type.
void write_selected_image_type(ImageViewRef<float> const& out_img,
                               double output_nodata_value,
                               Options & opt) {
  
  // Set up our output image object
  TerminalProgressCallback tpc("asp", "\t    Mosaic:");

  // Since we blend large portions, it is convenient to use large
  // tiles upon writing.  Yet, those are hard to process later, for
  // example, by stereo_gui. So, later we will rewrite the tiles
  // with smaller blocks.
  int min_tile_size = 2*opt.blend_radius;
  min_tile_size = std::max(opt.raster_tile_size[0], min_tile_size);
  min_tile_size = std::max(opt.raster_tile_size[1], min_tile_size);
  fix_tile_multiple(min_tile_size);
  
  vw_out() << "Using temporary block size: " << min_tile_size << std::endl;
  
  bool has_georef = false;
  bool has_nodata = true;
  vw::cartography::GeoReference georef;
  
  boost::algorithm::to_lower(opt.output_type);
  
  // Write to disk using the specified output data type.
  if (opt.output_type == "float32") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image, out_img,
                               has_georef, georef, has_nodata,
                               output_nodata_value, opt, tpc);
  else if (opt.output_type == "byte") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image,
                               per_pixel_filter(out_img,
                                                RoundAndClamp<uint8, float>()),
                               has_georef, georef, has_nodata, 
                               vw::round_and_clamp<uint8>(output_nodata_value),
                               opt, tpc);
  else if (opt.output_type == "uint16") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image,
                               per_pixel_filter(out_img,
                                                RoundAndClamp<uint16, float>()),
                               has_georef, georef, has_nodata, 
                               vw::round_and_clamp<uint16>(output_nodata_value),
                               opt, tpc);
  else if (opt.output_type == "int16") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image,
                               per_pixel_filter(out_img,
                                                RoundAndClamp<int16, float>()),
                               has_georef, georef, has_nodata, 
                               vw::round_and_clamp<int16>(output_nodata_value),
                               opt, tpc);
  
  else if (opt.output_type == "uint32") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image,
                               per_pixel_filter(out_img,
                                                RoundAndClamp<uint32, float>()),
                               has_georef, georef, has_nodata, 
                               vw::round_and_clamp<uint32>(output_nodata_value),
                               opt, tpc);
  else if (opt.output_type == "int32") 
    asp::saveWithTempBigBlocks(min_tile_size, opt.output_image,
                               per_pixel_filter(out_img,
                                                RoundAndClamp<int32, float>()),
                               has_georef, georef, has_nodata, 
                               vw::round_and_clamp<int32>(output_nodata_value),
                               opt, tpc);
  else
    vw_throw( NoImplErr() << "Unsupported output type: " << opt.output_type << ".\n" );
  
} // End function write_selected_image_type

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  // Add the reverse option
  general_options.add( vw::GdalWriteOptionsDescription(opt) );
  general_options.add_options()
    ("orientation", po::value(&opt.orientation)->default_value("horizontal"),
     "Choose a supported image layout from [horizontal].")
    ("reverse", po::bool_switch(&opt.reverse)->default_value(false),
     "Mosaic the images in reverse order.")
    ("rotate", po::bool_switch(&opt.rotate)->default_value(false),
     "After mosaicking, rotate the image by 180 degrees around its center.")
    ("rotate-90", po::bool_switch(&opt.rotate90)->default_value(false),
     "After mosaicking, rotate the image by 90 degrees clockwise around its center.")
    ("rotate-90-ccw", po::bool_switch(&opt.rotate90ccw)->default_value(false),
     "After mosaicking, rotate the image by 90 degrees counter-clockwise around its center.")
    ("use-affine-transform", po::bool_switch(&opt.use_affine_transform)->default_value(false),
     "Solve for full affine transforms between segments instead of a simpler rotate+translate transform.")
    ("overlap-width", po::value(&opt.overlap_width)->default_value(2000),
          "Select the size of the overlap region to use.")
    ("blend-radius", po::value(&opt.blend_radius)->default_value(0),
          "Size to perform blending over.  Default is the overlap width.")
    ("output-image,o", po::value(&opt.output_image)->default_value(""),
     "Specify the output image.")
    ("ip-per-tile",          po::value(&opt.ip_per_tile)->default_value(0),
     "How many interest points to detect in each 1024^2 image tile (default: automatic determination).")
    ("ot",  po::value(&opt.output_type)->default_value("Float32"),
     "Output data type. Supported types: Byte, UInt16, Int16, UInt32, Int32, Float32. If the output type is a kind of integer, values are rounded and then clamped to the limits of that type.")
    ("band", po::value(&opt.band)->default_value(1), 
     "Which band (channel) to use (for multi-spectral images). The band count starts from 1.")
    ("input-nodata-value", po::value(&opt.input_nodata_value),
     "Nodata value to use on input; input pixel values less than or equal to this are considered invalid.")
    ("output-nodata-value", po::value(&opt.output_nodata_value),
     "Nodata value to use on output.")
    ("output-prefix", po::value(&opt.out_prefix)->default_value(""),
     "If specified, save here the interest point matches used in mosaicking.")
    ("num-ransac-iterations", po::value(&opt.num_ransac_iterations)->default_value(1000),
     "How many iterations to perform in RANSAC when finding interest point matches.")
    ("inlier-threshold", po::value(&opt.inlier_threshold)->default_value(10.0),
     "The inlier threshold (in pixels) to separate inliers from outliers when computing "
     "interest point matches. A smaller threshold will result in fewer inliers.");
 
  po::options_description positional("");
  positional.add_options()
    ("input-files", po::value(&opt.image_files));

  po::positional_options_description positional_desc;
  positional_desc.add("input-files", -1);

  std::string usage("image_mosaic <images> [options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  opt.has_input_nodata_value  = vm.count("input-nodata-value" );
  opt.has_output_nodata_value = vm.count("output-nodata-value");

  if ( opt.image_files.empty() )
    vw_throw( ArgumentErr() << "No images to mosaic.\n" << usage << general_options );

  if ( opt.output_image.empty() )
    vw_throw( ArgumentErr() << "Missing output image name.\n" << usage << general_options );

  if ( opt.blend_radius == 0 ) {
    opt.blend_radius = opt.overlap_width;
    vw_out() << "Using blend radius: " << opt.blend_radius << std::endl;
  }

  int check = 0; // Count the number of rotate options specified.
  if (opt.rotate     ) ++check;
  if (opt.rotate90   ) ++check;
  if (opt.rotate90ccw) ++check;
  if (check > 1)
    vw_throw( ArgumentErr() << "Cannot specify more than one rotate option.\n"
                            << usage << general_options );

  // Reverse the order of files
  if (opt.reverse) {
    size_t len = opt.image_files.size();
    for (size_t it = 0; it < len; it++) {
      size_t it2 = len-1-it;
      if (it < it2)
        std::swap(opt.image_files[it], opt.image_files[it2]);
    }
  }
  
  // Create the output directory
  vw::create_out_dir(opt.output_image);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.output_image);
}

int main( int argc, char *argv[] ) {

  Options opt;
  
  try {

    // Find command line options
    handle_arguments( argc, argv, opt );

    // Compute the transforms between all of the images on disk
    std::vector<boost::shared_ptr<vw::Transform> > transforms;
    std::vector<BBox2i>          bboxes;
    Vector2i                     output_image_size;
    compute_all_image_positions(opt, transforms, bboxes, output_image_size);

    // Get handles to all of the input images.
    size_t num_images = opt.image_files.size();
    std::vector<ImageViewRef<PixelMask<float>>> images(num_images);
    double nodata = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < num_images; i++) {
      // Apply a nodata mask here
      ImageViewRef<float> temp;
      get_input_image(opt.image_files[i], opt, temp, nodata);
      images[i] = create_mask(temp, nodata);
    }

    // If nodata was not provided, take one from the input images.
    double output_nodata_value = nodata;
    if (opt.has_output_nodata_value)
      output_nodata_value = opt.output_nodata_value;

    // Set up our output image object
    vw_out() << "Writing: " << opt.output_image << std::endl;
    TerminalProgressCallback tpc("asp", "\t    Mosaic:");
    ImageViewRef<float> out_img = 
        ImageMosaicView<PixelMask<float>>(images, transforms, bboxes,
                                          opt.blend_radius, output_image_size,
                                          opt.output_nodata_value);

    if (opt.rotate)
      //write_selected_image_type(ImageRotateView<float>(out_img), opt.output_nodata_value, opt);
      write_selected_image_type(vw::rotate_180(out_img), opt.output_nodata_value, opt);
    else
      if (opt.rotate90)
        write_selected_image_type(vw::rotate_90_cw(out_img), opt.output_nodata_value, opt);
      else
        if (opt.rotate90ccw)
          write_selected_image_type(vw::rotate_90_ccw(out_img), opt.output_nodata_value, opt);
        else
          write_selected_image_type(out_img, opt.output_nodata_value, opt);

  } ASP_STANDARD_CATCHES;
  return 0;
}

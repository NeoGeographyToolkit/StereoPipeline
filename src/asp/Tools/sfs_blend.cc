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

/// \file sfs_blend.cc
///

// A tool to take an SfS-produced DEM, and replace in the areas in
// permanent shadow this DEM with the original LOLA DEM, with a
// transition at the boundary.

// It uses the Euclidean distance to the boundary, which is better
// than the Manhattan distance employed by grassfire.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <limits>
#include <algorithm>

#include <vw/FileIO/DiskImageManager.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/Algorithms2.h>
#include <vw/Image/Filter.h>
#include <vw/Cartography/GeoTransform.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/math/special_functions/erf.hpp>
#include <boost/program_options.hpp>

#include <boost/filesystem/convenience.hpp>

using namespace std;
using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

GeoReference read_georef(std::string const& file) {
  // Read a georef, and check for success
  GeoReference geo;
  bool is_good = read_georeference(geo, file);
  if (!is_good)
    vw_throw(ArgumentErr() << "No georeference found in " << file << ".\n");
    
  // This is a bug fix. The georef pixel size in y must be negative
  // for the image to be oriented correctly. 
  if (geo.transform()(1, 1) > 0)
    vw_throw(ArgumentErr() << "The georeference in " << file 
              << " has a positive pixel size in y. "
              << "This is unexpected. Normally it is negative since the (0, 0) "
              << "pixel is in the upper-left. Check your DEM pixel size with "
              << "gdalinfo. Cannot continue.\n");

  return geo;
}

struct Options: vw::GdalWriteOptions {
  string sfs_dem, lola_dem, max_lit_image_mosaic, output_dem, output_weight;
  double image_threshold, weight_blur_sigma, lit_blend_length,
    shadow_blend_length, min_blend_size;
  Options(): image_threshold(0.0), weight_blur_sigma(0.0), lit_blend_length(0.0),
             shadow_blend_length(0.0), min_blend_size(0.0) {}
};

// The workhorse of this code, do the blending
class SfsBlendView: public ImageViewBase<SfsBlendView>{
  
  ImageViewRef<float> m_sfs_dem, m_lola_dem, m_image_mosaic;
  float m_sfs_nodata, m_lola_nodata, m_weight_nodata;
  int m_extra;
  bool m_save_weight;
  Options const& m_opt;
  
  typedef float PixelT;
  
public:
  SfsBlendView(ImageViewRef<float> sfs_dem, ImageViewRef<float> lola_dem,
               ImageViewRef<float> image_mosaic,
               float sfs_nodata, float lola_nodata, float weight_nodata, int extra,
               bool save_weight, Options const& opt):
    m_sfs_dem(sfs_dem), m_lola_dem(lola_dem), m_image_mosaic(image_mosaic),
    m_sfs_nodata(sfs_nodata), m_lola_nodata(lola_nodata),
    m_weight_nodata(weight_nodata), m_extra(extra),
    m_save_weight(save_weight), m_opt(opt) {}

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<SfsBlendView> pixel_accessor;
  
  inline int32 cols() const { return m_sfs_dem.cols(); }
  inline int32 rows() const { return m_sfs_dem.rows(); }
  inline int32 planes() const { return 1; }
  
  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }
  
  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "SfsBlendView::operator()(...) is not implemented");
    return pixel_type();
  }
  
  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    BBox2i biased_box = bbox;
    biased_box.expand(m_extra);
    biased_box.crop(bounding_box(m_sfs_dem));

    // Make crops in memory (from references)
    ImageView<pixel_type> sfs_dem_crop = crop(m_sfs_dem, biased_box);
    ImageView<pixel_type> lola_dem_crop = crop(m_lola_dem, biased_box);
    ImageView<pixel_type> image_mosaic_crop = crop(m_image_mosaic, biased_box);

    // The mask of lit pixels
    ImageView<PixelMask<pixel_type>> mask
      = create_mask_less_or_equal(image_mosaic_crop, m_opt.image_threshold);
    
    // The mask of unlit pixels
    ImageView< PixelMask<pixel_type> > inv_mask = vw::copy(mask);
    for (int col = 0; col < inv_mask.cols(); col++) {
      for (int row = 0; row < inv_mask.rows(); row++) {
        if (is_valid(mask(col, row))) {
          inv_mask(col, row) = 0;
          inv_mask(col, row).invalidate();
        } else {
          inv_mask(col, row) = 1;
          inv_mask(col, row).validate();
        }
      }
    }
    
    // The grassfire weight positive in the lit region, with zero at the light-shadow
    // boundary
    bool no_zero_at_border = true; // don't decrease the weights to zero at image border

    // Fill small holes that we don't blend in those, then compute the distance
    // to the remaining holes
    ImageView<pixel_type> lit_grass_dist
      = vw::grassfire(vw::copy(vw::fill_holes_grass(mask, m_opt.min_blend_size)),
                      no_zero_at_border);

    // The grassfire weights are positive in the shadow region, with
    // zero at the light-shadow boundary
    ImageView<pixel_type> shadow_grass_dist = vw::grassfire(inv_mask, no_zero_at_border);

    // Find the clamped signed distance to the boundary. Note that our
    // boundary is in fact two pixel wide at the light-shadow
    // interface, given how lit_grass_dist and shadow_grass_dist are
    // defined as the negation of each other. The boundary is the set
    // of pixels where both of these are <= 1.
    ImageView<float> dist_to_bd;
    dist_to_bd.set_size(sfs_dem_crop.cols(), sfs_dem_crop.rows());
    for (int col = 0; col < sfs_dem_crop.cols(); col++) {
      
      for (int row = 0; row < sfs_dem_crop.rows(); row++) {

        if (lit_grass_dist(col, row) > 1.5*m_opt.lit_blend_length) {
          // Too far in the lit region
          dist_to_bd(col, row) = m_opt.lit_blend_length; // clamp at the blending length
          continue;
        }
        
        if (shadow_grass_dist(col, row) > 1.5*m_opt.shadow_blend_length) {
          // Too far in the shadow region
          dist_to_bd(col, row) = -m_opt.shadow_blend_length;
          continue;
        }
        
        // Find the shortest Euclidean distance to the no-data region.
        double max_dist = std::max(m_opt.lit_blend_length, m_opt.shadow_blend_length);
        double signed_dist = 0.0;
        if (lit_grass_dist(col, row) > 0) {
          signed_dist = m_opt.lit_blend_length;
        } else if (shadow_grass_dist(col, row) > 0) {
          signed_dist = -m_opt.shadow_blend_length;
        }
        
        for (int col2 = std::max(0.0, col - max_dist);
             col2 <= std::min(sfs_dem_crop.cols() - 1.0, col + max_dist);
             col2++) {

          // Estimate the range of rows for the circle with given radius
          // at given col value.
          double ht_val = ceil(sqrt(double(max_dist * max_dist) -
                                    double((col - col2) * (col - col2))));
          
          for (int row2 = std::max(0.0, row - ht_val);
               row2 <= std::min(sfs_dem_crop.rows() - 1.0, row + ht_val);
               row2++) {

            if (lit_grass_dist(col2, row2) > 1 || shadow_grass_dist(col2, row2) > 1) 
              continue; // not at the boundary

            // See if the current point is closer than anything so far
            double curr_dist = sqrt( double(col - col2) * (col - col2) +
                                     double(row - row2) * (row - row2) );
            if (lit_grass_dist(col, row) > 0) {
              if (curr_dist < signed_dist) 
                signed_dist = curr_dist;
            } else if (shadow_grass_dist(col, row) > 0) {
              if (curr_dist < -signed_dist) 
                signed_dist = -curr_dist;
            }
            
          }
        }

        // The closest we've got
        dist_to_bd(col, row) = signed_dist;
      }
    }

    // Apply the blur
    if (m_opt.weight_blur_sigma > 0)
      dist_to_bd = vw::gaussian_filter(dist_to_bd, m_opt.weight_blur_sigma);

    // Do the blending
    ImageView<float> blended_dem;
    blended_dem.set_size(sfs_dem_crop.cols(), sfs_dem_crop.rows());
    for (int col = 0; col < sfs_dem_crop.cols(); col++) {
      for (int row = 0; row < sfs_dem_crop.rows(); row++) {

        if (!m_save_weight)
          blended_dem(col, row) = m_sfs_nodata;
        else
          blended_dem(col, row) = m_weight_nodata;

        // The signed distance to the boundary is modified so that the smallest
        // value is 0, the largest is 1, and in a band close to the boundary it
        // transitions from 0 to 1.
        float weight = (dist_to_bd(col, row) + m_opt.shadow_blend_length) /
          (m_opt.shadow_blend_length + m_opt.lit_blend_length);

        // These are not strictly necessary but enforce them
        if (weight > 1.0)
          weight = 1.0;
        if (weight < 1e-7) // take into account the float error 
          weight = 0.0;
          
        // Handle no-data values. These are not meant to happen, but do this just in case.
        if (sfs_dem_crop(col, row) == m_sfs_nodata)
          weight = 0.0; // Use LOLA
        
        if (lola_dem_crop(col, row) == m_lola_nodata) 
          continue;

        if (!m_save_weight) 
          blended_dem(col, row)
            = weight * sfs_dem_crop(col, row) + (1.0 - weight) * lola_dem_crop(col, row);
        else
          blended_dem(col, row) = weight;
      }
    }
    
    return prerasterize_type(blended_dem, -biased_box.min().x(), -biased_box.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("Options");
  general_options.add_options()
    ("sfs-dem", po::value<string>(&opt.sfs_dem),
     "The SfS DEM to process.")
    ("lola-dem", po::value<string>(&opt.lola_dem),
     "The LOLA DEM to use to fill in the regions in permanent shadow.")
    ("max-lit-image-mosaic", po::value<string>(&opt.max_lit_image_mosaic),
     "The maximally lit image mosaic to use to determine the permanently shadowed regions.")
    ("image-threshold",  po::value<double>(&opt.image_threshold)->default_value(0.0),
     "The value separating permanently shadowed pixels from lit pixels in the maximally lit image mosaic.")
    ("lit-blend-length", po::value<double>(&opt.lit_blend_length)->default_value(0.0),
     "The length, in pixels, over which to blend the SfS and LOLA DEMs at the boundary of the permanently shadowed region towards the lit region.")
    ("shadow-blend-length", po::value<double>(&opt.shadow_blend_length)->default_value(0.0),
     "The length, in pixels, over which to blend the SfS and LOLA DEMs at the boundary of the permanently shadowed region towards the shadowed region.")
    ("weight-blur-sigma", po::value<double>(&opt.weight_blur_sigma)->default_value(0.0),
     "The standard deviation of the Gaussian used to blur the weight that performs the transition from the SfS to the LOLA DEM. A higher value results in a smoother transition (this does not smooth the DEMs). The extent of the blur is about 7 times this deviation, though it tapers fast to 0 before that. Set to 0 to not use this operation.")
    ("min-blend-size", po::value<double>(&opt.min_blend_size)->default_value(0.0),
     "Do not apply blending in shadowed areas of dimensions less than this, hence keeping there the SfS DEM.")
    ("output-dem", po::value(&opt.output_dem), "The blended output DEM to save.")
    ("output-weight", po::value(&opt.output_weight), "The weight showing the proportion of the SfS DEM in the blend with the LOLA DEM (1 is for purely SfS and 0 is for purely LOLA).");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);
  
  // Error checking
  if (opt.sfs_dem == "" || opt.lola_dem == "" ||
      opt.max_lit_image_mosaic == "" || opt.output_dem == "" || opt.output_weight == "")
    vw_throw(ArgumentErr() << "Not all input or output files were specified.\n"
                           << usage << general_options );
  if (opt.lit_blend_length <= 0)
    vw_throw(ArgumentErr() << "The lit blending length must be positive.\n"
                           << usage << general_options );
  if (opt.shadow_blend_length <= 0)
    vw_throw(ArgumentErr() << "The shadow blending length must be positive.\n"
                           << usage << general_options );
  if (opt.image_threshold <= 0)
    vw_throw(ArgumentErr() << "The image threshold must be positive.\n"
                           << usage << general_options );

  // Create the output directory
  vw::create_out_dir(opt.output_dem);
} // End function handle_arguments

#if 0
// Experimental code for deepening a crater in permanent shadow.  Fit
// in that area a cone-like shape with given slope and round its top
// using the arctan function. Very rough first attempt, but gives
// something plausible.

int main(int argc, char * argv[]){

  std::string id = argv[1];

  std::string img_file = argv[2]; // "clip0_maxlit.tif";
  DiskImageView<float> img(img_file);
  std::cout << "image is " << img_file << std::endl;
  std::cout << "image size " << img.cols() << ' ' << img.rows() << std::endl;
  
  std::string dem_file = argv[3]; // "sfs_clip0_exp0.25x/run-DEM-final.tif";
  ImageView<float> dem = copy(DiskImageView<float>(dem_file));
  std::cout << "dem is " << dem_file << std::endl;
  std::cout << "image size " << dem.cols() << ' ' << dem.rows() << std::endl;

  double thresh = atof(argv[4]);
  std::cout << "thresh is " << thresh << std::endl;
  int num_smooth = atof(argv[5]);
  std::cout << "num smooth passes " << num_smooth << std::endl;
  double slope = atof(argv[6]);
  std::cout << "slope is " << slope << std::endl;

  // Find the boundary
  std::vector<Vector2> pts;
  for (int col = 1; col < dem.cols() - 1; col++) {
    for (int row = 1; row < dem.rows() - 1; row++) {

      if (img(col, row) <= thresh) 
        continue;
      
      if (img(col-1, row) <= thresh ||
          img(col+1, row) <= thresh ||
          img(col, row-1) <= thresh ||
          img(col, row+1) <= thresh) {
        pts.push_back(Vector2(col, row));
      }
    }
  }

  // Find the distance to boundary using a brute force method
  // Points where above thresh are 0.
  // TODO(oalexan1): This can be made so much more efficient!
  float max_wt = 0.0;
  ImageView<float> wts(dem.cols(), dem.rows());
  for (int col = 0; col < wts.cols(); col++) {
    for (int row = 0; row < wts.rows(); row++) {
      wts(col, row) = 1e+6;

      Vector2 p1(col, row);
      for (size_t it = 0; it < pts.size(); it++) {
        Vector2 p2(pts[it][0], pts[it][1]);
        float dist = norm_2(p1 - p2);
        wts(col, row) = std::min(wts(col, row), dist);
      }

      // Adjust the steepness of the weight
      wts(col, row) *= slope; 

      // Flip the sign inside the hole, and make the weights negative outside
      if (img(col, row) <= thresh)
        wts(col, row) *= -1.0;
      else
        wts(col, row) = 0.0;
      
      max_wt = std::max(std::abs(wts(col, row)), max_wt);
    }
  }
  std::cout << "Max wt is " << max_wt << std::endl;

  // Make the tip of the weight rounder between max_wt/2 and max_wt.
  // TODO(oalexan1): Scaling by 2 changes the slope.
  // Maybe should use 0.5*atan(2*x)
  
  for (int col = 0; col < wts.cols(); col++) {
    for (int row = 0; row < wts.rows(); row++) {
      double val = -wts(col, row)/max_wt; // between 0 and 1
      val *= 2.0; // between 0 and 2
      val = atan(val); // between 0 and pi/2. Inflection point is at r/2.
      val = (max_wt/2.0) * (2.0/M_PI) * val;  // rounded at the top, max val is max_wt/2.0.
      wts(col, row) = -val; // make it negative again
    }
  }
  
  // Make the weights a little smooth not too far from boundary
  ImageView<float> curr_wts = copy(wts);
  for (int pass  =  0; pass < num_smooth; pass++) {
    for (int col = 1; col < wts.cols() - 1; col++) {
      for (int row = 1; row < wts.rows() - 1; row++) {
        
        bool is_far = true;
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            if (wts(col + i, row + j) < 0.0)
              is_far = false;
          }
        }
        
        if (is_far)
          continue;
        
        curr_wts(col, row) = 0.0;
        double sum = 0.0;
        for (int i = -1; i <= 1; i++) {
          for (int j = -1; j <= 1; j++) {
            double p = i * i + j * j;
            double wt = 1.0 / (1.0 + p*p);
            curr_wts(col, row) += wts(col + i, row + j) *wt;
            sum += wt;
          }
        }
        curr_wts(col, row) /= sum;
        
      }
    }

    wts = copy(curr_wts);
  }

  // Add the weights to the DEM, which pushes them down
  for (int col = 0; col < wts.cols(); col++) {
    for (int row = 0; row < wts.rows(); row++) {
      dem(col, row) += wts(col, row);
    }
  }
  
  float nodata = -1.0;
  bool has_nodata = false;
  vw::cartography::GeoReference georef;
  bool has_georef = vw::cartography::read_georeference(georef, dem_file);
  vw::GdalWriteOptions opt;

  std::string wts_file = "weights" + id + ".tif";
  vw_out() << "Writing: " << wts_file << std::endl;
  block_write_gdal_image(wts_file, wts, has_georef, georef, has_nodata, nodata, opt,
                         TerminalProgressCallback("wts", ": "));
  
  std::string fixed_dem_file = "fixed_dem" + id + ".tif";
  TerminalProgressCallback tpc("fixed_dem", ": ");
  vw_out() << "Writing: " << fixed_dem_file << std::endl;
  block_write_gdal_image(fixed_dem_file, dem, has_georef, georef, has_nodata, nodata, opt,
                         TerminalProgressCallback("fixed_dem", ": "));
  
  
  return 0;
}
#endif

int main(int argc, char *argv[]) {

  Options opt;
  
  try{

    handle_arguments(argc, argv, opt);

    vw_out() << "Reading SfS DEM: " << opt.sfs_dem << std::endl;
    DiskImageView<float> sfs_dem(opt.sfs_dem);

    
    vw_out() << "Reading LOLA DEM: " << opt.lola_dem << std::endl;
    DiskImageView<float> lola_dem(opt.lola_dem);
    
    vw_out() << "Reading maximally-lit image mosaic: " << opt.max_lit_image_mosaic << std::endl;
    DiskImageView<float> image_mosaic(opt.max_lit_image_mosaic);

    if (sfs_dem.cols() != lola_dem.cols() || sfs_dem.rows() != lola_dem.rows())
      vw_throw(ArgumentErr() << "The SfS DEM and LOLA DEM must have the same dimensions.");
    
    if (sfs_dem.cols() != image_mosaic.cols() || sfs_dem.rows() != image_mosaic.rows())
      vw_throw(ArgumentErr() << "The SfS DEM and image mosaic must have the same dimensions.");

    GeoReference sfs_georef   = read_georef(opt.sfs_dem);
    GeoReference lola_georef  = read_georef(opt.lola_dem);
    GeoReference image_georef = read_georef(opt.max_lit_image_mosaic);
    if (sfs_georef.proj4_str() != lola_georef.proj4_str() ||
        sfs_georef.proj4_str() != image_georef.proj4_str())
      vw_throw(ArgumentErr() << "The SfS DEM, LOLA DEM, and image mosaic "
               << "must have the same PROJ4 string.");

    // All these must be on the same grid, or else the blending will be wrong.
    // Allow some tolerance here as sometimes products created with different
    // tools can differ a bit. 1e-10 degrees is about 0.01 mm on Earth's surface.
    vw::Vector2 sfs_corner = sfs_georef.pixel_to_point(Vector2(0, 0));
    vw::Vector2 lola_corner = lola_georef.pixel_to_point(Vector2(0, 0));
    vw::Vector2 image_corner = image_georef.pixel_to_point(Vector2(0, 0));
    if (norm_2(sfs_corner - lola_corner) > 1e-10 || norm_2(sfs_corner - image_corner) > 1e-10) 
      vw_throw(ArgumentErr() << "The SfS DEM, LOLA DEM, and image mosaic "
               << "must be on the same grid.");

    float sfs_nodata = -1.0, lola_nodata = -1.0, image_nodata = -1.0;
    DiskImageResourceGDAL sfs_rsrc(opt.sfs_dem);
    if (sfs_rsrc.has_nodata_read())
      sfs_nodata = sfs_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The SfS DEM does not have a no-data value.");
    DiskImageResourceGDAL lola_rsrc(opt.lola_dem);
    if (lola_rsrc.has_nodata_read())
      lola_nodata = lola_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The LOLA DEM does not have a no-data value.");
    DiskImageResourceGDAL image_rsrc(opt.max_lit_image_mosaic);
    if (image_rsrc.has_nodata_read())
      image_nodata = image_rsrc.nodata_read();
    else
      vw_throw(ArgumentErr() << "The maximally-lit mosaic does not have a no-data value.");
    
    // When processing the DEM tile by tile, need to see further in
    // each tile because of blending and blurring
    int extra = 2*opt.lit_blend_length + 2*opt.shadow_blend_length + opt.min_blend_size;
    if (opt.weight_blur_sigma > 0)
      extra += vw::compute_kernel_size(opt.weight_blur_sigma);

    // Write bigger tiles to make the processing with the extra margin
    // more efficient.
    int block_size = 256 + 2 * extra;
    block_size = 16*ceil(block_size/16.0); // internal constraint

    vw_out() << "Writing: " << opt.output_dem << std::endl;
    bool has_georef = true, has_nodata = true;
    TerminalProgressCallback tpc("asp", ": ");
    float weight_nodata = -1.0;
    bool save_weight = false;
    asp::save_with_temp_big_blocks(block_size,
                                   opt.output_dem,
                                   SfsBlendView(sfs_dem, lola_dem, image_mosaic,
                                                sfs_nodata, lola_nodata, weight_nodata,
                                                extra, save_weight, opt),
                                   has_georef, sfs_georef,
                                   has_nodata, sfs_nodata, opt, tpc);

    // Write the weight. Have to rerun the same logic due to ASP's limitations,
    // it cannot write two large files at the same time.
    vw_out() << "Writing the blending weight: "
             << opt.output_weight << std::endl;
    save_weight = true;
    asp::save_with_temp_big_blocks(block_size,
                                   opt.output_weight,
                                   SfsBlendView(sfs_dem, lola_dem, image_mosaic,
                                                sfs_nodata, lola_nodata, weight_nodata,
                                                extra, save_weight, opt),
                                   has_georef, sfs_georef,
                                   has_nodata, weight_nodata, opt, tpc);
    
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

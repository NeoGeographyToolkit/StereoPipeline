// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// \file SfsImageProc.h
/// Image processing routines for SfS

#ifndef __SFS_IMAGE_PROC_H__
#define __SFS_IMAGE_PROC_H__

#include <vw/Image/ImageViewRef.h>
#include <vw/Image/PerPixelViews.h>

#include <string>
#include <vector>
#include <set>

namespace vw {
  class GdalWriteOptions;
  namespace cartography {
    class GeoReference;
  }
}

namespace asp {

class SfsOptions;

typedef vw::ImageViewRef<vw::PixelMask<float>> MaskedImgRefT;
typedef vw::ImageView<vw::PixelMask<double>> MaskedDblImgT;
typedef vw::ImageView<double> DblImgT;

// Calculate exposure and haze, and decide if an image should be skipped.
void calcExposureHazeSkipImages(// Inputs
                                asp::MaskedDblImgT const& intensity,
                                asp::MaskedDblImgT const& reflectance,
                                double mean_albedo,
                                int image_iter,
                                std::vector<std::string> const& input_images,
                                // In-out
                                std::vector<double>      & local_exposures_vec,
                                std::vector<double>      & local_haze_vec,
                                std::vector<std::string> & used_images,
                                std::set<int>            & skip_images,
                                std::vector<std::string> & skipped_images);

// Compute mean and standard deviation of two images. Do it where both are valid.
void calcJointStats(MaskedDblImgT const& I1,
                    MaskedDblImgT const& I2,
                    double & mean1, double & std1,
                    double & mean2, double & std2);

void maxImage(int cols, int rows,
              std::set<int> const& skip_images,
              std::vector<vw::ImageView<double>> const& images,
              vw::ImageView<double> & max_image);

// Find the per-pixel maximum of a set of masked images
void maxImage(int cols, int rows,
              std::set<int> const& skip_images,
              std::vector<MaskedDblImgT> const& meas_intensities,
              vw::ImageView<double> & max_intensity);

// See the .cc file for the documentation.
void adjustBorderlineDataWeights(int cols, int rows,
                                 int blending_dist, double blending_power,
                                 vw::GdalWriteOptions const& opt,
                                 vw::cartography::GeoReference const& geo,
                                 std::set<int> const& skip_images,
                                 std::string const& out_prefix, // for debug data
                                 std::vector<std::string> const& input_images,
                                 std::vector<std::string> const& input_cameras,
                                 std::vector<vw::ImageView<double>> & ground_weights);

// This will adjust the weights to account for borderline pixels and low-light conditions.
// The blend weights and masked images are modified in place.
void handleBorderlineAndLowLight(SfsOptions & opt,
                                 int num_images,
                                 vw::ImageView<double> const& dem,
                                 vw::cartography::GeoReference const& geo,
                                 std::vector<vw::BBox2i> const& crop_boxes,
                                 std::vector<MaskedDblImgT> const& meas_intensities,
                                 std::vector<MaskedDblImgT> const& comp_intensities,
                                 // Outputs
                                 float & img_nodata_val,
                                 std::vector<MaskedImgRefT> & masked_images,
                                 std::vector<vw::ImageView<double>> & blend_weights,
                                 bool & blend_weight_is_ground_weight,
                                 std::vector<vw::ImageView<double>> & ground_weights);

// See the .cc file for the documentation.
vw::ImageView<double> blendingWeights(MaskedImgRefT const& img,
                                      double blending_dist,
                                      double blending_power,
                                      int min_blend_size);

// Find the points on a given DEM that are shadowed by other points of
// the DEM.  Start marching from the point on the DEM on a ray towards
// the sun in small increments, until hitting the maximum DEM height.
bool isInShadow(int col, int row, vw::Vector3 const& sunPos,
                vw::ImageView<double> const& dem, double max_dem_height,
                double gridx, double gridy,
                vw::cartography::GeoReference const& geo);

void areInShadow(vw::Vector3 const& sunPos, vw::ImageView<double> const& dem,
                 double gridx, double gridy,
                 vw::cartography::GeoReference const& geo,
                 vw::ImageView<float> & shadow);

// Prototype code to identify permanently shadowed areas
// and deepen the craters there. Needs to be integrated
// and tested with various shapes of the deepened crater.
void deepenCraters(std::string const& dem_file,
                   std::vector<std::string> const& image_files,
                   double sigma,
                   std::string const& max_img_file,
                   std::string const& grass_file,
                   std::string const& out_dem_file);

// Sample large DEMs. Keep about num_samples row and column samples.
void calcSampleRates(vw::ImageViewRef<double> const& dem, int num_samples,
                     int & sample_col_rate, int & sample_row_rate);

// TODO(oalexan1): The albedo must have its own no-data value.
// Must check the albedo has everywhere valid values.
double meanAlbedo(vw::ImageView<double> const& dem,
                  vw::ImageView<double> const& albedo,
                  double dem_nodata_val);

double maxDemHeight(vw::ImageView<double> const& dem);

// Find the clamped signed distance to the boundary. Here it is assumed that
// there is a region in which lit_grass_dist is positive, while in its
// complement the shadow_grass_dist is positive. The boundary is where both are
// no more than 1 in value. Likely this logic could be made more generic.
vw::ImageView<double> calcClampedBdDist(vw::ImageView<float> const& lit_grass_dist,
                                        vw::ImageView<float> const& shadow_grass_dist,
                                        double lit_blend_length,
                                        double shadow_blend_length);

// Calc the weight for option --curvature-in-shadow-weight
void calcCurvatureInShadowWeight(asp::SfsOptions const& opt,
                                 vw::ImageView<int> const& lit_image_mask,
                                 vw::cartography::GeoReference const& geo,
                                 vw::ImageView<double> & curvature_in_shadow_weight);

// Compute a full-resolution image by specific interpolation into a low-resolution
// one. The full-res image may not fit in memory, so we need to compute it in tiles.
// See computeReflectanceAndIntensity() for low-res vs full-res relationship.
class SfsInterpView: public vw::ImageViewBase<SfsInterpView> {
  int m_full_res_cols, m_full_res_rows;
  int m_sample_col_rate, m_sample_row_rate;
  vw::ImageView<float> const& m_lowres_img;
  typedef float PixelT;

public:
  SfsInterpView(int full_res_cols, int full_res_rows,
                int sample_col_rate, int sample_row_rate,
                vw::ImageView<float> const& lowres_img);

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<SfsInterpView> pixel_accessor;

  inline vw::int32 cols() const { return m_full_res_cols; }
  inline vw::int32 rows() const { return m_full_res_rows; }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  pixel_type operator()(double/*i*/, double/*j*/, vw::int32/*p*/ = 0) const;

  typedef vw::CropView<vw::ImageView<pixel_type>> prerasterize_type;
  prerasterize_type prerasterize(vw::BBox2i const& bbox) const;

  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

} // end namespace asp

#endif // __SFS_IMAGE_PROC_H__

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

// TODO(oalexan1): Move image logic to SfsImageProc.cc. Add also
// SfS camera approx logic, and SfS cost function logic.
// Remove all logic with multiple DEM clips, it turned out not to work.
// Remove all floating of cameras from the code and the doc, one has to use bundle adjust.
// Then also remove all the logic using unadjusted cameras except the place
// when the sun positions are read.
// Evaluate RPC accuracy.
// Deal with outliers in image intensity.
// The rpc approximation should also approximate the adjusted cameras if those won't float.
// This approximation needs to remember is domain of validity.
// TODO: Ensure the output DEM is float. Check its no-data value.
// TODO: Study more floating model coefficients.
// TODO: Study why using tabulated camera model and multiple resolutions does
// not work as well as it should.
// TODO: When using approx camera, we assume the DEM and image grids are very similar.
// TODO: Remove warning from the approx camera
// TODO: Make it possible to initialize a DEM from scratch.
// TODO: Study more the multi-resolution approach.
// TODO: Must specify in the SfS doc that the Lunar-Lambertian model fails at poles
// TODO: If this code becomes multi-threaded, need to keep in mind
// that camera models are shared and modified, so
// this may cause problems.
// TODO: How to relax conditions at the boundary to improve the accuracy?
// TODO: Study if blurring the input images improves the fit.
// TODO: Add --orthoimage option, and make it clear where the final DEM is.
// Same for albedo. The other info should be printed only in debug mode.
// Implement multi-grid for SfS, it should help with bad initial DEM and bad
// camera alignment.
// TODO: Save final DEM and final ortho images.
// Say that if the camera are bundle adjusted, sfs can further improve
// the camera positions to make the results more self consistent,
// but this works only if the cameras are reasonably accurate to start with.
// TODO: Study the effect of reading the images as double as opposed to float.
// TODO: Study the effect of using bicubic interpolation.
// TODO: Study phaseCoeffC1, etc.
// TODO: Find a good automatic value for the smoothness weight.
// TODO: How to change the smoothness weight if resolution changes?
// How to change the smoothness weight if the number of images changes?
// TODO: Investigate the sign of the normal.
// TODO: Loop over all images when doing sfs.
// TODO: Check that we are within image boundaries when interpolating.
// TODO: Radiometric calibration of images.
// TODO: Add various kind of loss function.
// TODO: Study the normal computation formula.
// TODO: Move some code to Core.
// TODO: Make it work with non-ISIS cameras.
// TODO: Clean up some of the classes, not all members are needed.

// Paper suggested by Randy that does both bundle adjustment and sfs
// at the same time: https://www.sciencedirect.com/science/article/pii/S0924271619302771

/// \file sfs.cc

// Turn off warnings from boost and other packages
#if defined(__GNUC__) || defined(__GNUG__)
#define LOCAL_GCC_VERSION (__GNUC__ * 10000             \
                           + __GNUC_MINOR__ * 100       \
                           + __GNUC_PATCHLEVEL__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic push
#endif
#if LOCAL_GCC_VERSION >= 40202
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
#endif

#include <vw/Image/MaskViews.h>
#include <vw/Image/AntiAliasing.h>
#include <vw/Image/InpaintView.h>
#include <vw/Image/DistanceFunction.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Core/CmdUtils.h>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/SfsImageProc.h>
#include <asp/Camera/RPCModelGen.h>

#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <iostream>
#include <stdexcept>
#include <string>
#include<sys/types.h>

#if defined(__GNUC__) || defined(__GNUG__)
#if LOCAL_GCC_VERSION >= 40600
#pragma GCC diagnostic pop
#endif
#undef LOCAL_GCC_VERSION
#endif

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int g_num_locks = 0;
int g_warning_count = 0;
int g_max_warning_count = 1000;
const size_t g_num_model_coeffs = 16;
const size_t g_max_num_haze_coeffs = 6; // see nonlin_reflectance()

// If the blend weight is ground weight, rather than image weight,
// use it as it is, without projecting into camera and interpolating.
// It is a lot of work to pass this all over the place.
bool g_blend_weight_is_ground_weight = false;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

typedef ImageViewRef<PixelMask<float>> MaskedImgT;
typedef ImageViewRef<double> DoubleImgT;

namespace vw { namespace camera {

  // A base approx camera model class that will factor out some functionality
  // from the two approx camera model classes we have below.
  class ApproxBaseCameraModel: public CameraModel {

  protected:
    BBox2i m_img_bbox;
    mutable BBox2 m_point_box, m_crop_box;
    bool m_model_is_valid;
    boost::shared_ptr<CameraModel> m_exact_unadjusted_camera;
    AdjustedCameraModel m_exact_adjusted_camera;

  public:

    ApproxBaseCameraModel(AdjustedCameraModel const& exact_adjusted_camera,
                          boost::shared_ptr<CameraModel> exact_unadjusted_camera,
                          BBox2i img_bbox): m_exact_adjusted_camera(exact_adjusted_camera),
                                            m_exact_unadjusted_camera(exact_unadjusted_camera),
                                            m_img_bbox(img_bbox){}
    
    // The range of pixels in the image we are actually expected to use.
    // Note that the function returns an alias, so that we can modify the
    // crop box from outside.
    BBox2 & crop_box(){
      m_crop_box.crop(m_img_bbox);
      return m_crop_box;
    }

    bool model_is_valid(){
      return m_model_is_valid;
    }
    
    boost::shared_ptr<CameraModel> exact_unadjusted_camera() const{
      return m_exact_unadjusted_camera;
    }
    
    AdjustedCameraModel exact_adjusted_camera() const{
      return m_exact_adjusted_camera;
    }
    
  };
    
  // This class provides an approximation for the point_to_pixel()
  // function of an ISIS camera around a current DEM. The algorithm
  // works by tabulation of point_to_pixel and pixel_to_vector values
  // at the mean dem height.
  class ApproxCameraModel: public ApproxBaseCameraModel {
    mutable Vector3 m_mean_dir; // mean vector from camera to ground
    GeoReference m_geo;
    double m_mean_ht;
    mutable ImageView< PixelMask<Vector3> > m_pixel_to_vec_mat;
    mutable ImageView< PixelMask<Vector2> > m_point_to_pix_mat;
    double m_approx_table_gridx, m_approx_table_gridy;
    bool m_use_rpc_approximation, m_use_semi_approx;
    vw::Mutex& m_camera_mutex;
    Vector2 m_uncompValue;
    mutable int m_begX, m_endX, m_begY, m_endY;
    mutable bool m_compute_mean, m_stop_growing_range;
    mutable int m_count;
    boost::shared_ptr<asp::RPCModel> m_rpc_model;
    
    bool comp_rpc_approx_table(AdjustedCameraModel const& adj_camera,
                               boost::shared_ptr<CameraModel> exact_unadjusted_camera,
                               BBox2i img_bbox,
                               ImageView<double> const& dem,
                               GeoReference const& geo,
                               double rpc_penalty_weight){

      try {
        // Generate point pairs
        std::vector<Vector3> all_llh;
        std::vector<Vector2> all_pixels;

        vw_out() << "Projecting pixels into the camera to generate the RPC model.\n";
        vw::TerminalProgressCallback tpc("asp", "\t--> ");
        double inc_amount = 1.0 / double(dem.cols());
        tpc.report_progress(0);

        // If the DEM is too big, we need to skip points. About
        // 40,000 points should be good enough to determine 78 RPC
        // coefficients.
        double num = 200.0;
        double delta_col = std::max(1.0, dem.cols()/double(num));
        double delta_row = std::max(1.0, dem.rows()/double(num));
        BBox3 llh_box;
        BBox2 pixel_box;
        for (double dcol = 0; dcol < dem.cols(); dcol += delta_col) {
          for (double drow = 0; drow < dem.rows(); drow += delta_row) {
            int col = dcol, row = drow; // cast to int
            Vector2 pix(col, row);
            Vector2 lonlat = geo.pixel_to_lonlat(pix);
          
            // Lon lat height
            Vector3 llh;
            llh[0] = lonlat[0]; llh[1] = lonlat[1]; llh[2] = dem(col, row);
            Vector3 xyz = geo.datum().geodetic_to_cartesian(llh);

            // Go back to llh. This is a bugfix for the 360 deg offset problem.
            llh = geo.datum().cartesian_to_geodetic(xyz);
          
            Vector2 cam_pix = exact_unadjusted_camera->point_to_pixel(xyz);
            //if (!m_img_bbox.contains(cam_pix)) 
            //  continue; // skip out of range pixels? Not a good idea.
         
            if (m_img_bbox.contains(cam_pix)) 
              m_crop_box.grow(cam_pix);

            all_llh.push_back(llh);
            all_pixels.push_back(cam_pix);

            llh_box.grow(llh);
            pixel_box.grow(cam_pix);
          }

          tpc.report_incremental_progress( inc_amount );
        }
        tpc.report_finished();
      
        BBox2 ll_box;
        ll_box.min() = subvector(llh_box.min(), 0, 2);
        ll_box.max() = subvector(llh_box.max(), 0, 2);

        BBox2 cropped_pixel_box = pixel_box;
        cropped_pixel_box.crop(m_img_bbox);
        if (cropped_pixel_box.empty()) {
          vw_out() << "No points fall into the camera.\n";
          return false;
        }

        if (ll_box.empty()) {
          vw_out() << "Empty lon-lat box.\n";
          return false;
        }

        // This is a bugfix. The RPC approximation works best when the
        // input llh points are in an llh box whose sides are vertical
        // and horizontal, rather than in a box which is rotated.
        vw_out() << "Re-projecting pixels into the camera to improve accuracy.\n";
        vw::TerminalProgressCallback tpc2("asp", "\t--> ");
        double inc_amount2 = 1.0 / double(num);
        tpc2.report_progress(0);
        llh_box = BBox3();
        pixel_box = BBox2();
        all_llh.clear();
        all_pixels.clear();
        ImageViewRef<double> interp_dem
          = interpolate(dem, BicubicInterpolation(), ConstantEdgeExtension());
        double delta_lon = (ll_box.max()[0] - ll_box.min()[0])/double(num);
        double delta_lat = (ll_box.max()[1] - ll_box.min()[1])/double(num);
        for (double lon = ll_box.min()[0]; lon <= ll_box.max()[0] + delta_lon; lon += delta_lon) {
          for (double lat = ll_box.min()[1]; lat <= ll_box.max()[1] + delta_lat; lat += delta_lat) {

            Vector2 pix = geo.lonlat_to_pixel(Vector2(lon, lat));
            if (pix[0] < 0 || pix[0] > dem.cols()-1) continue;
            if (pix[1] < 0 || pix[1] > dem.rows()-1) continue;
            double ht = interp_dem(pix[0], pix[1]);

            // Lon lat height
            Vector3 llh;
            llh[0] = lon; llh[1] = lat; llh[2] = ht;
            Vector3 xyz = geo.datum().geodetic_to_cartesian(llh);

            // Later we will project DEM points into the adjusted camera.
            // That is the same as projecting adjusted points into the exact camera.
            // Hence, develop the RPC approximation using adjusted points.
            // TODO: Maybe we should also ensure that the unadjusted xyz is
            // also part of the model building? But probably not, as usually
            // adjustments change very little, and this will increase run-time by 2x.
            xyz = adj_camera.adjusted_point(xyz);
            
            // Go back to llh. This is a bugfix for the 360 deg offset problem.
            llh = geo.datum().cartesian_to_geodetic(xyz);

            Vector2 cam_pix = exact_unadjusted_camera->point_to_pixel(xyz);
            //if (!m_img_bbox.contains(cam_pix)) 
            //  continue; // skip out of range pixels? Not a good idea.
         
            if (m_img_bbox.contains(cam_pix)) 
              m_crop_box.grow(cam_pix);

            all_llh.push_back(llh);
            all_pixels.push_back(cam_pix);
            
            llh_box.grow(llh);
            pixel_box.grow(cam_pix);
          }
          
          tpc2.report_incremental_progress( inc_amount2 );
        }
        tpc2.report_finished();

        cropped_pixel_box = pixel_box;
        cropped_pixel_box.crop(m_img_bbox);
        if (cropped_pixel_box.empty()) {
          vw_out() << "No points fall into the camera.\n";
          return false;
        }

        ll_box.min() = subvector(llh_box.min(), 0, 2);
        ll_box.max() = subvector(llh_box.max(), 0, 2);
        if (ll_box.empty()) {
          vw_out() << "Empty lon-lat box.\n";
          return false;
        }
        
        Vector3 llh_scale  = (llh_box.max() - llh_box.min())/2.0; // half range
        Vector3 llh_offset = (llh_box.max() + llh_box.min())/2.0; // center point
      
        Vector2 pixel_scale  = (pixel_box.max() - pixel_box.min())/2.0; // half range 
        Vector2 pixel_offset = (pixel_box.max() + pixel_box.min())/2.0; // center point

        // Ensure we never divide by zero. For example, if the input dem heights are all constant,
        // then the height scale will be zero from above.
        for (size_t i = 0; i < llh_scale.size(); i++) 
          if (llh_scale[i] == 0) llh_scale[i] = 1;
        for (size_t i = 0; i < pixel_scale.size(); i++) 
          if (pixel_scale[i] == 0) pixel_scale[i] = 1;
        
        vw_out() << "Lon-lat-height box for the RPC approx: " << llh_box   << std::endl;
        vw_out() << "Camera pixel box for the RPC approx:   " << pixel_box << std::endl;

        Vector<double> normalized_llh;
        Vector<double> normalized_pixels;
        int num_total_pts = all_llh.size();
        normalized_llh.set_size(asp::RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
        normalized_pixels.set_size(asp::RPCModel::IMAGE_COORD_SIZE*num_total_pts
                                   + asp::RpcSolveLMA::NUM_PENALTY_TERMS);
        for (size_t i = 0; i < normalized_pixels.size(); i++) {
          // Important: The extra penalty terms are all set to zero here.
          normalized_pixels[i] = 0.0; 
        }
      
        // Form the arrays of normalized pixels and normalized llh
        for (int pt = 0; pt < num_total_pts; pt++) {
    
          // Normalize the pixel to -1 <> 1 range
          Vector3 llh_n   = elem_quot(all_llh[pt]    - llh_offset,   llh_scale);
          Vector2 pixel_n = elem_quot(all_pixels[pt] - pixel_offset, pixel_scale);
          subvector(normalized_llh, asp::RPCModel::GEODETIC_COORD_SIZE*pt,
                    asp::RPCModel::GEODETIC_COORD_SIZE) = llh_n;
          subvector(normalized_pixels, asp::RPCModel::IMAGE_COORD_SIZE*pt,
                    asp::RPCModel::IMAGE_COORD_SIZE   ) = pixel_n;
    
        }

        // Find the RPC coefficients
        asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
        std::string output_prefix = "";
        vw_out() << "Generating the RPC approximation using " << num_total_pts << " point pairs.\n";
        asp::gen_rpc(// Inputs
                     rpc_penalty_weight, output_prefix,
                     normalized_llh, normalized_pixels,  
                     llh_scale, llh_offset, pixel_scale, pixel_offset,
                     // Outputs
                     line_num, line_den, samp_num, samp_den);
      
        m_rpc_model = boost::shared_ptr<asp::RPCModel>
          (new asp::RPCModel(geo.datum(), line_num, line_den,
                             samp_num, samp_den, pixel_offset, pixel_scale,
                             llh_offset, llh_scale));
      } catch (std::exception const& e) {
        vw_out() << e.what() << std::endl;
        return false;
      }      
      
      return true;
    }
    
    void comp_entries_in_table() const{
      for (int x = m_begX; x <= m_endX; x++) {
        for (int y = m_begY; y <= m_endY; y++) {
      
          // This will be useful when we invoke this function repeatedly
          if (m_point_to_pix_mat(x, y).child() != m_uncompValue) {
            continue;
          }
      
          Vector2 pt(m_point_box.min().x() + x*m_approx_table_gridx,
                     m_point_box.min().y() + y*m_approx_table_gridy);
          Vector2 lonlat = m_geo.point_to_lonlat(pt);
          Vector3 xyz = m_geo.datum().geodetic_to_cartesian
            (Vector3(lonlat[0], lonlat[1], m_mean_ht));
          bool success = true;
          Vector2 pix;
          Vector3 vec;
          try {
            pix = m_exact_unadjusted_camera->point_to_pixel(xyz);
            //if (true || m_img_bbox.contains(pix))  // Need to think more here
            vec = m_exact_unadjusted_camera->pixel_to_vector(pix);
            //else
            // success = false;
            
          }catch(...){
            success = false;
          }
          if (success) {
            m_pixel_to_vec_mat(x, y) = vec;
            m_point_to_pix_mat(x, y) = pix;
            m_pixel_to_vec_mat(x, y).validate();
            m_point_to_pix_mat(x, y).validate();
            if (m_compute_mean) {
              m_mean_dir += vec; // only when the point projects inside the camera?
              if (m_img_bbox.contains(pix)) 
                m_crop_box.grow(pix);
              m_count++;
            }
          }else{
            m_pixel_to_vec_mat(x, y).invalidate();
            m_point_to_pix_mat(x, y).invalidate();
          }
        }
      }
      
    }
    
  public:

    ApproxCameraModel(AdjustedCameraModel const& exact_adjusted_camera,
                      boost::shared_ptr<CameraModel> exact_unadjusted_camera,
                      BBox2i img_bbox, 
                      ImageView<double> const& dem,
                      GeoReference const& geo,
                      double nodata_val,
                      bool use_rpc_approximation, bool use_semi_approx,
                      double rpc_penalty_weight,
                      vw::Mutex &camera_mutex):
      ApproxBaseCameraModel(exact_adjusted_camera, exact_unadjusted_camera, img_bbox),
      m_geo(geo),
      m_use_rpc_approximation(use_rpc_approximation),
      m_use_semi_approx(use_semi_approx),
      m_camera_mutex(camera_mutex) {

      // Initialize members of the base class
      m_model_is_valid = true;
      
      int big = 1e+8;
      m_uncompValue = Vector2(-big, -big);
      m_compute_mean = true; // We'll set this to false when we finish estimating the mean
      m_stop_growing_range = false; // stop when it does not help
      
      if (dynamic_cast<IsisCameraModel*>(exact_unadjusted_camera.get()) == NULL)
        vw_throw( ArgumentErr()
                  << "ApproxCameraModel: Expecting an unadjusted camera model.\n");

      // Compute the mean DEM height.
      // We expect all DEM entries to be valid.
      m_mean_ht = 0;
      double num = 0.0;
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          if (dem(col, row) == nodata_val)
            vw_throw( ArgumentErr()
                      << "ApproxCameraModel: Expecting a DEM without nodata values.\n");
          m_mean_ht += dem(col, row);
          num += 1.0;
        }
      }
      if (num > 0) m_mean_ht /= num;

      // The area we're supposed to work around
      m_point_box = m_geo.pixel_to_point_bbox(bounding_box(dem));
      double wx = m_point_box.width(), wy = m_point_box.height();
      m_approx_table_gridx = wx/std::max(dem.cols(), 1);
      m_approx_table_gridy = wy/std::max(dem.rows(), 1);

      if (m_approx_table_gridx == 0 || m_approx_table_gridy == 0) {
        vw_throw( ArgumentErr()
                  << "ApproxCameraModel: Expecting a positive grid size.\n");
      }

      // Expand the box, as later the DEM will change. 
      double extra = 1.00; // may need to lower here!
      m_point_box.min().x() -= extra*wx; m_point_box.max().x() += extra*wx;
      m_point_box.min().y() -= extra*wy; m_point_box.max().y() += extra*wy;
      wx = m_point_box.width();
      wy = m_point_box.height();

      vw_out() << "Approximation proj box: " << m_point_box << std::endl;

      if (m_use_semi_approx)
        return;
      
      // Bypass everything if doing RPC
      if (m_use_rpc_approximation) {
        m_model_is_valid = comp_rpc_approx_table(exact_adjusted_camera,
                                                 exact_unadjusted_camera, m_img_bbox,
                                                 dem,  geo, rpc_penalty_weight);
        
        // Ensure the box is valid
        //if (m_crop_box.empty()) m_crop_box = BBox2(0, 0, 2, 2);
    
#if 1
        // Expand the box a bit, as later the DEM will change and values at some
        // new pixels will be needed.
        m_crop_box.crop(m_img_bbox);
        if (!m_crop_box.empty()) {
          double wd = m_crop_box.width();
          double ht = m_crop_box.height();
          m_crop_box.min().x() -= extra*wd; m_crop_box.max().x() += extra*wd;
          m_crop_box.min().y() -= extra*ht; m_crop_box.max().y() += extra*ht;
          m_crop_box = grow_bbox_to_int(m_crop_box);
        }
        m_crop_box.crop(m_img_bbox);
#endif

        return;
      }
      
      // We will tabulate the point_to_pixel function at a multiple of
      // the grid, and we'll use interpolation for anything in
      // between.
      //m_approx_table_gridx /= 2.0; m_approx_table_gridy /= 2.0; // fine
      m_approx_table_gridx *= 2.0; m_approx_table_gridy *= 2.0; // coarse. good enough.

      int numx = wx/m_approx_table_gridx;
      int numy = wy/m_approx_table_gridy;

      vw_out() << "Lookup table dimensions: " << numx << ' ' << numy << std::endl;

      // Choose f so that the width from m_begX to m_endX is 2 x original wx
      double f = 0; // (extra-0.5)/(2.0*extra+1.0);
      m_begX = f*numx; m_endX = std::min((1.0-f)*numx, numx-1.0);
      m_begY = f*numy; m_endY = std::min((1.0-f)*numy, numy-1.0);
      
      //vw_out() << "Size of actually pre-computed table: "
      //           << m_endX - m_begX << ' ' << m_endY - m_begY << std::endl;
      
      // Mark all values as uncomputed and invalid
      m_pixel_to_vec_mat.set_size(numx, numy);
      m_point_to_pix_mat.set_size(numx, numy);
      for (int x = 0; x < numx; x++) {
        for (int y = 0; y < numy; y++) {
          m_point_to_pix_mat(x, y) = m_uncompValue;
          m_point_to_pix_mat(x, y).invalidate();
        }
      }
      
      // Fill in the table. Find along the way the mean direction from
      // the camera to the ground. Invalid values will be masked.
      m_count = 0;
      m_mean_dir = Vector3();
      comp_entries_in_table();
      m_mean_dir /= std::max(1, m_count);
      m_mean_dir = m_mean_dir/norm_2(m_mean_dir);
      m_compute_mean = false; // done computing the mean
      
      // Ensure the box is valid
      //if (m_crop_box.empty()) m_crop_box = BBox2(0, 0, 2, 2);

#if 1
      // Expansion should not be necessary, as we already expanded
      // m_point_box and we used that expanded box to compute m_crop_box.
      m_crop_box.crop(m_img_bbox);
      if (!m_crop_box.empty()) {
        // Expand the box a bit, as later the DEM will change and values at some
        // new pixels will be needed.
        double wd = m_crop_box.width();
        double ht = m_crop_box.height();
        double extra2 = 0.25; // still, just in case, a bit more expansion
        m_crop_box.min().x() -= extra2*wd; m_crop_box.max().x() += extra2*wd;
        m_crop_box.min().y() -= extra2*ht; m_crop_box.max().y() += extra2*ht;
        m_crop_box = grow_bbox_to_int(m_crop_box);
      }
      m_crop_box.crop(m_img_bbox);
#endif

      return;
    }

    // We have tabulated point_to_pixel at the mean dem height.
    // Look-up point_to_pixel for the current point by first
    // intersecting the ray from the current point to the camera
    // with the datum at that height. We don't know that ray,
    // so we iterate to find it.
    virtual Vector2 point_to_pixel(Vector3 const& xyz) const{

      if (m_use_semi_approx){
        vw::Mutex::Lock lock(m_camera_mutex);
        g_num_locks++;
        return m_exact_unadjusted_camera->point_to_pixel(xyz);
      }
      
      if (m_use_rpc_approximation) 
        return m_rpc_model->point_to_pixel(xyz);
      
      // TODO: What happens if we use bicubic interpolation?
      InterpolationView<EdgeExtensionView< ImageView< PixelMask<Vector3> >, ConstantEdgeExtension >, BilinearInterpolation> pixel_to_vec_interp
        = interpolate(m_pixel_to_vec_mat, BilinearInterpolation(),
                      ConstantEdgeExtension());

      InterpolationView<EdgeExtensionView< ImageView< PixelMask<Vector2> >, ConstantEdgeExtension >, BilinearInterpolation> point_to_pix_interp
        = interpolate(m_point_to_pix_mat, BilinearInterpolation(),
                      ConstantEdgeExtension());

      Vector3 dir = m_mean_dir;
      Vector2 pix;
      double major_radius = m_geo.datum().semi_major_axis() + m_mean_ht;
      double minor_radius = m_geo.datum().semi_minor_axis() + m_mean_ht;
      for (size_t i = 0; i < 10; i++) {

        Vector3 S = xyz - 1.1*major_radius*dir; // push the point outside the sphere
        if (norm_2(S) <= major_radius) {
          // should not happen. Return the exact solution.
          {
            vw::Mutex::Lock lock(m_camera_mutex);
            g_num_locks++;
            if (g_warning_count < g_max_warning_count) {
              g_warning_count++;
              vw_out(WarningMessage) << "3D point is inside the planet.\n";
            }
            return m_exact_unadjusted_camera->point_to_pixel(xyz);
          }
        }

        Vector3 datum_pt = datum_intersection(major_radius, minor_radius, S, dir);
        Vector3 llh = m_geo.datum().cartesian_to_geodetic(datum_pt);
        Vector2 pt = m_geo.lonlat_to_point(subvector(llh, 0, 2));

        // Indices
        double x = (pt.x() - m_point_box.min().x())/m_approx_table_gridx;
        double y = (pt.y() - m_point_box.min().y())/m_approx_table_gridy;

        bool out_of_range = ( x < 0 || x >= m_pixel_to_vec_mat.cols()-1 ||
                              y < 0 || y >= m_pixel_to_vec_mat.rows()-1 );

        bool out_of_comp_range = (x < m_begX || x >= m_endX-1 ||
                                  y < m_begY || y >= m_endY-1);

        // If we are not out of range, but we need to expand the computed table, do that
        if (!m_stop_growing_range && !out_of_range && out_of_comp_range) {
          vw::Mutex::Lock lock(m_camera_mutex);
          g_num_locks++;
          if (g_warning_count < g_max_warning_count) {
            g_warning_count++;
            vw_out(WarningMessage) << "Pixel outside of computed range. "
                                   << "Growing the computed table." << std::endl;
            vw_out(WarningMessage) << "Start table: " << m_begX << ' ' << m_begY << ' '
                                   << m_endX << ' ' << m_endY << std::endl;
          }
          
          // If we have to expand, do it by a lot
          int extrax = std::max(10, int(0.1*(m_endX - m_begX)));
          int extray = std::max(10, int(0.1*(m_endY - m_begY)));

          int old_begX = m_begX, old_begY = m_begY;
          int old_endX = m_endX, old_endY = m_endY;

          m_begX = std::min(m_begX, int(floor(x))) - extrax; m_begX = std::max(0, m_begX);
          m_begY = std::min(m_begY, int(floor(y))) - extray; m_begY = std::max(0, m_begY);
      
          m_endX = std::max(m_endX, int(ceil(x))) + extrax;
          m_endX = std::min(m_pixel_to_vec_mat.cols()-1, m_endX);

          m_endY = std::max(m_endY, int(ceil(y))) + extray;
          m_endY = std::min(m_pixel_to_vec_mat.rows()-1, m_endY);
      
          if (g_warning_count < g_max_warning_count) {
            vw_out(WarningMessage) << "Updated table: " << m_begX << ' ' << m_begY << ' '
                                   << m_endX << ' ' << m_endY << std::endl;
          }
          comp_entries_in_table();

          // Update this
          out_of_comp_range = (x < m_begX || x >= m_endX-1 ||
                               y < m_begY || y >= m_endY-1);

          // Avoid an infinite loop if we can't grow the table
          if (old_begX == m_begX && old_begY == m_begY &&
              old_endX == m_endX && old_endY == m_endY ) {
            m_stop_growing_range = true;
          }
        }

        if (out_of_range || out_of_comp_range){
          vw::Mutex::Lock lock(m_camera_mutex);
          g_num_locks++;
          if (g_warning_count < g_max_warning_count) {
            g_warning_count++;
            vw_out(WarningMessage) << "Pixel outside of range. Current values and range: "  << ' '
                                   << x << ' ' << y << ' '
                                   << m_pixel_to_vec_mat.cols() << ' ' << m_pixel_to_vec_mat.rows()
                                   << std::endl;
          }
          return m_exact_unadjusted_camera->point_to_pixel(xyz);
        }
        PixelMask<Vector3> masked_dir = pixel_to_vec_interp(x, y);
        PixelMask<Vector2> masked_pix = point_to_pix_interp(x, y);

        if (is_valid(masked_dir) && is_valid(masked_pix)) {
          dir = masked_dir.child();
          pix = masked_pix.child();
        }else{
          {
            vw::Mutex::Lock lock(m_camera_mutex);
            g_num_locks++;
            if (g_warning_count < g_max_warning_count) {
              g_warning_count++;
              vw_out(WarningMessage) << "Invalid ground to camera direction: "
                                     << masked_dir << ' ' << masked_pix << std::endl;
            }
            return m_exact_unadjusted_camera->point_to_pixel(xyz);
          }
        }
      }

      return pix;
    }

    virtual ~ApproxCameraModel(){}
    virtual std::string type() const{ return "ApproxIsis"; }

    virtual Vector3 pixel_to_vector(Vector2 const& pix) const {

      if (m_use_semi_approx) {
        vw::Mutex::Lock lock(m_camera_mutex);
        g_num_locks++;
        return this->exact_unadjusted_camera()->pixel_to_vector(pix);
      }

      if (m_use_rpc_approximation){
        return m_rpc_model->pixel_to_vector(pix);
      }
      
      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      if (g_warning_count < g_max_warning_count) {
        g_warning_count++;
        vw_out(WarningMessage) << "Invoked exact camera model pixel_to_vector for pixel: "
                               << pix << std::endl;
      }
      return this->exact_unadjusted_camera()->pixel_to_vector(pix);
    }

    virtual Vector3 camera_center(Vector2 const& pix) const{
      // It is tricky to approximate the camera center
      //if (m_use_rpc_approximation){
      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      //vw_out(WarningMessage) << "Invoked the camera center function for pixel: "
      //                       << pix << std::endl;
      return this->exact_unadjusted_camera()->camera_center(pix);
      //return m_rpc_model->camera_center(pix);
      //}

#if 0
      // TODO: Is this function invoked? Should just the underlying exact model
      // camera center be used all the time?
      InterpolationView<EdgeExtensionView< ImageView< PixelMask<Vector3> >, ConstantEdgeExtension >, BilinearInterpolation> camera_center_interp
        = interpolate(m_camera_center_mat, BilinearInterpolation(),
                      ConstantEdgeExtension());
      double lx = pix[0] - m_crop_box.min().x();
      double ly = pix[1] - m_crop_box.min().y();
      if (0 <= lx && lx < m_camera_center_mat.cols() - 1 &&
          0 <= ly && ly < m_camera_center_mat.rows() - 1 ) {
        PixelMask<Vector3> ctr = camera_center_interp(lx, ly);
        if (is_valid(ctr))
          return ctr.child();
      }
#endif
      
      {
        // Failed to interpolate
        vw::Mutex::Lock lock(m_camera_mutex);
        g_num_locks++;
        if (g_warning_count < g_max_warning_count) {
          g_warning_count++;
          vw_out(WarningMessage) << "Invoked the camera center function for pixel: "
                                 << pix << std::endl;
        }
        return this->exact_unadjusted_camera()->camera_center(pix);
      }

    }

    virtual Quat camera_pose(Vector2 const& pix) const{
      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      if (g_warning_count < g_max_warning_count) {
        g_warning_count++;
        vw_out(WarningMessage) << "Invoked the camera pose function for pixel: "
                               << pix << std::endl;
      }
      return this->exact_unadjusted_camera()->camera_pose(pix);
    }

  };

  // TODO(oalexan1): Must use the adjusted model in the camera center
  // and camera pose functions!
  // This class provides an approximation for an adjusted ISIS camera
  // model around a current DEM. Unlike the ApproxCameraModel class,
  // here the adjusted camera is approximated, not the unadjusted one,
  // hence the adjustments and the cameras themselves cannot be
  // floated with this class. Keeping the cameras fixed allows the
  // domain of approximation to be narrower so using less memory. The
  // algorithm works by tabulation of point_to_pixel and
  // pixel_to_vector values at the mean dem height.
  class ApproxAdjustedCameraModel: public ApproxBaseCameraModel {
    mutable Vector3 m_mean_dir; // mean vector from camera to ground
    GeoReference m_geo;
    double m_mean_ht;
    mutable ImageView< PixelMask<Vector3> > m_pixel_to_vec_mat;
    mutable ImageView< PixelMask<Vector2> > m_point_to_pix_mat;
    double m_approx_table_gridx, m_approx_table_gridy;
    vw::Mutex& m_camera_mutex;
    Vector2 m_uncompValue;
    mutable int m_begX, m_endX, m_begY, m_endY;
    mutable int m_count;
    
    void comp_entries_in_table() const{
      for (int x = m_begX; x <= m_endX; x++) {
        for (int y = m_begY; y <= m_endY; y++) {
      
          // This will be useful when we invoke this function repeatedly
          if (m_point_to_pix_mat(x, y).child() != m_uncompValue) {
            continue;
          }
      
          Vector2 pt(m_point_box.min().x() + x*m_approx_table_gridx,
                     m_point_box.min().y() + y*m_approx_table_gridy);
          Vector2 lonlat = m_geo.point_to_lonlat(pt);
          Vector3 xyz = m_geo.datum().geodetic_to_cartesian
            (Vector3(lonlat[0], lonlat[1], m_mean_ht));
          bool success = true;
          Vector2 pix;
          Vector3 vec;
          try {
            pix = m_exact_adjusted_camera.point_to_pixel(xyz);
            //if (true || m_img_bbox.contains(pix))  // Need to think more here
            vec = m_exact_adjusted_camera.pixel_to_vector(pix);
            //else
            // success = false;
            
          }catch(...){
            success = false;
          }
          if (success) {
            m_pixel_to_vec_mat(x, y) = vec;
            m_point_to_pix_mat(x, y) = pix;
            m_pixel_to_vec_mat(x, y).validate();
            m_point_to_pix_mat(x, y).validate();
            m_mean_dir += vec; // only when the point projects inside the camera?
            if (m_img_bbox.contains(pix)) 
              m_crop_box.grow(pix);
            m_count++;
          }else{
            m_pixel_to_vec_mat(x, y).invalidate();
            m_point_to_pix_mat(x, y).invalidate();
          }
        }
      }
      
    }
    
  public:

    ApproxAdjustedCameraModel(AdjustedCameraModel const& exact_adjusted_camera,
                              boost::shared_ptr<CameraModel> exact_unadjusted_camera,
                              BBox2i img_bbox, 
                              ImageView<double> const& dem,
                              GeoReference const& geo,
                              double nodata_val,
                              vw::Mutex &camera_mutex):
      ApproxBaseCameraModel(exact_adjusted_camera, exact_unadjusted_camera, img_bbox),
      m_geo(geo), m_camera_mutex(camera_mutex) {

      // Initialize members of the base class
      m_model_is_valid = true;

      int big = 1e+8;
      m_uncompValue = Vector2(-big, -big);
      
      if (dynamic_cast<AdjustedCameraModel*>(exact_unadjusted_camera.get()) != NULL)
        vw_throw( ArgumentErr()
                  << "ApproxAdjustedCameraModel: Expecting an unadjusted camera model.\n");

      // Compute the mean DEM height.
      // We expect all DEM entries to be valid.
      m_mean_ht = 0;
      double num = 0.0;
      for (int col = 0; col < dem.cols(); col++) {
        for (int row = 0; row < dem.rows(); row++) {
          if (dem(col, row) == nodata_val)
            vw_throw( ArgumentErr()
                      << "ApproxAdjustedCameraModel: Expecting a DEM without nodata values.\n");
          m_mean_ht += dem(col, row);
          num += 1.0;
        }
      }
      if (num > 0) m_mean_ht /= num;

      // The area we're supposed to work around
      m_point_box = m_geo.pixel_to_point_bbox(bounding_box(dem));
      double wx = m_point_box.width(), wy = m_point_box.height();
      m_approx_table_gridx = wx/std::max(dem.cols(), 1);
      m_approx_table_gridy = wy/std::max(dem.rows(), 1);

      if (m_approx_table_gridx == 0 || m_approx_table_gridy == 0) {
        vw_throw( ArgumentErr()
                  << "ApproxAdjustedCameraModel: Expecting a positive grid size.\n");
      }

      // Expand the box, as later the DEM will change. 
      double extra = 0.5;
      m_point_box.min().x() -= extra*wx; m_point_box.max().x() += extra*wx;
      m_point_box.min().y() -= extra*wy; m_point_box.max().y() += extra*wy;
      wx = m_point_box.width();
      wy = m_point_box.height();

      vw_out() << "Approximation proj box: " << m_point_box << std::endl;

      // We will tabulate the point_to_pixel function at a multiple of
      // the grid, and we'll use interpolation for anything in
      // between.
      //m_approx_table_gridx /= 2.0; m_approx_table_gridy /= 2.0; // fine
      m_approx_table_gridx *= 2.0; m_approx_table_gridy *= 2.0; // Coarse. Good enough.

      int numx = wx/m_approx_table_gridx;
      int numy = wy/m_approx_table_gridy;

      vw_out() << "Lookup table dimensions: " << numx << ' ' << numy << std::endl;

      m_begX = 0; m_endX = numx-1;
      m_begY = 0; m_endY = numy-1;
      
      //vw_out() << "Size of actually pre-computed table: "
      //           << m_endX - m_begX << ' ' << m_endY - m_begY << std::endl;
      
      // Mark all values as uncomputed and invalid
      m_pixel_to_vec_mat.set_size(numx, numy);
      m_point_to_pix_mat.set_size(numx, numy);
      for (int x = 0; x < numx; x++) {
        for (int y = 0; y < numy; y++) {
          m_point_to_pix_mat(x, y) = m_uncompValue;
          m_point_to_pix_mat(x, y).invalidate();
        }
      }
      
      // Fill in the table. Find along the way the mean direction from
      // the camera to the ground. Invalid values will be masked.
      m_count = 0;
      m_mean_dir = Vector3();
      comp_entries_in_table();
      m_mean_dir /= std::max(1, m_count);
      m_mean_dir = m_mean_dir/norm_2(m_mean_dir);
      
      m_crop_box.crop(m_img_bbox);

      return;
    }

    // We have tabulated point_to_pixel at the mean dem height.
    // Look-up point_to_pixel for the current point by first
    // intersecting the ray from the current point to the camera
    // with the datum at that height. We don't know that ray,
    // so we iterate to find it.
    virtual Vector2 point_to_pixel(Vector3 const& xyz) const{

      // TODO: What happens if we use bicubic interpolation?
      InterpolationView<EdgeExtensionView< ImageView< PixelMask<Vector3> >, ConstantEdgeExtension >, BilinearInterpolation> pixel_to_vec_interp
        = interpolate(m_pixel_to_vec_mat, BilinearInterpolation(),
                      ConstantEdgeExtension());

      InterpolationView<EdgeExtensionView< ImageView< PixelMask<Vector2> >, ConstantEdgeExtension >, BilinearInterpolation> point_to_pix_interp
        = interpolate(m_point_to_pix_mat, BilinearInterpolation(),
                      ConstantEdgeExtension());

      Vector3 dir = m_mean_dir;
      Vector2 pix;
      double major_radius = m_geo.datum().semi_major_axis() + m_mean_ht;
      double minor_radius = m_geo.datum().semi_minor_axis() + m_mean_ht;
      for (size_t i = 0; i < 10; i++) {

        Vector3 S = xyz - 1.1*major_radius*dir; // push the point outside the sphere
        if (norm_2(S) <= major_radius) {
          // should not happen. Return the exact solution.
          {
            vw::Mutex::Lock lock(m_camera_mutex);
            g_num_locks++;
            if (g_warning_count < g_max_warning_count) {
              g_warning_count++;
              vw_out(WarningMessage) << "3D point is inside the planet.\n";
            }
            return m_exact_adjusted_camera.point_to_pixel(xyz);
          }
        }

        Vector3 datum_pt = datum_intersection(major_radius, minor_radius, S, dir);
        Vector3 llh = m_geo.datum().cartesian_to_geodetic(datum_pt);
        Vector2 pt = m_geo.lonlat_to_point(subvector(llh, 0, 2));

        // Indices
        double x = (pt.x() - m_point_box.min().x())/m_approx_table_gridx;
        double y = (pt.y() - m_point_box.min().y())/m_approx_table_gridy;

        bool out_of_range = (x < m_begX || x >= m_endX-1 ||
                             y < m_begY || y >= m_endY-1);

        // If out of range, return the exact result. This should be very slow.
        // The hope is that it will be very rare.
        if (out_of_range){
          vw::Mutex::Lock lock(m_camera_mutex);
          g_num_locks++;
          if (g_warning_count < g_max_warning_count) {
            g_warning_count++;
            vw_out(WarningMessage) << "Pixel outside of range. Current values and range: "  << ' '
                                   << x << ' ' << y << ' '
                                   << m_pixel_to_vec_mat.cols() << ' ' << m_pixel_to_vec_mat.rows()
                                   << std::endl;
          }
          return m_exact_adjusted_camera.point_to_pixel(xyz);
        }
        
        PixelMask<Vector3> masked_dir = pixel_to_vec_interp(x, y);
        PixelMask<Vector2> masked_pix = point_to_pix_interp(x, y);
        if (is_valid(masked_dir) && is_valid(masked_pix)) {
          dir = masked_dir.child();
          pix = masked_pix.child();
        }else{
          {
            vw::Mutex::Lock lock(m_camera_mutex);
            g_num_locks++;
            if (g_warning_count < g_max_warning_count) {
              g_warning_count++;
              vw_out(WarningMessage) << "Invalid ground to camera direction: "
                                     << masked_dir << ' ' << masked_pix << std::endl;
            }
            return m_exact_adjusted_camera.point_to_pixel(xyz);
          }
        }
      }

      return pix;
    }

    virtual ~ApproxAdjustedCameraModel(){}
    virtual std::string type() const{ return "ApproxAdjustedIsis"; }

    virtual Vector3 pixel_to_vector(Vector2 const& pix) const {

      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      if (g_warning_count < g_max_warning_count) {
        g_warning_count++;
        vw_out(WarningMessage) << "Invoked exact camera model pixel_to_vector for pixel: "
                               << pix << std::endl;
      }
      // TODO(oalexan1): Put here the exact adjusted camera!
      return this->exact_unadjusted_camera()->pixel_to_vector(pix);
    }

    virtual Vector3 camera_center(Vector2 const& pix) const{
      // It is tricky to approximate the camera center
      // TODO(oalexan1): Must apply the adjustment here?
      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      // TODO(oalexan1): Put here the exact adjusted camera!
      return this->exact_unadjusted_camera()->camera_center(pix);
    }

    virtual Quat camera_pose(Vector2 const& pix) const{
      // TODO(oalexan1): Must apply the adjustment here?!!!
      vw::Mutex::Lock lock(m_camera_mutex);
      g_num_locks++;
      if (g_warning_count < g_max_warning_count) {
        g_warning_count++;
        vw_out(WarningMessage) << "Invoked the camera pose function for pixel: "
                               << pix << std::endl;
      }
      // TODO(oalexan1): Put here the exact adjusted camera!
      return this->exact_unadjusted_camera()->camera_pose(pix);
    }

  };
  
}}

// Get the memory usage for the given process. This is for debugging, not used
// in production code. It does not work on OSX.
void callTop() {

  std::ostringstream os;
  int pid = getpid();
  os << pid;
  
  std::string cmd = "top -b -n 1 | grep -i ' sfs' | grep -i '" + os.str() + "'";
  std::string ans = vw::exec_cmd(cmd.c_str());
  vw_out() << "Memory usage: " << cmd << " " << ans << std::endl;
}


// Compute mean and standard deviation of two images. Do it where both are valid.
template <class ImageT>
void compute_image_stats(ImageT const& I1, ImageT const& I2,
                         double & mean1, double & stdev1,
                         double & mean2, double & stdev2) {

  if (I1.cols() != I2.cols() || I1.rows() != I2.rows()) 
    vw_throw(ArgumentErr() << "Expecting two input images of same size.\n");
  
  mean1 = 0; stdev1 = 0;
  mean2 = 0; stdev2 = 0;
  
  double sum1 = 0.0, sum2 = 0.0, sum1_sq = 0.0, sum2_sq = 0.0, count = 0.0;
  for (int col = 0; col < I1.cols(); col++){
    for (int row = 0; row < I1.rows(); row++){
      
      if (!is_valid(I1(col, row)) || !is_valid(I2(col, row))) continue;
                    
      count++;
      
      double val1 = I1(col, row); sum1 += val1; sum1_sq += val1*val1;
      double val2 = I2(col, row); sum2 += val2; sum2_sq += val2*val2;
    }
  }

  if (count > 0){
    mean1 = sum1/count; stdev1 = sqrt(sum1_sq/count - mean1*mean1);
    mean2 = sum2/count; stdev2 = sqrt(sum2_sq/count - mean2*mean2);
  }

}

struct Options : public vw::GdalWriteOptions {
  std::string input_dems_str, image_list, camera_list, out_prefix, stereo_session, bundle_adjust_prefix;
  std::vector<std::string> input_dems, input_images, input_cameras;
  std::string shadow_thresholds, custom_shadow_threshold_list, max_valid_image_vals, skip_images_str, image_exposure_prefix, model_coeffs_prefix, model_coeffs, image_haze_prefix, sun_positions_list;
  std::vector<float> shadow_threshold_vec, max_valid_image_vals_vec;
  std::vector<double> image_exposures_vec;
  std::vector<std::vector<double>> image_haze_vec;
  std::vector<double> model_coeffs_vec;
  std::vector<std::set<int>> skip_images;
  int max_iterations, max_coarse_iterations, reflectance_type, coarse_levels,
    blending_dist, min_blend_size, num_haze_coeffs;
  bool float_albedo, float_exposure, float_cameras, float_all_cameras, model_shadows,
    save_computed_intensity_only, estimate_slope_errors, estimate_height_errors,
    compute_exposures_only,
    save_dem_with_nodata, use_approx_camera_models, use_approx_adjusted_camera_models,
    use_rpc_approximation, use_semi_approx,
    crop_input_images, allow_borderline_data, float_dem_at_boundary, boundary_fix,
    fix_dem, float_reflectance_model, float_sun_position, query, save_sparingly,
    float_haze;
    
  double smoothness_weight, steepness_factor, curvature_in_shadow,
    curvature_in_shadow_weight,
    lit_curvature_dist, shadow_curvature_dist, gradient_weight,
    blending_power, integrability_weight, smoothness_weight_pq, init_dem_height,
    nodata_val, initial_dem_constraint_weight, albedo_constraint_weight,
    albedo_robust_threshold,
    camera_position_step_size, rpc_penalty_weight, rpc_max_error,
    unreliable_intensity_threshold, robust_threshold, shadow_threshold;
  vw::BBox2 crop_win;
  vw::Vector2 height_error_params;
  
  Options():max_iterations(0), max_coarse_iterations(0), reflectance_type(0),
            coarse_levels(0), blending_dist(0), blending_power(2.0),
            min_blend_size(0), num_haze_coeffs(0),
            float_albedo(false), float_exposure(false), float_cameras(false),
            float_all_cameras(false),
            model_shadows(false), 
            save_computed_intensity_only(false),
            estimate_slope_errors(false),
            estimate_height_errors(false),
            compute_exposures_only(false),
            save_dem_with_nodata(false),
            use_approx_camera_models(false),
            use_approx_adjusted_camera_models(false),
            use_rpc_approximation(false),
            use_semi_approx(false),
            crop_input_images(false),
            allow_borderline_data(false), 
            float_dem_at_boundary(false), boundary_fix(false), fix_dem(false),
            float_reflectance_model(false), float_sun_position(false),
            query(false), save_sparingly(false), float_haze(false),
            smoothness_weight(0), steepness_factor(1.0),
            curvature_in_shadow(0), curvature_in_shadow_weight(0.0),
            lit_curvature_dist(0.0), shadow_curvature_dist(0.0),
            gradient_weight(0.0), integrability_weight(0), smoothness_weight_pq(0),
            initial_dem_constraint_weight(0.0),
            albedo_constraint_weight(0.0), albedo_robust_threshold(0.0),
            camera_position_step_size(1.0), rpc_penalty_weight(0.0),
            rpc_max_error(0.0),
            unreliable_intensity_threshold(0.0),
            crop_win(BBox2i(0, 0, 0, 0)){}
};

struct GlobalParams{
  int reflectanceType;
  // Two parameters used in the formula for the Lunar-Lambertian
  // reflectance
  double phaseCoeffC1, phaseCoeffC2;
};

struct ModelParams {
  vw::Vector3 sunPosition; //relative to the center of the Moon
  ModelParams(){}
  ~ModelParams(){}
};

// Make the reflectance nonlinear using a rational function
double nonlin_reflectance(double reflectance, double exposure,
                          double steepness_factor,
                          double const* haze, int num_haze_coeffs){

  // Make the exposure smaller. This will result in higher reflectance
  // to compensate, as intensity = exposure * reflectance, hence
  // steeper terrain. Things become more complicated if the haze
  // and nonlinear reflectance is modeled.
  exposure /= steepness_factor;
  
  double r = reflectance; // for short
  if (num_haze_coeffs == 0) return (exposure*r);
  if (num_haze_coeffs == 1) return (exposure*r + haze[0]);
  if (num_haze_coeffs == 2) return (exposure*r + haze[0])/(haze[1]*r + 1);
  if (num_haze_coeffs == 3) return (haze[2]*r*r + exposure*r + haze[0])/(haze[1]*r + 1);
  if (num_haze_coeffs == 4) return (haze[2]*r*r + exposure*r + haze[0])/(haze[3]*r*r + haze[1]*r + 1);
  if (num_haze_coeffs == 5) return (haze[4]*r*r*r + haze[2]*r*r + exposure*r + haze[0])/(haze[3]*r*r + haze[1]*r + 1);
  if (num_haze_coeffs == 6) return (haze[4]*r*r*r + haze[2]*r*r + exposure*r + haze[0])/(haze[5]*r*r*r + haze[3]*r*r + haze[1]*r + 1);
    
  vw_throw(ArgumentErr() << "Invalid value for the number of haze coefficients.\n");
  return 0;
}
                          
enum {NO_REFL = 0, LAMBERT, LUNAR_LAMBERT, HAPKE, ARBITRARY_MODEL, CHARON};

// computes the Lambertian reflectance model (cosine of the light
// direction and the normal to the Moon) Vector3 sunpos: the 3D
// coordinates of the Sun relative to the center of the Moon Vector2
// lon_lat is a 2D vector. First element is the longitude and the
// second the latitude.
//author Ara Nefian
double
computeLambertianReflectanceFromNormal(Vector3 sunPos, Vector3 xyz,
                                       Vector3 normal) {
  double reflectance;
  Vector3 sunDirection = normalize(sunPos-xyz);

  reflectance = sunDirection[0]*normal[0] + sunDirection[1]*normal[1] + sunDirection[2]*normal[2];

  return reflectance;
}


double computeLunarLambertianReflectanceFromNormal(Vector3 const& sunPos,
                                                   Vector3 const& viewPos,
                                                   Vector3 const& xyz,
                                                   Vector3 const& normal,
                                                   double phaseCoeffC1,
                                                   double phaseCoeffC2,
                                                   double & alpha,
                                                   const double * reflectance_model_coeffs) {
  double reflectance;
  double L;

  double len = dot_prod(normal, normal);
  if (abs(len - 1.0) > 1.0e-4){
    std::cerr << "Error: Expecting unit normal in the reflectance computation, in "
              << __FILE__ << " at line " << __LINE__ << std::endl;
    exit(1);
  }

  //compute /mu_0 = cosine of the angle between the light direction and the surface normal.
  //sun coordinates relative to the xyz point on the Moon surface
  Vector3 sunDirection = normalize(sunPos-xyz);
  double mu_0 = dot_prod(sunDirection, normal);

  //double tol = 0.3;
  //if (mu_0 < tol){
  //  // Sun is too low, reflectance is too close to 0, the albedo will be inaccurate
  //  return 0.0;
  // }

  //compute  /mu = cosine of the angle between the viewer direction and the surface normal.
  //viewer coordinates relative to the xyz point on the Moon surface
  Vector3 viewDirection = normalize(viewPos-xyz);
  double mu = dot_prod(viewDirection,normal);

  //compute the phase angle (alpha) between the viewing direction and the light source direction
  double deg_alpha;
  double cos_alpha;

  cos_alpha = dot_prod(sunDirection, viewDirection);
  if ((cos_alpha > 1)||(cos_alpha< -1)){
    printf("cos_alpha error\n");
  }

  alpha     = acos(cos_alpha);  // phase angle in radians
  deg_alpha = alpha*180.0/M_PI; // phase angle in degrees

  //printf("deg_alpha = %f\n", deg_alpha);

  //Bob Gaskell's model
  //L = exp(-deg_alpha/60.0);

  //Alfred McEwen's model
  double O = reflectance_model_coeffs[0]; // 1
  double A = reflectance_model_coeffs[1]; //-0.019;
  double B = reflectance_model_coeffs[2]; // 0.000242;//0.242*1e-3;
  double C = reflectance_model_coeffs[3]; // -0.00000146;//-1.46*1e-6;

  L = O + A*deg_alpha + B*deg_alpha*deg_alpha + C*deg_alpha*deg_alpha*deg_alpha;
 
  //printf(" deg_alpha = %f, L = %f\n", deg_alpha, L);

  //if (mu_0 < 0.0){
  //  return 0.0;
  // }

  //  if (mu < 0.0){ //emission angle is > 90
  //  mu = 0.0;
  //}

  //if (mu_0 + mu == 0){
  //  //printf("negative reflectance\n");
  //  return 0.0;
  //}
  //else{
  reflectance = 2*L*mu_0/(mu_0+mu) + (1-L)*mu_0;
  //}
  
  //if (mu < 0 || mu_0 < 0 || mu_0 + mu <= 0 ||  reflectance <= 0 || reflectance != reflectance){
  if (mu_0 + mu == 0 || reflectance != reflectance){
    return 0.0;
  }

  // Attempt to compensate for points on the terrain being too bright
  // if the sun is behind the spacecraft as seen from those points.

  //reflectance *= std::max(0.4, exp(-alpha*alpha));
  reflectance *= ( exp(-phaseCoeffC1*alpha) + phaseCoeffC2 );

  return reflectance;
}

// Hapke's model. See: An Experimental Study of Light Scattering by Large,
// Irregular Particles Audrey F. McGuire, Bruce W. Hapke. 1995. The reflectance
// used is R(g), in equation above Equation 21. The p(g) function is given by
// Equation (14), yet this one uses an old convention. The updated p(g) is given
// in: Spectrophotometric properties of materials observed by Pancam on the Mars
// Exploration Rovers: 1. Spirit. JR Johnson, 2006. We Use the two-term p(g),
// and the parameter c, not c'=1-c. We also use the values of w(=omega), b, and
// c from that table.

// Note that we use the updated Hapke model, having the term B(g). This one is
// given in "Modeling spectral and bidirectional reflectance", Jacquemoud, 1992.
// It has the params B0 and h. The ultimate reference is probably Hapke, 1986,
// having all pieces in one place, but that one is not available. 

// We use mostly the parameter values for omega, b, c, B0 and h from: Surface
// reflectance of Mars observed by CRISM/MRO: 2. Estimation of surface
// photometric properties in Gusev Crater and Meridiani Planum by J. Fernando.
// See equations (1), (2) and (4) in that paper.

// Example values for the params: w=omega=0.68, b=0.17, c=0.62, B0=0.52, h=0.52.

// We don't use equation (3) from that paper, we use instead what they call
// the formula H93, which is the H(x) from McGuire and Hapke 1995 mentioned
// above. See the complete formulas below.

// The Fernando paper has a factor S, which is not present in the 1992
// Jacquemoud paper, so we don't use it either here.
double computeHapkeReflectanceFromNormal(Vector3 const& sunPos,
                                         Vector3 const& viewPos,
                                         Vector3 const& xyz,
                                         Vector3 const& normal,
                                         double phaseCoeffC1,
                                         double phaseCoeffC2,
                                         double & alpha,
                                         const double * reflectance_model_coeffs) {

  double len = dot_prod(normal, normal);
  if (abs(len - 1.0) > 1.0e-4){
    std::cerr << "Error: Expecting unit normal in the reflectance computation, in "
              << __FILE__ << " at line " << __LINE__ << std::endl;
    exit(1);
  }

  //compute mu_0 = cosine of the angle between the light direction and the surface normal.
  //sun coordinates relative to the xyz point on the Moon surface
  Vector3 sunDirection = normalize(sunPos-xyz);
  double mu_0 = dot_prod(sunDirection, normal);

  //compute mu = cosine of the angle between the viewer direction and the surface normal.
  //viewer coordinates relative to the xyz point on the Moon surface
  Vector3 viewDirection = normalize(viewPos-xyz);
  double mu = dot_prod(viewDirection,normal);

  //compute the phase angle (g) between the viewing direction and the light source direction
  // in radians
  double cos_g = dot_prod(sunDirection, viewDirection);
  double g = acos(cos_g);  // phase angle in radians

  // Hapke params
  double omega = std::abs(reflectance_model_coeffs[0]); // also known as w
  double b     = std::abs(reflectance_model_coeffs[1]);
  double c     = std::abs(reflectance_model_coeffs[2]);
  // The older Hapke model lacks the B0 and h terms
  double B0    = std::abs(reflectance_model_coeffs[3]);
  double h     = std::abs(reflectance_model_coeffs[4]);   

  double J = 1.0; // does not matter, we'll factor out the constant scale as camera exposures anyway
  
  // The P(g) term
  double Pg 
    = (1.0 - c) * (1.0 - b*b) / pow(1.0 + 2.0*b*cos_g + b*b, 1.5)
    + c         * (1.0 - b*b) / pow(1.0 - 2.0*b*cos_g + b*b, 1.5);
    
  // The B(g) term
  double Bg = B0 / ( 1.0 + (1.0/h)*tan(g/2.0) );

  double H_mu0 = (1.0 + 2*mu_0) / (1.0 + 2*mu_0 * sqrt(1.0 - omega));
  double H_mu  = (1.0 + 2*mu  ) / (1.0 + 2*mu   * sqrt(1.0 - omega));

  // The reflectance
  double R = (J*omega/4.0/M_PI) * ( mu_0/(mu_0+mu) ) * ( (1.0 + Bg)*Pg + H_mu0*H_mu - 1.0 );
  
  return R;
}

// Use the following model:
// Reflectance = f(alpha) * A * mu_0 /(mu_0 + mu) + (1-A) * mu_0
// The value of A is either 1 (the so-called lunar-model), or A=0.7.
// f(alpha) = 0.63.
double computeCharonReflectanceFromNormal(Vector3 const& sunPos,
                                          Vector3 const& viewPos,
                                          Vector3 const& xyz,
                                          Vector3 const& normal,
                                          double phaseCoeffC1,
                                          double phaseCoeffC2,
                                          double & alpha,
                                          const double * reflectance_model_coeffs) {

  double len = dot_prod(normal, normal);
  if (abs(len - 1.0) > 1.0e-4){
    std::cerr << "Error: Expecting unit normal in the reflectance computation, in "
              << __FILE__ << " at line " << __LINE__ << std::endl;
    exit(1);
  }

  //compute mu_0 = cosine of the angle between the light direction and the surface normal.
  //sun coordinates relative to the xyz point on the Moon surface
  Vector3 sunDirection = normalize(sunPos-xyz);
  double mu_0 = dot_prod(sunDirection, normal);

  //compute mu = cosine of the angle between the viewer direction and the surface normal.
  //viewer coordinates relative to the xyz point on the Moon surface
  Vector3 viewDirection = normalize(viewPos-xyz);
  double mu = dot_prod(viewDirection,normal);

  // Charon model params
  double A       = std::abs(reflectance_model_coeffs[0]); // albedo 
  double f_alpha = std::abs(reflectance_model_coeffs[1]); // phase function 

  double reflectance = f_alpha*A*mu_0 / (mu_0 + mu) + (1.0 - A)*mu_0;
  
  if (mu_0 + mu == 0 || reflectance != reflectance){
    return 0.0;
  }

  return reflectance;
}

double computeArbitraryLambertianReflectanceFromNormal(Vector3 const& sunPos,
                                                       Vector3 const& viewPos,
                                                       Vector3 const& xyz,
                                                       Vector3 const& normal,
                                                       double phaseCoeffC1,
                                                       double phaseCoeffC2,
                                                       double & alpha,
                                                       const double * reflectance_model_coeffs) {
  double reflectance;

  double len = dot_prod(normal, normal);
  if (abs(len - 1.0) > 1.0e-4){
    std::cerr << "Error: Expecting unit normal in the reflectance computation, in "
              << __FILE__ << " at line " << __LINE__ << std::endl;
    exit(1);
  }

  //compute /mu_0 = cosine of the angle between the light direction and the surface normal.
  //sun coordinates relative to the xyz point on the Moon surface
  //Vector3 sunDirection = -normalize(sunPos-xyz);
  Vector3 sunDirection = normalize(sunPos-xyz);
  double mu_0 = dot_prod(sunDirection, normal);

  //double tol = 0.3;
  //if (mu_0 < tol){
  //  // Sun is too low, reflectance is too close to 0, the albedo will be inaccurate
  //  return 0.0;
  // }

  //compute  /mu = cosine of the angle between the viewer direction and the surface normal.
  //viewer coordinates relative to the xyz point on the Moon surface
  Vector3 viewDirection = normalize(viewPos-xyz);
  double mu = dot_prod(viewDirection,normal);

  //compute the phase angle (alpha) between the viewing direction and the light source direction
  double deg_alpha;
  double cos_alpha;

  cos_alpha = dot_prod(sunDirection,viewDirection);
  if ((cos_alpha > 1)||(cos_alpha< -1)){
    printf("cos_alpha error\n");
  }

  alpha     = acos(cos_alpha);  // phase angle in radians
  deg_alpha = alpha*180.0/M_PI; // phase angle in degrees

  //printf("deg_alpha = %f\n", deg_alpha);

  //Bob Gaskell's model
  //L = exp(-deg_alpha/60.0);

  //Alfred McEwen's model
  double O1 = reflectance_model_coeffs[0]; // 1
  double A1 = reflectance_model_coeffs[1]; // -0.019;
  double B1 = reflectance_model_coeffs[2]; // 0.000242;//0.242*1e-3;
  double C1 = reflectance_model_coeffs[3]; // -0.00000146;//-1.46*1e-6;
  double D1 = reflectance_model_coeffs[4]; 
  double E1 = reflectance_model_coeffs[5]; 
  double F1 = reflectance_model_coeffs[6]; 
  double G1 = reflectance_model_coeffs[7]; 

  double O2 = reflectance_model_coeffs[8];  // 1
  double A2 = reflectance_model_coeffs[9];  // -0.019;
  double B2 = reflectance_model_coeffs[10]; // 0.000242;//0.242*1e-3;
  double C2 = reflectance_model_coeffs[11]; // -0.00000146;//-1.46*1e-6;
  double D2 = reflectance_model_coeffs[12]; 
  double E2 = reflectance_model_coeffs[13]; 
  double F2 = reflectance_model_coeffs[14]; 
  double G2 = reflectance_model_coeffs[15]; 
  
  double L1 = O1 + A1*deg_alpha + B1*deg_alpha*deg_alpha + C1*deg_alpha*deg_alpha*deg_alpha;
  double K1 = D1 + E1*deg_alpha + F1*deg_alpha*deg_alpha + G1*deg_alpha*deg_alpha*deg_alpha;
  if (K1 == 0) K1 = 1;
    
  double L2 = O2 + A2*deg_alpha + B2*deg_alpha*deg_alpha + C2*deg_alpha*deg_alpha*deg_alpha;
  double K2 = D2 + E2*deg_alpha + F2*deg_alpha*deg_alpha + G2*deg_alpha*deg_alpha*deg_alpha;
  if (K2 == 0) K2 = 1;
  
  //printf(" deg_alpha = %f, L = %f\n", deg_alpha, L);

  //if (mu_0 < 0.0){
  //  return 0.0;
  // }

  //  if (mu < 0.0){ //emission angle is > 90
  //  mu = 0.0;
  //}

  //if (mu_0 + mu == 0){
  //  //printf("negative reflectance\n");
  //  return 0.0;
  //}
  //else{
  reflectance = 2*L1*mu_0/(mu_0+mu)/K1 + (1-L2)*mu_0/K2;
  //}
  
  //if (mu < 0 || mu_0 < 0 || mu_0 + mu <= 0 ||  reflectance <= 0 || reflectance != reflectance){
  if (mu_0 + mu == 0 || reflectance != reflectance){
    return 0.0;
  }

  // Attempt to compensate for points on the terrain being too bright
  // if the sun is behind the spacecraft as seen from those points.

  //reflectance *= std::max(0.4, exp(-alpha*alpha));
  reflectance *= ( exp(-phaseCoeffC1*alpha) + phaseCoeffC2 );

  return reflectance;
}

double ComputeReflectance(Vector3 const& cameraPosition,
                          Vector3 const& normal, Vector3 const& xyz,
                          ModelParams const& input_img_params,
                          GlobalParams const& global_params,
                          double & phase_angle,
                          const double * reflectance_model_coeffs) {
  double input_img_reflectance;

  switch ( global_params.reflectanceType )
    {
    case LUNAR_LAMBERT:
      input_img_reflectance
        = computeLunarLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                      cameraPosition,
                                                      xyz,  normal,
                                                      global_params.phaseCoeffC1,
                                                      global_params.phaseCoeffC2,
                                                      phase_angle, // output
                                                      reflectance_model_coeffs);
      break;
    case ARBITRARY_MODEL:
      input_img_reflectance
        = computeArbitraryLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                          cameraPosition,
                                                          xyz,  normal,
                                                          global_params.phaseCoeffC1,
                                                          global_params.phaseCoeffC2,
                                                          phase_angle, // output
                                                          reflectance_model_coeffs);
      break;
    case HAPKE:
      input_img_reflectance
        = computeHapkeReflectanceFromNormal(input_img_params.sunPosition,
                                            cameraPosition,
                                            xyz,  normal,
                                            global_params.phaseCoeffC1,
                                            global_params.phaseCoeffC2,
                                            phase_angle, // output
                                            reflectance_model_coeffs);
      break;
    case CHARON:
      input_img_reflectance
        = computeCharonReflectanceFromNormal(input_img_params.sunPosition,
                                             cameraPosition,
                                             xyz,  normal,
                                             global_params.phaseCoeffC1,
                                             global_params.phaseCoeffC2,
                                             phase_angle, // output
                                             reflectance_model_coeffs);
      break;
    case LAMBERT:
      input_img_reflectance
        = computeLambertianReflectanceFromNormal(input_img_params.sunPosition,
                                                 xyz, normal);
      break;

    default:
      input_img_reflectance = 1;
    }

  return input_img_reflectance;
}

// Use this struct to keep track of height errors.
struct HeightErrEstim {

  HeightErrEstim(int num_cols, int num_rows, int num_height_samples_in,
                 double max_height_error_in, double nodata_height_val_in,
                 ImageView<double> * albedo_in,
                 Options * opt_in) {
    num_height_samples = num_height_samples_in; // TODO(oalexan1): This must be a parameter
    max_height_error   = max_height_error_in;   // TODO(oalexan1): This must be a parameter
    nodata_height_val  = nodata_height_val_in;
    
    albedo = albedo_in;
    opt = opt_in;
    
    image_iter = 0; // will be modified later

    height_error_vec.set_size(num_cols, num_rows);
    for (int col = 0; col < num_cols; col++) {
      for (int row = 0; row < num_rows; row++) {
        height_error_vec(col, row)[0] = -max_height_error;
        height_error_vec(col, row)[1] =  max_height_error;
      }
    }
  }
  
  int num_height_samples;
  ImageView<double> * albedo;
  Options * opt;
  ImageView<Vector2> height_error_vec;
  int image_iter;
  double max_height_error;
  double nodata_height_val;
};

// Use this struct to keep track of slope errors.
struct SlopeErrEstim {

  SlopeErrEstim(int num_cols, int num_rows, int num_a_samples_in, int num_b_samples_in,
                ImageView<double> * albedo_in, Options * opt_in) {
    num_a_samples = num_a_samples_in;
    num_b_samples = num_b_samples_in;
    albedo = albedo_in;
    opt = opt_in;

    image_iter = 0; // will be modified later

    // The maximum possible deviation from the normal in degrees
    max_angle = 90.0; 
    
    slope_errs.resize(num_cols);
    for (int col = 0; col < num_cols; col++) {
      slope_errs[col].resize(num_rows);
      for (int row = 0; row < num_rows; row++) {
        slope_errs[col][row].resize(num_b_samples, max_angle);
      }
    }
  }
  
  int num_a_samples, num_b_samples;
  ImageView<double> * albedo;
  Options * opt;
  std::vector< std::vector< std::vector<double> > > slope_errs;
  int image_iter;
  double max_angle;
};

// Given the normal (slope) to the SfS DEM, find how different
// a slope can be from this before the computed intensity
// due to that slope is bigger than max_intensity_err.
void estimateSlopeError(Vector3 const& cameraPosition,
                        Vector3 const& normal, Vector3 const& xyz,
                        ModelParams const& local_model_params,
                        GlobalParams const& global_params,
                        const double * reflectance_model_coeffs,
                        double meas_intensity,
                        double max_intensity_err,
                        int col, int row, int image_iter,
                        Options & opt,
                        ImageView<double> & albedo,
                        SlopeErrEstim * slopeErrEstim){
  
  // Find the angle u from the normal to the z axis, and the angle v
  // from the x axis to the projection of the normal in the xy plane.
  double u = acos(normal[2]);
  double v = 0.0;
  if (normal[0] != 0.0 || normal[1] != 0.0) 
    v = atan2(normal[1], normal[0]);

  double cv = cos(v), sv = sin(v), cu = cos(u), su = sin(u);
  Vector3 n(cv*su, sv*su, cu);

  // Sanity check, these angles should give us back the normal
  if (norm_2(normal - n) > 1e-8) 
    vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );
    
  // Find the rotation R that transforms the vector (0, 0, 1) to the normal
  vw::Matrix3x3 R1, R2, R;
  
  R1[0][0] = cv;  R1[0][1] = -sv; R1[0][2] = 0;
  R1[1][0] = sv;  R1[1][1] =  cv; R1[1][2] = 0;
  R1[2][0] = 0;   R1[2][1] =  0;  R1[2][2] = 1;
  
  R2[0][0] = cu;  R2[0][1] =  0;  R2[0][2] = su;
  R2[1][0] = 0;   R2[1][1] =  1;  R2[1][2] = 0;
  R2[2][0] = -su; R2[2][1] =  0;  R2[2][2] = cu;

  R = R1 * R2;

  // We must have R * n0 = n
  Vector3 n0(0, 0, 1);
  if (norm_2(R*n0 - n) > 1e-8) 
    vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );
  
  int num_a_samples = slopeErrEstim->num_a_samples;
  int num_b_samples = slopeErrEstim->num_b_samples;

  int num_cols = slopeErrEstim->slope_errs.size();
  int num_rows = slopeErrEstim->slope_errs[0].size();
  int num_b_samples2 = slopeErrEstim->slope_errs[0][0].size();

  if (num_b_samples != num_b_samples2)
    vw_throw( LogicErr()
              << "Book-keeping failure in estimating the slope error!\n");
  
  // Sample the set of unit vectors w which make the angle 'a' with
  // the normal. For that, start with w having angle 'a' with the z
  // axis, and angle 'b' between the projection of w onto the xy plane
  // and the x axis. Then apply the rotation R to it which will make
  // the angle between w and the normal be 'a'. By varying 'b' we will
  // sample all such angles.
  double deg2rad = M_PI/180.0;
  for (int b_iter = 0; b_iter < num_b_samples; b_iter++) {
    
    double b = 360.0 * double(b_iter)/num_b_samples;
    double cb = cos(deg2rad * b), sb = sin(deg2rad * b);
    
    for (int a_iter = 0; a_iter < num_a_samples; a_iter++) {
      
      double a = 90.0 * double(a_iter)/num_a_samples;

      if (slopeErrEstim->slope_errs[col][row][b_iter] < a) {
        // We already determined that the slope error can't be as big as
        // a, so there is no point to explore bigger angles
        break;
      }

      double ca = cos(deg2rad * a), sa = sin(deg2rad * a);

      Vector3 w(cb*sa, sb*sa, ca);
      w = R*w;

      // Compute here dot product from w to n. Should be cos(a) for all b.
      double prod = dot_prod(w, normal);
      if (std::abs(prod - ca) > 1e-8)
        vw_throw( LogicErr() << "Book-keeping error in slope estimation.\n" );

      // Compute the reflectance with the given normal
      double phase_angle = 0.0;
      PixelMask<double> reflectance = ComputeReflectance(cameraPosition,
                                                         w, xyz, local_model_params,
                                                         global_params, phase_angle,
                                                         reflectance_model_coeffs);
      reflectance.validate();

      double comp_intensity = albedo(col, row) *
        nonlin_reflectance(reflectance, opt.image_exposures_vec[image_iter],
                           opt.steepness_factor,
                           &opt.image_haze_vec[image_iter][0], opt.num_haze_coeffs);

      if (std::abs(comp_intensity - meas_intensity) > max_intensity_err) {
        // We exceeded the error budget, hence this is an upper bound on the slope
        slopeErrEstim->slope_errs[col][row][b_iter] = a;
        break;
      }
      
    }
  }
}

// Given the normal (height) to the SfS DEM, find how different
// a height can be from this before the computed intensity
// due to that height is bigger than max_intensity_err.
void estimateHeightError(ImageView<double> const& dem,
                         cartography::GeoReference const& geo,
                         Vector3 const& cameraPosition,
                         ModelParams const& local_model_params,
                         GlobalParams const& global_params,
                         const double * reflectance_model_coeffs,
                         double meas_intensity,
                         double max_intensity_err,
                         int col, int row, int image_iter,
                         Options & opt,
                         ImageView<double> & albedo,
                         HeightErrEstim * heightErrEstim){

  // Look at the neighbors
  int cols[] = {col - 1, col,     col,     col + 1};
  int rows[] = {row,     row - 1, row + 1, row};
  
  for (int it = 0; it < 4; it++) {

    int colx = cols[it], rowx = rows[it];

    // Can't be at edges as need to compute normals
    if (colx <= 0 || rowx <= 0 || colx >= dem.cols() - 1 || rowx >= dem.rows() - 1)
      continue;

    // Perturb the height down and up
    for (int sign = -1; sign <= 1; sign += 2) {
      for (int height_it = 0; height_it < heightErrEstim->num_height_samples; height_it++) {
        double dh = sign * heightErrEstim->max_height_error
          * double(height_it)/double(heightErrEstim->num_height_samples);

        if (sign == -1) {
          if (dh < heightErrEstim->height_error_vec(colx, rowx)[0]) {
            // We already determined dh can't go as low, so stop here
            break;
          }
        } else if (sign == 1) {
          if (dh > heightErrEstim->height_error_vec(colx, rowx)[1]) {
            break;
          }
        }

        // Determine where to add the dh. Recall that we compute the intensity
        // at (col, row), while perturbing the dem height at (colx, rowx)
        double left_dh = 0, center_dh = 0, right_dh = 0, bottom_dh = 0, top_dh = 0;
        if      (colx == col - 1 && rowx == row    ) left_dh   = dh; 
        else if (colx == col     && rowx == row    ) center_dh = dh; // won't be reached
        else if (colx == col + 1 && rowx == row    ) right_dh  = dh; 
        else if (colx == col     && rowx == row + 1) bottom_dh = dh; 
        else if (colx == col     && rowx == row - 1) top_dh    = dh; 
        
        double left_h   = dem(col - 1, row)     + left_dh;
        double center_h = dem(col,     row)     + center_dh;
        double right_h  = dem(col + 1, row)     + right_dh;
        double bottom_h = dem(col,     row + 1) + bottom_dh;
        double top_h    = dem(col,     row - 1) + top_dh;

        // TODO(oalexan1): Make this into a function to avoid code duplication!
        
        // The xyz position at the center grid point
        Vector2 lonlat = geo.pixel_to_lonlat(Vector2(col, row));
        double h = center_h;
        Vector3 lonlat3 = Vector3(lonlat(0), lonlat(1), h);
        Vector3 base = geo.datum().geodetic_to_cartesian(lonlat3);

        // The xyz position at the left grid point
        lonlat = geo.pixel_to_lonlat(Vector2(col-1, row));
        h = left_h;
        lonlat3 = Vector3(lonlat(0), lonlat(1), h);
        Vector3 left = geo.datum().geodetic_to_cartesian(lonlat3);

        // The xyz position at the right grid point
        lonlat = geo.pixel_to_lonlat(Vector2(col+1, row));
        h = right_h;
        lonlat3 = Vector3(lonlat(0), lonlat(1), h);
        Vector3 right = geo.datum().geodetic_to_cartesian(lonlat3);

        // The xyz position at the bottom grid point
        lonlat = geo.pixel_to_lonlat(Vector2(col, row+1));
        h = bottom_h;
        lonlat3 = Vector3(lonlat(0), lonlat(1), h);
        Vector3 bottom = geo.datum().geodetic_to_cartesian(lonlat3);

        // The xyz position at the top grid point
        lonlat = geo.pixel_to_lonlat(Vector2(col, row-1));
        h = top_h;
        lonlat3 = Vector3(lonlat(0), lonlat(1), h);
        Vector3 top = geo.datum().geodetic_to_cartesian(lonlat3);

        // four-point normal (centered)
        Vector3 dx = right - left;
        Vector3 dy = bottom - top;

        Vector3 normal = -normalize(cross_prod(dx, dy)); // so normal points up

        double phase_angle = 0.0;
        PixelMask<double> reflectance = ComputeReflectance(cameraPosition,
                                                           normal, base, local_model_params,
                                                           global_params, phase_angle,
                                                           reflectance_model_coeffs);
        reflectance.validate();

        double comp_intensity = albedo(col, row) *
          nonlin_reflectance(reflectance, opt.image_exposures_vec[image_iter],
                             opt.steepness_factor,
                             &opt.image_haze_vec[image_iter][0], opt.num_haze_coeffs);

        if (std::abs(comp_intensity - meas_intensity) > max_intensity_err) {
          // We exceeded the error budget, record the dh at which it happens
          if (sign == -1) {
            heightErrEstim->height_error_vec(colx, rowx)[0] = dh;
          } else if (sign == 1) {
            heightErrEstim->height_error_vec(colx, rowx)[1] = dh;
          }
                
          break;
        }

      }
    }
  }
}

bool computeReflectanceAndIntensity(double left_h, double center_h, double right_h,
                                    double bottom_h, double top_h,
                                    bool use_pq, double p, double q, // dem partial derivatives
                                    int col, int row,
                                    ImageView<double>         const& dem,
                                    cartography::GeoReference const& geo,
                                    bool model_shadows,
                                    double max_dem_height,
                                    double gridx, double gridy,
                                    ModelParams  const & model_params,
                                    GlobalParams const & global_params,
                                    BBox2i       const & crop_box,
                                    MaskedImgT   const & image,
                                    DoubleImgT   const & blend_weight,
                                    CameraModel  const * camera,
                                    double       const * scaled_sun_posn,
                                    PixelMask<double>  & reflectance,
                                    PixelMask<double>  & intensity,
                                    double             & ground_weight,
                                    const double       * reflectance_model_coeffs,
                                    SlopeErrEstim      * slopeErrEstim = NULL,
                                    HeightErrEstim     * heightErrEstim = NULL) {

  // Set output values
  reflectance = 0.0; reflectance.invalidate();
  intensity   = 0.0; intensity.invalidate();
  ground_weight = 0.0;
  
  if (col >= dem.cols() - 1 || row >= dem.rows() - 1) return false;
  if (crop_box.empty()) return false;

  if (use_pq) {
    // p is defined as (right_h - left_h)/(2*gridx)
    // so, also, p = (right_h - center_h)/gridx
    // Hence, we get the formulas below in terms of p and q.
    right_h  = center_h + gridx*p;
    left_h   = center_h - gridx*p;
    top_h    = center_h + gridy*q;
    bottom_h = center_h - gridy*q;
  }

  // The xyz position at the center grid point
  Vector2 lonlat = geo.pixel_to_lonlat(Vector2(col, row));
  double h = center_h;
  Vector3 lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 base = geo.datum().geodetic_to_cartesian(lonlat3);

  // The xyz position at the left grid point
  lonlat = geo.pixel_to_lonlat(Vector2(col-1, row));
  h = left_h;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 left = geo.datum().geodetic_to_cartesian(lonlat3);

  // The xyz position at the right grid point
  lonlat = geo.pixel_to_lonlat(Vector2(col+1, row));
  h = right_h;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 right = geo.datum().geodetic_to_cartesian(lonlat3);

  // The xyz position at the bottom grid point
  lonlat = geo.pixel_to_lonlat(Vector2(col, row+1));
  h = bottom_h;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 bottom = geo.datum().geodetic_to_cartesian(lonlat3);

  // The xyz position at the top grid point
  lonlat = geo.pixel_to_lonlat(Vector2(col, row-1));
  h = top_h;
  lonlat3 = Vector3(lonlat(0), lonlat(1), h);
  Vector3 top = geo.datum().geodetic_to_cartesian(lonlat3);

  // four-point normal (centered)
  Vector3 dx = right - left;
  Vector3 dy = bottom - top;

  Vector3 normal = -normalize(cross_prod(dx, dy)); // so normal points up

  ModelParams local_model_params = model_params;

  // Update the sun position using the scaled sun position variable 
  for (int it = 0; it < 3; it++) 
    local_model_params.sunPosition[it] = scaled_sun_posn[it] * model_params.sunPosition[it]; 
    
  // Update the camera position for the given pixel (camera position
  // is pixel-dependent for linescan cameras).
  Vector2 pix;
  Vector3 cameraPosition;
  try {
    pix = camera->point_to_pixel(base);
    
    // Need camera center only for Lunar Lambertian
    if (global_params.reflectanceType != LAMBERT)
      cameraPosition = camera->camera_center(pix);
    
  } catch(...){
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }
  
  double phase_angle = 0.0;
  reflectance = ComputeReflectance(cameraPosition,
                                   normal, base, local_model_params,
                                   global_params, phase_angle,
                                   reflectance_model_coeffs);
  reflectance.validate();


  // Since our image is cropped
  pix -= crop_box.min();

  // Check for out of range
  if (pix[0] < 0 || pix[0] >= image.cols() - 1 || pix[1] < 0 || pix[1] >= image.rows() - 1) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }

  InterpolationView<EdgeExtensionView<MaskedImgT, ConstantEdgeExtension>, BilinearInterpolation>
    interp_image = interpolate(image, BilinearInterpolation(),
                               ConstantEdgeExtension());
  intensity = interp_image(pix[0], pix[1]); // this interpolates

  if (g_blend_weight_is_ground_weight) {
    if (blend_weight.cols() != dem.cols() || blend_weight.rows() != dem.rows()) 
      vw::vw_throw(vw::ArgumentErr() << "Ground weight must have the same size as the DEM.\n");
    ground_weight = blend_weight(col, row);
  } else {
    InterpolationView<EdgeExtensionView<DoubleImgT, ConstantEdgeExtension>, BilinearInterpolation>
      interp_weight = interpolate(blend_weight, BilinearInterpolation(),
                                  ConstantEdgeExtension());
    if (blend_weight.cols() > 0 && blend_weight.rows() > 0) // The weight may not exist
      ground_weight = interp_weight(pix[0], pix[1]); // this interpolates
    else
      ground_weight = 1.0;
  }
  
  // Note that we allow negative reflectance. It will hopefully guide
  // the SfS solution the right way.
  if (!is_valid(intensity)) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    ground_weight = 0.0;
    return false;
  }

  if (model_shadows) {
    bool inShadow = asp::isInShadow(col, row, local_model_params.sunPosition,
                                    dem, max_dem_height, gridx, gridy,
                                    geo);

    if (inShadow) {
      // The reflectance is valid, it is just zero
      reflectance = 0;
      reflectance.validate();
    }
  }

  if (slopeErrEstim != NULL && is_valid(intensity) && is_valid(reflectance)) {
    
    int image_iter = slopeErrEstim->image_iter;
    Options & opt = *slopeErrEstim->opt; // alias
    ImageView<double> & albedo = *slopeErrEstim->albedo; // alias
    double comp_intensity = albedo(col, row) *
      nonlin_reflectance(reflectance, opt.image_exposures_vec[image_iter],
                         opt.steepness_factor,
                         &opt.image_haze_vec[image_iter][0], opt.num_haze_coeffs);

    // We use twice the discrepancy between the computed and measured intensity
    // as a measure for how far is overall the computed intensity allowed
    // to diverge from the measured intensity
    double max_intensity_err = 2.0 * std::abs(intensity.child() - comp_intensity);
    estimateSlopeError(cameraPosition,
                       normal, base, local_model_params,
                       global_params,
                       reflectance_model_coeffs,
                       intensity.child(),
                       max_intensity_err,
                       col, row, image_iter,
                       opt, albedo,
                       slopeErrEstim);
  }
  
  if (heightErrEstim != NULL && is_valid(intensity) && is_valid(reflectance)) {
    
    int image_iter = heightErrEstim->image_iter;
    Options & opt = *heightErrEstim->opt; // alias
    ImageView<double> & albedo = *heightErrEstim->albedo; // alias
    double comp_intensity = albedo(col, row) *
      nonlin_reflectance(reflectance, opt.image_exposures_vec[image_iter],
                         opt.steepness_factor,
                         &opt.image_haze_vec[image_iter][0], opt.num_haze_coeffs);
    
    // We use twice the discrepancy between the computed and measured intensity
    // as a measure for how far is overall the computed intensity allowed
    // to diverge from the measured intensity
    double max_intensity_err = 2.0 * std::abs(intensity.child() - comp_intensity);

    estimateHeightError(dem, geo,  
                        cameraPosition, local_model_params,  global_params,  
                        reflectance_model_coeffs, intensity.child(),  
                        max_intensity_err, col, row, image_iter, opt,  albedo,  
                        heightErrEstim);
  }
  
  return true;
}

void computeReflectanceAndIntensity(ImageView<double> const& dem,
                                    ImageView<Vector2> const& pq,
                                    cartography::GeoReference const& geo,
                                    bool model_shadows,
                                    double & max_dem_height, // alias
                                    double gridx, double gridy,
                                    int sample_col_rate, int sample_row_rate,
                                    ModelParams const& model_params,
                                    GlobalParams const& global_params,
                                    BBox2i const& crop_box,
                                    MaskedImgT const  & image,
                                    DoubleImgT const  & blend_weight,
                                    CameraModel const * camera,
                                    double     const  * scaled_sun_posn,
                                    ImageView<PixelMask<double>> & reflectance,
                                    ImageView<PixelMask<double>> & intensity,
                                    ImageView<double>            & ground_weight,
                                    const double   * reflectance_model_coeffs,
                                    SlopeErrEstim  * slopeErrEstim = NULL,
                                    HeightErrEstim * heightErrEstim = NULL) {
  
  // Update max_dem_height
  max_dem_height = -std::numeric_limits<double>::max();
  if (model_shadows) {
    for (int col = 0; col < dem.cols(); col += sample_col_rate) {
      for (int row = 0; row < dem.rows(); row += sample_row_rate) {
        if (dem(col, row) > max_dem_height) {
          max_dem_height = dem(col, row);
        }
      }
    }
    vw_out() << "Maximum DEM height: " << max_dem_height << std::endl;
  }
  
  // Init the reflectance and intensity as invalid. Do it at all grid
  // points, not just where we sample, to ensure that these quantities
  // are fully initialized.
  reflectance.set_size(dem.cols(), dem.rows());
  intensity.set_size(dem.cols(), dem.rows());
  ground_weight.set_size(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      reflectance(col, row).invalidate();
      intensity(col, row).invalidate();
      ground_weight(col, row) = 0.0;
    }
  }

  bool use_pq = (pq.cols() > 0 && pq.rows() > 0);
  for (int col = 1; col < dem.cols() - 1; col += sample_col_rate) {
    for (int row = 1; row < dem.rows() - 1; row += sample_row_rate) {
      
      double pval = 0, qval = 0;
      if (use_pq) {
        pval = pq(col, row)[0];
        qval = pq(col, row)[1];
      }
      computeReflectanceAndIntensity(dem(col-1, row), dem(col, row), dem(col+1, row),
                                     dem(col, row+1), dem(col, row-1),
                                     use_pq, pval, qval,
                                     col, row, dem,  geo,
                                     model_shadows, max_dem_height,
                                     gridx, gridy,
                                     model_params, global_params,
                                     crop_box, image, blend_weight, camera,
                                     scaled_sun_posn, 
                                     reflectance(col, row), intensity(col, row),
                                     ground_weight(col, row),
                                     reflectance_model_coeffs,
                                     slopeErrEstim,
                                     heightErrEstim);
    }
  }
  
  return;
}

std::string exposure_file_name(std::string const& prefix){
  return prefix + "-exposures.txt";
}

std::string haze_file_name(std::string const& prefix){
  return prefix + "-haze.txt";
}

std::string model_coeffs_file_name(std::string const& prefix){
  return prefix + "-model_coeffs.txt";
}

// Form a finer resolution image with given dimensions from a coarse image.
// Use constant edge extension.
void interp_image(ImageView<double> const& coarse_image, double scale,
                  ImageView<double> & fine_image){

  ImageViewRef<double> coarse_interp = interpolate(coarse_image,
                                                   BicubicInterpolation(), ConstantEdgeExtension());
  for (int col = 0; col < fine_image.cols(); col++) {
    for (int row = 0; row < fine_image.rows(); row++) {
      fine_image(col, row) = coarse_interp(col*scale, row*scale);
    }
  }
}

void save_exposures(std::string const& out_prefix,
                    std::vector<std::string> const& input_images,
                    std::vector<double> const& exposures){
  std::string exposure_file = exposure_file_name(out_prefix);
  vw_out() << "Writing: " << exposure_file << std::endl;
  std::ofstream exf(exposure_file.c_str());
  exf.precision(18);
  for (size_t image_iter = 0; image_iter < exposures.size(); image_iter++)
    exf << input_images[image_iter] << " " << exposures[image_iter] << "\n";
  exf.close();
}

// Find the sun azimuth and elevation at the lon-lat position of the
// center of the DEM. The result can change depending on the DEM.
void sun_angles(Options const& opt,
                ImageView<double> const& dem, double nodata_val, GeoReference const& georef,
                boost::shared_ptr<CameraModel> cam,
                Vector3 const& sun_pos,
                double & azimuth, double &elevation){

  int cols = dem.cols(), rows = dem.rows();
  if (cols <= 0 || rows <= 0)
    vw_throw( ArgumentErr() << "Expecting a non-empty DEM.\n" );

  // Find lon-lat-height in the middle of the DEM
  Vector2 ll = georef.pixel_to_lonlat(Vector2(cols/2.0, rows/2.0));
  double height = dem(cols/2.0, rows/2.0);
  if (height == nodata_val)
    height = 0;
  Vector3 llh(ll[0], ll[1], height);

  Vector3 xyz = georef.datum().geodetic_to_cartesian(llh); // point on the planet
  Vector3 east(-xyz[1], xyz[0], 0);

  Vector3 sun_dir = sun_pos - xyz;

  //double prod = dot_prod(sun_dir, east)
  //  / sqrt ( dot_prod(east, east) * dot_prod(sun_dir, sun_dir));
  //double angle = (180.0/M_PI) * acos (prod);

  // Projection in the tangent plane
  //  Vector3 proj = sun_dir - xyz*dot_prod(sun_dir, xyz)/dot_prod(xyz, xyz);

  //prod = dot_prod(proj, east)
  //  / sqrt ( dot_prod(east, east) * dot_prod(proj, proj));

  //double angle2 = (180.0/M_PI) * acos (prod);

  // Find the sun direction in the North-East-Down coordinate system
  vw::Matrix3x3 Ned2Ecef = georef.datum().lonlat_to_ned_matrix(llh);
  Vector3 sun_dir_ned = inverse(Ned2Ecef) * sun_dir;
  
  if (sun_dir_ned[0] == 0 && sun_dir_ned[1] == 0)
    azimuth = 0;
  else
    azimuth = (180.0/M_PI) * atan2(sun_dir_ned[1], sun_dir_ned[0]);

  double L = norm_2(subvector(sun_dir_ned, 0, 2));
  elevation = (180.0/M_PI) * atan2(-sun_dir_ned[2], L);
}

// A function to invoke at every iteration of ceres.
// We need a lot of global variables to do something useful.
Options                               const * g_opt = NULL;
int                                           g_iter = -1;
std::vector<ImageView<double>>              * g_dem = NULL;
std::vector<ImageView<Vector2>>             * g_pq = NULL;
std::vector<ImageView<double>>              * g_albedo = NULL;
std::vector<cartography::GeoReference> const * g_geo = NULL;
GlobalParams                           const * g_global_params = NULL;
std::vector<ModelParams>               const * g_model_params = NULL;
std::vector<std::vector<BBox2i>>       const * g_crop_boxes = NULL;
std::vector<std::vector<MaskedImgT>>   const * g_masked_images = NULL;
std::vector<std::vector<DoubleImgT>>   const * g_blend_weights = NULL;
std::vector<std::vector<boost::shared_ptr<CameraModel>> > * g_cameras = NULL;
double                                       * g_dem_nodata_val = NULL;
float                                        * g_img_nodata_val = NULL;
std::vector<double>                          * g_exposures = NULL;
std::vector<std::vector<double>>             * g_haze = NULL;
std::vector<double>                          * g_adjustments = NULL;
std::vector<double>                          * g_scaled_sun_posns = NULL;
std::vector<double>                          * g_max_dem_height = NULL;
double                                       * g_gridx = NULL;
double                                       * g_gridy = NULL;
int                                            g_level = -1;
bool                                           g_final_iter = false;
double                                       * g_reflectance_model_coeffs = NULL; 

// When floating the camera position and orientation, multiply the
// position variables by this factor times
// opt.camera_position_step_size to give it a greater range in
// searching (it makes more sense to wiggle the camera position by say
// 1 meter than by a tiny fraction of one millimeter).
double g_position_scale_factor = 1e+6;

class SfsCallback: public ceres::IterationCallback {
public:
  virtual ceres::CallbackReturnType operator()
    (const ceres::IterationSummary& summary) {

    g_iter++;

    vw_out() << "Finished iteration: " << g_iter << std::endl;
    // callTop();

    if (!g_opt->save_computed_intensity_only)
      save_exposures(g_opt->out_prefix, g_opt->input_images, *g_exposures);

    if (g_opt->num_haze_coeffs > 0 && !g_opt->save_computed_intensity_only) {
      std::string haze_file = haze_file_name(g_opt->out_prefix);
      vw_out() << "Writing: " << haze_file << std::endl;
      std::ofstream hzf(haze_file.c_str());
      hzf.precision(18);
      for (size_t image_iter = 0; image_iter < (*g_haze).size(); image_iter++) {
        hzf << g_opt->input_images[image_iter];
        for (size_t hiter = 0; hiter < (*g_haze)[image_iter].size(); hiter++) {
          hzf << " " << (*g_haze)[image_iter][hiter];
        }
        hzf << "\n";
      }
      hzf.close();
    }
    
    std::string model_coeffs_file = model_coeffs_file_name(g_opt->out_prefix);
    if (!g_opt->save_computed_intensity_only) {
      vw_out() << "Writing: " << model_coeffs_file << std::endl;
      std::ofstream mcf(model_coeffs_file.c_str());
      mcf.precision(18);
      for (size_t coeff_iter = 0; coeff_iter < g_num_model_coeffs; coeff_iter++){
        mcf << g_reflectance_model_coeffs[coeff_iter] << " ";
      }
      mcf << "\n";
      mcf.close();
    }
    
    //vw_out() << "Model coefficients: "; 
    //for (size_t i = 0; i < g_num_model_coeffs; i++)
    //  vw_out() << g_reflectance_model_coeffs[i] << " ";
    //vw_out() << std::endl;
    
    //if (!g_opt->use_approx_adjusted_camera_models) {
    //  vw_out() << "cam adj: ";
    //  for (int s = 0; s < int((*g_adjustments).size()); s++) {
    //    vw_out() << (*g_adjustments)[s] << " ";
    // }
    // vw_out() << std::endl;
    //}
    
    //vw_out() << "scaled sun position: ";
    //for (int s = 0; s < int((*g_scaled_sun_posns).size()); s++) 
    //  vw_out() << (*g_scaled_sun_posns)[s] << " ";
    //vw_out() << std::endl;

    int num_dems = (*g_dem).size();
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
      // Apply the most recent adjustments to the cameras.
      for (size_t image_iter = 0; image_iter < (*g_masked_images)[dem_iter].size(); image_iter++) {
        if (g_opt->skip_images[dem_iter].find(image_iter) !=
            g_opt->skip_images[dem_iter].end()) continue;

        if (!g_opt->use_approx_adjusted_camera_models) {
          // When we use approx adjusted camera models we never change the adjustments,
          // hence this step is not necessary
          AdjustedCameraModel * icam
            = dynamic_cast<AdjustedCameraModel*>((*g_cameras)[dem_iter][image_iter].get());
          if (icam == NULL)
            vw_throw(ArgumentErr() << "Expecting an adjusted camera model.\n");
          Vector3 translation;
          Vector3 axis_angle;
          for (int param_iter = 0; param_iter < 3; param_iter++) {
            translation[param_iter]
              = (g_position_scale_factor*g_opt->camera_position_step_size)*
              (*g_adjustments)[6*image_iter + 0 + param_iter];
            axis_angle[param_iter] = (*g_adjustments)[6*image_iter + 3 + param_iter];
          }
          icam->set_translation(translation);
          icam->set_axis_angle_rotation(axis_angle);
        }
      }

      std::ostringstream os;
      if (!g_final_iter) {
        os << "-iter" << g_iter;
      }else{
        os << "-final";
      }

      // Note that for level 0 we don't append the level as part of
      // the filename. This way, whether we have levels or not,
      // the lowest level is always named consistently.
      if ((*g_opt).coarse_levels > 0 && g_level > 0) os << "-level" << g_level;
      if (num_dems > 1)                              os << "-clip"  << dem_iter;

      std::string iter_str = os.str();

      // The DEM with no-data where there are no valid image pixels
      ImageView<double> dem_nodata;
      if (g_opt->save_dem_with_nodata) {
        dem_nodata = ImageView<double>((*g_dem)[dem_iter].cols(), (*g_dem)[dem_iter].rows());
        fill(dem_nodata, *g_dem_nodata_val);
      }
        
      bool has_georef = true, has_nodata = true;
      TerminalProgressCallback tpc("asp", ": ");
      if ( (!g_opt->save_sparingly || g_final_iter) && !g_opt->save_computed_intensity_only ) {
        std::string out_dem_file = g_opt->out_prefix + "-DEM"
          + iter_str + ".tif";
        vw_out() << "Writing: " << out_dem_file << std::endl;
        block_write_gdal_image(out_dem_file, (*g_dem)[dem_iter], has_georef, (*g_geo)[dem_iter],
                               has_nodata, *g_dem_nodata_val,
                               *g_opt, tpc);
      }
      
      if ((!g_opt->save_sparingly || (g_final_iter && g_opt->float_albedo)) &&
          !g_opt->save_computed_intensity_only ) {
        std::string out_albedo_file = g_opt->out_prefix + "-comp-albedo"
          + iter_str + ".tif";
        vw_out() << "Writing: " << out_albedo_file << std::endl;
        block_write_gdal_image(out_albedo_file, (*g_albedo)[dem_iter], has_georef,
                               (*g_geo)[dem_iter],
                               has_nodata, *g_dem_nodata_val,
                               *g_opt, tpc);
      }

      // Print reflectance and other things
      for (size_t image_iter = 0; image_iter < (*g_masked_images)[dem_iter].size(); image_iter++) {

        if (g_opt->skip_images[dem_iter].find(image_iter) !=
            g_opt->skip_images[dem_iter].end()) {
          continue;
        }
      
        ImageView<PixelMask<double>> reflectance, intensity, comp_intensity;
        ImageView<double> ground_weight;

        std::string out_camera_file
          = asp::bundle_adjust_file_name(g_opt->out_prefix,
                                         g_opt->input_images[image_iter],
                                         g_opt->input_cameras[image_iter]);
        
        if (!g_opt->use_approx_adjusted_camera_models) {
          // Save the camera adjustments for the current iteration.
          // When we use approx adjusted camera models this is not necessary
          // since then the cameras don't change.
          //std::string out_camera_file = g_opt->out_prefix + "-camera"
          //+ iter_str2 + ".adjust";
          //vw_out() << "Writing: " << out_camera_file << std::endl;
          AdjustedCameraModel * icam
            = dynamic_cast<AdjustedCameraModel*>((*g_cameras)[dem_iter][image_iter].get());
          if (icam == NULL)
            vw_throw( ArgumentErr() << "Expecting an adjusted camera.\n");
          Vector3 translation = icam->translation();
          Quaternion<double> rotation = icam->rotation();
          //asp::write_adjustments(out_camera_file, translation, rotation);
          
          // Used to save adjusted files in the format <out prefix>-<input-img>.adjust
          // so we can later read them with --bundle-adjust-prefix to be
          // used in another SfS run. Don't do that anymore as normally
          // the adjustments don't change.
          //if (g_level == 0) {
          //if (!g_opt->save_computed_intensity_only){
          //  vw_out() << "Writing: " << out_camera_file << std::endl;
          //  asp::write_adjustments(out_camera_file, translation, rotation);
          //}
        }
        
        if (g_opt->save_sparingly && !g_opt->save_dem_with_nodata) 
          continue; // don't write too many things
        
        // Manufacture an output prefix for the other data associated with this camera
        std::string iter_str2 = fs::path(out_camera_file).replace_extension("").string();
        iter_str2 += iter_str;
    
        // Compute reflectance and intensity with optimized DEM
        int sample_col_rate = 1, sample_row_rate = 1;
        computeReflectanceAndIntensity((*g_dem)[dem_iter], (*g_pq)[dem_iter],
                                       (*g_geo)[dem_iter],
                                       g_opt->model_shadows,
                                       (*g_max_dem_height)[dem_iter],
                                       *g_gridx, *g_gridy,
                                       sample_col_rate, sample_row_rate,
                                       (*g_model_params)[image_iter],
                                       *g_global_params,
                                       (*g_crop_boxes)[dem_iter][image_iter],
                                       (*g_masked_images)[dem_iter][image_iter],
                                       (*g_blend_weights)[dem_iter][image_iter],
                                       (*g_cameras)[dem_iter][image_iter].get(),
                                       &(*g_scaled_sun_posns)[3*image_iter],
                                       reflectance, intensity, ground_weight, 
                                       g_reflectance_model_coeffs);

        // dem_nodata equals to dem if the image has valid pixels and no shadows
        if (g_opt->save_dem_with_nodata) {
          for (int col = 0; col < reflectance.cols(); col++) {
            for (int row = 0; row < reflectance.rows(); row++) {
              if (is_valid(reflectance(col, row))) 
                dem_nodata(col, row) = (*g_dem)[dem_iter](col, row);
            }
          }
        }

        if (g_opt->save_sparingly)
          continue;
        
        // Find the computed intensity
        comp_intensity.set_size(reflectance.cols(), reflectance.rows());
        for (int col = 0; col < comp_intensity.cols(); col++) {
          for (int row = 0; row < comp_intensity.rows(); row++) {
            comp_intensity(col, row)
              = (*g_albedo)[dem_iter](col, row) *
              nonlin_reflectance(reflectance(col, row), (*g_exposures)[image_iter],
                                 g_opt->steepness_factor,
                                 &(*g_haze)[image_iter][0], g_opt->num_haze_coeffs);
          }
        }

        std::string out_meas_intensity_file = iter_str2 + "-meas-intensity.tif";
        vw_out() << "Writing: " << out_meas_intensity_file << std::endl;
        block_write_gdal_image(out_meas_intensity_file,
                               apply_mask(intensity, *g_img_nodata_val),
                               has_georef, (*g_geo)[dem_iter], has_nodata,
                               *g_img_nodata_val, *g_opt, tpc);
    
        std::string out_comp_intensity_file = iter_str2 + "-comp-intensity.tif";
        vw_out() << "Writing: " << out_comp_intensity_file << std::endl;
        block_write_gdal_image(out_comp_intensity_file,
                               apply_mask(comp_intensity, *g_img_nodata_val),
                               has_georef, (*g_geo)[dem_iter], has_nodata, *g_img_nodata_val,
                               *g_opt, tpc);

        if (g_opt->save_computed_intensity_only) 
          continue; // don't write too many things

        std::string out_weight_file = iter_str2 + "-blending-weight.tif";
        vw_out() << "Writing: " << out_weight_file << std::endl;
        block_write_gdal_image(out_weight_file,
                               ground_weight,
                               has_georef, (*g_geo)[dem_iter], has_nodata, *g_img_nodata_val,
                               *g_opt, tpc);

        std::string out_reflectance_file = iter_str2 + "-reflectance.tif";
        vw_out() << "Writing: " << out_reflectance_file << std::endl;
        block_write_gdal_image(out_reflectance_file,
                               apply_mask(reflectance, *g_img_nodata_val),
                               has_georef, (*g_geo)[dem_iter], has_nodata, *g_img_nodata_val,
                               *g_opt, tpc);


        // Find the measured normalized albedo, after correcting for
        // reflectance.
        ImageView<double> measured_albedo;
        measured_albedo.set_size(reflectance.cols(), reflectance.rows());
        for (int col = 0; col < measured_albedo.cols(); col++) {
          for (int row = 0; row < measured_albedo.rows(); row++) {
            if (!is_valid(reflectance(col, row)))
              measured_albedo(col, row) = 1;
            else
              measured_albedo(col, row) = intensity(col, row) /
                nonlin_reflectance(reflectance(col, row), (*g_exposures)[image_iter],
                                   g_opt->steepness_factor,
                                   &(*g_haze)[image_iter][0], g_opt->num_haze_coeffs);
          }
        }
        std::string out_albedo_file = iter_str2 + "-meas-albedo.tif";
        vw_out() << "Writing: " << out_albedo_file << std::endl;
        block_write_gdal_image(out_albedo_file, measured_albedo,
                               has_georef, (*g_geo)[dem_iter], has_nodata, 0, *g_opt, tpc);


        double imgmean, imgstdev, refmean, refstdev;
        compute_image_stats(intensity, comp_intensity, imgmean, imgstdev, refmean, refstdev);
        vw_out() << "meas image mean and std: " << imgmean << ' ' << imgstdev
                 << std::endl;
        vw_out() << "comp image mean and std: " << refmean << ' ' << refstdev
                 << std::endl;

        vw_out() << "Exposure for image " << image_iter << ": "
                 << (*g_exposures)[image_iter] << std::endl;

        if (g_opt->num_haze_coeffs > 0) {
          vw_out() << "Haze for image " << image_iter << ":";
          for (size_t hiter = 0; hiter < (*g_haze)[image_iter].size(); hiter++) {
            vw_out() << " " << (*g_haze)[image_iter][hiter];
          }
          vw_out() << std::endl;
        }
      }

      if (g_opt->save_dem_with_nodata) {
        if ( !g_opt->save_sparingly || g_final_iter ) {
          std::string out_dem_nodata_file = g_opt->out_prefix + "-DEM-nodata"
            + iter_str + ".tif";
          vw_out() << "Writing: " << out_dem_nodata_file << std::endl;
          TerminalProgressCallback tpc("asp", ": ");
          block_write_gdal_image(out_dem_nodata_file, dem_nodata,
                                 has_georef, (*g_geo)[dem_iter],
                                 has_nodata, *g_dem_nodata_val,
                                 *g_opt, tpc);
        }
      }
    }
    
    return ceres::SOLVER_CONTINUE;
  }
};

// See SmoothnessError() for the definitions of bottom, top, etc.
template <typename F, typename G>
inline bool
calc_intensity_residual(const F* const exposure,
                        const F* const haze,
                        const G* const left,
                        const G* const center,
                        const G* const right,
                        const G* const bottom,
                        const G* const top,
                        bool use_pq,
                        const G* const pq, // partial derivatives of the dem in x and y
                        const G* const albedo,
                        const F* const camera_adjustments,
                        const F* const scaled_sun_posn,
                        const G* const reflectance_model_coeffs, 
                        int m_col, int m_row,
                        ImageView<double>                 const & m_dem,            // alias
                        cartography::GeoReference         const & m_geo,            // alias
                        bool                                      m_model_shadows,
                        double                                    m_camera_position_step_size,
                        double                            const & m_max_dem_height, // alias
                        double                                    m_gridx,
                        double                                    m_gridy,
                        GlobalParams                      const & m_global_params,  // alias
                        ModelParams                       const & m_model_params,   // alias
                        BBox2i                                    m_crop_box,
                        MaskedImgT                        const & m_image,          // alias
                        DoubleImgT                        const & m_blend_weight,   // alias
                        boost::shared_ptr<CameraModel>    const & m_camera,         // alias
                        F* residuals) {
  
  // Default residuals. Using here 0 rather than some big number tuned out to
  // work better than the alternative.
  residuals[0] = F(0.0);
  try{

    // Initialize this variable to something for now, it does not
    // matter yet to what.  We just don't want it to go out of scope.
    AdjustedCameraModel adj_cam_copy(m_camera);

    // This is a bit tricky. If use adjusted approximate camera model
    // (then the cameras never change), use the camera passed
    // in. Else, create a copy of this camera to avoid issues when
    // using multiple threads. In that case we copy just the
    // adjustment parameters, the pointer to the underlying ISIS
    // camera is shared.
    CameraModel * camera = NULL;

    if (g_opt->use_approx_adjusted_camera_models) {
      camera = (CameraModel*)(m_camera.get());
    }else{
      AdjustedCameraModel * adj_cam
        = dynamic_cast<AdjustedCameraModel*>(m_camera.get());
      if (adj_cam == NULL)
        vw_throw( ArgumentErr() << "Expecting an adjusted camera.\n");

      // We overwrite the dummy value of adj_cam_copy with a deep copy
      // of adj_cam.
      adj_cam_copy = *adj_cam;
      
      // Apply current adjustments to the camera
      Vector3 axis_angle;
      Vector3 translation;
      for (int param_iter = 0; param_iter < 3; param_iter++) {
        translation[param_iter]
          = (g_position_scale_factor*m_camera_position_step_size)*camera_adjustments[param_iter];
        axis_angle[param_iter] = camera_adjustments[3 + param_iter];
      }
      adj_cam_copy.set_translation(translation);
      adj_cam_copy.set_axis_angle_rotation(axis_angle);
      
      camera = &adj_cam_copy;
    }
    
    PixelMask<double> reflectance(0), intensity(0);
    double ground_weight = 0;

    // Need to be careful not to access an array which does not exist
    G p = 0, q = 0;
    if (use_pq) {
      p = pq[0];
      q = pq[1];
    }
    
    bool success =
      computeReflectanceAndIntensity(left[0], center[0], right[0],
                                     bottom[0], top[0],
                                     use_pq, p, q,
                                     m_col, m_row,  m_dem, m_geo,
                                     m_model_shadows, m_max_dem_height,
                                     m_gridx, m_gridy,
                                     m_model_params,  m_global_params,
                                     m_crop_box, m_image, m_blend_weight, camera,
                                     scaled_sun_posn,
                                     reflectance, intensity, ground_weight, reflectance_model_coeffs);
      
    if (g_opt->unreliable_intensity_threshold > 0){
      if (is_valid(intensity) && intensity.child() <= g_opt->unreliable_intensity_threshold &&
          intensity.child() >= 0) {
        ground_weight *=
          pow(intensity.child()/g_opt->unreliable_intensity_threshold, 2.0);
      }
    }
      
    if (success && is_valid(intensity) && is_valid(reflectance))
      residuals[0] = ground_weight * (intensity - albedo[0] *
                               nonlin_reflectance(reflectance.child(), exposure[0],
                                                  g_opt->steepness_factor,
                                                  haze, g_opt->num_haze_coeffs));
    

  } catch (const camera::PointToPixelErr& e) {
    // To be able to handle robustly DEMs that extend beyond the camera,
    // always return true when we fail to project, but with zero residual.
    // This needs more study.
    residuals[0] = F(0.0);
    return true;
  }

  return true;
}

// Discrepancy between measured and computed intensity.
// sum_i | I_i - albedo * nonlin_reflectance(reflectance_i, exposures[i], haze, num_haze_coeffs) |^2
struct IntensityError {
  IntensityError(int col, int row,
                 ImageView<double> const& dem,
                 cartography::GeoReference const& geo,
                 bool model_shadows,
                 double camera_position_step_size,
                 double const& max_dem_height, // note: this is an alias
                 double gridx, double gridy,
                 GlobalParams const& global_params,
                 ModelParams const& model_params,
                 BBox2i const& crop_box,
                 MaskedImgT const& image,
                 DoubleImgT const& blend_weight,
                 double * scaled_sun_posn, 
                 boost::shared_ptr<CameraModel> const& camera):
    m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_global_params(global_params),
    m_model_params(model_params),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_scaled_sun_posn(scaled_sun_posn),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure,
                  const F* const haze,
                  const F* const left,
                  const F* const center,
                  const F* const right,
                  const F* const bottom,
                  const F* const top,
                  const F* const albedo,
                  const F* const camera_adjustments,
                  //const F* const scaled_sun_posn,
                  const F* const reflectance_model_coeffs,
                  F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;

    //const F* const haze = NULL;
    return calc_intensity_residual(exposure, haze,
                                   left, center, right, bottom, top,
                                   use_pq, pq,
                                   albedo, camera_adjustments,
                                   m_scaled_sun_posn,
                                   reflectance_model_coeffs,
                                   m_col, m_row,  
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,  
                                   m_camera_position_step_size,  
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,  
                                   m_global_params,   // alias
                                   m_model_params,    // alias
                                   m_crop_box,  
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int col, int row,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     GlobalParams const& global_params,
                                     ModelParams const& model_params,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     double * scaled_sun_posn, 
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityError,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 1, 1, 1, 1, 1, 6, g_num_model_coeffs>
            (new IntensityError(col, row, dem, geo,
                                model_shadows,
                                camera_position_step_size,
                                max_dem_height,
                                gridx, gridy,
                                global_params, model_params,
                                crop_box, image, blend_weight, scaled_sun_posn, camera)));
  }

  int m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  GlobalParams                      const & m_global_params;  // alias
  ModelParams                       const & m_model_params;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  double                                  * m_scaled_sun_posn;   //  pointer
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// A variation of the intensity error where only the DEM is floated
struct IntensityErrorFloatDemOnly {
  IntensityErrorFloatDemOnly(int col, int row,
                             ImageView<double> const& dem,
                             double albedo,
                             double * reflectance_model_coeffs, 
                             double * exposure, 
                             double * haze, 
                             double * camera_adjustments, 
                             cartography::GeoReference const& geo,
                             bool model_shadows,
                             double camera_position_step_size,
                             double const& max_dem_height, // note: this is an alias
                             double gridx, double gridy,
                             GlobalParams const& global_params,
                             ModelParams const& model_params,
                             BBox2i const& crop_box,
                             MaskedImgT const& image,
                             DoubleImgT const& blend_weight,
                             double * scaled_sun_posn, 
                             boost::shared_ptr<CameraModel> const& camera):
    m_col(col), m_row(row), m_dem(dem),
    m_albedo(albedo), m_reflectance_model_coeffs(reflectance_model_coeffs),
    m_exposure(exposure), m_haze(haze), m_camera_adjustments(camera_adjustments),
    m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_global_params(global_params),
    m_model_params(model_params),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_scaled_sun_posn(scaled_sun_posn),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const left,
                  const F* const center,
                  const F* const right,
                  const F* const bottom,
                  const F* const top,
                  F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;

    return calc_intensity_residual(m_exposure, m_haze,
                                   left, center, right, bottom, top,
                                   use_pq, pq,
                                   &m_albedo, m_camera_adjustments,
                                   m_scaled_sun_posn,
                                   m_reflectance_model_coeffs,
                                   m_col, m_row,  
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,  
                                   m_camera_position_step_size,  
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,  
                                   m_global_params,   // alias
                                   m_model_params,    // alias
                                   m_crop_box,  
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int col, int row,
                                     ImageView<double> const& dem,
                                     double albedo,
                                     double * reflectance_model_coeffs, 
                                     double * exposure, 
                                     double * haze, 
                                     double * camera_adjustments, 
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     GlobalParams const& global_params,
                                     ModelParams const& model_params,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     double * scaled_sun_posn, 
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityErrorFloatDemOnly,
            ceres::CENTRAL, 1, 1, 1, 1, 1, 1>
            (new IntensityErrorFloatDemOnly(col, row, dem,
                                            albedo, reflectance_model_coeffs,
                                            exposure, haze, camera_adjustments,
                                            geo,
                                            model_shadows,
                                            camera_position_step_size,
                                            max_dem_height,
                                            gridx, gridy,
                                            global_params, model_params,
                                            crop_box, image, blend_weight, scaled_sun_posn,
                                            camera)));
  }

  int                                       m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  double                                    m_albedo;
  double                                  * m_reflectance_model_coeffs;
  double                                  * m_exposure;
  double                                  * m_haze;
  double                                  * m_camera_adjustments;
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  GlobalParams                      const & m_global_params;  // alias
  ModelParams                       const & m_model_params;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  double                                  * m_scaled_sun_posn;   // pointer
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// A variation of IntensityError where albedo, dem, and model params are fixed.
struct IntensityErrorFixedMost {
  IntensityErrorFixedMost(int col, int row,
                          ImageView<double> const& dem,
                          double albedo,
                          double * reflectance_model_coeffs, 
                          cartography::GeoReference const& geo,
                          bool model_shadows,
                          double camera_position_step_size,
                          double const& max_dem_height, // note: this is an alias
                          double gridx, double gridy,
                          GlobalParams const& global_params,
                          ModelParams const& model_params,
                          BBox2i const& crop_box,
                          MaskedImgT const& image,
                          DoubleImgT const& blend_weight,
                          boost::shared_ptr<CameraModel> const& camera):
    m_col(col), m_row(row), m_dem(dem),
    m_albedo(albedo), m_reflectance_model_coeffs(reflectance_model_coeffs), 
    m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_global_params(global_params),
    m_model_params(model_params),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_camera(camera) {}

  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure,
                  const F* const haze,
                  const F* const camera_adjustments,
                  const F* const scaled_sun_posn,
                  F* residuals) const {

    // For this error we do not use p and q, hence just use a placeholder.
    bool use_pq = false;
    const F * const pq = NULL;
    
    return calc_intensity_residual(exposure, haze,
                                   &m_dem(m_col-1, m_row),            // left
                                   &m_dem(m_col, m_row),              // center
                                   &m_dem(m_col+1, m_row),            // right
                                   &m_dem(m_col, m_row+1),            // bottom
                                   &m_dem(m_col, m_row-1),            // top
                                   use_pq, pq,
                                   &m_albedo,
                                   camera_adjustments,
                                   scaled_sun_posn,
                                   m_reflectance_model_coeffs,
                                   m_col, m_row,  
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,  
                                   m_camera_position_step_size,  
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,  
                                   m_global_params,  // alias
                                   m_model_params,  // alias
                                   m_crop_box,  
                                   m_image,  // alias
                                   m_blend_weight,  // alias
                                   m_camera,  // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int col, int row,
                                     ImageView<double> const& dem,
                                     double albedo,
                                     double * reflectance_model_coeffs, 
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     GlobalParams const& global_params,
                                     ModelParams const& model_params,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityErrorFixedMost,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 6, 3>
            (new IntensityErrorFixedMost(col, row, dem, albedo, reflectance_model_coeffs, geo,
                                         model_shadows,
                                         camera_position_step_size,
                                         max_dem_height,
                                         gridx, gridy,
                                         global_params, model_params,
                                         crop_box, image, blend_weight, camera)));
  }

  int m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  double                                    m_albedo;
  double                                  * m_reflectance_model_coeffs; 
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  GlobalParams                      const & m_global_params;  // alias
  ModelParams                       const & m_model_params;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// A variant of the intensity error when we float the partial derviatives
// in x and in y of the dem, which we call p and q.  
struct IntensityErrorPQ {
  IntensityErrorPQ(int col, int row,
                   ImageView<double> const& dem,
                   cartography::GeoReference const& geo,
                   bool model_shadows,
                   double camera_position_step_size,
                   double const& max_dem_height, // note: this is an alias
                   double gridx, double gridy,
                   GlobalParams const& global_params,
                   ModelParams const& model_params,
                   BBox2i const& crop_box,
                   MaskedImgT const& image,
                   DoubleImgT const& blend_weight,
                   boost::shared_ptr<CameraModel> const& camera):
    m_col(col), m_row(row), m_dem(dem), m_geo(geo),
    m_model_shadows(model_shadows),
    m_camera_position_step_size(camera_position_step_size),
    m_max_dem_height(max_dem_height),
    m_gridx(gridx), m_gridy(gridy),
    m_global_params(global_params),
    m_model_params(model_params),
    m_crop_box(crop_box),
    m_image(image), m_blend_weight(blend_weight),
    m_camera(camera) {}
  
  // See SmoothnessError() for the definitions of bottom, top, etc.
  template <typename F>
  bool operator()(const F* const exposure,
                  const F* const haze,                  
                  const F* const center_h,
                  const F* const pq,                 // array of length 2 
                  const F* const albedo,
                  const F* const camera_adjustments, // array of length 6
                  const F* const scaled_sun_posn,       // array of length 3
                  const F* const reflectance_model_coeffs,
                  F* residuals) const {

    bool use_pq = true;

    F v = 0;
    return calc_intensity_residual(exposure, haze, &v, center_h, &v, &v, &v,
                                   use_pq, pq,
                                   albedo, camera_adjustments,
                                   scaled_sun_posn,
                                   reflectance_model_coeffs,
                                   m_col, m_row,  
                                   m_dem,  // alias
                                   m_geo,  // alias
                                   m_model_shadows,  
                                   m_camera_position_step_size,  
                                   m_max_dem_height,  // alias
                                   m_gridx, m_gridy,  
                                   m_global_params,   // alias
                                   m_model_params,    // alias
                                   m_crop_box,  
                                   m_image,           // alias
                                   m_blend_weight,    // alias
                                   m_camera,          // alias
                                   residuals);
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(int col, int row,
                                     ImageView<double> const& dem,
                                     vw::cartography::GeoReference const& geo,
                                     bool model_shadows,
                                     double camera_position_step_size,
                                     double const& max_dem_height, // alias
                                     double gridx, double gridy,
                                     GlobalParams const& global_params,
                                     ModelParams const& model_params,
                                     BBox2i const& crop_box,
                                     MaskedImgT const& image,
                                     DoubleImgT const& blend_weight,
                                     boost::shared_ptr<CameraModel> const& camera){
    return (new ceres::NumericDiffCostFunction<IntensityErrorPQ,
            ceres::CENTRAL, 1, 1, g_max_num_haze_coeffs, 1, 2, 1, 6, 3, g_num_model_coeffs>
            (new IntensityErrorPQ(col, row, dem, geo,
                                  model_shadows,
                                  camera_position_step_size,
                                  max_dem_height,
                                  gridx, gridy,
                                  global_params, model_params,
                                  crop_box, image, blend_weight, camera)));
  }

  int m_col, m_row;
  ImageView<double>                 const & m_dem;            // alias
  cartography::GeoReference         const & m_geo;            // alias
  bool                                      m_model_shadows;
  double                                    m_camera_position_step_size;
  double                            const & m_max_dem_height; // alias
  double                                    m_gridx, m_gridy;
  GlobalParams                      const & m_global_params;  // alias
  ModelParams                       const & m_model_params;   // alias
  BBox2i                                    m_crop_box;
  MaskedImgT                        const & m_image;          // alias
  DoubleImgT                        const & m_blend_weight;   // alias
  boost::shared_ptr<CameraModel>    const & m_camera;         // alias
};

// The smoothness error is the sum of squares of
// the 4 second order partial derivatives, with a weight:
// error = smoothness_weight * ( u_xx^2 + u_xy^2 + u_yx^2 + u_yy^2 )

// We will use finite differences to compute these.
// Consider a grid point and its neighbors, 9 points in all.
//
// bl   = u(c-1, r+1)  bottom = u(c, r+1) br    = u(c+1,r+1)
// left = u(c-1, r  )  center = u(c, r  ) right = u(c+1,r  )
// tl   = u(c-1, r-1)  top    = u(c, r-1) tr    = u(c+1,r-1)
//
// See https://en.wikipedia.org/wiki/Finite_difference
// for the obtained formulas.

struct SmoothnessError {
  SmoothnessError(double smoothness_weight, double gridx, double gridy):
    m_smoothness_weight(smoothness_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bl,   const T* const bottom,    const T* const br,
                  const T* const left, const T* const center,    const T* const right,
                  const T* const tl,   const T* const top,       const T* const tr,
                  T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    residuals[0] = (left[0] + right[0] - 2*center[0])/m_gridx/m_gridx;   // u_xx
    residuals[1] = (br[0] + tl[0] - bl[0] - tr[0] )/4.0/m_gridx/m_gridy; // u_xy
    residuals[2] = residuals[1];                                         // u_yx
    residuals[3] = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy
    
    for (int i = 0; i < 4; i++)
      residuals[i] *= m_smoothness_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<SmoothnessError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1>
            (new SmoothnessError(smoothness_weight, gridx, gridy)));
  }

  double m_smoothness_weight, m_gridx, m_gridy;
};

// The gradient error is the sum of squares of
// the first order partial derivatives, with a weight:
// error = gradient_weight * (u_x^2 + u_y^2)

// We will use finite differences to compute these. See
// SmoothnessError() for more details.
struct GradientError {
  GradientError(double gradient_weight, double gridx, double gridy):
    m_gradient_weight(gradient_weight), m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const center,
                  const T* const right, const T* const top, T* residuals) const {

    // This results in a smoother solution than using centered differences
    residuals[0] = (right[0]  - center[0])/m_gridx; // u_x
    residuals[1] = (center[0] - left[0]  )/m_gridx; // u_x
    residuals[2] = (top[0]    - center[0])/m_gridy; // u_y
    residuals[3] = (center[0] - bottom[0])/m_gridy; // u_y
    
    for (int i = 0; i < 4; i++)
      residuals[i] *= m_gradient_weight;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double gradient_weight,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<GradientError,
            ceres::CENTRAL, 4, 1, 1, 1, 1, 1>
            (new GradientError(gradient_weight, gridx, gridy)));
  }

  double m_gradient_weight, m_gridx, m_gridy;
};

// Try to make the DEM in shadow have positive curvature. The error term is
// (curvature_weight *(terrain_xx + terrain_xy - curvature))^2 in the shadow,
// and not used in lit areas.
struct CurvatureInShadowError {
  CurvatureInShadowError(double curvature_in_shadow, double curvature_in_shadow_weight,
                         double gridx, double gridy):
    m_curvature_in_shadow(curvature_in_shadow),
    m_curvature_in_shadow_weight(curvature_in_shadow_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const center,
                  const T* const right, const T* const top, T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    double u_xx = (left[0] + right[0] - 2*center[0])/m_gridx/m_gridx;   // u_xx
    double u_yy = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy
    
    residuals[0] = m_curvature_in_shadow_weight*(u_xx + u_yy - m_curvature_in_shadow);

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double curvature_in_shadow,
                                     double curvature_in_shadow_weight,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<CurvatureInShadowError,
            ceres::CENTRAL, 1, 1, 1, 1, 1, 1>
            (new CurvatureInShadowError(curvature_in_shadow, curvature_in_shadow_weight,
                                        gridx, gridy)));
  }

  double m_curvature_in_shadow, m_curvature_in_shadow_weight, m_gridx, m_gridy;
};

struct SmoothnessErrorPQ {
  SmoothnessErrorPQ(double smoothness_weight_pq, double gridx, double gridy):
    m_smoothness_weight_pq(smoothness_weight_pq),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom_pq, const T* const left_pq, const T* const right_pq,
                  const T* const top_pq, T* residuals) const {

    // Normalize by grid size seems to make the functional less
    // sensitive to the actual grid size used.
    residuals[0] = (right_pq[0] - left_pq[0])/(2*m_gridx);   // p_x
    residuals[1] = (top_pq[0] - bottom_pq[0])/(2*m_gridy);   // p_y
    residuals[2] = (right_pq[1] - left_pq[1])/(2*m_gridx);   // q_x
    residuals[3] = (top_pq[1] - bottom_pq[1])/(2*m_gridy);   // q_y
    
    for (int i = 0; i < 4; i++)
      residuals[i] *= m_smoothness_weight_pq;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double smoothness_weight_pq,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<SmoothnessErrorPQ,
            ceres::CENTRAL, 4, 2, 2, 2, 2>
            (new SmoothnessErrorPQ(smoothness_weight_pq, gridx, gridy)));
  }

  double m_smoothness_weight_pq, m_gridx, m_gridy;
};

// The integrability error is the discrepancy between the
// independently optimized gradients p and q, and the partial
// derivatives of the dem, here denoted by u.
// error = integrability_weight * ( (u_x - p)^2 + (u_y - q)^2 )

// See SmoothnessError for the notation below. 

struct IntegrabilityError {
  IntegrabilityError(double integrability_weight, double gridx, double gridy):
    m_integrability_weight(integrability_weight),
    m_gridx(gridx), m_gridy(gridy) {}

  template <typename T>
  bool operator()(const T* const bottom, const T* const left, const T* const right,
                  const T* const top, const T* const pq, 
                  T* residuals) const {

    residuals[0] = (right[0] - left[0])/(2*m_gridx) - pq[0];
    residuals[1] = (top[0] - bottom[0])/(2*m_gridy) - pq[1];
    
    for (int i = 0; i < 2; i++)
      residuals[i] *= m_integrability_weight;
    
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double integrability_weight,
                                     double gridx, double gridy){
    return (new ceres::NumericDiffCostFunction<IntegrabilityError,
            ceres::CENTRAL, 2, 1, 1, 1, 1, 2>
            (new IntegrabilityError(integrability_weight, gridx, gridy)));
  }

  double m_integrability_weight, m_gridx, m_gridy;
};


// A cost function that will penalize deviating too much from the original DEM height.
struct HeightChangeError {
  HeightChangeError(double orig_height, double initial_dem_constraint_weight):
    m_orig_height(orig_height), m_initial_dem_constraint_weight(initial_dem_constraint_weight){}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_orig_height)*m_initial_dem_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double orig_height,
                                     double initial_dem_constraint_weight){
    return (new ceres::NumericDiffCostFunction<HeightChangeError,
            ceres::CENTRAL, 1, 1>
            (new HeightChangeError(orig_height, initial_dem_constraint_weight)));
  }

  double m_orig_height, m_initial_dem_constraint_weight;
};

// A cost function that will penalize deviating too much from the initial albedo.
struct AlbedoChangeError {
  AlbedoChangeError(double initial_albedo, double albedo_constraint_weight):
    m_initial_albedo(initial_albedo), m_albedo_constraint_weight(albedo_constraint_weight){}

  template <typename T>
  bool operator()(const T* const center, T* residuals) const {
    residuals[0] = (center[0] - m_initial_albedo)*m_albedo_constraint_weight;
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(double initial_albedo,
                                     double albedo_constraint_weight){
    return (new ceres::NumericDiffCostFunction<AlbedoChangeError,
            ceres::CENTRAL, 1, 1>
            (new AlbedoChangeError(initial_albedo, albedo_constraint_weight)));
  }

  double m_initial_albedo, m_albedo_constraint_weight;
};

// Given a DEM, estimate the median grid size in x and in y in meters.
// Given that the DEM heights are in meters as well, having these grid sizes
// will make it possible to handle heights and grids in same units.
void compute_grid_sizes_in_meters(ImageView<double> const& dem,
                                  GeoReference const& geo,
                                  double nodata_val,
                                  double & gridx, double & gridy){

  // Initialize the outputs
  gridx = 0; gridy = 0;

  // Estimate the median height
  std::vector<double> heights;
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      double h = dem(col, row);
      if (h == nodata_val) continue;
      heights.push_back(h);
    }
  }
  double median_height = 0.0;
  int len = heights.size();
  if (len > 0) {
    std::sort(heights.begin(), heights.end());
    median_height = 0.5*(heights[(len-1)/2] + heights[len/2]);
  }

  // Find the grid sizes by estimating the Euclidean distances
  // between points of a DEM at constant height.
  std::vector<double> gridx_vec, gridy_vec;
  for (int col = 0; col < dem.cols()-1; col++) {
    for (int row = 0; row < dem.rows()-1; row++) {

      // The xyz position at the center grid point
      Vector2 lonlat = geo.pixel_to_lonlat(Vector2(col, row));
      Vector3 lonlat3 = Vector3(lonlat(0), lonlat(1), median_height);
      Vector3 base = geo.datum().geodetic_to_cartesian(lonlat3);

      // The xyz position at the right grid point
      lonlat = geo.pixel_to_lonlat(Vector2(col+1, row));
      lonlat3 = Vector3(lonlat(0), lonlat(1), median_height);
      Vector3 right = geo.datum().geodetic_to_cartesian(lonlat3);

      // The xyz position at the bottom grid point
      lonlat = geo.pixel_to_lonlat(Vector2(col, row+1));
      lonlat3 = Vector3(lonlat(0), lonlat(1), median_height);
      Vector3 bottom = geo.datum().geodetic_to_cartesian(lonlat3);

      gridx_vec.push_back(norm_2(right-base));
      gridy_vec.push_back(norm_2(bottom-base));
    }
  }

  // Median grid size
  if (!gridx_vec.empty()) gridx = gridx_vec[gridx_vec.size()/2];
  if (!gridy_vec.empty()) gridy = gridy_vec[gridy_vec.size()/2];
}

void read_sun_positions_from_list(Options const& opt,
                                  std::vector<ModelParams> &model_params) {

  // Initialize the sun position with something (the planet center)
  int num_images = opt.input_images.size();
  model_params.resize(num_images);
  for (int it = 0; it < num_images; it++) {
    model_params[it].sunPosition = Vector3();  
  }
  
  if (opt.sun_positions_list == "") 
    return; // nothing to do
  
  // First read the positions in a map, as they may be out of order
  std::map<std::string, vw::Vector3> sun_positions_map;
  std::ifstream ifs(opt.sun_positions_list.c_str());
  std::string filename;
  double x, y, z;
  while (ifs >> filename >> x >> y >> z)
    sun_positions_map[filename] = Vector3(x, y, z);

  if (sun_positions_map.size() != opt.input_images.size())
    vw_throw(ArgumentErr() << "Expecting to find in file: " << opt.sun_positions_list
             << " as many sun positions as there are images.\n");
  
  // Put the sun positions in model_params.
  for (int it = 0; it < num_images; it++) {
    auto map_it = sun_positions_map.find(opt.input_images[it]);
    if (map_it == sun_positions_map.end()) 
      vw_throw(ArgumentErr() << "Could not read the Sun position from file: "
               << opt.sun_positions_list << " for image: " << opt.input_images[it] << ".\n");

    model_params[it].sunPosition = map_it->second;
  }
}

Vector3 sun_position_from_camera(boost::shared_ptr<CameraModel> camera) {

  // Remove any adjustment to get to the camera proper
  boost::shared_ptr<CameraModel> ucam = unadjusted_model(camera);

  // Try isis
  IsisCameraModel* isis_cam = dynamic_cast<IsisCameraModel*>(ucam.get());
  if (isis_cam != NULL)
    return isis_cam->sun_position();

  // Try csm
  asp::CsmModel* csm_cam = dynamic_cast<asp::CsmModel*>(ucam.get());
  if (csm_cam != NULL)
    return csm_cam->sun_position();

  // No luck. Later there will be a complaint.
  return vw::Vector3();
}

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-dem,i",  po::value(&opt.input_dems_str),
     "The input DEM(s) to refine using SfS. If more than one, their list should be in quotes.")
    ("image-list", po::value(&opt.image_list)->default_value(""),
     "A file containing the list of images, when they are too many to specify on the command line. Use space or newline as separator. See also --camera-list and --mapprojected-data-list.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of cameras, when they are too many to specify on the command "
     "line. If the images have embedded camera information, such as for ISIS, this file must "
     "be empty but must be specified if --image-list is specified.")
    ("output-prefix,o", po::value(&opt.out_prefix),
     "Prefix for output filenames.")
    ("max-iterations,n", po::value(&opt.max_iterations)->default_value(10),
     "Set the maximum number of iterations. Normally 5-10 iterations is enough, even when convergence is not reached, as the solution usually improves quickly at first and only very fine refinements happen later.")
    ("reflectance-type", po::value(&opt.reflectance_type)->default_value(1),
     "Reflectance type (0 = Lambertian, 1 = Lunar-Lambert, 2 = Hapke, 3 = Experimental extension of Lunar-Lambert, 4 = Charon model (a variation of Lunar-Lambert)).")
    ("smoothness-weight", po::value(&opt.smoothness_weight)->default_value(0.04),
     "The weight given to the cost function term which consists of sums of squares of second-order derivatives. A larger value will result in a smoother solution with fewer artifacts. See also --gradient-weight.")
    ("initial-dem-constraint-weight", po::value(&opt.initial_dem_constraint_weight)->default_value(0),
     "A larger value will try harder to keep the SfS-optimized DEM closer to the initial guess DEM. A value between 0.0001 and 0.001 may work, unless your initial DEM is very unreliable.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustments obtained by previously running bundle_adjust with this output prefix.")
    ("float-albedo",   po::bool_switch(&opt.float_albedo)->default_value(false)->implicit_value(true),
     "Float the albedo for each pixel. Will give incorrect results if only one image is present. The albedo is normalized, its nominal value is 1.")
    ("float-exposure",   po::bool_switch(&opt.float_exposure)->default_value(false)->implicit_value(true),
     "Float the exposure for each image. Will give incorrect results if only one image is present. It usually gives marginal results.")
    ("float-cameras",   po::bool_switch(&opt.float_cameras)->default_value(false)->implicit_value(true),
     "Float the camera pose for each image except the first one. It is suggested that this option be avoided and bundle adjustment be used instead.")
    ("float-all-cameras",   po::bool_switch(&opt.float_all_cameras)->default_value(false)->implicit_value(true),
     "Float the camera pose for each image, including the first one. Experimental. It is suggested to avoid this option.")
    ("model-shadows",   po::bool_switch(&opt.model_shadows)->default_value(false)->implicit_value(true),
     "Model the fact that some points on the DEM are in the shadow (occluded from the Sun).")
    ("compute-exposures-only",   po::bool_switch(&opt.compute_exposures_only)->default_value(false)->implicit_value(true),
     "Quit after saving the exposures. This should be done once for a big DEM, before using these for small sub-clips without recomputing them.")

    ("save-computed-intensity-only",   po::bool_switch(&opt.save_computed_intensity_only)->default_value(false)->implicit_value(true),
     "Save the computed (simulated) image intensities for given DEM, "
     "images, cameras, and reflectance model, without refining the "
     "DEM. The exposures will be computed along the way unless specified "
     "via --image-exposures-prefix, and will be saved to <output prefix>-exposures.txt.")
    
    ("estimate-slope-errors",   po::bool_switch(&opt.estimate_slope_errors)->default_value(false)->implicit_value(true),
     "Estimate the error for each slope (normal to the DEM). This is experimental.")
    ("estimate-height-errors",   po::bool_switch(&opt.estimate_height_errors)->default_value(false)->implicit_value(true),
     "Estimate the SfS DEM height uncertainty by finding the height perturbation (in meters) at each grid point which will make at least one of the simulated images at that point change by more than twice the discrepancy between the unperturbed simulated image and the measured image. The SfS DEM must be provided via the -i option. The number of iterations, blending parameters (--blending-dist, etc.), and smoothness weight are ignored. Results are not computed at image pixels in shadow. This produces <output prefix>-height-error.tif. No SfS DEM is computed.")
    ("height-error-params", po::value(&opt.height_error_params)->default_value(Vector2(5.0,1000.0), "5.0 1000"),
     "Specify the largest height deviation to examine (in meters), and how many samples to use from 0 to that height.")
    ("sun-positions", po::value(&opt.sun_positions_list)->default_value(""),
     "A file having on each line an image name and three values in double precision specifying the Sun position in meters in ECEF coordinates (origin is planet center). Use a space as separator. If not provided, these will be read from the camera files for ISIS and CSM models.")
    ("shadow-thresholds", po::value(&opt.shadow_thresholds)->default_value(""),
     "Optional shadow thresholds for the input images (a list of real values in quotes, one per image).")
    ("shadow-threshold", po::value(&opt.shadow_threshold)->default_value(-1),
     "A shadow threshold to apply to all images instead of using individual thresholds. (Must be positive.)")
    ("custom-shadow-threshold-list", po::value(&opt.custom_shadow_threshold_list)->default_value(""),
     "A list having one image and one shadow threshold per line. For the images specified here, override the shadow threshold supplied by other means with this value.")
    ("max-valid-image-vals", po::value(&opt.max_valid_image_vals)->default_value(""),
     "Optional values for the largest valid image value in each image (a list of real values in quotes, one per image).")
    ("robust-threshold", po::value(&opt.robust_threshold)->default_value(-1.0),
     "If positive, set the threshold for the robust measured-to-simulated intensity difference (using the Cauchy loss). Any difference much larger than this will be penalized. A good value may be 5% to 25% of the average image value or the same fraction of the computed image exposure values.")
    ("albedo-constraint-weight", po::value(&opt.albedo_constraint_weight)->default_value(0),
     "If floating the albedo, a larger value will try harder to keep the optimized albedo close to the nominal value of 1.")
    ("albedo-robust-threshold", po::value(&opt.albedo_robust_threshold)->default_value(0),
     "If floating the albedo and this threshold is positive, apply a Cauchy loss with this threshold to the product of the albedo difference and the albedo constraint weight.")
    ("unreliable-intensity-threshold", po::value(&opt.unreliable_intensity_threshold)->default_value(0.0),
     "Intensities lower than this will be considered unreliable and given less weight.")
    ("skip-images", po::value(&opt.skip_images_str)->default_value(""), "Skip images with these indices (indices start from 0).")
    ("save-dem-with-nodata",   po::bool_switch(&opt.save_dem_with_nodata)->default_value(false)->implicit_value(true),
     "Save a copy of the DEM while using a no-data value at a DEM grid point where all images show shadows. To be used if shadow thresholds are set.")
    ("use-approx-camera-models",   po::bool_switch(&opt.use_approx_camera_models)->default_value(false)->implicit_value(true),
     "Use approximate camera models for speed. Only with ISIS .cub cameras.")
    ("use-rpc-approximation",   po::bool_switch(&opt.use_rpc_approximation)->default_value(false)->implicit_value(true),
     "Use RPC approximations for the camera models instead of approximate tabulated camera models (invoke with --use-approx-camera-models). This is broken and should not be used.")
    ("rpc-penalty-weight", po::value(&opt.rpc_penalty_weight)->default_value(0.1),
     "The RPC penalty weight to use to keep the higher-order RPC coefficients small, if the RPC model approximation is used. Higher penalty weight results in smaller such coefficients.")
    ("rpc-max-error", po::value(&opt.rpc_max_error)->default_value(2),
     "Skip the current camera if the maximum error between a camera model and its RPC approximation is larger than this.")
    ("use-semi-approx",   po::bool_switch(&opt.use_semi_approx)->default_value(false)->implicit_value(true),
     "This is an undocumented experiment.")
    ("coarse-levels", po::value(&opt.coarse_levels)->default_value(0),
     "Solve the problem on a grid coarser than the original by a factor of 2 to this power, then refine the solution on finer grids. It is suggested to not use this option.")
    ("max-coarse-iterations", po::value(&opt.max_coarse_iterations)->default_value(10),
     "How many iterations to do at levels of resolution coarser than the final result.")
    ("crop-input-images",   po::bool_switch(&opt.crop_input_images)->default_value(false)->implicit_value(true),
     "Crop the images to a region that was computed to be large enough, and keep them fully in memory, for speed.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(0),
     "Give less weight to image pixels close to no-data or boundary values. Enabled only when crop-input-images is true, for performance reasons. Blend over this many pixels.")
    ("blending-power", po::value(&opt.blending_power)->default_value(2.0),
     "A higher value will result in smoother blending.")
    ("min-blend-size", po::value(&opt.min_blend_size)->default_value(0),
     "Do not apply blending in shadowed areas of dimensions less than this.")
    ("allow-borderline-data",   po::bool_switch(&opt.allow_borderline_data)->default_value(false)->implicit_value(true),
     "At the border of the region where there are no lit pixels in any images, do not let the blending weights decay to 0. This noticeably improves the level of detail. The sfs_blend tool may need to be used to further tune this region.")
    ("steepness-factor", po::value(&opt.steepness_factor)->default_value(1.0),
     "Try to make the terrain steeper by this factor. This is not recommended in regular use.")
    ("curvature-in-shadow", po::value(&opt.curvature_in_shadow)->default_value(0.0),
     "Attempt to make the curvature of the DEM (the Laplacian) at points in shadow in all images equal to this value, which should make the DEM curve down.")
    ("curvature-in-shadow-weight", po::value(&opt.curvature_in_shadow_weight)->default_value(0.0),
     "The weight to give to the curvature in shadow constraint.")
    ("lit-curvature-dist", po::value(&opt.lit_curvature_dist)->default_value(0.0),
     "If using a curvature in shadow, start phasing it in this far from the shadow boundary in the lit region (in units of pixels).")
    ("shadow-curvature-dist", po::value(&opt.shadow_curvature_dist)->default_value(0.0),
     "If using a curvature in shadow, have it fully phased in this far from shadow boundary in the shadow region (in units of pixels).")
    ("image-exposures-prefix", po::value(&opt.image_exposure_prefix)->default_value(""),
     "Use this prefix to optionally read initial exposures (filename is <prefix>-exposures.txt).")
    ("model-coeffs-prefix", po::value(&opt.model_coeffs_prefix)->default_value(""),
     "Use this prefix to optionally read model coefficients from a file (filename is <prefix>-model_coeffs.txt).")
    ("model-coeffs", po::value(&opt.model_coeffs)->default_value(""),
     "Use the model coefficients specified as a list of numbers in quotes. Lunar-Lambertian: O, A, B, C, e.g., '1 -0.019 0.000242 -0.00000146'. Hapke: omega, b, c, B0, h, e.g., '0.68 0.17 0.62 0.52 0.52'. Charon: A, f(alpha), e.g., '0.7 0.63'.")
    ("num-haze-coeffs", po::value(&opt.num_haze_coeffs)->default_value(0),
     "Set this to 1 to model the problem as image = exposure * albedo * reflectance + haze, where haze is a single value for each image. This models a small quantity of stray light entering the lens due to scattering and other effects. Use --float-haze to actually optimize the haze (it starts as 0). It will be written as <output-prefix>-haze.txt (ignore all columns of numbers in that file except the first one).")
    ("float-haze",   po::bool_switch(&opt.float_haze)->default_value(false)->implicit_value(true),
     "If specified, float the haze coefficients as part of the optimization, if haze is modeled, so if --num-haze-coeffs is 1.")
    ("haze-prefix", po::value(&opt.image_haze_prefix)->default_value(""),
     "Use this prefix to read initial haze values (filename is <haze-prefix>-haze.txt). The file format is the same as what the tool writes itself, when triggered by the earlier options. If haze is modeled, it will be initially set to 0 unless read from such a file, and will be floated or not depending on whether --float-haze is on. The final haze values will be saved to <output prefix>-haze.txt.")
    ("init-dem-height", po::value(&opt.init_dem_height)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this value for initial DEM heights (measured in meters, relative to the datum). "
     "An input DEM still needs to be provided for georeference information.")
    ("crop-win", po::value(&opt.crop_win)->default_value(BBox2i(0, 0, 0, 0), "xoff yoff xsize ysize"),
     "Crop the input DEM to this region before continuing.")
    ("nodata-value", po::value(&opt.nodata_val)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this as the DEM no-data value, over-riding what is in the initial guess DEM.")
    ("float-dem-at-boundary",   po::bool_switch(&opt.float_dem_at_boundary)->default_value(false)->implicit_value(true),
     "Allow the DEM values at the boundary of the region to also float (not advised).")
    ("boundary-fix",   po::bool_switch(&opt.boundary_fix)->default_value(false)->implicit_value(true),
     "An attempt to let the DEM float at the boundary.")
    ("fix-dem",   po::bool_switch(&opt.fix_dem)->default_value(false)->implicit_value(true),
     "Do not float the DEM at all. Useful when floating the model params.")
    ("float-reflectance-model",   po::bool_switch(&opt.float_reflectance_model)->default_value(false)->implicit_value(true),
     "Allow the coefficients of the reflectance model to float (not recommended).")
    ("float-sun-position",   po::bool_switch(&opt.float_sun_position)->default_value(false)->implicit_value(true),
     "Allow the position of the sun to float.")
    ("integrability-constraint-weight", po::value(&opt.integrability_weight)->default_value(0.0),
     "Use the integrability constraint from Horn 1990 with this value of its weight.")
    ("smoothness-weight-pq", po::value(&opt.smoothness_weight_pq)->default_value(0.00),
     "Smoothness weight for p and q, when the integrability constraint "
     "is used. A larger value will result in a smoother solution "
     "(experimental).")
    ("query",   po::bool_switch(&opt.query)->default_value(false)->implicit_value(true),
     "Print some info and exit. Invoked from parallel_sfs.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("gradient-weight", po::value(&opt.gradient_weight)->default_value(0.0),
     "The weight given to the cost function term which consists of sums "
     "of squares of first-order derivatives. A larger value will result "
     "in shallower slopes but less noise. This can be used in conjunction with "
     "--smoothness-weight. It is suggested to experiment with this "
     "with a value of 0.0001 - 0.01, while reducing the "
     "smoothness weight to a very small value.")
    ("save-sparingly",   po::bool_switch(&opt.save_sparingly)->default_value(false)->implicit_value(true),
     "Avoid saving any results except the adjustments and the DEM, as that's a lot of files.")
    ("camera-position-step-size", po::value(&opt.camera_position_step_size)->default_value(1.0),
     "Larger step size will result in more aggressiveness in varying the camera position if it is being floated (which may result in a better solution or in divergence).");

  general_options.add( vw::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-images", po::value(&opt.input_images));

  po::positional_options_description positional_desc;
  positional_desc.add("input-images", -1);

  std::string usage("-i <input DEM> -n <max iterations> -o <output prefix> <images> [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);


  if (opt.float_all_cameras)
    opt.float_cameras = true;

  std::istringstream idem(opt.input_dems_str);
  std::string dem;
  while (idem >> dem) opt.input_dems.push_back(dem);
    
  // Sanity checks. Put this early, before separating images from cameras, as that
  // function can print a message not reflecting the true issue of missing the DEM.
  if (opt.input_dems.empty())
    vw_throw( ArgumentErr() << "Missing input DEM(s).\n"
              << usage << general_options );

  // Separate the cameras from the images
  std::vector<std::string> inputs = opt.input_images;

  if (!opt.image_list.empty()) {
    // Read the images and cameras and put them in 'inputs' to be parsed later
    if (opt.camera_list.empty())
      vw_throw(ArgumentErr()
               << "The option --image-list must be invoked together with --camera-list.\n");
    if (!inputs.empty())
      vw_throw(ArgumentErr() << "The option --image-list was specified, but also "
               << "images or cameras on the command line.\n");
    asp::read_list(opt.image_list, inputs);
    std::vector<std::string> tmp;
    asp::read_list(opt.camera_list, tmp);
    for (size_t it = 0; it < tmp.size(); it++) 
      inputs.push_back(tmp[it]);
  }
  
  bool ensure_equal_sizes = true;
  asp::separate_images_from_cameras(inputs,
                                    opt.input_images, opt.input_cameras, // outputs
                                    ensure_equal_sizes); 
  
  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
              << usage << general_options );

  if (opt.max_iterations < 0)
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
              << usage << general_options );

  if (opt.input_images.empty())
    vw_throw( ArgumentErr() << "Missing input images.\n"
              << usage << general_options );

  if (opt.smoothness_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative smoothness weight.\n");

  if (opt.gradient_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative gradient weight.\n");

  if (opt.integrability_weight < 0) 
    vw_throw(ArgumentErr() << "Expecting a non-negative integrability weight.\n");

  if (opt.float_sun_position)
    vw_throw(ArgumentErr() << "Floating sun positions is currently disabled.\n");
  
  if ( opt.float_haze && opt.num_haze_coeffs == 0 ) 
    vw_throw(ArgumentErr() << "Haze cannot be floated unless there is at least one haze coefficient.\n");
  if ( opt.image_haze_prefix != "" && opt.num_haze_coeffs == 0  )
    vw_throw(ArgumentErr() << "Haze cannot be read unless there is at least one haze coefficient.\n");
  
  if (opt.use_rpc_approximation) 
    vw_throw(ArgumentErr() << "The RPC approximation is broken.\n");

  // When we use approximate cameras, and the cameras are fixed, use an approximation
  // for the adjusted camera rather than for the unadjusted one. This uses less memory.
  if (opt.use_approx_camera_models && !opt.float_cameras &&
      !opt.use_rpc_approximation && !opt.use_semi_approx) {
    opt.use_approx_camera_models = false;
    opt.use_approx_adjusted_camera_models = true;
  }

  // Curvature in shadow params
  if (opt.curvature_in_shadow < 0.0 || opt.curvature_in_shadow_weight < 0.0) 
    vw_throw(ArgumentErr() << "Cannot have negative curvature in shadow or its weight.\n");
  if (opt.lit_curvature_dist < 0.0 || opt.shadow_curvature_dist < 0.0) 
    vw_throw(ArgumentErr() << "Cannot have negative curvature distances.\n");    
  if (opt.curvature_in_shadow > 0.0 &&
      opt.shadow_curvature_dist + opt.lit_curvature_dist <= 0.0)
    vw_throw(ArgumentErr() << "When modeling curvature in shadow, expecting a "
             << "positive value of shadow-curvature-dist or list-curvature-dist.\n");    

  if (opt.steepness_factor <= 0.0) 
    vw_throw(ArgumentErr() << "The steepness factor must be positive.\n");    
      
  if (opt.compute_exposures_only){
    if (opt.use_approx_camera_models ||
        opt.use_approx_adjusted_camera_models ||
        opt.use_rpc_approximation ||
        opt.crop_input_images ||
        opt.use_semi_approx) {
      vw_out(WarningMessage) << "When computing exposures only, not using approximate "
                             << "camera models or cropping input images.\n";
      opt.use_approx_camera_models = false;
      opt.use_approx_adjusted_camera_models = false;
      opt.use_rpc_approximation = false;
      opt.crop_input_images = false;
      opt.use_semi_approx = false;
      opt.blending_dist = 0;
      opt.allow_borderline_data = false;
    }

    if (!opt.crop_win.empty()) {
      vw_throw(ArgumentErr() << "When computing exposures only, cannot crop the "
               << "input DEM as that will give wrong results. Use the full DEM.\n");
    }
  }
  
  if (opt.blending_dist > 0 && !opt.crop_input_images) 
    vw_throw(ArgumentErr() << "A blending distance is only supported with --crop-input-images.\n");
  
  if (opt.crop_input_images && (opt.float_cameras || opt.float_all_cameras))  
    vw_throw( ArgumentErr()
              << "Using cropped input images implies that the cameras are not floated.\n" );

  if (opt.allow_borderline_data && !opt.crop_input_images)
    vw_throw(ArgumentErr() << "Option --allow-borderline-data needs option "
             << "--crop-input-images.\n");

  if (opt.allow_borderline_data && opt.blending_dist <= 0) 
    vw::vw_throw(vw::ArgumentErr()
                 << "Option --allow-borderline-data needs a positive --blending-dist.\n");

  if (opt.allow_borderline_data && opt.coarse_levels > 0) 
    vw::vw_throw(vw::ArgumentErr() << "Option --allow-borderline-data cannot be "
                 << "used with multiple coarseness levels.\n");
  
  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Parse the shadow thresholds
  std::istringstream ist(opt.shadow_thresholds);
  opt.shadow_threshold_vec.clear();
  float val;
  while (ist >> val)
    opt.shadow_threshold_vec.push_back(val);
  if (!opt.shadow_threshold_vec.empty() &&
      opt.shadow_threshold_vec.size() != opt.input_images.size())
    vw_throw(ArgumentErr()
             << "If specified, there must be as many shadow thresholds as images.\n");

  // See if to use opt.shadow_threshold.
  if (opt.shadow_threshold > 0) {
    if (!opt.shadow_threshold_vec.empty())
      vw_throw(ArgumentErr()
               << "Cannot specify both --shadow-threshold and --shadow-thresholds.\n");
    while (opt.shadow_threshold_vec.size() < opt.input_images.size())
      opt.shadow_threshold_vec.push_back(opt.shadow_threshold);
  }

  // Default thresholds are the smallest float.
  // Maybe it should be 0?
  if (opt.shadow_threshold_vec.empty()) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      opt.shadow_threshold_vec.push_back(-std::numeric_limits<float>::max());
    }
  }

  // Override some shadow thresholds with custom versions
  if (!opt.custom_shadow_threshold_list.empty()) {
    std::map<std::string, double> custom_thresh;
    std::ifstream ifs(opt.custom_shadow_threshold_list);
    std::string image;
    double val;
    while (ifs >> image >> val) {
      custom_thresh[image] = val;
    }
    
    if (custom_thresh.empty()) 
      vw_throw(ArgumentErr() << "Could not read any data from: "
               << opt.custom_shadow_threshold_list << "\n");
    
    for (size_t it = 0; it < opt.input_images.size(); it++) {
      auto key = custom_thresh.find(opt.input_images[it]);
      if (key != custom_thresh.end()) {
        vw_out() << "Over-riding the shadow threshold for " << opt.input_images[it]
                 << " with: " << key->second << std::endl;
        opt.shadow_threshold_vec[it] = key->second;
      }
    }
  }
  
  // Parse max valid image vals
  std::istringstream ism(opt.max_valid_image_vals);
  opt.max_valid_image_vals_vec.clear();
  while (ism >> val)
    opt.max_valid_image_vals_vec.push_back(val);
  if (!opt.max_valid_image_vals_vec.empty() &&
      opt.max_valid_image_vals_vec.size() != opt.input_images.size())
    vw_throw(ArgumentErr()
             << "If specified, there must be as many max valid image vals as images.\n");

  if (opt.max_valid_image_vals_vec.empty()) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      opt.max_valid_image_vals_vec.push_back(std::numeric_limits<float>::max());
    }
  }

  for (size_t i = 0; i < opt.input_images.size(); i++) {
    vw_out() << "Shadow threshold and max valid value for " << opt.input_images[i] << ' '
             << opt.shadow_threshold_vec[i] << ' ' << opt.max_valid_image_vals_vec[i] << std::endl;
  }

  // Initial image exposures, if provided. First read them in a map,
  // as perhaps the initial exposures were created using more images
  // than what we have here. 
  std::string exposure_file = exposure_file_name(opt.image_exposure_prefix);
  opt.image_exposures_vec.clear();
  std::map<std::string, double> img2exp;
  std::string name;
  double dval;
  std::ifstream ise(exposure_file.c_str());
  int exp_count = 0;
  while (ise >> name >> dval){
    img2exp[name] = dval;
    exp_count++;
  }
  ise.close();
  if (exp_count > 0) {
    vw_out() << "Using exposures from: " << exposure_file << std::endl;
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      std::string img = opt.input_images[i];
      std::map<std::string, double>::iterator it = img2exp.find(img);
      if (it == img2exp.end()) {
        vw_throw(ArgumentErr()
                 << "Could not find the exposure for image: " << img << ".\n");
      }
      double exp_val = it->second;
      vw_out() << "Exposure for " << img << ": " << exp_val << std::endl;
      opt.image_exposures_vec.push_back(exp_val);
    }
  }
  if (opt.image_exposure_prefix != "" && exp_count == 0) 
    vw_throw(ArgumentErr()
             << "Could not find the exposures file: " << exposure_file << ".\n");
  
  if (opt.steepness_factor != 1.0) 
    vw_out() << "Making the terrain artificially steeper by factor: " << opt.steepness_factor
             << ".\n";
  
  // Initial image haze, if provided. First read them in a map,
  // as perhaps the initial haze were created using more images
  // than what we have here. 
  if (opt.num_haze_coeffs > 0) {
    std::string haze_file = haze_file_name(opt.image_haze_prefix);
    opt.image_haze_vec.clear();
    std::map< std::string, std::vector<double> > img2haze;
    std::ifstream ish(haze_file.c_str());
    int haze_count = 0;
    while(1){
      std::string line;
      std::getline(ish, line);
      std::istringstream hstream(line);
      if (! (hstream >> name) ) break;
      std::vector<double> haze_vec;
      while (hstream >> dval)
        haze_vec.push_back(dval);
      if (haze_vec.empty()) break;
      haze_count++;

      // Pad the haze vec
      while (haze_vec.size() < g_max_num_haze_coeffs) haze_vec.push_back(0);
      
      img2haze[name] = haze_vec;

      // All haze coefficients beyond the first num_haze_coeffs must
      // be zero, as that means we are reading results written with
      // different number of haze coeffs.
      for (size_t hiter = opt.num_haze_coeffs; hiter < g_max_num_haze_coeffs; hiter++) {
        if (haze_vec[hiter] != 0) 
          vw_throw(ArgumentErr() 
                   << "Found unexpected non-zero haze coefficient: " << haze_vec[hiter] << ".\n");
      }
      
    }
    ish.close();
    if (opt.image_haze_prefix != "" && haze_count == 0) 
      vw_throw(ArgumentErr()
               << "Could not find the haze file: " << haze_file << ".\n");

    
    if (haze_count > 0) {
      vw_out() << "Using haze from: " << haze_file << std::endl;
      for (size_t i = 0; i < opt.input_images.size(); i++) {
        std::string img = opt.input_images[i];
        std::map< std::string, std::vector<double> >::iterator it = img2haze.find(img);
        if (it == img2haze.end()) {
          vw_throw(ArgumentErr()
                   << "Could not find the haze for image: " << img << ".\n");
        }
        std::vector<double> haze_vec = it->second;
        vw_out() << "Haze for " << img << ":";
        for (size_t hiter = 0; hiter < haze_vec.size(); hiter++) 
          vw_out() << " " << haze_vec[hiter];
        vw_out() << std::endl;
        opt.image_haze_vec.push_back(haze_vec);
      }
    }
  }
  
  // Initial model coeffs, if passed on the command line
  if (opt.model_coeffs != "") {
    vw_out() << "Parsing model coefficients: " << opt.model_coeffs << std::endl;
    std::istringstream is(opt.model_coeffs);
    double val;
    while( is >> val){
      opt.model_coeffs_vec.push_back(val);
    }
  }

  // Initial model coefficients, if provided in the file
  if (opt.model_coeffs_prefix != "") {
    std::string model_coeffs_file = model_coeffs_file_name(opt.model_coeffs_prefix);
    vw_out() << "Reading model coefficients from file: " << model_coeffs_file << std::endl;
    std::ifstream ism(model_coeffs_file.c_str());
    opt.model_coeffs_vec.clear();
    while (ism >> dval)
      opt.model_coeffs_vec.push_back(dval);
    ism.close();
    if ( opt.model_coeffs_vec.empty()) {
      vw_throw(ArgumentErr() << "Could not read model coefficients from: " << model_coeffs_file << ".\n");
    }
  }

  if (!opt.model_coeffs_vec.empty()) {
    // Pad with zeros if needed, as the Lunar Lambertian has 4 params, while Hapke has 5 of them.
    // the Charon one has 2.
    while (opt.model_coeffs_vec.size() < g_num_model_coeffs)
      opt.model_coeffs_vec.push_back(0);

    if (opt.model_coeffs_vec.size() != g_num_model_coeffs)
      vw_throw(ArgumentErr()
               << "If specified, there must be " << g_num_model_coeffs << " coefficients.\n");
  }

  // Sanity check
  if (opt.camera_position_step_size <= 0) {
    vw_throw(ArgumentErr() << "Expecting a positive value for camera-position-step-size.\n");
  }

  if (opt.coarse_levels < 0) {
    vw_throw(ArgumentErr() << "Expecting the number of levels to be non-negative.\n");
  }

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  if (opt.input_images.size() <= 1 && opt.float_albedo && 
      opt.initial_dem_constraint_weight <= 0 && opt.albedo_constraint_weight <= 0.0)
    vw_throw(ArgumentErr()
             << "Floating the albedo is ill-posed for just one image without "
             << "the initial DEM constraint or the albedo constraint.\n");

  if (opt.input_images.size() <= 1 && opt.float_exposure && opt.initial_dem_constraint_weight <= 0)
    vw_throw(ArgumentErr()
             << "Floating the exposure is ill-posed for just one image.\n");

  if (opt.input_images.size() <= 1 && opt.float_dem_at_boundary)
    vw_throw(ArgumentErr()
             << "Floating the DEM at the boundary is ill-posed for just one image.\n");

  if (opt.boundary_fix && opt.integrability_weight == 0) {
    vw_throw(ArgumentErr()
             << "The boundary fix only works with the integrability constraint.\n");
  }
  
  // Start with given images to skip. Later, for each dem clip, we may
  // skip more, if those images do not overlap with the clip.
  int num_dems = opt.input_dems.size();
  opt.skip_images.resize(num_dems);
  std::set<int> curr_skip_images;
  if (opt.skip_images_str != "") {
    std::istringstream is(opt.skip_images_str);
    int val;
    while( is >> val)
      curr_skip_images.insert(val);
  }
  for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
    opt.skip_images[dem_iter] = curr_skip_images;
  }

  if (opt.estimate_slope_errors && opt.estimate_height_errors) 
    vw_throw( ArgumentErr() << "Cannot estimate both slope and height error at the same time.");

  if (opt.estimate_height_errors && opt.model_shadows) 
    vw_throw( ArgumentErr() << "Cannot estimate height error when modeling shadows.");
  
  if (opt.save_computed_intensity_only || opt.estimate_slope_errors || opt.estimate_height_errors){
    if (opt.max_iterations > 0 || opt.max_coarse_iterations > 0){
      vw_out(WarningMessage) << "Using 0 iterations.\n";
      opt.max_iterations = 0;
      opt.max_coarse_iterations = 0;
    }
    //if (!opt.model_shadows) 
    //  vw_out(WarningMessage) << "It is suggested that --model-shadows be used.\n";

    if (opt.coarse_levels > 0) {
      vw_out(WarningMessage) << "Using 0 coarse levels.\n";
      opt.coarse_levels = 0;
    }

    if (opt.use_approx_camera_models ||
        opt.use_approx_adjusted_camera_models ||
        opt.use_rpc_approximation ||
        opt.crop_input_images) {
      vw_out(WarningMessage) << "Not using approximate camera models or cropping input images.\n";
      opt.use_approx_camera_models = false;
      opt.use_approx_adjusted_camera_models = false;
      opt.use_rpc_approximation = false;
      opt.crop_input_images = false;
      opt.use_semi_approx = false;
    }
    
    //if (opt.image_exposures_vec.empty())
    //  vw_throw( ArgumentErr()
    // << "Expecting the exposures to be computed and passed in.\n" );
    
    if (opt.num_haze_coeffs > 0 && opt.image_haze_vec.empty())
      vw_throw( ArgumentErr()
                << "Expecting the haze to be computed and passed in.\n" );
  }
  
  
}

// Run sfs at a given coarseness level
void run_sfs_level(// Fixed inputs
                   int num_iterations, Options & opt,
                   std::vector<GeoReference> const& geo,
                   double smoothness_weight,
                   double dem_nodata_val,
                   std::vector< std::vector<BBox2i>>     const& crop_boxes,
                   std::vector< std::vector<MaskedImgT>> const& masked_images,
                   std::vector< std::vector<DoubleImgT>> const& blend_weights,
                   GlobalParams const& global_params,
                   std::vector<ModelParams> const & model_params,
                   std::vector< ImageView<double>> const& orig_dems, 
                   double initial_albedo,
                   ImageView<int> const& lit_image_mask,
                   ImageView<double> const& curvature_in_shadow_weight,
                   // Quantities that will float
                   std::vector< ImageView<double>> & dems,
                   std::vector< ImageView<double>> & albedos,
                   std::vector< std::vector<boost::shared_ptr<CameraModel>>> & cameras,
                   std::vector<double> & exposures,
                   std::vector< std::vector<double>> & haze,
                   std::vector<double> & scaled_sun_posns,
                   std::vector<double> & adjustments,
                   std::vector<double> & reflectance_model_coeffs){

  int num_images = opt.input_images.size();
  int num_dems   = dems.size();
  ceres::Problem problem;
  
  // Find the grid sizes in meters. Note that dem heights are in
  // meters too, so we treat both horizontal and vertical measurements
  // in same units.
  double gridx, gridy;
  compute_grid_sizes_in_meters(dems[0], geo[0], dem_nodata_val, gridx, gridy);
  vw_out() << "grid in x and y in meters: "
           << gridx << ' ' << gridy << std::endl;
  g_gridx = &gridx;
  g_gridy = &gridy;

  std::vector<double> max_dem_height(num_dems, -std::numeric_limits<double>::max());
  if (opt.model_shadows) {
    // Find the max DEM height
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) { 
      double curr_max_dem_height = -std::numeric_limits<double>::max();
      for (int col = 0; col < dems[dem_iter].cols(); col++) {
        for (int row = 0; row < dems[dem_iter].rows(); row++) {
          if (dems[dem_iter](col, row) > curr_max_dem_height) {
            curr_max_dem_height = dems[dem_iter](col, row);
          }
        }
      }
      max_dem_height[dem_iter] = curr_max_dem_height;
    }
  }
  g_max_dem_height = &max_dem_height;

  // See if a given image is used in at least one clip or skipped in
  // all of them
  std::vector<bool> use_image(num_images, false);
  int num_used = 0;
  for (int image_iter = 0; image_iter < num_images; image_iter++) {
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      if (opt.skip_images[dem_iter].find(image_iter) == opt.skip_images[dem_iter].end()){
        use_image[image_iter] = true;
        num_used++;
      }
    }
  }

  // We define p and q as the partial derivatives in x in y of the dem.
  // When using the integrability constraint, they are floated as variables
  // in their own right, while constrained to not go too far from the DEM.
  std::vector< ImageView<Vector2> > pq;  
  pq.resize(num_dems);
  if (opt.integrability_weight > 0) {
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      pq[dem_iter].set_size(dems[dem_iter].cols(), dems[dem_iter].rows());

      for (int col = 1; col < dems[dem_iter].cols()-1; col++) {
        for (int row = 1; row < dems[dem_iter].rows()-1; row++) {
          // Note that the top value is dems[dem_iter](col, row-1) and the 
          //            bottom value is dems[dem_iter](col, row+1).
          pq[dem_iter](col, row)[0] // same as (right - left)/(2*gridx)
            = (dems[dem_iter](col+1, row) - dems[dem_iter](col-1, row))/(2*gridx);
          pq[dem_iter](col, row)[1] // same as (top - bottom)/(2*gridy)
            = (dems[dem_iter](col, row-1) - dems[dem_iter](col, row+1))/(2*gridy);
        }
      }
    }
  }

  // Use a simpler cost function if only the DEM is floated. Not sure how much
  // of speedup that gives.
  bool float_dem_only = true;
  if (opt.float_albedo || opt.float_exposure || opt.float_cameras ||
      opt.float_all_cameras || opt.float_dem_at_boundary ||
      opt.boundary_fix || opt.fix_dem || opt.float_reflectance_model ||
      opt.float_sun_position || opt.float_haze || opt.integrability_weight > 0){
    float_dem_only = false;
  }
  
  std::set<int> use_dem, use_albedo; // to avoid a crash in Ceres when a param is fixed but not set
  
  for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
    
    int bd = 1;
    if (opt.boundary_fix) bd = 0;
    
    // Add a residual block for every grid point not at the boundary
    for (int col = bd; col < dems[dem_iter].cols()-bd; col++) {
      for (int row = bd; row < dems[dem_iter].rows()-bd; row++) {
        
        // Intensity error for each image
        for (int image_iter = 0; image_iter < num_images; image_iter++) {

          if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) {
            continue;
          }
          
          ceres::LossFunction* loss_function_img = NULL;
          if (opt.robust_threshold > 0) 
            loss_function_img = new ceres::CauchyLoss(opt.robust_threshold);
          
          if (float_dem_only) {
            ceres::CostFunction* cost_function_img =
              IntensityErrorFloatDemOnly::Create(col, row,
                                                 dems[dem_iter],
                                                 albedos[dem_iter](col, row), 
                                                 &reflectance_model_coeffs[0],
                                                 &exposures[image_iter],      // exposure
                                                 &haze[image_iter][0],        // haze
                                                 &adjustments[6*image_iter],  // camera adjustments
                                                 geo[dem_iter],
                                                 opt.model_shadows,
                                                 opt.camera_position_step_size,
                                                 max_dem_height[dem_iter],
                                                 gridx, gridy,
                                                 global_params, model_params[image_iter],
                                                 crop_boxes[dem_iter][image_iter],
                                                 masked_images[dem_iter][image_iter],
                                                 blend_weights[dem_iter][image_iter],
                                                 &scaled_sun_posns[3*image_iter], // sun positions
                                                 cameras[dem_iter][image_iter]);
            problem.AddResidualBlock(cost_function_img, loss_function_img,
                                     &dems[dem_iter](col-1, row),  // left
                                     &dems[dem_iter](col, row),    // center
                                     &dems[dem_iter](col+1, row),  // right
                                     &dems[dem_iter](col, row+1),  // bottom
                                     &dems[dem_iter](col, row-1)  // top
                                     );
            use_dem.insert(dem_iter); 
            
          }else if (opt.integrability_weight == 0){
            ceres::CostFunction* cost_function_img =
              IntensityError::Create(col, row, dems[dem_iter], geo[dem_iter],
                                     opt.model_shadows,
                                     opt.camera_position_step_size,
                                     max_dem_height[dem_iter],
                                     gridx, gridy,
                                     global_params, model_params[image_iter],
                                     crop_boxes[dem_iter][image_iter],
                                     masked_images[dem_iter][image_iter],
                                     blend_weights[dem_iter][image_iter],
                                     &scaled_sun_posns[3*image_iter], // sun positions
                                     cameras[dem_iter][image_iter]);
            problem.AddResidualBlock(cost_function_img, loss_function_img,
                                     &exposures[image_iter],       // exposure
                                     &haze[image_iter][0],         // haze
                                     &dems[dem_iter](col-1, row),  // left
                                     &dems[dem_iter](col, row),    // center
                                     &dems[dem_iter](col+1, row),  // right
                                     &dems[dem_iter](col, row+1),  // bottom
                                     &dems[dem_iter](col, row-1),  // top
                                     &albedos[dem_iter](col, row), // albedo
                                     &adjustments[6*image_iter],   // camera
                                     //&scaled_sun_posns[3*image_iter], // sun positions
                                     &reflectance_model_coeffs[0]);
            use_dem.insert(dem_iter); 
            use_albedo.insert(dem_iter);
          } else {
            // Use the integrability constraint
            ceres::CostFunction* cost_function_img =
              IntensityErrorPQ::Create(col, row, dems[dem_iter], geo[dem_iter],
                                       opt.model_shadows,
                                       opt.camera_position_step_size,
                                       max_dem_height[dem_iter],
                                       gridx, gridy,
                                       global_params, model_params[image_iter],
                                       crop_boxes[dem_iter][image_iter],
                                       masked_images[dem_iter][image_iter],
                                       blend_weights[dem_iter][image_iter],
                                       cameras[dem_iter][image_iter]);
            problem.AddResidualBlock(cost_function_img, loss_function_img,
                                     &exposures[image_iter],          // exposure
                                     &haze[image_iter][0],            // haze
                                     &dems[dem_iter](col, row),       // center
                                     &pq[dem_iter](col, row)[0],      // pq
                                     &albedos[dem_iter](col, row),    // albedo
                                     &adjustments[6*image_iter],      // camera
                                     &scaled_sun_posns[3*image_iter], // sun positions
                                     &reflectance_model_coeffs[0]);   // reflectance 
            
            
            use_dem.insert(dem_iter); 
            use_albedo.insert(dem_iter);
          }
          
        } // end iterating over images
        
        if (col > 0 && col < dems[dem_iter].cols()-1 &&
            row > 0 && row < dems[dem_iter].rows()-1 ) {
          
          // Smoothness penalty. We always add this, even if the weight is 0,
          // to make Ceres not complain about blocks not being set. 
          ceres::LossFunction* loss_function_sm = NULL;
          ceres::CostFunction* cost_function_sm =
            SmoothnessError::Create(smoothness_weight, gridx, gridy);
          problem.AddResidualBlock(cost_function_sm, loss_function_sm,
                                   &dems[dem_iter](col-1, row+1),  // bottom left
                                   &dems[dem_iter](col, row+1),    // bottom 
                                   &dems[dem_iter](col+1, row+1),  // bottom right
                                   &dems[dem_iter](col-1, row  ),  // left
                                   &dems[dem_iter](col, row  ),    // center
                                   &dems[dem_iter](col+1, row  ),  // right 
                                   &dems[dem_iter](col-1, row-1),  // top left
                                   &dems[dem_iter](col, row-1),    // top
                                   &dems[dem_iter](col+1, row-1)); // top right

          // Add curvature in shadow. Note that we use a per-pixel curvature_in_shadow_weight,
          // to gradually phase it in to avoid artifacts.
          if (opt.curvature_in_shadow_weight > 0.0 && curvature_in_shadow_weight(col, row) > 0) {
            ceres::LossFunction* loss_function_cv = NULL;
            ceres::CostFunction* cost_function_cv =
              CurvatureInShadowError::Create(opt.curvature_in_shadow,
                                             curvature_in_shadow_weight(col, row),
                                             gridx, gridy);
            problem.AddResidualBlock(cost_function_cv, loss_function_cv,
                                     &dems[dem_iter](col,   row+1),  // bottom 
                                     &dems[dem_iter](col-1, row),    // left
                                     &dems[dem_iter](col,   row),    // center
                                     &dems[dem_iter](col+1, row),    // right 
                                     &dems[dem_iter](col,   row-1)); // top
          }

          // Add gradient weight
          if (opt.gradient_weight > 0.0) {
            ceres::LossFunction* loss_function_grad = NULL;
            ceres::CostFunction* cost_function_grad =
              GradientError::Create(opt.gradient_weight, gridx, gridy);
            problem.AddResidualBlock(cost_function_grad, loss_function_grad,
                                     &dems[dem_iter](col,   row+1),  // bottom 
                                     &dems[dem_iter](col-1, row),    // left
                                     &dems[dem_iter](col,   row),    // center
                                     &dems[dem_iter](col+1, row),    // right 
                                     &dems[dem_iter](col,   row-1)); // top
          }
        
          if (opt.integrability_weight > 0) {
            ceres::LossFunction* loss_function_int = NULL;
            ceres::CostFunction* cost_function_int =
              IntegrabilityError::Create(opt.integrability_weight, gridx, gridy);
            problem.AddResidualBlock(cost_function_int, loss_function_int,
                                     &dems[dem_iter](col,   row+1),   // bottom
                                     &dems[dem_iter](col-1, row),     // left
                                     &dems[dem_iter](col+1, row),     // right
                                     &dems[dem_iter](col,   row-1),   // top
                                     &pq[dem_iter]  (col,   row)[0]); // pq

            if (opt.smoothness_weight_pq > 0) {
              ceres::LossFunction* loss_function_sm_pq = NULL;
              ceres::CostFunction* cost_function_sm_pq =
                SmoothnessErrorPQ::Create(opt.smoothness_weight_pq, gridx, gridy);
              problem.AddResidualBlock(cost_function_sm_pq, loss_function_sm_pq,
                                       &pq[dem_iter](col, row+1)[0],  // bottom 
                                       &pq[dem_iter](col-1, row)[0],  // left
                                       &pq[dem_iter](col+1, row)[0],  // right 
                                       &pq[dem_iter](col, row-1)[0]); // top
            }
          }
          
          use_dem.insert(dem_iter); 
          
          // Deviation from prescribed height constraint
          if (opt.initial_dem_constraint_weight > 0) {
            ceres::LossFunction* loss_function_hc = NULL;
            ceres::CostFunction* cost_function_hc =
              HeightChangeError::Create(orig_dems[dem_iter](col, row),
                                        opt.initial_dem_constraint_weight);
            problem.AddResidualBlock(cost_function_hc, loss_function_hc,
                                     &dems[dem_iter](col, row));
            use_dem.insert(dem_iter); 
          }
          
          // Deviation from prescribed albedo
          if (opt.float_albedo > 0 && opt.albedo_constraint_weight > 0) {
            
            ceres::LossFunction* loss_function_hc = NULL;
            if (opt.albedo_robust_threshold > 0)
              loss_function_hc = new ceres::CauchyLoss(opt.albedo_robust_threshold);
            ceres::CostFunction* cost_function_hc =
              AlbedoChangeError::Create(initial_albedo,
                                        opt.albedo_constraint_weight);
            problem.AddResidualBlock(cost_function_hc, loss_function_hc,
                                     &albedos[dem_iter](col, row));
            use_albedo.insert(dem_iter);
          }
        }
        
      } // end row iter
    } // end col iter
    
    // DEM at the boundary must be fixed.
    if (!opt.float_dem_at_boundary) {
      for (int col = 0; col < dems[dem_iter].cols(); col++) {
        for (int row = 0; row < dems[dem_iter].rows(); row++) {
          if (col == 0 || col == dems[dem_iter].cols() - 1 ||
              row == 0 || row == dems[dem_iter].rows() - 1 ) {
            if (use_dem.find(dem_iter) != use_dem.end())
              problem.SetParameterBlockConstant(&dems[dem_iter](col, row));
          }
        }
      }
    }
    
    if (opt.fix_dem) {
      for (int col = 0; col < dems[dem_iter].cols(); col++) {
        for (int row = 0; row < dems[dem_iter].rows(); row++) {
          if (use_dem.find(dem_iter) != use_dem.end())
            problem.SetParameterBlockConstant(&dems[dem_iter](col, row));
        }
      }
    }
    
    if (opt.initial_dem_constraint_weight <= 0 && num_used <= 1) {    

      if (opt.float_albedo && opt.albedo_constraint_weight <= 0) {
        vw_out() << "No DEM or albedo constraint is used, and there is at most one "
                 << "usable image. Fixing the albedo.\n";
        opt.float_albedo = false;
      }

      // If there's just one image, don't float the exposure, as the
      // problem is under-determined. If we float the albedo, we will
      // implicitly float the exposure, hence keep the exposure itself
      // fixed.
      if (opt.float_exposure) {
        vw_out() << "No DEM constraint is used, and there is at most one "
                 << "usable image. Fixing the exposure.\n";
        opt.float_exposure = false;
      }
    }
    
    // If to float the albedo
    if (!float_dem_only) {
      for (int col = 1; col < dems[dem_iter].cols() - 1; col++) {
        for (int row = 1; row < dems[dem_iter].rows() - 1; row++) {
          if (!opt.float_albedo && num_used > 0 && use_albedo.find(dem_iter) != use_albedo.end())
            problem.SetParameterBlockConstant(&albedos[dem_iter](col, row));
        }
      }
    }
    
  } // end iterating over DEMs

  if (!float_dem_only) {

    // If floating the DEM only, none of the below parameters are even added to the problem,
    // it does not make sense to check to keep them fixed or floating them.
    
    if (!opt.float_exposure){
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter]) problem.SetParameterBlockConstant(&exposures[image_iter]);
      }
    }
    if (!opt.float_haze){
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        if (use_image[image_iter]) problem.SetParameterBlockConstant(&haze[image_iter][0]);
      }
    }
    
    if (!opt.float_cameras) {
      vw_out() << "Not floating cameras." << std::endl;
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        if (use_image[image_iter]){
          problem.SetParameterBlockConstant(&adjustments[6*image_iter]);
        }
      }
    }else if (!opt.float_all_cameras){
      // Fix the first camera, let the other ones conform to it.
      // TODO: This needs further study.
      vw_out() << "Floating all cameras sans the first one." << std::endl;
      int image_iter = 0;
      if (use_image[image_iter]){
        problem.SetParameterBlockConstant(&adjustments[6*image_iter]);
      }
      
    }else{
      vw_out() << "Floating all cameras, including the first one." << std::endl;
    }

    // If to float the reflectance model coefficients
    if (!opt.float_reflectance_model && num_used > 0) {
      problem.SetParameterBlockConstant(&reflectance_model_coeffs[0]);
    }
    
    if (!opt.float_sun_position){
      if (opt.integrability_weight != 0){
        for (int image_iter = 0; image_iter < num_images; image_iter++) {
          if (use_image[image_iter]) problem.SetParameterBlockConstant(&scaled_sun_posns[3*image_iter]);
        }
      }
    }
  }
  
  if (opt.num_threads > 1 &&
      opt.stereo_session == "isis"  &&
      !opt.use_approx_camera_models &&
      !opt.use_approx_adjusted_camera_models) {
    vw_out() << "Using exact ISIS camera models. Can run with only a single thread.\n";
    opt.num_threads = 1;
  }

  vw_out() << "Using: " << opt.num_threads << " thread(s).\n";

  ceres::Solver::Options options;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;
  options.max_num_iterations = num_iterations;
  options.minimizer_progress_to_stdout = 1;
  options.num_threads = opt.num_threads;
  options.linear_solver_type = ceres::SPARSE_SCHUR;

  // Use a callback function at every iteration
  SfsCallback callback;
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  // A bunch of global variables to use in the callback
  g_dem            = &dems;
  g_pq             = &pq;
  g_albedo         = &albedos;
  g_geo            = &geo;
  g_global_params  = &global_params;
  g_model_params   = &model_params;
  g_crop_boxes     = &crop_boxes;
  g_masked_images  = &masked_images;
  g_blend_weights  = &blend_weights;
  g_cameras        = &cameras;
  g_iter           = -1; // reset the iterations for each level
  g_final_iter     = false;

  // Solve the problem if asked to do iterations. Otherwise
  // just keep the DEM at the initial guess, while saving
  // all the output data as if iterations happened.
  ceres::Solver::Summary summary;
  if (options.max_num_iterations > 0)
    ceres::Solve(options, &problem, &summary);

  // Save the final results
  g_final_iter = true;
  ceres::IterationSummary callback_summary;
  callback(callback_summary);
  
  vw_out() << summary.FullReport() << "\n" << std::endl;

  // callTop();
}

#if 0

// Function for highlighting no-data
template<class PixelT>
class NoData {
  typedef typename CompoundChannelType<PixelT>::type channel_type;
  channel_type m_nodata;
  typedef ChannelRange<channel_type> range_type;
public:
  NoData( channel_type nodata ) : m_nodata(nodata) {}

  template <class Args> struct result {
    typedef channel_type type;
  };

  inline channel_type operator()( channel_type const& val ) const {
    return (!(val != m_nodata && !std::isnan(val)))? range_type::max() : range_type::min();
  }
};

template <class ImageT, class NoDataT>
UnaryPerPixelView<ImageT,UnaryCompoundFunctor<NoData<typename ImageT::pixel_type>, typename ImageT::pixel_type>  >
inline nodata( ImageViewBase<ImageT> const& image, NoDataT nodata ) {
  typedef UnaryCompoundFunctor<NoData<typename ImageT::pixel_type>, typename ImageT::pixel_type> func_type;
  func_type func( nodata );
  return UnaryPerPixelView<ImageT,func_type>( image.impl(), func );
}

#endif

#if 0
// Prototype code to identify permanently shadowed areas
// and deepen the craters there. Needs to be integrated
// and tested with various shapes of the deepened crater.
void deepenCraters() {
  std::vector<std::string> image;

  std::string dem_file = argv[argc - 1];
  float dem_nodata_val = -std::numeric_limits<float>::max();
  if (vw::read_nodata_val(dem_file, dem_nodata_val)){
    vw_out() << "Dem nodata: " << dem_nodata_val << std::endl;
  }

  ImageView<PixelMask<float>> dem (create_mask(DiskImageView<float>(dem_file), dem_nodata_val));
  vw::cartography::GeoReference georef;
  if (!read_georeference(georef, dem_file))
    vw_throw( ArgumentErr() << "The input DEM " << dem_file << " has no georeference.\n" );

  // The maximum of all valid pixel values with no-data where there is no-valid data.
  ImageView<PixelMask<float>> max_img(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      max_img(col, row) = dem_nodata_val;
      max_img(col, row).invalidate();
    }
  }
  
  for (int i = 1; i < argc - 1; i++) {

    std::string img_file = argv[i];
    float img_nodata_val = -std::numeric_limits<float>::max();
    if (vw::read_nodata_val(img_file, img_nodata_val)){
      vw_out() << "Img nodata: " << img_nodata_val << std::endl;
    }
    
    ImageView<PixelMask<float>> img(create_mask(DiskImageView<float>(img_file), img_nodata_val));
    std::cout << "cols and rows are " << img.cols() << ' ' << img.rows() << std::endl;
    if (img.cols() != dem.cols() || img.rows() != dem.rows()) {
      vw_throw(ArgumentErr() << "Images and DEM must have same size.\n");
    }

    for (int col = 0; col < img.cols(); col++) {
      for (int row = 0; row < img.rows(); row++) {

        // Nothing to do if the current image has invalid data
        if (!is_valid(img(col, row)))
          continue; 

        // If the output image is not valid yet, copy the current image's valid pixel
        if (!is_valid(max_img(col, row) && img(col, row).child() > 0)) {
          max_img(col, row) = img(col, row);
          continue;
        }

        // Now both the current image and the output image are valid
        if (img(col, row).child() > max_img(col, row).child() &&
            img(col, row).child() > 0) {
          max_img(col, row) = img(col, row);
        }
        
      }
    }
  }

  // At the boundary the intensity is always invalid, but that is due to
  // computational limitations. Make it valid if we can.
  // TODO: Test here that the image has at least 3 rows and 3 cols!
  for (int col = 0; col < max_img.cols(); col++) {
    for (int row = 0; row < max_img.rows(); row++) {
      if ( (col == 0 || col == max_img.cols() - 1) ||
           (row == 0 || row == max_img.rows() - 1) ) {
        int next_col = col, next_row = row;
        if (col == 0) next_col = 1;
        if (col == max_img.cols() - 1) next_col = max_img.cols() - 2;
        if (row == 0) next_row = 1;
        if (row == max_img.rows() - 1) next_row = max_img.rows() - 2;

        if (!is_valid(max_img(col, row)) && is_valid(max_img(next_col, next_row))) 
          max_img(col, row) = max_img(next_col, next_row);
      }
    }
  }
  
  std::string max_img_file = "max_img.tif";
  bool has_nodata = true, has_georef = true;
  TerminalProgressCallback tpc("", "\t--> ");

  vw_out() << "Writing: " << max_img_file << "\n";
  
  block_write_gdal_image(max_img_file, apply_mask(max_img, dem_nodata_val),
                         has_georef, georef,
                         has_nodata, dem_nodata_val,
                         opt, tpc);

  ImageView<double> grass = grassfire(nodata(select_channel(max_img, 0), dem_nodata_val));

  // Scale as craters are shallow.
  // TODO: Need to think of a better algorithm!
  for (int col = 0; col < grass.cols(); col++) {
    for (int row = 0; row < grass.rows(); row++) {
      grass(col, row) *= 0.2;
    }
  }

  // Blur with a given sigma
  double sigma = atof(getenv("SIGMA"));
  //blur_weights(grass, sigma);
  ImageView<double> blurred_grass;
  if (sigma > 0) 
    blurred_grass = gaussian_filter(grass, sigma);
  else
    blurred_grass = copy(grass);
  
  std::string grass_file = "grass.tif";
  vw_out() << "Writing: " << grass_file << "\n";

  bool grass_has_nodata = false;
  block_write_gdal_image(grass_file, blurred_grass,
                         has_georef, georef,
                         grass_has_nodata, dem_nodata_val,
                         opt, tpc);

  // Bias the DEM by that grassfire height deepening the craters
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (is_valid(dem(col, row))) {
        dem(col, row).child() -= blurred_grass(col, row);
      }
    }
  }


  std::string out_dem_file = "out_dem.tif";
  vw_out() << "Writing: " << out_dem_file << "\n";
  
  block_write_gdal_image(out_dem_file, apply_mask(dem, dem_nodata_val),
                         has_georef, georef,
                         has_nodata, dem_nodata_val,
                         opt, tpc);

}
#endif

void setUpModelParams(GlobalParams & global_params, Options & opt) {
  if (opt.reflectance_type == 0)
    global_params.reflectanceType = LAMBERT;
  else if (opt.reflectance_type == 1)
    global_params.reflectanceType = LUNAR_LAMBERT;
  else if (opt.reflectance_type == 2)
    global_params.reflectanceType = HAPKE;
  else if (opt.reflectance_type == 3)
    global_params.reflectanceType = ARBITRARY_MODEL;
  else if (opt.reflectance_type == 4)
    global_params.reflectanceType = CHARON;
  else
    vw_throw( ArgumentErr() << "Expecting Lambertian or Lunar-Lambertian reflectance." );
  global_params.phaseCoeffC1 = 0; 
  global_params.phaseCoeffC2 = 0;
  
  // Default model coefficients, unless they were read already
  if (opt.model_coeffs_vec.empty()) {
    opt.model_coeffs_vec.resize(g_num_model_coeffs);
    if (global_params.reflectanceType == LUNAR_LAMBERT ||
        global_params.reflectanceType == ARBITRARY_MODEL ) {
      // Lunar lambertian or its crazy experimental generalization
      opt.model_coeffs_vec.resize(g_num_model_coeffs);
      opt.model_coeffs_vec[0] = 1;
      opt.model_coeffs_vec[1] = -0.019;
      opt.model_coeffs_vec[2] =  0.000242;   //0.242*1e-3;
      opt.model_coeffs_vec[3] = -0.00000146; //-1.46*1e-6;
      opt.model_coeffs_vec[4] = 1;
      opt.model_coeffs_vec[5] = 0;
      opt.model_coeffs_vec[6] = 0;
      opt.model_coeffs_vec[7] = 0;
      opt.model_coeffs_vec[8] = 1;
      opt.model_coeffs_vec[9] = -0.019;
      opt.model_coeffs_vec[10] =  0.000242;   //0.242*1e-3;
      opt.model_coeffs_vec[11] = -0.00000146; //-1.46*1e-6;
      opt.model_coeffs_vec[12] = 1;
      opt.model_coeffs_vec[13] = 0;
      opt.model_coeffs_vec[14] = 0;
      opt.model_coeffs_vec[15] = 0;
    }else if (global_params.reflectanceType == HAPKE) {
      opt.model_coeffs_vec[0] = 0.68; // omega (also known as w)
      opt.model_coeffs_vec[1] = 0.17; // b
      opt.model_coeffs_vec[2] = 0.62; // c
      opt.model_coeffs_vec[3] = 0.52; // B0
      opt.model_coeffs_vec[4] = 0.52; // h
    }else if (global_params.reflectanceType == CHARON) {
      opt.model_coeffs_vec.resize(g_num_model_coeffs);
      opt.model_coeffs_vec[0] = 0.7; // A
      opt.model_coeffs_vec[1] = 0.63; // f(alpha)
    }else if (global_params.reflectanceType != LAMBERT) {
      vw_throw( ArgumentErr() << "The Hapke model coefficients were not set. "
                << "Use the --model-coeffs option." );
    }
  }
}

int main(int argc, char* argv[]) {
  
  Stopwatch sw_total;
  sw_total.start();
  
  Options opt;
  g_opt = &opt;
  try {
    handle_arguments(argc, argv, opt);

    if (opt.compute_exposures_only && !opt.image_exposures_vec.empty()) {
      // TODO: This needs to be adjusted if haze is computed.
      vw_out() << "Exposures exist.";
      return 0;
    }

    // Set up model information
    GlobalParams global_params;
    setUpModelParams(global_params, opt);
    g_reflectance_model_coeffs = &opt.model_coeffs_vec[0];
    
    int num_dems = opt.input_dems.size();

    // Manage no-data
    double dem_nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
    if (vw::read_nodata_val(opt.input_dems[0], dem_nodata_val)){
      vw_out() << "Found DEM nodata value: " << dem_nodata_val << std::endl;
      if (std::isnan(dem_nodata_val)) {
        dem_nodata_val = -std::numeric_limits<float>::max(); // bugfix for NaN
        vw_out() << "Overwriting the nodata-value with: " << dem_nodata_val << "\n";
      }
    }
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      double curr_nodata_val = -std::numeric_limits<float>::max(); 
      if (vw::read_nodata_val(opt.input_dems[dem_iter], curr_nodata_val)){
        if (std::isnan(curr_nodata_val)) 
          curr_nodata_val = dem_nodata_val; // bugfix for NaN
        if (dem_nodata_val != curr_nodata_val) {
          vw_throw( ArgumentErr() << "All DEMs must have the same nodata value.\n" );
        }
      }
    }
    if (!boost::math::isnan(opt.nodata_val)) {
      dem_nodata_val = opt.nodata_val;
      vw_out() << "Over-riding the DEM nodata value with: " << dem_nodata_val << std::endl;
    }
    g_dem_nodata_val = &dem_nodata_val;
    
    // Prepare for multiple levels
    int levels = opt.coarse_levels;

    // Read the handles to the DEMs. Here we don't load them into
    // memory yet. We will later load into memory only cropped
    // versions if cropping is specified. This is to save on memory.
    std::vector< ImageViewRef<double> > dem_handles(num_dems);
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) 
      dem_handles[dem_iter] = DiskImageView<double>(opt.input_dems[dem_iter]);

    // There are multiple DEM clips, and multiple coarseness levels
    // for each DEM. Same about albedo and georeferences.
    std::vector< std::vector< ImageView<double> > >
      orig_dems(levels+1), dems(levels+1), albedos(levels+1);
    std::vector<std::vector<GeoReference>> geos(levels+1);
    for (int level = 0; level <= levels; level++) {
      orig_dems [level].resize(num_dems);
      dems      [level].resize(num_dems);
      albedos   [level].resize(num_dems);
      geos      [level].resize(num_dems);
    }
    
    if ( (!opt.crop_win.empty() || opt.query) && num_dems > 1) 
      vw_throw( ArgumentErr() << "Cannot run parallel_stereo with multiple DEM clips.\n" );

    // This must be done before the DEM is cropped. This stats is
    // queried from parallel_sfs.
    if (opt.query) {
      vw_out() << "dem_cols, " << dem_handles[0].cols() << std::endl;
      vw_out() << "dem_rows, " << dem_handles[0].rows() << std::endl;
    }

    // Adjust the crop win
    opt.crop_win.crop(bounding_box(dem_handles[0]));
    
    // Read the georeference 
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      if (!read_georeference(geos[0][dem_iter], opt.input_dems[dem_iter]))
        vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );
      
      // This is a bug fix. The georef pixel size in y must be negative
      // for the DEM to be oriented correctly. 
      if (geos[0][dem_iter].transform()(1, 1) > 0)
        vw_throw(ArgumentErr() << "The input DEM has a positive pixel size in y. "
                 << "This is unexpected. Normally it is negative since the (0, 0) "
                 << "pixel is in the upper-left. Check your DEM pixel size with "
                 << "gdalinfo. Cannot continue.\n");
       
      // Crop the DEM and georef if requested to given box.  The
      // cropped DEM (or uncropped if no cropping happens) is fully
      // loaded in memory.
      if (!opt.crop_win.empty()) {
        dems[0][dem_iter] = crop(dem_handles[dem_iter], opt.crop_win);
        geos[0][dem_iter] = crop(geos[0][dem_iter], opt.crop_win);
      }else{
        dems[0][dem_iter] = dem_handles[dem_iter]; // load in memory
      }
    
      // This can be useful
      vw_out() << "DEM cols and rows: " << dems[0][dem_iter].cols()  << ' '
               << dems[0][dem_iter].rows() << std::endl;
    }
    
    int min_dem_size = 5;
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      // See if to use a constant init value
      if (!boost::math::isnan(opt.init_dem_height)) {
        for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
          for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
            dems[0][dem_iter](col, row) = opt.init_dem_height;
          }
        }
      }

      // Refuse to run if there are no-data values
      for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
        for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
          if (dems[0][dem_iter](col, row) == dem_nodata_val ||
              std::isnan(dems[0][dem_iter](col, row))) {
            vw_throw( ArgumentErr() << "Found a no-data or NaN pixel in the DEM. Cannot continue. "
                      << "The dem_mosaic tool can be used to fill in holes. Then "
                      << "crop and use a clip from this DEM having only valid data.");
          }
        }
      }
      
      if (dems[0][dem_iter].cols() < min_dem_size ||
          dems[0][dem_iter].rows() < min_dem_size) {
        vw_throw( ArgumentErr() << "The input DEM with index "
                  << dem_iter << " is too small.\n" );
      }
    }

    // Read the sun positions from a list, if provided. Usually those
    // are read from the cameras, however, as done further down. 
    std::vector<ModelParams> model_params;
    read_sun_positions_from_list(opt, model_params);
    
    // Read in the camera models (and the sun positions, if not read from the list)
    int num_images = opt.input_images.size();
    std::vector<std::vector<boost::shared_ptr<CameraModel>>> cameras(num_dems);
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {

      cameras[dem_iter].resize(num_images);
      for (int image_iter = 0; image_iter < num_images; image_iter++){
    
        if (opt.skip_images[dem_iter].find(image_iter)
            != opt.skip_images[dem_iter].end()) continue;
    
                asp::SessionPtr session(asp::StereoSessionFactory::create
                           (opt.stereo_session, // in-out
                            opt,
                            opt.input_images[image_iter],
                            opt.input_images[image_iter],
                            opt.input_cameras[image_iter],
                            opt.input_cameras[image_iter],
                            opt.out_prefix));
    
        vw_out() << "Loading image and camera: " << opt.input_images[image_iter] << " "
                 <<  opt.input_cameras[image_iter] << " for DEM clip " << dem_iter << ".\n";
        cameras[dem_iter][image_iter] = session->camera_model(opt.input_images[image_iter],
                                                              opt.input_cameras[image_iter]);

        if (dem_iter == 0) {
          // Read the sun position from the camera if it is was not read from the list
          if (model_params[image_iter].sunPosition == Vector3())
            model_params[image_iter].sunPosition
              = sun_position_from_camera(cameras[dem_iter][image_iter]);

          // Sanity check
          if (model_params[image_iter].sunPosition == Vector3())
            vw_throw(ArgumentErr()
                     << "Could not read sun positions from list or from camera model files.\n");
            
          // Compute the azimuth and elevation
          double azimuth, elevation;
          sun_angles(opt, dems[0][dem_iter], dem_nodata_val, geos[0][dem_iter],
                     cameras[dem_iter][image_iter],
                     model_params[image_iter].sunPosition,
                     azimuth, elevation);
          
          // Print this. It will be used to organize the images by illumination
          // for bundle adjustment.
          // Since the sun position has very big values and we want to sort uniquely
          // the images by azimuth angle, use high precision below.
          vw_out().precision(17);
          vw_out() << "Sun position for: " << opt.input_images[image_iter] << " is "
                   << model_params[image_iter].sunPosition << "\n";
          vw_out() << "Sun azimuth and elevation for: "
                   << opt.input_images[image_iter] << " are " << azimuth
                   << " and " << elevation << " degrees.\n";
          vw_out().precision(6); // Go back to usual precision
        }
      }
    }

    // Stop here if all we wanted was some information
    if (opt.query) 
      return 0;

    // This check must be here, after we find the session
    if (opt.stereo_session != "isis" &&
        (opt.use_approx_camera_models || opt.use_approx_adjusted_camera_models ||
         opt.use_rpc_approximation || opt.use_semi_approx)) {
      vw_out() << "Computing approximate models works only with ISIS cameras. "
               << "Ignoring that option.\n";
      opt.use_approx_camera_models = false;
      opt.use_approx_adjusted_camera_models = false;
      opt.use_rpc_approximation = false;
      opt.use_semi_approx = false;
    }
      
    // Since we may float the cameras, ensure our camera models are
    // always adjustable. Note that if the user invoked this tool with
    // --bundle-adjust-prefix, the adjustments were already loaded
    // by now so the cameras are already adjustable. 
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end())
          continue;
        CameraModel * icam
          = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
        if (icam == NULL) {
          // Set a default identity adjustment
          Vector2 pixel_offset;
          Vector3 translation;
          Quaternion<double> rotation = Quat(math::identity_matrix<3>());
          // For clarity, first make a copy of the object that we will overwrite.
          // This may not be necessary but looks safer this way.
          boost::shared_ptr<CameraModel> cam_ptr = cameras[dem_iter][image_iter];
          cameras[dem_iter][image_iter] = boost::shared_ptr<CameraModel>
            (new AdjustedCameraModel(cam_ptr, translation,
                                     rotation, pixel_offset));
        }
      }
    }
    
    // Prepare for working at multiple levels
    int factor = 2;
    std::vector<int> factors;
    factors.push_back(1);
    for (int level = 1; level <= levels; level++)
      factors.push_back(factors[level-1]*factor);
    
    // We won't load the full images, just portions restricted
    // to the area we we will compute the DEM.
    std::vector<std::vector<std::vector<BBox2i>>> crop_boxes(levels+1);
    for (int level = 0; level <= levels; level++)
      crop_boxes[level].resize(num_dems);
    
    // The crop box starts as the original image bounding box. We'll shrink it later.
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        std::string img_file = opt.input_images[image_iter];
        crop_boxes[0][dem_iter].push_back(bounding_box(DiskImageView<float>(img_file)));
      }
    }
    
    // Ensure that no two threads can access an ISIS camera at the same time.
    // Declare the lock here, as we want it to live until the end of the program. 
    vw::Mutex camera_mutex;

    // callTop();
    
    // If to use approximate camera models or to crop input images
    if (opt.use_approx_camera_models || opt.use_approx_adjusted_camera_models) {

      // TODO(oalexan1): This code needs to be modularized.
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
        double max_approx_err = 0.0;
      
        for (int image_iter = 0; image_iter < num_images; image_iter++){
        
          if (opt.skip_images[dem_iter].find(image_iter)
              != opt.skip_images[dem_iter].end()) continue;
      
          // Here we make a copy, since soon cameras[dem_iter][image_iter] will be overwritten
          AdjustedCameraModel exact_adjusted_camera
            = *dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());

          boost::shared_ptr<CameraModel>
            exact_unadjusted_camera = exact_adjusted_camera.unadjusted_model();

          vw_out() << "Creating an approximate camera model for "
                   << opt.input_cameras[image_iter] << " and clip "
                   << opt.input_dems[dem_iter] <<".\n";
          BBox2i img_bbox = crop_boxes[0][dem_iter][image_iter];
          Stopwatch sw;
          sw.start();
          boost::shared_ptr<CameraModel> apcam;
          if (opt.use_approx_camera_models) {
            apcam = boost::shared_ptr<CameraModel>
              (new ApproxCameraModel(exact_adjusted_camera, exact_unadjusted_camera,
                                     img_bbox, dems[0][dem_iter],
                                     geos[0][dem_iter],
                                     dem_nodata_val, opt.use_rpc_approximation,
                                     opt.use_semi_approx,
                                     opt.rpc_penalty_weight, camera_mutex));
            
            // Copy the adjustments over to the approximate camera model
            Vector3 translation  = exact_adjusted_camera.translation();
            Quat rotation        = exact_adjusted_camera.rotation();
            Vector2 pixel_offset = exact_adjusted_camera.pixel_offset();
            double scale         = exact_adjusted_camera.scale();
            cameras[dem_iter][image_iter] = boost::shared_ptr<CameraModel>
              (new AdjustedCameraModel(apcam, translation,
                                       rotation, pixel_offset, scale));
          }else if (opt.use_approx_adjusted_camera_models){
            apcam = boost::shared_ptr<CameraModel>
              (new ApproxAdjustedCameraModel(exact_adjusted_camera, exact_unadjusted_camera,
                                             img_bbox,
                                             dems[0][dem_iter], geos[0][dem_iter],
                                             dem_nodata_val, camera_mutex));
            // Adjustments are already baked into the adjusted
            // approximate cameras, that is why the logic as above to
            // reincorporate the adjustments is not needed.
            cameras[dem_iter][image_iter] = apcam;
          }
          
          sw.stop();
          vw_out() << "Approximate model generation time: " << sw.elapsed_seconds()
                   << " s." << std::endl;
          
          // callTop();

          // Cast the pointer back to ApproxBaseCameraModel as we need that.
          ApproxBaseCameraModel* cam_ptr = dynamic_cast<ApproxBaseCameraModel*>(apcam.get());
          if (cam_ptr == NULL) 
            vw_throw( ArgumentErr() << "Expecting a ApproxBaseCameraModel." );

          bool model_is_valid = cam_ptr->model_is_valid();
          
          // Compared original and unadjusted models
          double max_curr_err = 0.0;

          // TODO: No need to test how unadjusted models compare for RPC,
          // test only the adjusted models. 
          if (model_is_valid) {
            // Recompute the crop box, can be done more reliably here
            if (opt.use_rpc_approximation || opt.use_semi_approx)
              cam_ptr->crop_box() = BBox2();
            for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
              for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
                Vector2 ll = geos[0][dem_iter].pixel_to_lonlat(Vector2(col, row));
                Vector3 xyz = geos[0][dem_iter].datum().geodetic_to_cartesian
                  (Vector3(ll[0], ll[1], dems[0][dem_iter](col, row)));

                if (opt.use_approx_camera_models) {
                  // For approx adjusted camera models we don't do this,
                  // as we don't approximate the unadjusted camera.
                  // Test how unadjusted models compare
                  Vector2 pix1 = exact_unadjusted_camera->point_to_pixel(xyz);
                  //if (!img_bbox.contains(pix1)) continue;
                  
                  Vector2 pix2 = apcam->point_to_pixel(xyz);
                  max_curr_err = std::max(max_curr_err, norm_2(pix1 - pix2));
                  
                  // Use these pixels to expand the crop box, as we
                  // now also know the adjustments.  This is a bug
                  // fix.
                  cam_ptr->crop_box().grow(pix1);
                  cam_ptr->crop_box().grow(pix2);
                }
                
                // Test how adjusted (exact and approximate) models compare
                Vector2 pix3 = exact_adjusted_camera.point_to_pixel(xyz);
                //if (!img_bbox.contains(pix3)) continue;
                Vector2 pix4 = cameras[dem_iter][image_iter]->point_to_pixel(xyz);
                max_curr_err = std::max(max_curr_err, norm_2(pix3 - pix4));

                cam_ptr->crop_box().grow(pix3);
                cam_ptr->crop_box().grow(pix4);
              }
            }

            cam_ptr->crop_box().crop(img_bbox);
            
            vw_out() << "Max approximate model error in pixels for: "
                     <<  opt.input_images[image_iter] << " and clip "
                     << opt.input_dems[dem_iter] << ": " << max_curr_err << std::endl;
          }else{
            vw_out() << "Invalid model for clip: " << dem_iter << ".\n";
          }
          
          if (max_curr_err > opt.rpc_max_error || !model_is_valid) {
            // This is a bugfix. When the DEM clip does not intersect the image,
            // the approx camera model has incorrect values.
            if (model_is_valid)
              vw_out() << "Error is too big.\n";
            vw_out() << "Skip image " << image_iter << " for clip "
                     << dem_iter << std::endl;
            opt.skip_images[dem_iter].insert(image_iter);
            cam_ptr->crop_box() = BBox2();
            max_curr_err = 0.0;
          }

          max_approx_err = std::max(max_approx_err, max_curr_err);
        
          if (opt.use_rpc_approximation && !cam_ptr->crop_box().empty()){
            // Grow the box just a bit more, to ensure we still see
            // enough of the images during optimization.
            double extra = 0.2;
            double extrax = extra*cam_ptr->crop_box().width();
            double extray = extra*cam_ptr->crop_box().height();
            cam_ptr->crop_box().min() -= Vector2(extrax, extray);
            cam_ptr->crop_box().max() += Vector2(extrax, extray);
          }
          cam_ptr->crop_box().crop(img_bbox);
          vw_out() << "Crop box dimensions: " << cam_ptr->crop_box() << std::endl;
    
          // Copy the crop box
          if (opt.crop_input_images)
            crop_boxes[0][dem_iter][image_iter].crop(cam_ptr->crop_box());

          // Skip images which result in empty crop boxes
          if (crop_boxes[0][dem_iter][image_iter].empty()) {
            opt.skip_images[dem_iter].insert(image_iter);
          }
          
        } // end iterating over images
        vw_out() << "Max total approximate model error in pixels: "
                 << max_approx_err << std::endl;
        
      } // end iterating over dem clips

      // end computing the approximate camera model
    } else if (opt.crop_input_images) {
      
      // We will arrive here if it is desired to crop the input images
      // without using an approximate model, such as for CSM.
      // Estimate the crop box by projecting the pixels in the exact
      // camera (with the adjustments applied, if present).
      
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        for (int image_iter = 0; image_iter < num_images; image_iter++){
          if (opt.skip_images[dem_iter].find(image_iter)
              != opt.skip_images[dem_iter].end()) continue;

          // Store the full image box, and initialize the crop box to an empty box
          BBox2i img_bbox = crop_boxes[0][dem_iter][image_iter];
          crop_boxes[0][dem_iter][image_iter] = BBox2i();
          
          for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
            for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
              Vector2 ll = geos[0][dem_iter].pixel_to_lonlat(Vector2(col, row));
              Vector3 xyz = geos[0][dem_iter].datum().geodetic_to_cartesian
                (Vector3(ll[0], ll[1], dems[0][dem_iter](col, row)));
              
              Vector2 pix = cameras[dem_iter][image_iter]->point_to_pixel(xyz);
              crop_boxes[0][dem_iter][image_iter].grow(pix); 
            }
          }

          // Double the box dimensions, just in case. Later the SfS heights
          // may change, and we may need to see beyond the given box
          double extraFactor = 0.5;
          double extrax = extraFactor * crop_boxes[0][dem_iter][image_iter].width();
          double extray = extraFactor * crop_boxes[0][dem_iter][image_iter].height();
          crop_boxes[0][dem_iter][image_iter].min() -= Vector2(extrax, extray);
          crop_boxes[0][dem_iter][image_iter].max() += Vector2(extrax, extray);

          // Crop to the bounding box of the image
          crop_boxes[0][dem_iter][image_iter].crop(img_bbox);
            
          vw_out() << "Estimated crop box for image " 
                   << opt.input_images[image_iter] << " and clip "
                   << opt.input_dems[dem_iter] << ": "
                   << crop_boxes[0][dem_iter][image_iter]
                   << std::endl;
          
          if (crop_boxes[0][dem_iter][image_iter].empty()) 
            opt.skip_images[dem_iter].insert(image_iter);
        }
      }
    }

    // Compute the boxes at lower resolutions
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
      // Make the crop boxes lower left corner be multiple of 2^level
      int last_factor = factors.back();
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        if (!crop_boxes[0][dem_iter][image_iter].empty()) {
          Vector2i mn = crop_boxes[0][dem_iter][image_iter].min();
          crop_boxes[0][dem_iter][image_iter].min()
            = last_factor*(floor(mn/double(last_factor)));
        }
      }
      
      // Crop boxes at the coarser resolutions
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        for (int level = 1; level <= levels; level++) {
          crop_boxes[level][dem_iter]
            .push_back(crop_boxes[0][dem_iter][image_iter]/factors[level]);
        }
      }
    }
    
    // Masked images and weights.
    std::vector<std::vector< std::vector<MaskedImgT> > > masked_images_vec(levels+1);
    std::vector<std::vector< std::vector<DoubleImgT> > > blend_weights_vec(levels+1);
    for (int level = levels; level >= 0; level--) {
      masked_images_vec[level].resize(num_dems);
      blend_weights_vec[level].resize(num_dems);
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        masked_images_vec[level][dem_iter].resize(num_images);
        blend_weights_vec[level][dem_iter].resize(num_images);
      }
    }
    
    float img_nodata_val = -std::numeric_limits<float>::max();
    for (int image_iter = 0; image_iter < num_images; image_iter++){
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end())
          continue;
       
        std::string img_file = opt.input_images[image_iter];
        if (vw::read_nodata_val(img_file, img_nodata_val)){
          //vw_out() << "Found image " << image_iter << " nodata value: "
          //         << img_nodata_val << std::endl;
        }
        // Model the shadow threshold
        float shadow_thresh = opt.shadow_threshold_vec[image_iter];
        if (opt.crop_input_images) {
          // Make a copy in memory for faster access
          if (!crop_boxes[0][dem_iter][image_iter].empty()) {
            ImageView<float> cropped_img = 
              crop(DiskImageView<float>(img_file), crop_boxes[0][dem_iter][image_iter]);
            masked_images_vec[0][dem_iter][image_iter]
              = create_pixel_range_mask2(cropped_img,
                                         std::max(img_nodata_val, shadow_thresh),
                                         opt.max_valid_image_vals_vec[image_iter]);

            // Compute blending weights only when cropping the
            // images. Otherwise the weights are too huge.
            if (opt.blending_dist > 0)
              blend_weights_vec[0][dem_iter][image_iter]
                = asp::blendingWeights(masked_images_vec[0][dem_iter][image_iter],
                                       opt.blending_dist, opt.blending_power,
                                       opt.min_blend_size);
          }
        }else{
          masked_images_vec[0][dem_iter][image_iter]
            = create_pixel_range_mask2(DiskImageView<float>(img_file),
                                       std::max(img_nodata_val, shadow_thresh),
                                       opt.max_valid_image_vals_vec[image_iter]);
        }
      }
    }
    g_img_nodata_val = &img_nodata_val;

    // Copy sun positions to an array
    std::vector<double> scaled_sun_posns(3*num_images);
    for (int image_iter = 0; image_iter < num_images; image_iter++){
      for (int it = 0; it < 3; it++) 
        scaled_sun_posns[3*image_iter + it] = 1; // model_params[image_iter].sunPosition[it];
    }    
    
    // Find the grid sizes in meters. Note that dem heights are in
    // meters too, so we treat both horizontal and vertical
    // measurements in same units.
    double gridx, gridy;
    compute_grid_sizes_in_meters(dems[0][0], geos[0][0], dem_nodata_val, gridx, gridy);
    vw_out() << "grid in x and y in meters: "
             << gridx << ' ' << gridy << std::endl;
    g_gridx = &gridx;
    g_gridy = &gridy;

    // Find the max DEM height
    std::vector<double> max_dem_height(num_dems, -std::numeric_limits<double>::max());
    if (opt.model_shadows) {
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        double curr_max_dem_height = -std::numeric_limits<double>::max();
        for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
          for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
            if (dems[0][dem_iter](col, row) > curr_max_dem_height) {
              curr_max_dem_height = dems[0][dem_iter](col, row);
            }
          }
        }
        max_dem_height[dem_iter] = curr_max_dem_height;
      }
    }
    g_max_dem_height = &max_dem_height;
    
    // Initial albedo. This will be updated later.
    double initial_albedo = 1.0;
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      albedos[0][dem_iter].set_size(dems[0][dem_iter].cols(), dems[0][dem_iter].rows());
      for (int col = 0; col < albedos[0][dem_iter].cols(); col++) {
        for (int row = 0; row < albedos[0][dem_iter].rows(); row++) {
          albedos[0][dem_iter](col, row) = initial_albedo;
        }
      }
    }
    
    // We have intensity = albedo * nonlin_reflectance(reflectance, exposure, haze, num_haze_coeffs)
    // Assume that haze is 0 to start with. Find the exposure as
    // mean(intensity)/mean(reflectance)/albedo. Use this to compute an
    // initial exposure and decide based on that which images to
    // skip. If the user provided initial exposures and haze, use those, but
    // still go through the motions to find the images to skip.
    vw_out() << "Computing exposures.\n";
    std::vector<double> local_exposures_vec(num_images, 0);
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      
      std::vector<double> exposures_per_dem;
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        
        if (opt.skip_images[dem_iter].find(image_iter) !=
            opt.skip_images[dem_iter].end()) continue;
        
        ImageView<PixelMask<double>> reflectance, intensity;
        ImageView<double> ground_weight;
        ImageView<Vector2> pq; // no need for these just for initialization
        
        // Sample the large DEMs. Keep about 200 row and column samples.
        int sample_col_rate = std::max((int)round(dems[0][dem_iter].cols()/200.0), 1);
        int sample_row_rate = std::max((int)round(dems[0][dem_iter].rows()/200.0), 1);
        computeReflectanceAndIntensity(dems[0][dem_iter], pq, geos[0][dem_iter],
                                       opt.model_shadows, max_dem_height[dem_iter],
                                       gridx, gridy, sample_col_rate, sample_row_rate,
                                       model_params[image_iter],
                                       global_params,
                                       crop_boxes[0][dem_iter][image_iter],
                                       masked_images_vec[0][dem_iter][image_iter],
                                       blend_weights_vec[0][dem_iter][image_iter],
                                       cameras[dem_iter][image_iter].get(),
                                       &scaled_sun_posns[3*image_iter],
                                       reflectance, intensity, ground_weight,
                                       &opt.model_coeffs_vec[0]);
        
        // TODO: Below is not the optimal way of finding the exposure!
        // Find it as the analytical minimum using calculus.
        double imgmean, imgstdev, refmean, refstdev;
        compute_image_stats(intensity, reflectance, imgmean, imgstdev, refmean, refstdev);
        double exposure = imgmean/refmean/initial_albedo;
        vw_out() << "img mean std: " << imgmean << ' ' << imgstdev << std::endl;
        vw_out() << "ref mean std: " << refmean << ' ' << refstdev << std::endl;
        vw_out() << "Local exposure for image " << image_iter << " and clip "
                 << dem_iter << ": " << exposure << std::endl;
    
        double big = 1e+100; // There's no way image exposure can be bigger than this
        bool is_good = ( 0 < exposure && exposure < big );
        if (is_good) {
          exposures_per_dem.push_back(exposure);
        }else{
          // Skip images with bad exposure. Apparently there is no good
          // imagery in the area.
          opt.skip_images[dem_iter].insert(image_iter);
          vw_out() << "Skip image " << image_iter << " for clip " << dem_iter << std::endl;
        }
      }
      
      // Out the exposures for this image on all clips, pick the median
      int len = exposures_per_dem.size();
      if (len > 0) {
        std::sort(exposures_per_dem.begin(), exposures_per_dem.end());
        local_exposures_vec[image_iter] = 
          0.5*(exposures_per_dem[(len-1)/2] + exposures_per_dem[len/2]);
        //vw_out() << "Median exposure for image " << image_iter << " on all clips: "
        //     << local_exposures_vec[image_iter] << std::endl;
      }
      
    }
    
    // Only overwrite the exposures if we don't have them supplied
    if (opt.image_exposures_vec.empty()) opt.image_exposures_vec = local_exposures_vec;

    for (size_t image_iter = 0; image_iter < opt.image_exposures_vec.size(); image_iter++) {
      vw_out() << "Image exposure for " << opt.input_images[image_iter] << ' '
               << opt.image_exposures_vec[image_iter] << std::endl;
    }

    // Initialize the haze as 0.
    if ((!opt.image_haze_vec.empty()) && (int)opt.image_haze_vec.size() != num_images)
      vw_throw(ArgumentErr() << "Expecting as many haze values as images.\n");
    if (opt.image_haze_vec.empty()) {
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        // Pad the haze vec
        std::vector<double> haze_vec;
        while (haze_vec.size() < g_max_num_haze_coeffs) haze_vec.push_back(0);
        opt.image_haze_vec.push_back(haze_vec);
      }
    }
    if (opt.compute_exposures_only) {
      save_exposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      // all done
      return 0;
    }

    // Need to compute the valid data image to be able to find the grid points always
    // in shadow, so when this image is zero.
    ImageView<int> lit_image_mask;
    if (opt.curvature_in_shadow_weight > 0.0) {
      if (num_dems > 1 || levels > 0) 
        vw_throw(ArgumentErr() << "Enforcing positive curvature in shadow does not work "
                 << "with more than one input DEM clip or a positive number of "
                 << "coarseness levels.\n");
      lit_image_mask.set_size(dems[0][0].cols(), dems[0][0].rows());
      for (int col = 0; col < lit_image_mask.cols(); col++) {
        for (int row = 0; row < lit_image_mask.rows(); row++) {
          lit_image_mask(col, row) = 0; // no valid points originally
        }
      }
    }

    // If opt.allow_borderline_data is true, create for each image that will not be skipped
    // a weight matrix with dimensions equal to DEM dimensions, that will be used instead
    // of weights in the camera image space. These are balanced among each other and give more
    // weight to barely lit and unlit nearby pixels.
    std::vector<ImageView<double>> ground_weights(num_images);
    
    // Note that below we may use the exposures computed at the previous step
    if (opt.save_computed_intensity_only || opt.estimate_slope_errors ||
        opt.estimate_height_errors || opt.curvature_in_shadow_weight > 0.0 ||
        opt.allow_borderline_data) {
      // In this case simply save the computed and actual intensity, and for most of these quit
      ImageView<PixelMask<double>> reflectance, meas_intensity, comp_intensity;
      ImageView<double> ground_weight;
      ImageView<Vector2> pq; // no need for these just for initialization
      int sample_col_rate = 1, sample_row_rate = 1;

      boost::shared_ptr<SlopeErrEstim> slopeErrEstim = boost::shared_ptr<SlopeErrEstim>(NULL);
      if (opt.estimate_slope_errors) {
        int num_a_samples = 90; // Sample the 0 to 90 degree range with this many samples
        int num_b_samples = 360; // sample the 0 to 360 degree range with this many samples
        slopeErrEstim = boost::shared_ptr<SlopeErrEstim>
          (new SlopeErrEstim(dems[0][0].cols(), dems[0][0].rows(),
                             num_a_samples, num_b_samples, &albedos[0][0], &opt));
      }
      
      boost::shared_ptr<HeightErrEstim> heightErrEstim = boost::shared_ptr<HeightErrEstim>(NULL);
      if (opt.estimate_height_errors) {
        double max_height_error  = opt.height_error_params[0];
        int num_height_samples   = opt.height_error_params[1];
        vw_out() << "Maximum height error to examine: " << max_height_error << "\n";
        vw_out() << "Number of samples to use from 0 to that height: " << num_height_samples
                 << "\n";
          
        double nodata_height_val = -1.0;
        heightErrEstim = boost::shared_ptr<HeightErrEstim>
          (new HeightErrEstim(dems[0][0].cols(), dems[0][0].rows(),
                              num_height_samples, max_height_error, nodata_height_val,
                              &albedos[0][0], &opt));
      }
      
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        
        if (opt.estimate_slope_errors) 
          slopeErrEstim->image_iter = image_iter;
        if (opt.estimate_height_errors) 
          heightErrEstim->image_iter = image_iter;

        // Find the reflectance and measured intensity (and work towards estimating the slopes
        // if asked to).
        computeReflectanceAndIntensity(dems[0][0], pq, geos[0][0],
                                       opt.model_shadows, max_dem_height[0],
                                       gridx, gridy, sample_col_rate, sample_row_rate,
                                       model_params[image_iter],
                                       global_params,
                                       crop_boxes[0][0][image_iter],
                                       masked_images_vec[0][0][image_iter],
                                       blend_weights_vec[0][0][image_iter],
                                       cameras[0][image_iter].get(),
                                       &scaled_sun_posns[3*image_iter],
                                       reflectance, meas_intensity, ground_weight,
                                       &opt.model_coeffs_vec[0],
                                       slopeErrEstim.get(), heightErrEstim.get());

        if (opt.skip_images[0].find(image_iter) == opt.skip_images[0].end() &&
            opt.allow_borderline_data) {
          // if not skipping, save the weight
          ground_weights[image_iter] = copy(ground_weight);
        }
        
        // Find the computed intensity.
        // TODO(oalexan1): Should one mark the no-data values rather than setting
        // them to 0? 
        comp_intensity.set_size(reflectance.cols(), reflectance.rows());
        for (int col = 0; col < comp_intensity.cols(); col++) {
          for (int row = 0; row < comp_intensity.rows(); row++) {
            comp_intensity(col, row)
              = albedos[0][0](col, row) *
              nonlin_reflectance(reflectance(col, row), opt.image_exposures_vec[image_iter],
                                 opt.steepness_factor,
                                 &opt.image_haze_vec[image_iter][0], opt.num_haze_coeffs);
          }
        }
        
        if (opt.curvature_in_shadow_weight > 0.0) {
          if (meas_intensity.cols() != lit_image_mask.cols() ||
              meas_intensity.rows() != lit_image_mask.rows()) {
            vw_throw(ArgumentErr()
                     << "Intensity image dimensions disagree with DEM clip dimensions.\n");
          }
          for (int col = 0; col < lit_image_mask.cols(); col++) {
            for (int row = 0; row < lit_image_mask.rows(); row++) {
              if (is_valid(meas_intensity(col, row))           || 
                  col == 0 || col == lit_image_mask.cols() - 1 ||
                  row == 0 || row == lit_image_mask.rows() - 1) {
                // Boundary pixels are declared lit. Otherwise they are
                // always unlit due to the peculiarities of how the intensity
                // is found at the boundary.
                lit_image_mask(col, row) = 1;
              }
            }
          }
        }
        
        if (opt.save_computed_intensity_only) {
          TerminalProgressCallback tpc("asp", ": ");
          bool has_georef = true, has_nodata = true;
          std::string out_camera_file
            = asp::bundle_adjust_file_name(opt.out_prefix,
                                           opt.input_images[image_iter],
                                           opt.input_cameras[image_iter]);
          std::string local_prefix = fs::path(out_camera_file).replace_extension("").string();
          std::string out_meas_intensity_file = local_prefix + "-meas-intensity.tif";
          vw_out() << "Writing: " << out_meas_intensity_file << std::endl;
          block_write_gdal_image(out_meas_intensity_file,
                                 apply_mask(meas_intensity, img_nodata_val),
                                 has_georef, geos[0][0], has_nodata,
                                 img_nodata_val, opt, tpc);
          
          std::string out_comp_intensity_file = local_prefix + "-comp-intensity.tif";
          vw_out() << "Writing: " << out_comp_intensity_file << std::endl;
          block_write_gdal_image(out_comp_intensity_file,
                                 apply_mask(comp_intensity, img_nodata_val),
                                 has_georef, geos[0][0], has_nodata, img_nodata_val,
                                 opt, tpc);
        }

      } // End iterating over images

      if (opt.estimate_slope_errors) {
        // Find the slope error as the maximum of slope errors in all directions
        // from the given slope.
        ImageView<float> slope_error;
        slope_error.set_size(reflectance.cols(), reflectance.rows());
        double nodata_slope_value = -1.0;
        for (int col = 0; col < slope_error.cols(); col++) {
          for (int row = 0; row < slope_error.rows(); row++) {
            slope_error(col, row) = nodata_slope_value;
            int num_samples = slopeErrEstim->slope_errs[col][row].size();
            for (int sample = 0; sample < num_samples; sample++) {
              slope_error(col, row)
                = std::max(double(slope_error(col, row)),
                           slopeErrEstim->slope_errs[col][row][sample]);
            }
          }
        }
        
        // Slope errors that are stuck at 90 degrees could not be estimated
        for (int col = 0; col < slope_error.cols(); col++) {
          for (int row = 0; row < slope_error.rows(); row++) {
            if (slope_error(col, row) == slopeErrEstim->max_angle)
              slope_error(col, row) = nodata_slope_value;
          }
        }
        
        TerminalProgressCallback tpc("asp", ": ");
        bool has_georef = true, has_nodata = true;
        std::string slope_error_file = opt.out_prefix + "-slope-error.tif";
        vw_out() << "Writing: " << slope_error_file << std::endl;
        block_write_gdal_image(slope_error_file,
                               slope_error, has_georef, geos[0][0], has_nodata,
                               nodata_slope_value, opt, tpc);
      }

      if (opt.estimate_height_errors) {
        // Find the height error from the range of heights
        ImageView<float> height_error;
        height_error.set_size(heightErrEstim->height_error_vec.cols(),
                              heightErrEstim->height_error_vec.rows());
        for (int col = 0; col < height_error.cols(); col++) {
          for (int row = 0; row < height_error.rows(); row++) {
            height_error(col, row)
              = std::max(-heightErrEstim->height_error_vec(col, row)[0],
                         heightErrEstim->height_error_vec(col, row)[1]);
            
            // When we are stuck at the highest error that means we could not
            // find it
            if (height_error(col, row) == heightErrEstim->max_height_error)
              height_error(col, row) = heightErrEstim->nodata_height_val;
          }
        }
        TerminalProgressCallback tpc("asp", ": ");
        bool has_georef = true, has_nodata = true;
        std::string height_error_file = opt.out_prefix + "-height-error.tif";
        vw_out() << "Writing: " << height_error_file << std::endl;
        block_write_gdal_image(height_error_file,
                               height_error,
                               has_georef, geos[0][0],
                               has_nodata, heightErrEstim->nodata_height_val,
                               opt, tpc);
      }
      
    } // End doing intensity computations and/or height and/or slope error estimations
      
    if (opt.save_computed_intensity_only || opt.estimate_slope_errors ||
        opt.estimate_height_errors) {
      save_exposures(opt.out_prefix, opt.input_images, opt.image_exposures_vec);
      // All done
      return 0;
    }

    if (opt.allow_borderline_data) {
      int cols = dems[0][0].cols(), rows = dems[0][0].rows();
      asp::adjustBorderlineDataWeights(cols, rows, opt.blending_dist, opt.blending_power,
                                       vw::GdalWriteOptions(opt), // slice
                                       geos[0][0],
                                       opt.skip_images[0],
                                       opt.out_prefix, // for debug data
                                       opt.input_images, opt.input_cameras, 
                                       ground_weights); // output

      // Use the ground weights from now on instead of blending weights.
      // Will overwrite the weights below.
      g_blend_weight_is_ground_weight = true;
      if (num_dems != 1) 
        vw::vw_throw(vw::ArgumentErr() << "Cannot use more than one DEM with "
                     << "--allow-borderline-data.\n");

      // Redo the image masks. Unlike before, the shadow threshold is set to 0
      // to allow shadow pixels. The weights will control how much of these
      // are actually used. This approach is better than a hard cutoff with the mask.
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
          if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end())
            continue;
          
          std::string img_file = opt.input_images[image_iter];
          vw::read_nodata_val(img_file, img_nodata_val);
          float shadow_thresh = 0.0; // Note how the shadow thresh is now 0, unlike before
          // Make a copy in memory for faster access
          if (!crop_boxes[0][dem_iter][image_iter].empty()) {
            ImageView<float> cropped_img = 
              crop(DiskImageView<float>(img_file), crop_boxes[0][dem_iter][image_iter]);
            masked_images_vec[0][dem_iter][image_iter]
              = create_pixel_range_mask2(cropped_img,
                                         std::max(img_nodata_val, shadow_thresh),
                                         opt.max_valid_image_vals_vec[image_iter]);

            // Overwrite the blending weights with ground weights
            blend_weights_vec[0][dem_iter][image_iter] = copy(ground_weights[image_iter]);
          }
        }
      }

      ground_weights.clear(); // not needed anymore
    } // end allow borderline data
    
    ImageView<double> curvature_in_shadow_weight;
    if (opt.curvature_in_shadow_weight > 0.0) {
      TerminalProgressCallback tpc("asp", ": ");
      bool has_georef = true, has_nodata = false;
      double nodata_val = -1; // will not be used
      std::string lit_image_mask_file = opt.out_prefix + "-lit_image_mask.tif";
      vw_out() << "Writing: " << lit_image_mask_file << std::endl;
      block_write_gdal_image(lit_image_mask_file, lit_image_mask,
                             has_georef, geos[0][0], has_nodata, nodata_val, opt, tpc);

      // Form the curvature_in_shadow_weight image. It will start at 0
      // at distance opt.lit_curvature_dist from the shadow
      // boundary in the lit area, and then reach value
      // opt.curvature_in_shadow_weight when at distance
      // opt.shadow_curvature_dist from the boundary in the shadowed
      // area. This is done to avoid boundary artifacts.
      double max_dist = std::max(opt.lit_curvature_dist, opt.shadow_curvature_dist);
      vw::bounded_signed_dist<int>(vw::create_mask(lit_image_mask, 0), max_dist,
                                   curvature_in_shadow_weight);
      // Do further adjustments
      for (int col = 0; col < curvature_in_shadow_weight.cols(); col++) {
        for (int row = 0; row < curvature_in_shadow_weight.rows(); row++) {
          double val = curvature_in_shadow_weight(col, row);
          val = std::min(val, opt.lit_curvature_dist);
          val = std::max(val, -opt.shadow_curvature_dist);
          val = (opt.lit_curvature_dist - val) /
            (opt.lit_curvature_dist + opt.shadow_curvature_dist);
          curvature_in_shadow_weight(col, row) = val * opt.curvature_in_shadow_weight;
        }
      }
      
      std::string curvature_in_shadow_weight_file = opt.out_prefix
        + "-curvature_in_shadow_weight.tif";
      vw_out() << "Writing: " << curvature_in_shadow_weight_file << std::endl;
      block_write_gdal_image(curvature_in_shadow_weight_file, curvature_in_shadow_weight,
                             has_georef, geos[0][0], has_nodata, nodata_val, opt, tpc);
    }
    
    if (opt.num_haze_coeffs > 0) {
      for (size_t image_iter = 0; image_iter < opt.image_haze_vec.size(); image_iter++) {
        vw_out() << "Image haze for " << opt.input_images[image_iter] << ':';
        for (size_t hiter = 0; hiter < opt.image_haze_vec[image_iter].size(); hiter++) {
          vw_out() << " " << opt.image_haze_vec[image_iter][hiter];
        }
        vw_out() << "\n";
      }
    }
    
    g_exposures        = &opt.image_exposures_vec;
    g_haze             = &opt.image_haze_vec;
    g_scaled_sun_posns = &scaled_sun_posns;
    
    // For images that we don't use, wipe the cameras and all other
    // info, as those take up memory (the camera is a table). 
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) {
          masked_images_vec[0][dem_iter][image_iter] = ImageView<PixelMask<float>>();
          blend_weights_vec[0][dem_iter][image_iter] = ImageView<double>();
          cameras[dem_iter][image_iter] = boost::shared_ptr<CameraModel>();
        }
      }
    }
    
    // The initial camera adjustments. They will be updated later.
    std::vector<double> adjustments(6*num_images, 0);
    for (int image_iter = 0; image_iter < num_images; image_iter++) {

      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) continue;
      
        Vector3 translation, axis_angle;
        Vector2 pixel_offset;

        if (!opt.use_approx_adjusted_camera_models) {
          AdjustedCameraModel * icam
            = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
          if (icam == NULL)
            vw_throw(ArgumentErr() << "Expecting an adjusted camera model.\n");
          translation = icam->translation();
          axis_angle = icam->rotation().axis_angle();
          pixel_offset = icam->pixel_offset();
        }else{
          ApproxAdjustedCameraModel * aapcam
            = dynamic_cast<ApproxAdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
          if (aapcam == NULL)
            vw_throw(ArgumentErr() << "Expecting an approximate adjusted camera model.\n");
          AdjustedCameraModel acam = aapcam->exact_adjusted_camera();
          translation = acam.translation();
          axis_angle = acam.rotation().axis_angle();
          pixel_offset = acam.pixel_offset();
        }

        // TODO(oalexan1): This does not appear necessary use adjusted approximate cameras.
        if (pixel_offset != Vector2())
          vw_throw(ArgumentErr() << "Expecting zero pixel offset.\n");
        for (int param_iter = 0; param_iter < 3; param_iter++) {
          adjustments[6*image_iter + 0 + param_iter]
            = translation[param_iter]/(g_position_scale_factor*opt.camera_position_step_size);
          adjustments[6*image_iter + 3 + param_iter] = axis_angle[param_iter];
        }
      }
    }
    g_adjustments = &adjustments;

    // Prepare data at each coarseness level
    // orig_dems will keep the input DEMs and won't change. Keep to the optimized
    // DEMs close to orig_dems. Make a deep copy below.
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      orig_dems[0][dem_iter] = copy(dems[0][dem_iter]);
    }
    
    double sub_scale = 1.0/factor;
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {

      for (int level = 1; level <= levels; level++) {
        geos[level][dem_iter] = resample(geos[level-1][dem_iter], sub_scale);
        orig_dems[level][dem_iter]
          = pixel_cast<double>(vw::resample_aa
                               (pixel_cast< PixelMask<double> >
                                (orig_dems[level-1][dem_iter]), sub_scale));
        dems[level][dem_iter] = copy(orig_dems[level][dem_iter]);
        
        // CERES won't be happy with tiny DEMs
        if (dems[level][dem_iter].cols() < min_dem_size || dems[level][dem_iter].rows()
            < min_dem_size) {
          levels = std::max(0, level-1);
          vw_out(WarningMessage) << "Reducing the number of coarse levels to "
                                 << levels << ".\n";
          geos.resize(levels+1);
          orig_dems.resize(levels+1);
          dems.resize(levels+1);
          albedos.resize(levels+1);
          masked_images_vec.resize(levels+1);
          blend_weights_vec.resize(levels+1);
          factors.resize(levels+1);
          break;
        }

        albedos[level][dem_iter]
          = pixel_cast<double>(vw::resample_aa
                               (pixel_cast< PixelMask<double>>
                                (albedos[level-1][dem_iter]), sub_scale));

        // We must write the subsampled images to disk, and then read
        // them back, as VW cannot access individual pixels of the
        // monstrosities created using the logic below, and even if it
        // could, it is best if resampling is done once, and offline,
        // rather than redoing it each time within the optimization
        // loop.
        for (int image_iter = 0; image_iter < num_images; image_iter++) {

          if (opt.skip_images[dem_iter].find(image_iter)
              != opt.skip_images[dem_iter].end()) continue;
        
          fs::path image_path(opt.input_images[image_iter]);
          std::ostringstream os; os << "-level" << level;
          if (num_dems > 1)      os << "-clip"  << dem_iter;
          std::string sub_image = opt.out_prefix + "-"
            + image_path.stem().string() + os.str() + ".tif";
          vw_out() << "Writing subsampled image: " << sub_image << "\n";
          bool has_img_georef = false;
          GeoReference img_georef;
          bool has_img_nodata = true;
          int tile_size = 256;
          int sub_threads = 1;
          TerminalProgressCallback tpc("asp", ": ");
          vw::cartography::block_write_gdal_image
            (sub_image,
             apply_mask
             (block_rasterize
              (vw::cache_tile_aware_render
               (vw::resample_aa
                (masked_images_vec[level-1][dem_iter][image_iter], sub_scale),
                Vector2i(tile_size, tile_size) * sub_scale),
               Vector2i(tile_size, tile_size), sub_threads), img_nodata_val),
             has_img_georef, img_georef, has_img_nodata, img_nodata_val, opt, tpc);
          
          // Read it right back
          if (opt.crop_input_images) {
            // Read it fully in memory, as we cropped it before
            ImageView<float> memory_img = copy(DiskImageView<float>(sub_image));
            masked_images_vec[level][dem_iter][image_iter]
              = create_mask(memory_img, img_nodata_val);
          }else{
            // Read just a handle, as the full image could be huge
            masked_images_vec[level][dem_iter][image_iter]
              = create_mask(DiskImageView<float>(sub_image), img_nodata_val);
          }
        
          if (blend_weights_vec[level-1][dem_iter][image_iter].cols() > 0 &&
              blend_weights_vec[level-1][dem_iter][image_iter].rows() > 0 ) {
            fs::path weight_path(opt.input_images[image_iter]);
            std::string sub_weight = opt.out_prefix + "-wt-"
              + weight_path.stem().string() + os.str() + ".tif";
            vw_out() << "Writing subsampled weight: " << sub_weight << "\n";
          
            vw::cartography::block_write_gdal_image
              (sub_weight,
               apply_mask
               (block_rasterize
                (vw::cache_tile_aware_render
                 (vw::resample_aa
                  (create_mask(blend_weights_vec[level-1][dem_iter][image_iter],
                               dem_nodata_val), sub_scale),
                  Vector2i(tile_size,tile_size) * sub_scale),
                 Vector2i(tile_size, tile_size), sub_threads), dem_nodata_val),
               has_img_georef, img_georef, has_img_nodata, dem_nodata_val, opt, tpc);

            ImageView<double> memory_weight = copy(DiskImageView<double>(sub_weight));
            blend_weights_vec[level][dem_iter][image_iter] = memory_weight;
          }
        
        }
      }
    }
    
    // Start going from the coarsest to the finest level
    for (int level = levels; level >= 0; level--) {

      g_level = level;

      int num_iterations;
      if (level == 0)
        num_iterations = opt.max_iterations;
      else
        num_iterations = opt.max_coarse_iterations;

      // Scale the cameras
      for (int image_iter = 0; image_iter < num_images; image_iter++) {
        for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
          
          if (opt.skip_images[dem_iter].find(image_iter) !=
              opt.skip_images[dem_iter].end()) continue;

          if (!opt.use_approx_adjusted_camera_models) {
            AdjustedCameraModel * adj_cam
              = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
            if (adj_cam == NULL)
              vw_throw( ArgumentErr() << "Expecting adjusted camera.\n");
            adj_cam->set_scale(factors[level]);
          }
        }
      }
      
      run_sfs_level(// Fixed inputs
                    num_iterations, opt, geos[level],
                    opt.smoothness_weight*factors[level]*factors[level],
                    dem_nodata_val, crop_boxes[level],
                    masked_images_vec[level], blend_weights_vec[level],
                    global_params, model_params,
                    orig_dems[level], initial_albedo,
                    lit_image_mask, curvature_in_shadow_weight,
                    // Quantities that will float
                    dems[level], albedos[level], cameras,
                    opt.image_exposures_vec,
                    opt.image_haze_vec,
                    scaled_sun_posns,
                    adjustments, opt.model_coeffs_vec);

      // TODO: Study this. Discarding the coarse DEM and exposure so
      // keeping only the cameras seem to work better.
      // Note that we overwrite dems[level-1] by resampling the coarser
      // dems[level], but we keep orig_dems[level-1] from the beginning.
      if (level > 0) {
        for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
          if (!opt.fix_dem)
            interp_image(dems[level][dem_iter],    sub_scale, dems[level-1][dem_iter]);
          if (opt.float_albedo)
            interp_image(albedos[level][dem_iter], sub_scale, albedos[level-1][dem_iter]);
        }
      }
      
    }

  } ASP_STANDARD_CATCHES;
  
  VW_OUT(DebugMessage, "asp") << "Number of times we used the global lock: "
                              << g_num_locks << std::endl;

  sw_total.stop();
  vw_out() << "Total elapsed time: " << sw_total.elapsed_seconds() << " s." << std::endl;
 
}

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


/// \file sfs.cc

// Turn off warnings from boost and other packages
#if defined(__GNUC__) || defined(__GNUG__)
#define LOCAL_GCC_VERSION (__GNUC__ * 10000                    \
			   + __GNUC_MINOR__ * 100              \
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
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Stopwatch.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/IsisIO/IsisCameraModel.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Camera/RPCModelGen.h>
#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
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

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

typedef ImageViewRef< PixelMask<float> > MaskedImgT;
typedef ImageViewRef<double> DoubleImgT;

// TODO: Study more floating model coefficients.
// TODO: Study why using tabulated camera model and multiple resolutions does
// not work as well as it should.
// TODO: When using approx camera, we assume the DEM and image grids are very similar.
// TODO: Remove warning from the approx camera
// TODO: Make it possible to initialize a DEM from scratch.
// TODO: Study more the multi-resolution approach.
// TODO: Must specify in the SfS doc that the lunar lambertian model fails at poles
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
// TODO: Document the bundle adjustment, and if necessary, manual ip selection.
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
// TODO: Handle the case when the DEM has no-data values.
// TODO: Add various kind of loss function.
// TODO: Study the normal computation formula.
// TODO: Move some code to Core.
// TODO: Make it work with non-ISIS cameras.
// TODO: Clean up some of the classes, not all members are needed.

namespace vw { namespace camera {

  // This class provides an approximation for the point_to_pixel()
  // function of an ISIS camera around a current DEM. The algorithm
  // works by tabulation of point_to_pixel and pixel_to_vector values
  // at the mean dem height.
  class ApproxCameraModel: public CameraModel {
    boost::shared_ptr<CameraModel>  m_exact_camera;
    mutable Vector3 m_mean_dir; // mean vector from camera to ground
    BBox2i m_img_bbox;
    GeoReference m_geo;
    double m_mean_ht;
    mutable ImageView< PixelMask<Vector3> > m_pixel_to_vec_mat;
    mutable ImageView< PixelMask<Vector2> > m_point_to_pix_mat;
    //ImageView< PixelMask<Vector3> > m_camera_center_mat;
    double m_approx_table_gridx, m_approx_table_gridy;
    mutable BBox2 m_point_box, m_crop_box;
    bool m_use_rpc_approximation, m_use_semi_approx;
    vw::Mutex& m_camera_mutex;
    Vector2 m_uncompValue;
    mutable int m_begX, m_endX, m_begY, m_endY;
    mutable bool m_compute_mean, m_stop_growing_range;
    mutable int m_count;
    boost::shared_ptr<asp::RPCModel> m_rpc_model;
    bool m_model_is_valid;
    
    bool comp_rpc_approx_table(AdjustedCameraModel const& adj_camera,
                               boost::shared_ptr<CameraModel> exact_camera,
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
          
            Vector2 cam_pix = exact_camera->point_to_pixel(xyz);
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

            Vector2 cam_pix = exact_camera->point_to_pixel(xyz);
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
	    pix = m_exact_camera->point_to_pixel(xyz);
            //if (true || m_img_bbox.contains(pix))  // Need to think more here
            vec = m_exact_camera->pixel_to_vector(pix);
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
    
    ApproxCameraModel(AdjustedCameraModel const& adj_camera,
                      boost::shared_ptr<CameraModel> exact_camera,
                      BBox2i img_bbox, 
		      ImageView<double> const& dem,
		      GeoReference const& geo,
		      double nodata_val,
		      bool use_rpc_approximation, bool use_semi_approx,
                      double rpc_penalty_weight,
		      vw::Mutex &camera_mutex):
      m_exact_camera(exact_camera), m_img_bbox(img_bbox), m_geo(geo),
      m_use_rpc_approximation(use_rpc_approximation),
      m_use_semi_approx(use_semi_approx),
      m_camera_mutex(camera_mutex), m_model_is_valid(true)
    {

      int big = 1e+8;
      m_uncompValue = Vector2(-big, -big);
      m_compute_mean = true; // We'll set this to false when we finish estimating the mean
      m_stop_growing_range = false; // stop when it does not help
      
      if (dynamic_cast<IsisCameraModel*>(exact_camera.get()) == NULL)
	vw_throw( ArgumentErr()
		  << "ApproxCameraModel: Expecting an unadjusted ISIS camera model.\n");

      //Compute the mean DEM height.
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
	m_model_is_valid = comp_rpc_approx_table(adj_camera, exact_camera, m_img_bbox,
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
      //	       << m_endX - m_begX << ' ' << m_endY - m_begY << std::endl;
      
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

#if 0 // takes too much memory
      // Tabulate the camera_center function
      m_camera_center_mat.set_size(m_crop_box.width(), m_crop_box.height());
      for (int col = 0; col < m_crop_box.width(); col++) {
	for (int row = 0; row < m_crop_box.height(); row++) {
	  bool success = true;
	  Vector3 ctr;
	  try {
	    ctr = m_exact_camera->camera_center(m_crop_box.min() + Vector2(col, row));
	  }catch(...){
	    success = false;
	  }
	  if (success) {
	    m_camera_center_mat(col, row) = ctr;
	    m_camera_center_mat(col, row).validate();
	  }else{
	    m_camera_center_mat(col, row).invalidate();
	  }
	}
      }
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
        return m_exact_camera->point_to_pixel(xyz);
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
	    return m_exact_camera->point_to_pixel(xyz);
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
	  return m_exact_camera->point_to_pixel(xyz);
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
	    return m_exact_camera->point_to_pixel(xyz);
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
        return this->exact_camera()->pixel_to_vector(pix);
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
      return this->exact_camera()->pixel_to_vector(pix);
    }

    virtual Vector3 camera_center(Vector2 const& pix) const{
      // It is tricky to approximate the camera center
      //if (m_use_rpc_approximation){
	vw::Mutex::Lock lock(m_camera_mutex);
	g_num_locks++;
	//vw_out(WarningMessage) << "Invoked the camera center function for pixel: "
        //                       << pix << std::endl;
	return this->exact_camera()->camera_center(pix);
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
	return this->exact_camera()->camera_center(pix);
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
      return this->exact_camera()->camera_pose(pix);
    }

    boost::shared_ptr<CameraModel> exact_camera() const{
      return m_exact_camera;
    }
  };
}}

// Execute a command and capture its output
// TODO: Move this to a better place.
std::string exec_cmd(const char* cmd) {
    char buffer[128];
    std::string result = "";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) throw std::runtime_error("popen() failed!");
    try {
        while (!feof(pipe)) {
            if (fgets(buffer, 128, pipe) != NULL)
                result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

// Get the memory usage for the given process.
void callTop() {

  std::ostringstream os;
  int pid = getpid();
  os << pid;
  
  std::string cmd = "top -b -n 1 | grep -i 'sfs ' | grep -i '" + os.str() + " '";
  std::string ans = exec_cmd(cmd.c_str());
  vw_out() << "Memory usage: " << cmd << " " << ans << std::endl;
}

// Pull the ISIS model from an adjusted IsisCameraModel or ApproxCameraModel
boost::shared_ptr<CameraModel> get_isis_cam(boost::shared_ptr<CameraModel> cam){

  AdjustedCameraModel * acam = dynamic_cast<AdjustedCameraModel*>(cam.get());
  if (acam == NULL)
    vw_throw( ArgumentErr() << "get_isis_cam: Expecting an adjusted camera model.\n" );

  boost::shared_ptr<CameraModel> ucam = acam->unadjusted_model();
  if (ucam.get() == NULL)
    vw_throw( ArgumentErr() << "get_isis_cam: Expecting a valid camera model.\n" );

  ApproxCameraModel * apcam = dynamic_cast<ApproxCameraModel*>(ucam.get());
  if (apcam != NULL)
    return apcam->exact_camera();

  if (dynamic_cast<IsisCameraModel*>(ucam.get()) == NULL)
    vw_throw( ArgumentErr() << "get_isis_cam: Expecting an ISIS camera model.\n" );

  return ucam;
}


// Compute mean and standard deviation of an image
template <class ImageT>
void compute_image_stats(ImageT const& I, double & mean, double & stdev){
  mean  = 0;
  stdev = 0;
  double sum = 0.0, sum2 = 0.0, count = 0.0;
  for (int col = 0; col < I.cols(); col++){
    for (int row = 0; row < I.rows(); row++){
      if (!is_valid(I(col, row))) continue;
      count++;
      double val = I(col, row);
      sum += val;
      sum2 += val*val;
    }
  }

  if (count > 0){
    mean = sum/count;
    stdev = sqrt( sum2/count - mean*mean );
  }

}

// Find the points on a given DEM that are shadowed by other points of
// the DEM.  Start marching from the point on the DEM on a ray towards
// the sun in small increments, until hitting the maximum DEM height.
bool isInShadow(int col, int row, Vector3 & sunPos,
		ImageView<double> const& dem, double max_dem_height,
		double gridx, double gridy,
		cartography::GeoReference const& geo){

  // Here bicubic interpolation won't work. It is easier to interpret
  // the DEM as piecewise-linear when dealing with rays intersecting
  // it.
  InterpolationView<EdgeExtensionView< ImageView<double>,
    ConstantEdgeExtension >, BilinearInterpolation>
    interp_dem = interpolate(dem, BilinearInterpolation(),
			     ConstantEdgeExtension());

  // The xyz position at the center grid point
  Vector2 dem_llh = geo.pixel_to_lonlat(Vector2(col, row));
  Vector3 dem_lonlat_height = Vector3(dem_llh(0), dem_llh(1), dem(col, row));
  Vector3 xyz = geo.datum().geodetic_to_cartesian(dem_lonlat_height);

  // Normalized direction from the view point
  Vector3 dir = sunPos - xyz;
  if (dir == Vector3())
    return false;
  dir = dir/norm_2(dir);

  // The projection of dir onto the tangent plane at xyz,
  // that is, the "horizontal" component at the current sphere surface.
  Vector3 dir2 = dir - dot_prod(dir, xyz)*xyz/dot_prod(xyz, xyz);

  // Ensure that we advance by at most half a grid point each time
  double delta = 0.5*std::min(gridx, gridy)/std::max(norm_2(dir2), 1e-16);

  // go along the ray. Don't allow the loop to go forever.
  for (int i = 1; i < 10000000; i++) {
    Vector3 ray_P = xyz + i * delta * dir;
    Vector3 ray_llh = geo.datum().cartesian_to_geodetic(ray_P);
    if (ray_llh[2] > max_dem_height) {
      // We're above the terrain, no point in continuing
      return false;
    }

    // Compensate for any longitude 360 degree offset, e.g., 270 deg vs -90 deg
    ray_llh[0] += 360.0*round((dem_llh[0] - ray_llh[0])/360.0);

    Vector2 ray_pix = geo.lonlat_to_pixel(Vector2(ray_llh[0], ray_llh[1]));

    if (ray_pix[0] < 0 || ray_pix[0] > dem.cols() - 1 ||
	ray_pix[1] < 0 || ray_pix[1] > dem.rows() - 1 ) {
      return false; // got out of the DEM, no point continuing
    }

    // Dem height at the current point on the ray
    double dem_h = interp_dem(ray_pix[0], ray_pix[1]);

    if (ray_llh[2] < dem_h) {
      // The ray goes under the DEM, so we are in shadow.
      return true;
    }
  }

  return false;
}

void areInShadow(Vector3 & sunPos, ImageView<double> const& dem,
		 double gridx, double gridy,
		 cartography::GeoReference const& geo,
		 ImageView<float> & shadow){

  // Find the max DEM height
  double max_dem_height = -std::numeric_limits<double>::max();
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      if (dem(col, row) > max_dem_height) {
	max_dem_height = dem(col, row);
      }
    }
  }

  shadow.set_size(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      shadow(col, row) = isInShadow(col, row, sunPos, dem,
				    max_dem_height, gridx, gridy, geo);
    }
  }
}

struct Options : public vw::cartography::GdalWriteOptions {
  std::string input_dems_str, out_prefix, stereo_session_string, bundle_adjust_prefix;
  std::vector<std::string> input_dems, input_images, input_cameras;
  std::string shadow_thresholds, max_valid_image_vals, skip_images_str, image_exposure_prefix,
    model_coeffs_prefix, model_coeffs;
  std::vector<float> shadow_threshold_vec, max_valid_image_vals_vec;
  std::vector<double> image_exposures_vec;
  std::vector<double> model_coeffs_vec;
  std::vector< std::set<int> > skip_images;

  int max_iterations, max_coarse_iterations, reflectance_type, coarse_levels, blending_dist,
    blending_power;
  bool float_albedo, float_exposure, float_cameras, float_all_cameras, model_shadows,
    save_computed_intensity_only,
    save_dem_with_nodata, use_approx_camera_models, use_rpc_approximation, use_semi_approx, crop_input_images,
    use_blending_weights,
    float_dem_at_boundary, fix_dem, float_reflectance_model, query, save_sparingly;
  double smoothness_weight, init_dem_height, nodata_val, initial_dem_constraint_weight,
    albedo_constraint_weight, camera_position_step_size, rpc_penalty_weight, unreliable_intensity_threshold;
  vw::BBox2 crop_win;

  Options():max_iterations(0), max_coarse_iterations(0), reflectance_type(0),
	    coarse_levels(0), blending_dist(10), blending_power(2),
            float_albedo(false), float_exposure(false), float_cameras(false),
            float_all_cameras(false),
	    model_shadows(false),
	    save_computed_intensity_only(false),
	    save_dem_with_nodata(false),
	    use_approx_camera_models(false),
	    use_rpc_approximation(false),
            use_semi_approx(false),
	    crop_input_images(false), use_blending_weights(false),
            float_dem_at_boundary(false), fix_dem(false),
            float_reflectance_model(false), query(false), save_sparingly(false),
	    smoothness_weight(0), initial_dem_constraint_weight(0.0),
	    albedo_constraint_weight(0.0),
	    camera_position_step_size(1.0), rpc_penalty_weight(0.0),
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
                                                   const double * coeffs) {
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
  double O = coeffs[0]; // 1
  double A = coeffs[1]; //-0.019;
  double B = coeffs[2]; // 0.000242;//0.242*1e-3;
  double C = coeffs[3]; // -0.00000146;//-1.46*1e-6;

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

// Hapke's model.
// See: An Experimental Study of Light Scattering by Large, Irregular Particles
// Audrey F. McGuire, Bruce W. Hapke. 1995. The reflectance used is R(g), in equation
// above Equation 21. The p(g) function is given by Equation (14), yet this one uses
// an old convention. The updated p(g) is given in:
// Spectrophotometric properties of materials observed by Pancam on the Mars Exploration Rovers: 1.
// Spirit. JR Johnson, 2006.
// We Use the two-term p(g), and the parameter c, not c'=1-c.
// We also use the values of w(=omega), b, and c from that table.
// Note that we use the updated Hapke model, having the term B(g). This one is given in
// "Modeling spectral and bidirectional reflectance", Jacquemoud, 1992. It has the params
// B0 and h.
// The ultimate reference is probably Hapke, 1986, having all pieces in one place, but
// that one is not available. 
// We use mostly the parameter values for omega, b, c, B0 and h from:
// Surface reflectance of Mars observed by CRISM/MRO: 2.
// Estimation of surface photometric properties in Gusev Crater and Meridiani Planum by J. Fernando. 
// See equations (1), (2) and (4) in that paper.
// Example values for the params: w=omega=0.68, b=0.17, c=0.62, B0=0.52, h=0.52.
// But we don't use equation (3) from that paper, we use instead what they call the formula H93,
// which is the H(x) from McGuire and Hapke 1995 mentioned above.
// See the complete formulas below.
double computeHapkeReflectanceFromNormal(Vector3 const& sunPos,
                                         Vector3 const& viewPos,
                                         Vector3 const& xyz,
                                         Vector3 const& normal,
                                         double phaseCoeffC1,
                                         double phaseCoeffC2,
                                         double & alpha,
                                         const double * coeffs) {

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
  double omega = std::abs(coeffs[0]); // also known as w
  double b     = std::abs(coeffs[1]);
  double c     = std::abs(coeffs[2]);
  double B0    = std::abs(coeffs[3]);    // The older Hapke model lacks the B0 and h terms
  double h     = std::abs(coeffs[4]);   

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
					  const double * coeffs) {

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
  double A       = std::abs(coeffs[0]); // albedo 
  double f_alpha = std::abs(coeffs[1]); // phase function 

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
                                                    const double * coeffs) {
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
  double O1 = coeffs[0]; // 1
  double A1 = coeffs[1]; //-0.019;
  double B1 = coeffs[2]; // 0.000242;//0.242*1e-3;
  double C1 = coeffs[3]; // -0.00000146;//-1.46*1e-6;
  double D1 = coeffs[4]; 
  double E1 = coeffs[5]; 
  double F1 = coeffs[6]; 
  double G1 = coeffs[7]; 

  double O2 = coeffs[8]; // 1
  double A2 = coeffs[9]; //-0.019;
  double B2 = coeffs[10]; // 0.000242;//0.242*1e-3;
  double C2 = coeffs[11]; // -0.00000146;//-1.46*1e-6;
  double D2 = coeffs[12]; 
  double E2 = coeffs[13]; 
  double F2 = coeffs[14]; 
  double G2 = coeffs[15]; 
  
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
                          const double * coeffs) {
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
                                                      coeffs);
      break;
    case ARBITRARY_MODEL:
      input_img_reflectance
	= computeArbitraryLambertianReflectanceFromNormal(input_img_params.sunPosition,
						      cameraPosition,
						      xyz,  normal,
						      global_params.phaseCoeffC1,
						      global_params.phaseCoeffC2,
						      phase_angle, // output
                                                      coeffs);
      break;
    case HAPKE:
      input_img_reflectance
	= computeHapkeReflectanceFromNormal(input_img_params.sunPosition,
                                            cameraPosition,
                                            xyz,  normal,
                                            global_params.phaseCoeffC1,
                                            global_params.phaseCoeffC2,
                                            phase_angle, // output
                                            coeffs);
      break;
    case CHARON:
      input_img_reflectance
	= computeCharonReflectanceFromNormal(input_img_params.sunPosition,
					     cameraPosition,
					     xyz,  normal,
					     global_params.phaseCoeffC1,
					     global_params.phaseCoeffC2,
					     phase_angle, // output
					     coeffs);
      break;
    case LAMBERT:
      input_img_reflectance
	= computeLambertianReflectanceFromNormal(input_img_params.sunPosition,
						 xyz,  normal);
      break;

    default:
      input_img_reflectance = 1;
    }

  return input_img_reflectance;
}

bool computeReflectanceAndIntensity(double left_h, double center_h, double right_h,
				    double bottom_h, double top_h,
				    int col, int row,
				    ImageView<double> const& dem,
				    cartography::GeoReference const& geo,
				    bool model_shadows,
				    double max_dem_height,
				    double gridx, double gridy,
				    ModelParams const& model_params,
				    GlobalParams const& global_params,
				    BBox2i const& crop_box,
				    MaskedImgT const & image,
				    DoubleImgT const & blend_weight,
				    CameraModel const* camera,
				    PixelMask<double> & reflectance,
				    PixelMask<double> & intensity,
				    double            & weight,
                                    const double * coeffs) {

  // Set output values
  reflectance = 0.0; reflectance.invalidate();
  intensity   = 0.0; intensity.invalidate();
  weight      = 0.0;
  
  if (col >= dem.cols() - 1 || row >= dem.rows() - 1) return false;
  if (crop_box.empty()) return false;
    
  // TODO: Investigate various ways of finding the normal.

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

#if 0
  // two-point normal
  Vector3 dx = right - base;
  Vector3 dy = bottom - base;
#else
  // four-point normal (centered)
  Vector3 dx = right - left;
  Vector3 dy = bottom - top;
#endif
  Vector3 normal = -normalize(cross_prod(dx, dy)); // so normal points up

  // Update the camera position for the given pixel (camera position
  // is pixel-dependent for for linescan cameras.
  ModelParams local_model_params = model_params;
  Vector2 pix;
  Vector3 cameraPosition;
  try {
    pix = camera->point_to_pixel(base);
    
    // Need camera center only for Lunar Lambertian
    if ( global_params.reflectanceType != LAMBERT ) {
      cameraPosition = camera->camera_center(pix);
    }
    
  } catch(...){
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    weight      = 0.0;
    return false;
  }
  
  double phase_angle;
  reflectance = ComputeReflectance(cameraPosition,
				   normal, base, local_model_params,
				   global_params, phase_angle,
                                   coeffs);
  reflectance.validate();


  // Since our image is cropped
  pix -= crop_box.min();

  // Check for out of range
  if (pix[0] < 0 || pix[0] >= image.cols()-1 ||
      pix[1] < 0 || pix[1] >= image.rows()-1) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    weight      = 0.0;
    return false;
  }

  InterpolationView<EdgeExtensionView<MaskedImgT, ConstantEdgeExtension>, BilinearInterpolation>
    interp_image = interpolate(image, BilinearInterpolation(),
			       ConstantEdgeExtension());
  intensity = interp_image(pix[0], pix[1]); // this interpolates

  InterpolationView<EdgeExtensionView<DoubleImgT, ConstantEdgeExtension>, BilinearInterpolation>
    interp_weight = interpolate(blend_weight, BilinearInterpolation(),
                                ConstantEdgeExtension());
  if (blend_weight.cols() > 0 && blend_weight.rows() > 0) // The weight may not exist
    weight = interp_weight(pix[0], pix[1]); // this interpolates
  else
    weight = 1.0;

  if (!is_valid(intensity)) {
    reflectance = 0.0; reflectance.invalidate();
    intensity   = 0.0; intensity.invalidate();
    weight      = 0.0;
    return false;
  }


  if (model_shadows) {
    bool inShadow = isInShadow(col, row, local_model_params.sunPosition,
			       dem, max_dem_height, gridx, gridy,
			       geo);

    if (inShadow) {
      // The reflectance is valid, it is just zero
      reflectance = 0;
      reflectance.validate();
    }
  }

  return true;
}

void computeReflectanceAndIntensity(ImageView<double> const& dem,
				    cartography::GeoReference const& geo,
				    bool model_shadows,
				    double & max_dem_height, // alias
				    double gridx, double gridy,
				    ModelParams const& model_params,
				    GlobalParams const& global_params,
				    BBox2i const& crop_box,
				    MaskedImgT const & image,
				    DoubleImgT const & blend_weight,
				    CameraModel const* camera,
				    ImageView< PixelMask<double> > & reflectance,
				    ImageView< PixelMask<double> > & intensity,
				    ImageView< double            > & weight,
                                    const double * coeffs) {

  // Update max_dem_height
  max_dem_height = -std::numeric_limits<double>::max();
  if (model_shadows) {
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
        if (dem(col, row) > max_dem_height) {
          max_dem_height = dem(col, row);
        }
      }
    }
    vw_out() << "Maximum DEM height: " << max_dem_height << std::endl;
  }
  
  // Init the reflectance and intensity as invalid
  reflectance.set_size(dem.cols(), dem.rows());
  intensity.set_size(dem.cols(), dem.rows());
  weight.set_size(dem.cols(), dem.rows());
  for (int col = 0; col < dem.cols(); col++) {
    for (int row = 0; row < dem.rows(); row++) {
      reflectance(col, row).invalidate();
      intensity(col, row).invalidate();
      weight(col, row) = 0.0;
    }
  }

  for (int col = 1; col < dem.cols()-1; col++) {
    for (int row = 1; row < dem.rows()-1; row++) {
      computeReflectanceAndIntensity(dem(col-1, row), dem(col, row),
				     dem(col+1, row),
				     dem(col, row+1), dem(col, row-1),
				     col, row, dem,  geo,
				     model_shadows, max_dem_height,
				     gridx, gridy,
				     model_params, global_params,
				     crop_box, image, blend_weight, camera,
				     reflectance(col, row), intensity(col, row),
                                     weight(col, row),
                                     coeffs);
    }
  }

  return;
}

std::string exposure_file_name(std::string const& prefix){
  return prefix + "-exposures.txt";
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

// A function to invoke at every iteration of ceres.
// We need a lot of global variables to do something useful.
int                                            g_iter = -1;
Options                                const * g_opt;
std::vector< ImageView<double> >             * g_dem;
std::vector< ImageView<double> >             * g_albedo;
std::vector<cartography::GeoReference> const * g_geo;
GlobalParams                           const * g_global_params;
std::vector<ModelParams>               const * g_model_params;
std::vector< std::vector<BBox2i> >     const * g_crop_boxes;
std::vector< std::vector<MaskedImgT> > const * g_masked_images;
std::vector< std::vector<DoubleImgT> > const * g_blend_weights;
std::vector< std::vector<boost::shared_ptr<CameraModel> > > * g_cameras;
double                                       * g_dem_nodata_val;
float                                        * g_img_nodata_val;
std::vector<double>                          * g_exposures;
std::vector<double>                          * g_adjustments;
std::vector<double>                          * g_max_dem_height;
double                                       * g_gridx;
double                                       * g_gridy;
int                                            g_level = -1;
bool                                           g_final_iter = false;
double                                       * g_coeffs; 

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
    callTop();

    std::string exposure_file = exposure_file_name(g_opt->out_prefix);
    vw_out() << "Writing: " << exposure_file << std::endl;
    std::ofstream exf(exposure_file.c_str());
    exf.precision(18);
    for (size_t image_iter = 0; image_iter < (*g_exposures).size(); image_iter++){
      exf << g_opt->input_images[image_iter] << " " << (*g_exposures)[image_iter] << "\n";
    }
    exf.close();
    
    std::string model_coeffs_file = model_coeffs_file_name(g_opt->out_prefix);
    vw_out() << "Writing: " << model_coeffs_file << std::endl;
    std::ofstream mcf(model_coeffs_file.c_str());
    mcf.precision(18);
    for (size_t coeff_iter = 0; coeff_iter < g_num_model_coeffs; coeff_iter++){
      mcf << g_coeffs[coeff_iter] << " ";
    }
    mcf << "\n";
    mcf.close();

    vw_out() << "Model coefficients: "; 
    for (size_t i = 0; i < g_num_model_coeffs; i++) vw_out() << g_coeffs[i] << " ";
    vw_out() << std::endl;
    
    vw_out() << "cam adj: ";
    for (int s = 0; s < int((*g_adjustments).size()); s++) {
      vw_out() << (*g_adjustments)[s] << " ";
    }
    vw_out() << std::endl;

    int num_dems = (*g_dem).size();
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
      // Apply the most recent adjustments to the cameras.
      for (size_t image_iter = 0; image_iter < (*g_masked_images)[dem_iter].size(); image_iter++) {
        if (g_opt->skip_images[dem_iter].find(image_iter) !=
            g_opt->skip_images[dem_iter].end()) continue;
       
        AdjustedCameraModel * icam
          = dynamic_cast<AdjustedCameraModel*>((*g_cameras)[dem_iter][image_iter].get());
        if (icam == NULL)
          vw_throw(ArgumentErr() << "Expecting adjusted camera models.\n");
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
      if ( !g_opt->save_sparingly || g_final_iter ) {
        std::string out_dem_file = g_opt->out_prefix + "-DEM"
          + iter_str + ".tif";
        vw_out() << "Writing: " << out_dem_file << std::endl;
        block_write_gdal_image(out_dem_file, (*g_dem)[dem_iter], has_georef, (*g_geo)[dem_iter],
                               has_nodata, *g_dem_nodata_val,
                               *g_opt, tpc);
      }
      
      if (!g_opt->save_sparingly || (g_final_iter && g_opt->float_albedo) ) {
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
      
        // Separate into blocks for each image
        vw_out() << "\n";

        ImageView< PixelMask<double> > reflectance, intensity, comp_intensity;
        ImageView< double            > blend_weight;

        // Save the camera adjustments for the current iteration
        //std::string out_camera_file = g_opt->out_prefix + "-camera"
	//+ iter_str2 + ".adjust";
        //vw_out() << "Writing: " << out_camera_file << std::endl;
        AdjustedCameraModel * icam
          = dynamic_cast<AdjustedCameraModel*>((*g_cameras)[dem_iter][image_iter].get());
        if (icam == NULL)
          vw_throw( ArgumentErr() << "Expecting adjusted camera.\n");
        Vector3 translation = icam->translation();
        Quaternion<double> rotation = icam->rotation();
        //asp::write_adjustments(out_camera_file, translation, rotation);

        // Save adjusted files in the format <out prefix>-<input-img>.adjust
        // so we can later read them with --bundle-adjust-prefix to be
        // used in another SfS run.
        //if (g_level == 0) {
	std::string out_camera_file
	  = asp::bundle_adjust_file_name(g_opt->out_prefix,
					 g_opt->input_images[image_iter],
					 g_opt->input_cameras[image_iter]);
	if (!g_opt->save_computed_intensity_only){
	  vw_out() << "Writing: " << out_camera_file << std::endl;
	  asp::write_adjustments(out_camera_file, translation, rotation);
	}

        if (g_opt->save_sparingly && !g_opt->save_dem_with_nodata) 
          continue; // don't write too many things
        
	// Manufacture an output prefix for the other data associated with this camera
	std::string iter_str2 = fs::path(out_camera_file).replace_extension("").string();
	iter_str2 += iter_str;
	
        // Compute reflectance and intensity with optimized DEM
        computeReflectanceAndIntensity((*g_dem)[dem_iter], (*g_geo)[dem_iter],
                                       g_opt->model_shadows,
                                       (*g_max_dem_height)[dem_iter],
                                       *g_gridx, *g_gridy,
                                       (*g_model_params)[image_iter],
                                       *g_global_params,
                                       (*g_crop_boxes)[dem_iter][image_iter],
                                       (*g_masked_images)[dem_iter][image_iter],
                                       (*g_blend_weights)[dem_iter][image_iter],
                                       (*g_cameras)[dem_iter][image_iter].get(),
                                       reflectance, intensity, blend_weight, 
                                       g_coeffs);

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
              = (*g_albedo)[dem_iter](col, row) * (*g_exposures)[image_iter]
              * reflectance(col, row);
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
                               blend_weight,
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
              measured_albedo(col, row)
                = intensity(col, row)/(reflectance(col, row)*(*g_exposures)[image_iter]);
          }
        }
	std::string out_albedo_file = iter_str2 + "-meas-albedo.tif";
        vw_out() << "Writing: " << out_albedo_file << std::endl;
        block_write_gdal_image(out_albedo_file, measured_albedo,
                               has_georef, (*g_geo)[dem_iter], has_nodata, 0, *g_opt, tpc);


        double imgmean, imgstdev, refmean, refstdev;
        compute_image_stats(intensity, imgmean, imgstdev);
        compute_image_stats(comp_intensity, refmean, refstdev);

        vw_out() << "meas image mean and std: " << imgmean << ' ' << imgstdev
                 << std::endl;
        vw_out() << "comp image mean and std: " << refmean << ' ' << refstdev
                 << std::endl;

        vw_out() << "Exposure for image " << image_iter << ": "
                 << (*g_exposures)[image_iter] << std::endl;

#if 0
        // Dump the points in shadow
        ImageView<float> shadow; // don't use int, scaled weirdly by ASP on reading
        Vector3 sunPos = (*g_model_params)[image_iter].sunPosition;
        areInShadow(sunPos, (*g_dem)[dem_iter], *g_gridx, *g_gridy,  (*g_geo)[dem_iter], shadow);

	std::string out_shadow_file = iter_str2 + "-shadow.tif";
        vw_out() << "Writing: " << out_shadow_file << std::endl;
        block_write_gdal_image(out_shadow_file, shadow, has_georef, (*g_geo)[dem_iter], has_nodata,
                               -std::numeric_limits<float>::max(), *g_opt, tpc);
#endif

      }

      if (g_opt->save_dem_with_nodata) {
        if ( !g_opt->save_sparingly || g_final_iter ) {
          std::string out_dem_nodata_file = g_opt->out_prefix + "-DEM-nodata"
            + iter_str + ".tif";
          vw_out() << "Writing: " << out_dem_nodata_file << std::endl;
          TerminalProgressCallback tpc("asp", ": ");
          block_write_gdal_image(out_dem_nodata_file, dem_nodata, has_georef, (*g_geo)[dem_iter],
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
  inline bool calc_residual(const F* const exposure,
                            const G* const left,
                            const G* const center,
                            const G* const right,
                            const G* const bottom,
                            const G* const top,
                            const G* const albedo,
                            const F* const adjustments, // camera adjustments
                            const G* const coeffs, // Lunar lambertian model coeffs
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

      AdjustedCameraModel * adj_cam
	= dynamic_cast<AdjustedCameraModel*>(m_camera.get());
      if (adj_cam == NULL)
	vw_throw( ArgumentErr() << "Expecting adjusted camera.\n");

      // We create a copy of this camera to avoid issues when using
      // multiple threads. We copy just the adjustment parameters,
      // the pointer to the underlying ISIS camera is shared.
      AdjustedCameraModel adj_cam_copy = *adj_cam;

      // Apply current adjustments to the camera
      Vector3 axis_angle;
      Vector3 translation;
      for (int param_iter = 0; param_iter < 3; param_iter++) {
	translation[param_iter]
	  = (g_position_scale_factor*m_camera_position_step_size)*adjustments[param_iter];
	axis_angle[param_iter] = adjustments[3 + param_iter];
      }
      adj_cam_copy.set_translation(translation);
      adj_cam_copy.set_axis_angle_rotation(axis_angle);

      PixelMask<double> reflectance, intensity;
      double weight;
      bool success =
	computeReflectanceAndIntensity(left[0], center[0], right[0],
				       bottom[0], top[0],
				       m_col, m_row,  m_dem, m_geo,
				       m_model_shadows, m_max_dem_height,
				       m_gridx, m_gridy,
				       m_model_params,  m_global_params,
				       m_crop_box, m_image, m_blend_weight, &adj_cam_copy,
				       reflectance, intensity, weight, coeffs);
      
      if (g_opt->unreliable_intensity_threshold > 0){
        if (is_valid(intensity) && intensity.child() <= g_opt->unreliable_intensity_threshold &&
            intensity.child() >= 0) {
          weight *=
          pow(intensity.child()/g_opt->unreliable_intensity_threshold, 2.0);
        }
      }
      
      if (success && is_valid(intensity) && is_valid(reflectance))
	residuals[0] = weight*(intensity - albedo[0]*exposure[0]*reflectance).child();
      
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
// sum_i | I_i - albedo * exposures[i] * reflectance_i |^2
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
		  const F* const left,
		  const F* const center,
		  const F* const right,
		  const F* const bottom,
		  const F* const top,
		  const F* const albedo,
		  const F* const adjustments, // camera adjustments
		  const F* const coeffs, // Lunar lambertian model coeffs
                  F* residuals) const {

    return calc_residual(exposure, left, center, right, bottom,  top, albedo,
                         adjustments,  // camera adjustments
                         coeffs,  // Lunar lambertian model coeffs
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
    return (new ceres::NumericDiffCostFunction<IntensityError,
	    ceres::CENTRAL, 1, 1, 1, 1, 1, 1, 1, 1, 6, g_num_model_coeffs>
	    (new IntensityError(col, row, dem, geo,
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

// A variation of IntensityError where albedo, dem, and model params are fixed.
struct IntensityErrorFixedMost {
  IntensityErrorFixedMost(int col, int row,
                          ImageView<double> const& dem,
                          double albedo,
                          double * coeffs, 
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
    m_albedo(albedo), m_coeffs(coeffs), 
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
		  const F* const adjustments, // camera adjustments
                  F* residuals) const {

    return calc_residual(exposure,
                         &m_dem(m_col-1, m_row),            // left
                         &m_dem(m_col, m_row),              // center
                         &m_dem(m_col+1, m_row),            // right
                         &m_dem(m_col, m_row+1),            // bottom
                         &m_dem(m_col, m_row-1),            // top
                         &m_albedo,
                         adjustments,  // camera adjustments
                         m_coeffs,  // Lunar lambertian model coeffs
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
                                     double * coeffs, 
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
	    ceres::CENTRAL, 1, 1, 6>
	    (new IntensityErrorFixedMost(col, row, dem, albedo, coeffs, geo,
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
  double                                  * m_coeffs; 
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
    try{

      // Normalize by grid size seems to make the functional less
      // sensitive to the actual grid size used.
      residuals[0] = (left[0] + right[0] - 2*center[0])/m_gridx/m_gridx;   // u_xx
      residuals[1] = (br[0] + tl[0] - bl[0] - tr[0] )/4.0/m_gridx/m_gridy; // u_xy
      residuals[2] = residuals[1];                                         // u_yx
      residuals[3] = (bottom[0] + top[0] - 2*center[0])/m_gridy/m_gridy;   // u_yy

      for (int i = 0; i < 4; i++)
	residuals[i] *= m_smoothness_weight;

    } catch (const camera::PointToPixelErr& e) {
      // Failed to compute the residuals
      residuals[0] = T(1e+20);
      residuals[1] = T(1e+20);
      residuals[2] = T(1e+20);
      residuals[3] = T(1e+20);
      return false;
    }

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

ImageView<double> comp_blending_weights(MaskedImgT const& img,
                                        double blending_dist,
                                        double blending_power){
 
//   if (img.cols() <= 2 || img.rows() <= 2) {
//     // The image is too small to have good weights. grassfire crashes.
//     ImageView<double> weights(img.cols(), img.rows());
//     for (int col = 0; col < weights.cols(); col++) {
//       for (int row = 0; row < weights.rows(); row++) {
// 	weights(col, row) = 0;
//       }
//     }
//     return weights;
//   }
  
  ImageView<double> weights = grassfire(img);

  for (int col = 0; col < weights.cols(); col++) {
    for (int row = 0; row < weights.rows(); row++) {
      weights(col, row) = pow( std::min(weights(col, row)/blending_dist, 1.0), blending_power );
    }
  }
  return weights;
}

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("input-dem,i",  po::value(&opt.input_dems_str),
     "The input DEM(s) to refine using SfS. If more than one, their list should be in quotes.")
    ("output-prefix,o", po::value(&opt.out_prefix),
     "Prefix for output filenames.")
    ("max-iterations,n", po::value(&opt.max_iterations)->default_value(100),
     "Set the maximum number of iterations.")
    ("reflectance-type", po::value(&opt.reflectance_type)->default_value(1),
     "Reflectance type (0 = Lambertian, 1 = Lunar-Lambert, 2 = Hapke, 3 = Experimental extension of Lunar-Lambert, 4 = Charon model (a variation of Lunar-Lambert)).")
    ("smoothness-weight", po::value(&opt.smoothness_weight)->default_value(0.04),
     "A larger value will result in a smoother solution.")
    ("initial-dem-constraint-weight", po::value(&opt.initial_dem_constraint_weight)->default_value(0),
     "A larger value will try harder to keep the SfS-optimized DEM closer to the initial guess DEM.")
    ("albedo-constraint-weight", po::value(&opt.albedo_constraint_weight)->default_value(0),
     "If floating the albedo, a larger value will try harder to keep the optimized albedo close to the nominal value of 1.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustments obtained by previously running bundle_adjust with this output prefix.")
    ("float-albedo",   po::bool_switch(&opt.float_albedo)->default_value(false)->implicit_value(true),
     "Float the albedo for each pixel. Will give incorrect results if only one image is present.")
    ("float-exposure",   po::bool_switch(&opt.float_exposure)->default_value(false)->implicit_value(true),
     "Float the exposure for each image. Will give incorrect results if only one image is present.")
    ("float-cameras",   po::bool_switch(&opt.float_cameras)->default_value(false)->implicit_value(true),
     "Float the camera pose for each image except the first one.")
    ("float-all-cameras",   po::bool_switch(&opt.float_all_cameras)->default_value(false)->implicit_value(true),
     "Float the camera pose for each image, including the first one. Experimental.")
    ("model-shadows",   po::bool_switch(&opt.model_shadows)->default_value(false)->implicit_value(true),
     "Model the fact that some points on the DEM are in the shadow (occluded from the Sun).")
    ("save-computed-intensity-only",   po::bool_switch(&opt.save_computed_intensity_only)->default_value(false)->implicit_value(true),
     "Do not run any optimization. Simply compute the intensity for a given DEM with exposures, camera positions, etc, coming from a previous SfS run. Useful with --model-shadows.")
    ("shadow-thresholds", po::value(&opt.shadow_thresholds)->default_value(""),
     "Optional shadow thresholds for the input images (a list of real values in quotes, one per image).")
    ("max-valid-image-vals", po::value(&opt.max_valid_image_vals)->default_value(""),
     "Optional values for the largest valid image value in each image (a list of real values in quotes, one per image).")
    ("unreliable-intensity-threshold", po::value(&opt.unreliable_intensity_threshold)->default_value(0.0),
     "Intensities lower than this will be considered unreliable and given less weight.")
    ("skip-images", po::value(&opt.skip_images_str)->default_value(""), "Skip images with these indices (indices start from 0).")
    ("save-dem-with-nodata",   po::bool_switch(&opt.save_dem_with_nodata)->default_value(false)->implicit_value(true),
     "Save a copy of the DEM while using a no-data value at a DEM grid point where all images show shadows. To be used if shadow thresholds are set.")
    ("use-approx-camera-models",   po::bool_switch(&opt.use_approx_camera_models)->default_value(false)->implicit_value(true),
     "Use approximate camera models for speed.")
    ("use-rpc-approximation",   po::bool_switch(&opt.use_rpc_approximation)->default_value(false)->implicit_value(true),
     "Use RPC approximations for the camera models instead of approximate tabulated camera models (invoke with --use-approx-camera-models).")
    ("rpc-penalty-weight", po::value(&opt.rpc_penalty_weight)->default_value(0.1),
     "The RPC penalty weight to use to keep the higher-order RPC coefficients small, if the RPC model approximation is used. Higher penalty weight results in smaller such coefficients.")
    ("use-semi-approx",   po::bool_switch(&opt.use_semi_approx)->default_value(false)->implicit_value(true),
     "This is an undocumented experiment.")
    ("coarse-levels", po::value(&opt.coarse_levels)->default_value(0),
     "Solve the problem on a grid coarser than the original by a factor of 2 to this power, then refine the solution on finer grids.")
    ("max-coarse-iterations", po::value(&opt.max_coarse_iterations)->default_value(50),
     "How many iterations to do at levels of resolution coarser than the final result.")
    ("crop-input-images",   po::bool_switch(&opt.crop_input_images)->default_value(false)->implicit_value(true),
     "Crop the images to a region that was computed to be large enough, and keep them fully in memory, for speed.")
    ("use-blending-weights", po::bool_switch(&opt.use_blending_weights)->default_value(false)->implicit_value(true),
     "Give less weight to image pixels close to no-data or boundary values. Enabled only when crop-input-images is true, for performance reasons.")
    ("blending-dist", po::value(&opt.blending_dist)->default_value(10),
     "Over how many pixels to blend.")
    ("blending-power", po::value(&opt.blending_power)->default_value(2),
     "A higher value will result in smoother blending.")
    ("image-exposures-prefix", po::value(&opt.image_exposure_prefix)->default_value(""),
     "Use this prefix to optionally read initial exposures (filename is <prefix>-exposures.txt).")
    ("model-coeffs-prefix", po::value(&opt.model_coeffs_prefix)->default_value(""),
     "Use this prefix to optionally read model coefficients from a file (filename is <prefix>-model_coeffs.txt).")
    ("model-coeffs", po::value(&opt.model_coeffs)->default_value(""),
     "Use the model coefficients specified as a list of numbers in quotes. Lunar-Lambertian: O, A, B, C, e.g., '1 0.019 0.000242 -0.00000146'. Hapke: omega, b, c, B0, h, e.g., '0.68 0.17 0.62 0.52 0.52'. Charon: A, f(alpha), e.g., '0.7 0.63'.")
    ("init-dem-height", po::value(&opt.init_dem_height)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this value for initial DEM heights. An input DEM still needs to be provided for georeference information.")
    ("crop-win", po::value(&opt.crop_win)->default_value(BBox2i(0, 0, 0, 0), "startx starty stopx stopy"),
       "Crop the input DEM to this region before continuing.")
    ("nodata-value", po::value(&opt.nodata_val)->default_value(std::numeric_limits<double>::quiet_NaN()),
     "Use this as the DEM no-data value, over-riding what is in the initial guess DEM.")
    ("float-dem-at-boundary",   po::bool_switch(&opt.float_dem_at_boundary)->default_value(false)->implicit_value(true),
     "Allow the DEM values at the boundary of the region to also float (not advised).")
    ("fix-dem",   po::bool_switch(&opt.fix_dem)->default_value(false)->implicit_value(true),
     "Do not float the DEM at all. Useful when floating the model params.")
    ("float-reflectance-model",   po::bool_switch(&opt.float_reflectance_model)->default_value(false)->implicit_value(true),
     "Allow the coefficients of the reflectance model to float (not recommended).")
    ("query",   po::bool_switch(&opt.query)->default_value(false)->implicit_value(true),
     "Print some info and exit. Invoked from parallel_sfs.")
    ("save-sparingly",   po::bool_switch(&opt.save_sparingly)->default_value(false)->implicit_value(true),
     "Avoid saving any results except the adjustments and the DEM, as that's a lot of files.")
    ("camera-position-step-size", po::value(&opt.camera_position_step_size)->default_value(1.0),
     "Larger step size will result in more aggressiveness in varying the camera position if it is being floated (which may result in a better solution or in divergence).");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

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
  
  // If the images are tif files, and the cameras are cub files, separate them
  std::vector<std::string> images, cameras;
  for (size_t i = 0; i < opt.input_images.size(); i++) {
    std::string file = opt.input_images[i];
    if (asp::has_cam_extension(file))          // can be .cub
      cameras.push_back(file);
    else if (asp::has_image_extension(file))   // can be .cub
      images.push_back(file);
    else
      vw_throw( ArgumentErr() << "Invalid image or camera file: " << file << ".\n"
		<< usage << general_options );
  }
  if (images.empty())
    images = cameras;
  if (images.size() != cameras.size()) {
    vw_throw( ArgumentErr() << "Expecting as many images as cameras.\n"
	      << usage << general_options );
  }
  opt.input_images = images;
  opt.input_cameras = cameras;

  std::istringstream idem(opt.input_dems_str);
  std::string dem;
  while (idem >> dem) opt.input_dems.push_back(dem);
    
  // Sanity checks
  if (opt.input_dems.empty())
    vw_throw( ArgumentErr() << "Missing input DEM(s).\n"
	      << usage << general_options );
  if (opt.out_prefix.empty())
    vw_throw( ArgumentErr() << "Missing output prefix.\n"
	      << usage << general_options );
  if (opt.max_iterations < 0)
    vw_throw( ArgumentErr() << "The number of iterations must be non-negative.\n"
	      << usage << general_options );
  if (opt.input_images.empty())
  vw_throw( ArgumentErr() << "Missing input images.\n"
	    << usage << general_options );

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Parse shadow thresholds
  std::istringstream ist(opt.shadow_thresholds);
  opt.shadow_threshold_vec.clear();
  float val;
  while (ist >> val)
    opt.shadow_threshold_vec.push_back(val);
  if (!opt.shadow_threshold_vec.empty() &&
      opt.shadow_threshold_vec.size() != opt.input_images.size())
    vw_throw(ArgumentErr()
	     << "If specified, there must be as many shadow thresholds as images.\n");

  // Default thresholds are the smallest float.
  // Maybe it should be 0?
  if (opt.shadow_threshold_vec.empty()) {
    for (size_t i = 0; i < opt.input_images.size(); i++) {
      opt.shadow_threshold_vec.push_back(-std::numeric_limits<float>::max());
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

  if (opt.input_images.size() <=1 && opt.float_exposure)
    vw_throw(ArgumentErr()
	     << "Floating the exposure is ill-posed for just one image.\n");

  if (opt.input_images.size() <=1 && opt.float_dem_at_boundary)
    vw_throw(ArgumentErr()
	     << "Floating the DEM at the boundary is ill-posed for just one image.\n");

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

  if (opt.save_computed_intensity_only){
    if (opt.max_iterations > 0 || opt.max_coarse_iterations > 0){
      vw_out(WarningMessage) << "Using 0 iterations.\n";
      opt.max_iterations = 0;
      opt.max_coarse_iterations = 0;
    }
    if (!opt.model_shadows) {
      vw_out(WarningMessage) << "It is suggested that --model-shadows be used.\n";
    }

    if (opt.coarse_levels > 0) {
      vw_out(WarningMessage) << "Using 0 coarse levels.\n";
      opt.coarse_levels = 0;
    }

    if (opt.use_approx_camera_models || opt.use_rpc_approximation || opt.crop_input_images) {
      vw_out(WarningMessage) << "Not using approximate camera models or cropping input images.\n";
      opt.use_approx_camera_models = false;
      opt.use_rpc_approximation = false;
      opt.crop_input_images = false;
      opt.use_semi_approx = false;
    }
    
    if (opt.image_exposures_vec.empty())
      vw_throw( ArgumentErr()
		<< "Expecting the exposures to be computed and passed in.\n" );
    
  }
  if (opt.crop_input_images && !opt.use_approx_camera_models) {
    vw_throw( ArgumentErr()
              << "Using cropped input images implies using an approximate camera model.\n" );
  }
  
}

// Run sfs at a given coarseness level
void run_sfs_level(// Fixed inputs
		   int num_iterations, Options & opt,
		   std::vector<GeoReference> const& geo,
		   double smoothness_weight,
		   double dem_nodata_val,
		   std::vector< std::vector<BBox2i>     > const& crop_boxes,
		   std::vector< std::vector<MaskedImgT> > const& masked_images,
		   std::vector< std::vector<DoubleImgT> > const& blend_weights,
		   GlobalParams const& global_params,
		   std::vector<ModelParams> const & model_params,
		   std::vector< ImageView<double> > const& orig_dems, 
		   double initial_albedo,
		   // Quantities that will float
		   std::vector< ImageView<double> > & dems,
		   std::vector< ImageView<double> > & albedos,
		   std::vector< std::vector<boost::shared_ptr<CameraModel> > > & cameras,
		   std::vector<double> & exposures,
		   std::vector<double> & adjustments,
                   std::vector<double> & coeffs){

  int num_images = opt.input_images.size();
  int num_dems   = dems.size();
  ceres::Problem problem;
  
  // Find the grid sizes in meters. Note that dem heights are in
  // meters too, so we treat both horizontal and vertical
  // measurements in same units.
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

  // When albedo, dem, model, are fixed, we will not even set these as variables.
  bool fix_most = (!opt.float_albedo && opt.fix_dem && !opt.float_reflectance_model);

  std::set<int> use_dem, use_albedo; // to avoid a crash in Ceres when a param is fixed but not set
  
  for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
    
    // Add a residual block for every grid point not at the boundary
    for (int col = 1; col < dems[dem_iter].cols()-1; col++) {
      for (int row = 1; row < dems[dem_iter].rows()-1; row++) {

        // Intensity error for each image
        for (int image_iter = 0; image_iter < num_images; image_iter++) {

          if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) {
            continue;
          }
        
          ceres::LossFunction* loss_function_img = NULL;
          if (!fix_most) {
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
                                     cameras[dem_iter][image_iter]);
            problem.AddResidualBlock(cost_function_img, loss_function_img,
                                     &exposures[image_iter],      // exposure
                                     &dems[dem_iter](col-1, row),            // left
                                     &dems[dem_iter](col, row),              // center
                                     &dems[dem_iter](col+1, row),            // right
                                     &dems[dem_iter](col, row+1),            // bottom
                                     &dems[dem_iter](col, row-1),            // top
                                     &albedos[dem_iter](col, row),           // albedo
                                     &adjustments[6*image_iter],  // camera
                                     &coeffs[0]);                 // reflectance model coeffs
            use_dem.insert(dem_iter); 
            use_albedo.insert(dem_iter);
          }else{
            ceres::CostFunction* cost_function_img =
              IntensityErrorFixedMost::Create(col, row, dems[dem_iter],
                                              albedos[dem_iter](col, row), // albedo
                                              &coeffs[0],                  // reflectance model coeffs
                                              geo[dem_iter],
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
                                     &exposures[image_iter],      // exposure
                                     &adjustments[6*image_iter]  // camera
                                     );
            
          }
        } // end iterating over images

        if (!fix_most) {
          // Smoothness penalty
          ceres::LossFunction* loss_function_sm = NULL;
          ceres::CostFunction* cost_function_sm =
            SmoothnessError::Create(smoothness_weight, gridx, gridy);
          problem.AddResidualBlock(cost_function_sm, loss_function_sm,
                                   &dems[dem_iter](col-1, row+1), &dems[dem_iter](col, row+1),
                                   &dems[dem_iter](col+1, row+1),
                                   &dems[dem_iter](col-1, row  ), &dems[dem_iter](col, row  ),
                                   &dems[dem_iter](col+1, row  ),
                                   &dems[dem_iter](col-1, row-1), &dems[dem_iter](col, row-1),
                                   &dems[dem_iter](col+1, row-1));
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
    if (!fix_most) {
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
    }
    
    if (opt.initial_dem_constraint_weight <= 0 && num_used <= 1) {    

      if (opt.float_albedo && opt.albedo_constraint_weight <= 0) {
        vw_out() << "No DEM or albedo constraint is used, and there is at most one "
                 << "usable image. Fixing the albedo.\n";
        opt.float_albedo = false;
      }

      if (opt.float_exposure) {
        vw_out() << "No DEM constraint is used, and there is at most one "
                 << "usable image. Fixing the exposure.\n";
        opt.float_exposure = false;
      }
    }
    
    // If to float the albedo
    if (!fix_most) {
      for (int col = 1; col < dems[dem_iter].cols() - 1; col++) {
        for (int row = 1; row < dems[dem_iter].rows() - 1; row++) {
          if (!opt.float_albedo && num_used > 0 && use_albedo.find(dem_iter) != use_albedo.end())
            problem.SetParameterBlockConstant(&albedos[dem_iter](col, row));
        }
      }
    }
  } // end iterating over DEMs

  // If there's just one image, don't float the exposure, as the
  // problem is under-determined. If we float the albedo, we will
  // implicitly float the exposure, hence keep the exposure itself
  // fixed.
  if (!opt.float_exposure){
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      if (use_image[image_iter]) problem.SetParameterBlockConstant(&exposures[image_iter]);
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
  if (!fix_most) {
    if (!opt.float_reflectance_model && num_used > 0) {
      problem.SetParameterBlockConstant(&coeffs[0]);
    }
  }
  
  if (opt.num_threads > 1 && !opt.use_approx_camera_models) {
    vw_out() << "Using exact ISIS camera models. Can run with only a single thread.\n";
    opt.num_threads = 1;
  }
  vw_out() << "Using: " << opt.num_threads << " threads.\n";

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
  g_opt            = &opt;
  g_dem            = &dems;
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
}

int main(int argc, char* argv[]) {
  
  Stopwatch sw_total;
  sw_total.start();
  
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    GlobalParams global_params;
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
      vw_throw( ArgumentErr()
		<< "Expecting Lambertian or Lunar-Lambertian reflectance." );
    global_params.phaseCoeffC1 = 0; //1.383488;
    global_params.phaseCoeffC2 = 0; //0.501149;
    
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
    g_coeffs = &opt.model_coeffs_vec[0];
    
    int num_dems = opt.input_dems.size();

    // Manage no-data
    double dem_nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
    if (vw::read_nodata_val(opt.input_dems[0], dem_nodata_val)){
      vw_out() << "Found DEM nodata value: " << dem_nodata_val << std::endl;
    }
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      double curr_nodata_val = -std::numeric_limits<float>::max(); 
      if (vw::read_nodata_val(opt.input_dems[dem_iter], curr_nodata_val)){
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

    // There are multiple DEM clips, and multiple coarseness levels
    // for each DEM. Same about albedo and georeferences.
    std::vector< std::vector< ImageView<double> > >
      orig_dems(levels+1), dems(levels+1), albedos(levels+1);
    std::vector< std::vector< GeoReference > > geos(levels+1);
    for (int level = 0; level <= levels; level++) {
      orig_dems [level].resize(num_dems);
      dems      [level].resize(num_dems);
      albedos   [level].resize(num_dems);
      geos      [level].resize(num_dems);
    }
    
    // Read the DEMs
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) 
      dems[0][dem_iter] = copy(DiskImageView<double>(opt.input_dems[dem_iter]));
    
    if ( (!opt.crop_win.empty() || opt.query) && num_dems > 1) 
      vw_throw( ArgumentErr() << "Cannot run parallel_stereo with multiple DEM clips.\n" );

    // This must be done before the DEM is cropped. This stats is
    // queried from parallel_sfs.
    if (opt.query) {
      vw_out() << "dem_cols, " << dems[0][0].cols() << std::endl;
      vw_out() << "dem_rows, " << dems[0][0].rows() << std::endl;
      return 0;
    }

    // Adjust the crop win
    opt.crop_win.crop(bounding_box(dems[0][0]));
    
    // Read the georeference. 
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      if (!read_georeference(geos[0][dem_iter], opt.input_dems[dem_iter]))
        vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );
      
      // Crop the DEM and georef if requested to given box
      if (!opt.crop_win.empty()) {
        dems[0][dem_iter] = crop(dems[0][dem_iter], opt.crop_win);
        geos[0][dem_iter] = crop(geos[0][dem_iter], opt.crop_win);
      }
    
      // This can be useful
      vw_out() << "DEM cols and rows: " << dems[0][dem_iter].cols()  << ' '
               << dems[0][dem_iter].rows() << std::endl;

    }
    
    // Replace no-data values with the min valid value. That is because
    // no-data values usually come from shadows, and those are low-lying.
    // This works better than using the mean value.
    // TODO: Maybe do hole-filling instead.
    int min_dem_size = 5;
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      double min_val = std::numeric_limits<double>::max(), mean = 0, num = 0;
      for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
        for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
          if (dems[0][dem_iter](col, row) != dem_nodata_val) {
            mean += dems[0][dem_iter](col, row);
            num += 1;
            min_val = std::min(min_val, dems[0][dem_iter](col, row));
          }
        }
      }
      if (num > 0) mean /= num;
      for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
        for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
          if (dems[0][dem_iter](col, row) == dem_nodata_val) {
            dems[0][dem_iter](col, row) = min_val;
          }
        }
      }

      // See if to use a constant init value
      if (!boost::math::isnan(opt.init_dem_height)) {
        for (int col = 0; col < dems[0][dem_iter].cols(); col++) {
          for (int row = 0; row < dems[0][dem_iter].rows(); row++) {
            dems[0][dem_iter](col, row) = opt.init_dem_height;
          }
        }
      }
  
      if (dems[0][dem_iter].cols() < min_dem_size ||
          dems[0][dem_iter].rows() < min_dem_size) {
        vw_throw( ArgumentErr() << "The input DEM with index "
                  << dem_iter << " is too small.\n" );
      }
    }
  
    // Read in the camera models for the input images.
    int num_images = opt.input_images.size();
    std::vector< std::vector< boost::shared_ptr<CameraModel> > > cameras(num_dems);
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {

      cameras[dem_iter].resize(num_images);
      for (int image_iter = 0; image_iter < num_images; image_iter++){
	
	if (opt.skip_images[dem_iter].find(image_iter)
	    != opt.skip_images[dem_iter].end()) continue;
	
	typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
	SessionPtr session(asp::StereoSessionFactory::create
			   (opt.stereo_session_string, opt,
			    opt.input_images[image_iter],
			    opt.input_images[image_iter],
			    opt.input_cameras[image_iter],
			    opt.input_cameras[image_iter],
			    opt.out_prefix));
	
	vw_out() << "Loading image and camera: " << opt.input_images[image_iter] << " "
		 <<  opt.input_cameras[image_iter] << " for DEM clip " << dem_iter << ".\n";
	cameras[dem_iter][image_iter] = session->camera_model(opt.input_images[image_iter],
							      opt.input_cameras[image_iter]);
      }
    }
    
    // Since we may float the cameras, ensure our camera models are
    // always adjustable.
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) continue;
        CameraModel * icam
          = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
        if (icam == NULL) {
          // Set a default identity adjustment
          Vector2 pixel_offset;
          Vector3 translation;
          Quaternion<double> rotation = Quat(math::identity_matrix<3>());
          cameras[dem_iter][image_iter] = boost::shared_ptr<CameraModel>
            (new AdjustedCameraModel(cameras[dem_iter][image_iter], translation,
                                     rotation, pixel_offset));
        }
      }
    }
    
    // Prepare for working at multiple levels
    int factor = 2;
    std::vector<int> factors;
    factors.push_back(1);
    for (int level = 1; level <= levels; level++) {
      factors.push_back(factors[level-1]*factor);
    }
    
    // We won't load the full images, just portions restricted
    // to the area we we will compute the DEM.
    std::vector<std::vector< std::vector<BBox2i> > > crop_boxes(levels+1);
    for (int level = 0; level <= levels; level++) {
      crop_boxes[level].resize(num_dems);
    }
    
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

    // If to use approximate camera models
    if (opt.use_approx_camera_models) {

      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
        double max_approx_err = 0.0;
      
        for (int image_iter = 0; image_iter < num_images; image_iter++){
        
          if (opt.skip_images[dem_iter].find(image_iter)
              != opt.skip_images[dem_iter].end()) continue;
      
          // Here we make a copy, since soon cameras[dem_iter][image_iter] will be overwritten
          AdjustedCameraModel adj_cam
            = *dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());

          boost::shared_ptr<CameraModel> exact_cam = adj_cam.unadjusted_model();
          if (dynamic_cast<IsisCameraModel*>(exact_cam.get()) == NULL)
            vw_throw( ArgumentErr() << "Expecting an ISIS camera model.\n" );

          vw_out() << "Creating an approximate camera model for "
                   << opt.input_cameras[image_iter] << " and clip "
                   << opt.input_dems[dem_iter] <<".\n";
          BBox2i img_bbox = crop_boxes[0][dem_iter][image_iter];
          Stopwatch sw;
          sw.start();
          boost::shared_ptr<CameraModel> apcam
            (new ApproxCameraModel(adj_cam, exact_cam, img_bbox, dems[0][dem_iter], geos[0][dem_iter],
                                   dem_nodata_val, opt.use_rpc_approximation, opt.use_semi_approx,
                                   opt.rpc_penalty_weight, camera_mutex));
          sw.stop();
          vw_out() << "Approximate model generation time: " << sw.elapsed_seconds()
                   << " s." << std::endl;

          // Copy the adjustments over to the approximate camera model
          Vector3 translation  = adj_cam.translation();
          Quat rotation        = adj_cam.rotation();
          Vector2 pixel_offset = adj_cam.pixel_offset();
          double scale         = adj_cam.scale();

          cameras[dem_iter][image_iter] = boost::shared_ptr<CameraModel>
            (new AdjustedCameraModel(apcam, translation,
                                     rotation, pixel_offset, scale));

          // Cast the pointer back to AdjustedCameraModel as we need that.
          ApproxCameraModel* cam_ptr = dynamic_cast<ApproxCameraModel*>(apcam.get());
          if (cam_ptr == NULL) 
            vw_throw( ArgumentErr() << "Expecting an ApproxCameraModel." );

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

                // Test how unadjusted models compare
                Vector2 pix1 = exact_cam->point_to_pixel(xyz);
                //if (!img_bbox.contains(pix1)) continue;

                Vector2 pix2 = apcam->point_to_pixel(xyz);
                max_curr_err = std::max(max_curr_err, norm_2(pix1 - pix2));
                //std::cout << "orig and approx1 " << pix1 << ' ' << pix2 << ' '
                //          << norm_2(pix1-pix2) << std::endl;

                // Test how adjusted models compare
                Vector2 pix3 = adj_cam.point_to_pixel(xyz);
                //if (!img_bbox.contains(pix3)) continue;
                Vector2 pix4 = cameras[dem_iter][image_iter]->point_to_pixel(xyz);
                max_curr_err = std::max(max_curr_err, norm_2(pix3 - pix4));
                //std::cout << "orig and approx2 " << pix3 << ' ' << pix4 << ' '
                //          << norm_2(pix3-pix4) << std::endl;

                // Use these pixels to expand the crop box, as we now also know the adjustments.
                // This is a bug fix.
                cam_ptr->crop_box().grow(pix1);
                cam_ptr->crop_box().grow(pix2);
                cam_ptr->crop_box().grow(pix3);
                cam_ptr->crop_box().grow(pix4);
              }
            }

            vw_out() << "Max approximate model error in pixels for: "
                     <<  opt.input_cameras[image_iter] << " and clip "
                     << opt.input_dems[dem_iter] << ": " << max_curr_err << std::endl;
          }else{
            vw_out() << "Invalid model for clip: " << dem_iter << ".\n";
          }
          
          if (max_curr_err > 2.0 || !model_is_valid) {
            // This is a bugfix. When the DEM clip does not intersect the image,
            // the approx camera model has incorrect values.
            if (model_is_valid)
              vw_out() << "Error is too big.\n";
	    vw_out() << "Skip image " << image_iter << " for clip " << dem_iter << std::endl;
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
        vw_out() << "Max total approximate model error in pixels: " << max_approx_err << std::endl;
        
      } // end iterating over dem clips
    } // end computing the approximate camera model
    
    for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
      // Make the crop boxes lower left corner be multiple of 2^level
      int last_factor = factors.back();
      for (int image_iter = 0; image_iter < num_images; image_iter++){
        if (!crop_boxes[0][dem_iter][image_iter].empty()) {
          Vector2i mn = crop_boxes[0][dem_iter][image_iter].min();
          crop_boxes[0][dem_iter][image_iter].min() = last_factor*(floor(mn/double(last_factor)));
        }
      }
      
      // Crop boxes at the coarser resolutions
      for (int image_iter = 0; image_iter < num_images; image_iter++){
	for (int level = 1; level <= levels; level++) {
	  crop_boxes[level][dem_iter].push_back(crop_boxes[0][dem_iter][image_iter]/factors[level]);
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
      
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) continue;
       
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
					 opt.max_valid_image_vals_vec[image_iter]
					 );
            
            // Compute blending weights only when using an approx camera model and
            // cropping the images. Otherwise the weights are too huge.
            if (opt.use_blending_weights)
              blend_weights_vec[0][dem_iter][image_iter]
                = comp_blending_weights(masked_images_vec[0][dem_iter][image_iter],
                                        opt.blending_dist, opt.blending_power);
          }
        }else{
          masked_images_vec[0][dem_iter][image_iter]
            = create_pixel_range_mask2(DiskImageView<float>(img_file),
				       std::max(img_nodata_val, shadow_thresh),
				       opt.max_valid_image_vals_vec[image_iter]
				       );
        }
      }
    }
    g_img_nodata_val = &img_nodata_val;

    // Get the sun and camera positions from the ISIS cube
    std::vector<ModelParams> model_params;
    model_params.resize(num_images);
    for (int image_iter = 0; image_iter < num_images; image_iter++){
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
	if (opt.skip_images[dem_iter].find(image_iter) !=
	    opt.skip_images[dem_iter].end()) continue;
	IsisCameraModel* icam
	  = dynamic_cast<IsisCameraModel*>(get_isis_cam(cameras[dem_iter][image_iter]).get());
	model_params[image_iter].sunPosition = icam->sun_position();
      }
      
      vw_out() << "Sun position for image: " << image_iter << " "
	       << model_params[image_iter].sunPosition << std::endl;
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
    
    // We have intensity = reflectance*exposure*albedo. Find the exposure as
    // mean(intensity)/mean(reflectance)/albedo. Use this to compute an
    // initial exposure and decide based on that which images to
    // skip. If the user provided initial exposures, use those, but
    // still go through the motions to find the images to skip.
    if (!opt.save_computed_intensity_only) {
      
      std::vector<double> local_exposures_vec(num_images, 0);
      for (int image_iter = 0; image_iter < num_images; image_iter++) {

	std::vector<double> exposures_per_dem;
	for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
      
	  if (opt.skip_images[dem_iter].find(image_iter) !=
	      opt.skip_images[dem_iter].end()) continue;
      
	  ImageView< PixelMask<double> > reflectance, intensity;
	  ImageView<double> weight;
	  computeReflectanceAndIntensity(dems[0][dem_iter], geos[0][dem_iter],
					 opt.model_shadows, max_dem_height[dem_iter],
					 gridx, gridy,
					 model_params[image_iter],
					 global_params,
					 crop_boxes[0][dem_iter][image_iter],
					 masked_images_vec[0][dem_iter][image_iter],
					 blend_weights_vec[0][dem_iter][image_iter],
					 cameras[dem_iter][image_iter].get(),
					 reflectance, intensity, weight,
					 &opt.model_coeffs_vec[0]);

          // TODO: Below is not the optimal way of finding the exposure!
          // Find it as the analytical minimum using calculus.
	  double imgmean, imgstdev, refmean, refstdev;
	  compute_image_stats(intensity, imgmean, imgstdev);
	  compute_image_stats(reflectance, refmean, refstdev);
	  double exposure = imgmean/refmean/initial_albedo;
	  vw_out() << "img mean std: " << imgmean << ' ' << imgstdev << std::endl;
	  vw_out() << "ref mean std: " << refmean << ' ' << refstdev << std::endl;
	  vw_out() << "Local exposure for image " << image_iter << " and clip "
		   << dem_iter << ": " << exposure << std::endl;
	
	  double big = 1e+10; // There's no way image exposure can be bigger than this
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
	  //	 << local_exposures_vec[image_iter] << std::endl;
	}
      }
    
      if (opt.image_exposures_vec.empty()) opt.image_exposures_vec = local_exposures_vec;
    }
    
    for (size_t image_iter = 0; image_iter < opt.image_exposures_vec.size(); image_iter++) {
      vw_out() << "Image exposure for " << opt.input_images[image_iter] << ' '
               << opt.image_exposures_vec[image_iter] << std::endl;
    }
    g_exposures = &opt.image_exposures_vec;
    
    // For images that we don't use, wipe the cameras and all other
    // info, as those take up memory (the camera is a table). 
    for (int image_iter = 0; image_iter < num_images; image_iter++) {
      for (int dem_iter = 0; dem_iter < num_dems; dem_iter++) {
        if (opt.skip_images[dem_iter].find(image_iter) != opt.skip_images[dem_iter].end()) {
          masked_images_vec[0][dem_iter][image_iter] = ImageView< PixelMask<float> >();
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
      
        AdjustedCameraModel * icam
          = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
        if (icam == NULL)
          vw_throw(ArgumentErr() << "Expecting adjusted camera models.\n");
        Vector3 translation = icam->translation();
        Vector3 axis_angle = icam->rotation().axis_angle();
        Vector2 pixel_offset = icam->pixel_offset();
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

        albedos[level][dem_iter] = pixel_cast<double>(vw::resample_aa
                                                      (pixel_cast< PixelMask<double> >
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
	  
          AdjustedCameraModel * adj_cam
            = dynamic_cast<AdjustedCameraModel*>(cameras[dem_iter][image_iter].get());
          if (adj_cam == NULL)
            vw_throw( ArgumentErr() << "Expecting adjusted camera.\n");
          adj_cam->set_scale(factors[level]);
        }
      }
      
      run_sfs_level(// Fixed inputs
                    num_iterations, opt, geos[level],
                    opt.smoothness_weight*factors[level]*factors[level],
                    dem_nodata_val, crop_boxes[level],
                    masked_images_vec[level], blend_weights_vec[level],
                    global_params, model_params,
                    orig_dems[level], initial_albedo,
                    // Quantities that will float
                    dems[level], albedos[level], cameras,
                    opt.image_exposures_vec,
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

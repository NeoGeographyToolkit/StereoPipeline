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

#include <vw/Camera/CameraModel.h>
#include <vw/Stereo/StereoView.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/InterestPoint/InterestData.h>

#include <asp/Camera/RPCModel.h>
#include <asp/Core/DisparityProcessing.h>
#include <asp/Core/Bathymetry.h>
#include <asp/Tools/stereo.h>
#include <asp/Tools/jitter_adjust.h>
#include <asp/Tools/ccd_adjust.h>

// We must have the implementations of all sessions for triangulation
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Sessions/StereoSessionMapProj.h>
#include <asp/Sessions/StereoSessionIsis.h>
#include <asp/Sessions/StereoSessionNadirPinhole.h>
#include <asp/Sessions/StereoSessionPinhole.h>
#include <asp/Sessions/StereoSessionRPC.h>
#include <asp/Sessions/StereoSessionASTER.h>
#include <xercesc/util/PlatformUtils.hpp>
#include <ctime>

using namespace vw;
using namespace asp;
using namespace std;

typedef typename StereoSession::tx_type TXT;

namespace asp{

enum OUTPUT_CLOUD_TYPE {FULL_CLOUD, BATHY_CLOUD, TOPO_CLOUD};

/// The main class for taking in a set of disparities and returning a
/// point cloud via joint triangulation.
template <class DisparityImageT, class StereoModelT>
class StereoTXAndErrorView:
  public ImageViewBase<StereoTXAndErrorView<DisparityImageT, StereoModelT> > {
  vector<DisparityImageT> m_disparity_maps;
  vector<TXT>  m_transforms; // e.g., map-projection or homography to undo
  StereoModelT m_stereo_model;
  bool         m_is_map_projected;
  bool         m_bathy_correct;
  OUTPUT_CLOUD_TYPE m_cloud_type;
  ImageViewRef< PixelMask<float> > m_left_aligned_bathy_mask;
  ImageViewRef< PixelMask<float> > m_right_aligned_bathy_mask;

  typedef typename DisparityImageT::pixel_type DPixelT;

public:

  typedef Vector6 pixel_type;
  typedef Vector6 result_type;
  typedef ProceduralPixelAccessor<StereoTXAndErrorView> pixel_accessor;

  /// Constructor
  StereoTXAndErrorView(vector<DisparityImageT> const& disparity_maps,
                       vector<TXT>             const& transforms,
                       StereoModelT            const& stereo_model,
                       bool is_map_projected,
                       bool bathy_correct, OUTPUT_CLOUD_TYPE cloud_type,
                       ImageViewRef< PixelMask<float> > left_aligned_bathy_mask,
                       ImageViewRef< PixelMask<float> > right_aligned_bathy_mask):
    m_disparity_maps(disparity_maps),
    m_transforms(transforms),
    m_stereo_model(stereo_model),
    m_is_map_projected(is_map_projected),
    m_bathy_correct(bathy_correct),
    m_cloud_type(cloud_type),
    m_left_aligned_bathy_mask(left_aligned_bathy_mask),
    m_right_aligned_bathy_mask(right_aligned_bathy_mask) {

    // Sanity check
    for (int p = 1; p < (int)m_disparity_maps.size(); p++){
      if (m_disparity_maps[0].cols() != m_disparity_maps[p].cols() ||
          m_disparity_maps[0].rows() != m_disparity_maps[p].rows()   )
        vw_throw( ArgumentErr() << "In multi-view triangulation, all disparities "
                  << "must have the same dimensions.\n" );
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
    pixVec[0] = m_transforms[0]->reverse(Vector2(i,j)); // De-warp "left" pixel
    for (int c = 0; c < num_disp; c++){
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
    // TODO(oalexan1): Wipe any mention of bathy from the base stereo model,
    // and here employ a regular model which knows nothing about bathy
    // and a bathy stereo model to be called as needed.
    Vector3 errorVec;
    pixel_type result;
    bool do_bathy = false;
    bool did_bathy = false;
    if (!m_bathy_correct) {
      try {
        subvector(result,0,3) = m_stereo_model(pixVec, errorVec, do_bathy, did_bathy);
        subvector(result,3,3) = errorVec;
      }catch(...) {
        return result;
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

    // See if both the left and right matching pixels are in the aligned
    // bathymetry masks which means bathymetry correction should happen.
    Vector2 rpix = lpix + stereo::DispHelper(disp);
    Vector2 irpix(round(rpix.x()), round(rpix.y())); // integer version

    // Do bathy only when the mask is invalid (under water)
    do_bathy = (!is_valid(m_left_aligned_bathy_mask(lpix.x(), lpix.y())) &&
                0 <= irpix.x() && irpix.x() < m_right_aligned_bathy_mask.cols() &&
                0 <= irpix.y() && irpix.y() < m_right_aligned_bathy_mask.rows() &&
                !is_valid(m_right_aligned_bathy_mask(irpix.x(), irpix.y())));

    // See if we actually need to do anything
    if ((m_cloud_type == BATHY_CLOUD && !do_bathy) ||
        (m_cloud_type == TOPO_CLOUD && do_bathy)) {
      // There is no point in continuing, as we won't get what is asked
      subvector(result, 0, 3) = Vector3(0, 0, 0);
      subvector(result, 3, 3) = Vector3(0, 0, 0);
      return result;
    }

    // Do the triangulation. The did_bathy variable may change. 
    did_bathy = false;
    subvector(result, 0, 3) = m_stereo_model(pixVec, errorVec, do_bathy, did_bathy);
    subvector(result, 3, 3) = errorVec;

    // If we wanted to do bathy correction and it did not happen, or the opposite,
    // don't return the computed answer
    if ((m_cloud_type == BATHY_CLOUD && !did_bathy) ||
        (m_cloud_type == TOPO_CLOUD && did_bathy)) {
      subvector(result, 0, 3) = Vector3(0, 0, 0);
      subvector(result, 3, 3) = Vector3(0, 0, 0);
      return result;
    }
    
    return result; // Contains location and error vector
  }
  
  typedef StereoTXAndErrorView<ImageViewRef<DPixelT>, StereoModelT> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const {
    return PreRasterHelper( bbox, m_transforms );
  }
  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
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
  prerasterize_type PreRasterHelper( BBox2i const& bbox, vector<T> const& transforms) const {

    ImageViewRef< PixelMask<float> > in_memory_left_aligned_bathy_mask;
    ImageViewRef< PixelMask<float> > in_memory_right_aligned_bathy_mask;
    
    // Code for NON-MAP-PROJECTED session types.
    if (m_is_map_projected == false) {
      // We explicitly bring in-memory the disparities for the current box
      // to speed up processing later, and then we pretend this is the entire
      // image by virtually enlarging it using a CropView.

      vector<ImageViewRef<DPixelT>> disparity_cropviews;
      for (int p = 0; p < (int)m_disparity_maps.size(); p++){
        ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ));
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

      return prerasterize_type(disparity_cropviews, transforms, m_stereo_model,
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
    // - Without some sort of duplication function in the transform base class we need
    //   to manually copy the Map2CamTrans type which is pretty hacky.
    vector<T> transforms_copy(transforms.size());
    for (size_t i = 0; i < transforms.size(); ++i) {
      transforms_copy[i] = make_transform_copy(transforms[i]);
    }
    // As a side effect this call makes transforms_copy create a local cache we want later
    transforms_copy[0]->reverse_bbox(bbox); 

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

      ImageView<DPixelT> clip( crop( m_disparity_maps[p], bbox ));
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

    return prerasterize_type(disparity_cropviews, transforms_copy, m_stereo_model,
                             m_is_map_projected, m_bathy_correct, m_cloud_type,
                             in_memory_left_aligned_bathy_mask, in_memory_right_aligned_bathy_mask);
  } // End function PreRasterHelper() DGMapRPC version

}; // End class StereoTXAndErrorView

/// Just a wrapper function for StereoTXAndErrorView view construction
template <class DisparityT, class StereoModelT>
StereoTXAndErrorView<DisparityT, StereoModelT>
stereo_error_triangulate(vector<DisparityT> const& disparities,
                         vector<TXT>        const& transforms,
                         StereoModelT       const& model,
                         bool is_map_projected,
                         bool bathy_correct,
                         OUTPUT_CLOUD_TYPE cloud_type,
                         ImageViewRef< PixelMask<float> > left_aligned_bathy_mask,
                         ImageViewRef< PixelMask<float> > right_aligned_bathy_mask) {
  
  typedef StereoTXAndErrorView<DisparityT, StereoModelT> result_type;
  return result_type(disparities, transforms, model, is_map_projected,
                     bathy_correct, cloud_type, left_aligned_bathy_mask, right_aligned_bathy_mask);
}

// TODO(oalexan1): Move this out of here
/// Bin the disparities, and from each bin get a disparity value.
/// This will create a correspondence from the left to right image,
/// which we save in the match format.
/// When gen_triplets is true, and there are many overlapping images,
/// try hard to have many IP with the property that each such IP is seen
/// in more than two images. This helps with bundle adjustment.
template <class DisparityT>
void compute_matches_from_disp(vector<ASPGlobalOptions> const& opt_vec,
                               vector<DisparityT> const& disparities,
                               vector<TXT>        const& transforms,
                               std::string        const& match_file,
                               int                const  max_num_matches,
                               bool                      gen_triplets
                               ) {

  VW_ASSERT( disparities.size() == 1 && transforms.size() == 2,
               vw::ArgumentErr() << "Expecting two images and one disparity.\n" );
  DisparityT const& disp = disparities[0]; // pull the disparity

  // Transforms to compensate for alignment
  TXT left_trans  = transforms[0];
  TXT right_trans = transforms[1];

  bool usePinholeEpipolar = ( (stereo_settings().alignment_method == "epipolar") &&
                              ( opt_vec[0].session->name() == "pinhole" ||
                                opt_vec[0].session->name() == "nadirpinhole") );
  if (usePinholeEpipolar) {
    StereoSessionPinhole* pinPtr = dynamic_cast<StereoSessionPinhole*>(opt_vec[0].session.get());
    if (pinPtr == NULL) 
      vw_throw(ArgumentErr() << "Expected a pinhole camera.\n");
    pinPtr->pinhole_cam_trans(left_trans, right_trans);
  }

  std::vector<vw::ip::InterestPoint> left_ip, right_ip;

  if (!gen_triplets) {

    double num_pixels = double(disp.cols()) * double(disp.rows());
    double bin_len = sqrt(num_pixels/std::min(double(max_num_matches), num_pixels));
    VW_ASSERT( bin_len >= 1.0, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

    int lenx = round( disp.cols()/bin_len ); lenx = std::max(1, lenx);
    int leny = round( disp.rows()/bin_len ); leny = std::max(1, leny);

    // Iterate over bins.

    vw_out() << "Computing interest point matches based on disparity.\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(lenx);
    tpc.report_progress(0);

    for (int binx = 0; binx < lenx; binx++) {

      // Pick the disparity at the center of the bin
      int posx = round( (binx+0.5)*bin_len );

      for (int biny = 0; biny < leny; biny++) {

        int posy = round( (biny+0.5)*bin_len );

        if (posx >= disp.cols() || posy >= disp.rows()) 
          continue;
        typedef typename DisparityT::pixel_type DispPixelT;
        DispPixelT dpix = disp(posx, posy);
        if (!is_valid(dpix))
          continue;

        // De-warp left and right pixels to be in the camera coordinate system
        Vector2 left_pix  = left_trans->reverse ( Vector2(posx, posy) );
        Vector2 right_pix = right_trans->reverse( Vector2(posx, posy) + stereo::DispHelper(dpix) );

        left_ip.push_back(ip::InterestPoint(left_pix.x(), left_pix.y()));
        right_ip.push_back(ip::InterestPoint(right_pix.x(), right_pix.y()));
      }

      tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();

  } else{

    // First create ip with left_ip being at integer multiple of bin size.
    // Then do the same for right_ip. This way there is a symmetry
    // and predictable location for ip. So if three images overlap,
    // a feature can often be seen in many of them whether a given
    // image is left in some pairs or right in some others.

    // Note that the code above is modified in subtle ways.

    // Need these to not insert an ip twice, as then bundle_adjust
    // will wipe both copies
    std::map<double, double> left_done, right_done;
    
    // Start with the left
    {
      DiskImageView<float> left_img(opt_vec[0].in_file1);
    
      double num_pixels = double(left_img.cols()) * double(left_img.rows());
      int bin_len = round(sqrt(num_pixels/std::min(double(max_num_matches), num_pixels)));
      VW_ASSERT( bin_len >= 1, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

      int lenx = round( left_img.cols()/bin_len ); lenx = std::max(1, lenx);
      int leny = round( left_img.rows()/bin_len ); leny = std::max(1, leny);

      // Iterate over bins.

      vw_out() << "Computing interest point matches based on disparity.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(lenx);
      tpc.report_progress(0);

      for (int binx = 0; binx <= lenx; binx++) {

        int posx = binx*bin_len; // integer multiple of bin length

        for (int biny = 0; biny <= leny; biny++) {

          int posy = biny*bin_len; // integer multiple of bin length

          if (posx >= left_img.cols() || posy >= left_img.rows()) 
            continue;

          Vector2 left_pix(posx, posy);
          Vector2 trans_left_pix, trans_right_pix, right_pix;
        
          typedef typename DisparityT::pixel_type DispPixelT;

          // Make the left pixel go to the disparity domain. Find the corresponding
          // right pixel. And make that one go to the right image domain.
          trans_left_pix = round(left_trans->forward(left_pix));
          if (trans_left_pix[0] < 0 || trans_left_pix[0] >= disp.cols()) continue;
          if (trans_left_pix[1] < 0 || trans_left_pix[1] >= disp.rows()) continue;
          DispPixelT dpix = disp(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix = right_trans->reverse(trans_right_pix);

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress( inc_amount );
      }
      tpc.report_finished();
    }
    
    // Now create ip in predictable location for the right image.This is hard,
    // as the disparity goes from left to right, so we need to examine every disparity.
    typedef typename DisparityT::pixel_type DispPixelT;
    ImageView<DispPixelT> disp_copy = copy(disp);
    {
      DiskImageView<float> right_img(opt_vec[0].in_file2);
    
      double num_pixels = double(right_img.cols()) * double(right_img.rows());
      int bin_len = round(sqrt(num_pixels/std::min(double(max_num_matches), num_pixels)));
      VW_ASSERT( bin_len >= 1, vw::ArgumentErr() << "Expecting bin_len >= 1.\n" );

      // Iterate over disparity.

      vw_out() << "Doing a second pass. This will be very slow.\n";
      vw::TerminalProgressCallback tpc("asp", "\t--> ");
      double inc_amount = 1.0 / double(disp_copy.cols());
      tpc.report_progress(0);

      for (int col = 0; col < disp_copy.cols(); col++) {
        for (int row = 0; row < disp_copy.rows(); row++) {

          Vector2 trans_left_pix(col, row);
          Vector2 left_pix, trans_right_pix, right_pix;

          DispPixelT dpix = disp_copy(trans_left_pix[0], trans_left_pix[1]);
          if (!is_valid(dpix))
            continue;

          // Compute the left and right pixels. 
          left_pix        = left_trans->reverse(trans_left_pix);
          trans_right_pix = trans_left_pix + stereo::DispHelper(dpix);
          right_pix       = right_trans->reverse(trans_right_pix);

          // If the right pixel is a multiple of the bin size, keep
          // it.
          right_pix = round(right_pix); // very important
          if ( int(right_pix[0]) % bin_len != 0 ) continue;
          if ( int(right_pix[1]) % bin_len != 0 ) continue;

          // Add this ip unless found already. This is clumsy, but we
          // can't use a set since there is no ordering for pairs.
          std::map<double, double>::iterator it;
          it = left_done.find(left_pix.x());
          if (it != left_done.end() && it->second == left_pix.y()) continue; 
          it = right_done.find(right_pix.x());
          if (it != right_done.end() && it->second == right_pix.y()) continue; 
          left_done[left_pix.x()] = left_pix.y();
          right_done[right_pix.x()] = right_pix.y();
          ip::InterestPoint lip(left_pix.x(), left_pix.y());
          ip::InterestPoint rip(right_pix.x(), right_pix.y());
          left_ip.push_back(lip); 
          right_ip.push_back(rip);
        }
        
        tpc.report_incremental_progress( inc_amount );
      }
      tpc.report_finished();
    }
    
  } // end considering multi-image friendly ip

  vw_out() << "Determined " << left_ip.size()
           << " interest point matches from disparity.\n";

  vw_out() << "Writing: " << match_file << std::endl;
  ip::write_binary_match_file(match_file, left_ip, right_ip);
}


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
    return UnaryPerPixelView<ImageT, PointAndErrorNorm>(image.impl(), PointAndErrorNorm());
  }

  template <class ImageT>
  void save_point_cloud(Vector3 const& shift, ImageT const& point_cloud,
                        string const& point_cloud_file,
                        ASPGlobalOptions const& opt) {

    vw_out() << "Writing point cloud: " << point_cloud_file << "\n";
    bool has_georef = true;
    cartography::GeoReference georef = opt.session->get_georef();

    bool has_nodata = false;
    double nodata = -std::numeric_limits<float>::max(); // smallest float

    if (opt.session->supports_multi_threading()){
      asp::block_write_approx_gdal_image
        (point_cloud_file, shift,
         stereo_settings().point_cloud_rounding_error,
         point_cloud,
         has_georef, georef, has_nodata, nodata,
         opt, TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }else{
      // ISIS does not support multi-threading
      asp::write_approx_gdal_image
        (point_cloud_file, shift,
         stereo_settings().point_cloud_rounding_error,
         point_cloud,
         has_georef, georef, has_nodata, nodata,
         opt, TerminalProgressCallback("asp", "\t--> Triangulating: "));
    }
  }

  Vector3 find_approx_points_median(vector<Vector3> const& points){

    // Find the median of the x coordinates of points, then of y, then of
    // z. Perturb the median a bit to ensure it is never exactly on top
    // of a real point, as in such a case after subtraction of that
    // point from median we'd get the zero vector which by convention
    // is invalid.

    if (points.empty())
      return Vector3();

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
               y != numy/2-r && y != numy/2+r)
            continue; // skip inner points

          if (x < 0 || y < 0 || x >= numx || y >= numy)
            continue; // out of bounds

          BBox2i box(x*tile_size[0], y*tile_size[1], tile_size[0], tile_size[1]);
          box.crop(bounding_box(point_cloud));

          // Crop to the cloud area actually having points
          box.crop(stereo_settings().trans_crop_win);

          // Triangulate in the existing box
          ImageView<Vector6> cropped_cloud = crop(point_cloud, box);
          for (int px = 0; px < cropped_cloud.cols(); px++){
            for (int py = 0; py < cropped_cloud.rows(); py++){
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

/// Main triangulation function
void stereo_triangulation(string const& output_prefix,
                          vector<ASPGlobalOptions> const& opt_vec ) {
    
    typedef StereoSession                       SessionT;
    
  try { // Outer try/catch

    const bool is_map_projected = opt_vec[0].session->isMapProjected();

    // Collect the images, cameras, and transforms. The left image is
    // the same in all n-1 stereo pairs forming the n images multiview
    // system. Same for cameras and transforms.
    vector<string> image_files, camera_files;
    vector< boost::shared_ptr<camera::CameraModel> > cameras;
    vector<typename SessionT::tx_type> transforms;
    for (int p = 0; p < (int)opt_vec.size(); p++){

      boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
      opt_vec[p].session->camera_models(camera_model1, camera_model2);

      boost::shared_ptr<SessionT> sPtr = opt_vec[p].session;

      if (p == 0){ // The first image is the "left" image for all pairs.
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

    // If the distance from the left camera center to a point is
    // greater than the universe radius, we remove that pixel and
    // replace it with a zero vector, which is the missing pixel value in the point_image.
    //
    // We apply the universe radius here and then write the result directly to a file on disk.
    stereo::UniverseRadiusFunc universe_radius_func(Vector3(),0,0);
    try{
      if ( stereo_settings().universe_center == "camera" ) {
        if (opt_vec[0].session->name() == "rpc") {
          vw_throw(InputErr() << "Stereo with RPC cameras cannot "
                              << "have the camera as the universe center.\n");
        }

        universe_radius_func = stereo::UniverseRadiusFunc(cameras[0]->camera_center(Vector2()),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      } else if ( stereo_settings().universe_center == "zero" ) {
        universe_radius_func = stereo::UniverseRadiusFunc(Vector3(),
                                                          stereo_settings().near_universe_radius,
                                                          stereo_settings().far_universe_radius);
      }
    } catch (std::exception &e) {
      vw_out() << e.what() << std::endl;
      vw_out(WarningMessage) << "Could not find the camera center. "
                             << "Will not be able to filter triangulated points by radius.\n";
    } // End try/catch

    std::vector<DispImageType> disparity_maps;
    for (int p = 0; p < (int)opt_vec.size(); p++)
      disparity_maps.push_back
        (opt_vec[p].session->pre_pointcloud_hook(opt_vec[p].out_prefix+"-F.tif"));

    // Create a disparity map with between the original unaligned images 
    if (stereo_settings().unalign_disparity) {
      VW_ASSERT( disparity_maps.size() == 1 && transforms.size() == 2,
                 vw::ArgumentErr() << "Expecting two images and one disparity.\n" );
      
      std::string unaligned_disp_file = asp::unwarped_disp_file(output_prefix,
                                                                opt_vec[0].in_file1, 
                                                                opt_vec[0].in_file2);
      ASPGlobalOptions opt = opt_vec[0];

      // Transforms to compensate for alignment
      TransPtr left_trans  = transforms[0];
      TransPtr right_trans = transforms[1];
      // Special case to overwrite the transforms
      bool is_map_projected = opt.session->isMapProjected();
      bool usePinholeEpipolar = ( (stereo_settings().alignment_method == "epipolar") &&
                                  ( opt.session->name() == "pinhole" ||
                                    opt.session->name() == "nadirpinhole") );
      if (usePinholeEpipolar) {
        StereoSessionPinhole* pinPtr = dynamic_cast<StereoSessionPinhole*>(opt.session.get());
        if (pinPtr == NULL) 
          vw_throw(ArgumentErr() << "Expected a pinhole camera.\n");
        pinPtr->pinhole_cam_trans(left_trans, right_trans);
      }

      unalign_disparity(is_map_projected, disparity_maps[0], left_trans, right_trans,  
                        opt,  unaligned_disp_file);
    }
    
    std::string match_file = ip::match_filename(output_prefix + "-disp",
                                                opt_vec[0].in_file1, 
                                                opt_vec[0].in_file2);

    // Pull matches from disparity. Highly experimental.
    if (stereo_settings().num_matches_from_disparity > 0 && 
        stereo_settings().num_matches_from_disp_triplets > 0) {
      vw_throw( ArgumentErr() << "Cannot have both --num-matches-from-disparity and  "
                              << "--num-matches-from-disp-triplets.\n" );
    }

    if (stereo_settings().num_matches_from_disparity > 0) {
      bool gen_triplets = false;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                stereo_settings().num_matches_from_disparity, gen_triplets);
    }
    if (stereo_settings().num_matches_from_disp_triplets > 0) {
      bool gen_triplets = true;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                stereo_settings().num_matches_from_disp_triplets, gen_triplets);
    }
    
    // Piecewise adjustments for jitter
    if (stereo_settings().image_lines_per_piecewise_adjustment > 0 &&
        !stereo_settings().skip_computing_piecewise_adjustments){

      // TODO: This must be proportional to how many adjustments have!
      double max_num_matches = stereo_settings().num_matches_for_piecewise_adjustment;

      bool gen_triplets = false;
      compute_matches_from_disp(opt_vec, disparity_maps, transforms, match_file,
                                max_num_matches, gen_triplets);

      int num_threads = opt_vec[0].num_threads;
      if (opt_vec[0].session->name() == "isis" || opt_vec[0].session->name() == "isismapisis")
        num_threads = 1;
      asp::jitter_adjust(image_files, camera_files, cameras,
                         output_prefix, opt_vec[0].session->name(),
                         match_file,  num_threads);
      //asp::ccd_adjust(image_files, camera_files, cameras, output_prefix,
      //                match_file,  num_threads);
    }

    if (stereo_settings().compute_piecewise_adjustments_only) {
      vw_out() << "Computed the piecewise adjustments. Will stop here." << endl;
      return;
    }

    // Reload the cameras, loading the piecewise corrections for jitter.
    if (stereo_settings().image_lines_per_piecewise_adjustment > 0) {

      stereo_settings().bundle_adjust_prefix = output_prefix; // trigger loading adj cams
      cameras.clear();
      for (int p = 0; p < (int)opt_vec.size(); p++){

        boost::shared_ptr<camera::CameraModel> camera_model1, camera_model2;
        opt_vec[p].session->camera_models(camera_model1, camera_model2);
        if (p == 0) // The first image is the "left" image for all pairs.
          cameras.push_back(camera_model1);
        cameras.push_back(camera_model2);
      }
    }

    if (is_map_projected)
      vw_out() << "\t--> Inputs are map projected" << std::endl;

    // Strip the smart pointers and form the stereo model
    std::vector<const vw::camera::CameraModel *> camera_ptrs;
    int num_cams = cameras.size();
    for (int c = 0; c < num_cams; c++) {
      camera_ptrs.push_back(cameras[c].get());
    }

    // Convert the angle tol to be in terms of dot product and pass it
    // to the stereo model.
    double angle_tol = vw::stereo::StereoModel::robust_1_minus_cos
      (stereo_settings().min_triangulation_angle*M_PI/180);

    // Create both a regular stereo model and a bathy stereo model. Will use
    // the latter only if we do bathymetry. There seems to be no elegant
    // way of doing that.
    vw::stereo::StereoModel stereo_model(camera_ptrs, stereo_settings().use_least_squares,
                                         angle_tol);
    asp::BathyStereoModel bathy_stereo_model(camera_ptrs, stereo_settings().use_least_squares,
                                             angle_tol);
    
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
      
    ImageViewRef< PixelMask<float> > left_aligned_bathy_mask, right_aligned_bathy_mask;
    
    std::vector<double> bathy_plane;
    bool use_curved_water_surface = false; // may change below
    vw::cartography::GeoReference water_surface_projection;
    bool bathy_correct = opt_vec[0].session->do_bathymetry();
    if (bathy_correct) {

      if (disparity_maps.size() != 1)
        vw_throw(ArgumentErr() << "Bathymetry correction does not work with multiview stereo\n");

      read_bathy_plane(stereo_settings().bathy_plane,
                       bathy_plane, use_curved_water_surface,
                       water_surface_projection);

      asp::read_vec(stereo_settings().bathy_plane, bathy_plane);
      
      opt_vec[0].session->read_aligned_bathy_masks(left_aligned_bathy_mask,
                                                   right_aligned_bathy_mask); 
      
      if (left_aligned_bathy_mask.cols() != disparity_maps[0].cols() ||
          left_aligned_bathy_mask.rows() != disparity_maps[0].rows() )
        vw_throw( ArgumentErr() << "The dimensions of disparity and left "
                  << "aligned bathymetry mask must agree.\n");
      
      // Pass bathy data to the stereo model
      bathy_stereo_model.set_bathy(stereo_settings().refraction_index, bathy_plane,
                                   use_curved_water_surface, water_surface_projection);
    }

    // Apply radius function and stereo model in one go
    vw_out() << "\t--> Generating a 3D point cloud." << endl;
    ImageViewRef<Vector6> point_cloud;
    if (!bathy_correct) 
      point_cloud = per_pixel_filter
        (stereo_error_triangulate
         (disparity_maps, transforms, stereo_model, is_map_projected, bathy_correct,
          cloud_type, left_aligned_bathy_mask, right_aligned_bathy_mask),
         universe_radius_func);
    else
      point_cloud = per_pixel_filter
        (stereo_error_triangulate
         (disparity_maps, transforms, bathy_stereo_model, is_map_projected, bathy_correct,
          cloud_type, left_aligned_bathy_mask, right_aligned_bathy_mask),
         universe_radius_func);
    
    // If we crop the left and right images, at each run we must
    // recompute the cloud center, as the cropping windows may have changed.
    bool crop_left  = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    bool crop_right = (stereo_settings().right_image_crop_win != BBox2i(0, 0, 0, 0));

    // Compute the point cloud center, unless done by now
    Vector3 cloud_center = Vector3();
    if (!stereo_settings().save_double_precision_point_cloud){
      string cloud_center_file = output_prefix + "-PC-center.txt";
      
      if (!read_point(cloud_center_file, cloud_center) || crop_left || crop_right){
        if (!stereo_settings().skip_point_cloud_center_comp) {
          cloud_center = find_point_cloud_center(opt_vec[0].raster_tile_size, point_cloud);
          write_point(cloud_center_file, cloud_center);
        }
      }
    }
    if (stereo_settings().compute_point_cloud_center_only){
      vw_out() << "Computed the point cloud center. Will stop here." << endl;
      return;
    }

    // We are supposed to do the triangulation in trans_crop_win only
    // so force rasterization in that box only using crop().
    BBox2i cbox = stereo_settings().trans_crop_win;
    string point_cloud_file = output_prefix + "-PC.tif";
    if (stereo_settings().compute_error_vector){

      if (num_cams > 2)
        vw_out(WarningMessage) << "For more than two cameras, the error "
                               << "vector between rays is not meaningful. "
                               << "Setting it to (err_len, 0, 0)." << endl;

      ImageViewRef<Vector6> crop_pc = crop(point_cloud, cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, opt_vec[0]);
    }else{
      ImageViewRef<Vector4> crop_pc = crop(point_and_error_norm(point_cloud), cbox);
      save_point_cloud(cloud_center, crop_pc, point_cloud_file, opt_vec[0]);
    } // End if/else

    // Must print this at the end, as it contains statistics on the number of rejected points.
    vw_out() << "\t--> " << universe_radius_func;

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at point cloud stage "
              << "-- could not read input files.\n"
              << e.what() << "\nExiting.\n\n" );
  } // End outer try/catch
} // End function stereo_triangulation()

} // End namespace asp

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 4 --> TRIANGULATION \n";

    stereo_register_sessions();

    // Unlike other stereo executables, triangulation can handle multiple images and cameras.
    bool verbose = false;
    vector<ASPGlobalOptions> opt_vec;
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
    vector<ASPGlobalOptions> opt_vec_new;
    for (int p = 0; p < (int)opt_vec.size(); p++){
      if (fs::exists(opt_vec[p].out_prefix+"-F.tif"))
        opt_vec_new.push_back(opt_vec[p]);
    }
    opt_vec = opt_vec_new;
    if (opt_vec.empty())
      vw_throw( ArgumentErr() << "No valid F.tif files found.\n" );

    // Triangulation uses small tiles.
    //---------------------------------------------------------
    int ts = ASPGlobalOptions::tri_tile_size();
    for (int s = 0; s < (int)opt_vec.size(); s++)
      opt_vec[s].raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------

    asp::stereo_triangulation(output_prefix, opt_vec);

    vw_out() << "\n[ " << current_posix_time_string() << " ] : TRIANGULATION FINISHED \n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}

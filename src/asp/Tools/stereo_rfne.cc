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


/// \file stereo_rfne.cc
///

#include <asp/Tools/stereo.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/SubpixelView.h>
#include <vw/Stereo/EMSubpixelCorrelatorView.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Core/LocalHomography.h>
#include <asp/Sessions/StereoSession.h>
#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace vw::stereo;
using namespace asp;
using namespace std;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

template <class Image1T, class Image2T>
ImageViewRef<PixelMask<Vector2f> >
refine_disparity(Image1T const& left_image,
                 Image2T const& right_image,
                 ImageViewRef< PixelMask<Vector2f> > const& integer_disp,
                 ASPGlobalOptions const& opt, bool verbose){

  ImageViewRef<PixelMask<Vector2f> > refined_disp = integer_disp;

  PrefilterModeType prefilter_mode = 
    static_cast<vw::stereo::PrefilterModeType>(stereo_settings().pre_filter_mode);

  if ((stereo_settings().subpixel_mode == 0) || (stereo_settings().subpixel_mode > 5)) {
    // Do nothing (includes SGM specific subpixel modes)
    if (verbose)
      vw_out() << "\t--> Skipping subpixel mode.\n";
  }
  if (stereo_settings().subpixel_mode == 1) {
    // Parabola
    
    if (verbose) {
      vw_out() << "\t--> Using parabola subpixel mode.\n";
      if (stereo_settings().pre_filter_mode == 2)
        vw_out() << "\t--> Using LOG pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
      else if (stereo_settings().pre_filter_mode == 1)
        vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
      else
        vw_out() << "\t--> NO preprocessing" << endl;
    } 
    
    refined_disp = parabola_subpixel( integer_disp,
                                      left_image, right_image,
                                      prefilter_mode, stereo_settings().slogW,
                                      stereo_settings().subpixel_kernel );
    
  } // End parabola cases
  if (stereo_settings().subpixel_mode == 2) {
    // Bayes EM
    if (verbose){
      vw_out() << "\t--> Using affine adaptive subpixel mode\n";
      vw_out() << "\t--> Forcing use of LOG filter with "
               << stereo_settings().slogW << " sigma blur.\n";
    }
    refined_disp =
      bayes_em_subpixel( integer_disp,
                         left_image, right_image,
                         prefilter_mode, stereo_settings().slogW,
                         stereo_settings().subpixel_kernel,
                         stereo_settings().subpixel_max_levels );

  } // End Bayes EM cases
  if (stereo_settings().subpixel_mode == 3) {
    // Fast affine
    if (verbose){
      vw_out() << "\t--> Using affine subpixel mode\n";
      vw_out() << "\t--> Forcing use of LOG filter with "
               << stereo_settings().slogW << " sigma blur.\n";
    }
    refined_disp =
      affine_subpixel( integer_disp,
                       left_image, right_image,
                       prefilter_mode, stereo_settings().slogW,
                       stereo_settings().subpixel_kernel,
                       stereo_settings().subpixel_max_levels );

  } // End Fast affine cases
  if (stereo_settings().subpixel_mode == 4) {
    // Lucas-Kanade
    if (verbose){
      vw_out() << "\t--> Using Lucas-Kanade subpixel mode\n";
      vw_out() << "\t--> Forcing use of LOG filter with "
               << stereo_settings().slogW << " sigma blur.\n";
    }
    refined_disp =
      lk_subpixel( integer_disp,
                   left_image, right_image,
                   prefilter_mode, stereo_settings().slogW,
                   stereo_settings().subpixel_kernel,
                   stereo_settings().subpixel_max_levels );

  } // End Lucas-Kanade cases
  if (stereo_settings().subpixel_mode == 5) {
    // Affine and Bayes subpixel refinement always use the LogPreprocessingFilter...
    if (verbose){
      vw_out() << "\t--> Using EM Subpixel mode "
               << stereo_settings().subpixel_mode << endl;
      vw_out() << "\t--> Mode 3 does internal preprocessing;"
               << " settings will be ignored. " << endl;
    }

    typedef stereo::EMSubpixelCorrelatorView<float32> EMCorrelator;
    EMCorrelator em_correlator(channels_to_planes(left_image),
                               channels_to_planes(right_image),
                               pixel_cast<PixelMask<Vector2f> >(integer_disp), -1);
    em_correlator.set_em_iter_max   (stereo_settings().subpixel_em_iter       );
    em_correlator.set_inner_iter_max(stereo_settings().subpixel_affine_iter   );
    em_correlator.set_kernel_size   (stereo_settings().subpixel_kernel        );
    em_correlator.set_pyramid_levels(stereo_settings().subpixel_pyramid_levels);

    DiskImageResourceOpenEXR em_disparity_map_rsrc(opt.out_prefix + "-F6.exr", em_correlator.format());

    block_write_image(em_disparity_map_rsrc, em_correlator,
                      TerminalProgressCallback("asp", "\t--> EM Refinement :"));

    DiskImageResource *em_disparity_map_rsrc_2 =
      DiskImageResourceOpenEXR::construct_open(opt.out_prefix + "-F6.exr");
    DiskImageView<PixelMask<Vector<float, 5> > > em_disparity_disk_image(em_disparity_map_rsrc_2);

    ImageViewRef<Vector<float, 3> > disparity_uncertainty =
      per_pixel_filter(em_disparity_disk_image,
                       EMCorrelator::ExtractUncertaintyFunctor());
    ImageViewRef<float> spectral_uncertainty =
      per_pixel_filter(disparity_uncertainty,
                       EMCorrelator::SpectralRadiusUncertaintyFunctor());
    write_image(opt.out_prefix+"-US.tif", spectral_uncertainty);
    write_image(opt.out_prefix+"-U.tif", disparity_uncertainty);

    refined_disp =
      per_pixel_filter(em_disparity_disk_image,
                       EMCorrelator::ExtractDisparityFunctor());
  } // End EM subpixel cases 
  if ((stereo_settings().subpixel_mode < 0) || (stereo_settings().subpixel_mode > 5)){
    if (verbose) {
      vw_out() << "\t--> Invalid Subpixel mode selection: " << stereo_settings().subpixel_mode << endl;
      vw_out() << "\t--> Doing nothing\n";
    }
  }

  return refined_disp;
}

// Perform refinement in each tile. If using local homography,
// apply the local homography transform for the given tile
// to the right image before doing refinement in that tile.
template <class Image1T, class Image2T, class SeedDispT>
class PerTileRfne: public ImageViewBase<PerTileRfne<Image1T, Image2T, SeedDispT> >{
  Image1T              m_left_image;
  Image2T              m_right_image;
  ImageViewRef<uint8>  m_right_mask;
  SeedDispT            m_integer_disp;
  SeedDispT            m_sub_disp;
  ImageView<Matrix3x3> m_local_hom;
  ASPGlobalOptions const&       m_opt;
  Vector2              m_upscale_factor;

public:
  PerTileRfne( ImageViewBase<Image1T>   const& left_image,
               ImageViewBase<Image2T>   const& right_image,
               ImageViewRef <uint8>     const& right_mask,
               ImageViewBase<SeedDispT> const& integer_disp,
               ImageViewBase<SeedDispT> const& sub_disp,
               ImageView    <Matrix3x3> const& local_hom,
               ASPGlobalOptions const& opt):
    m_left_image(left_image.impl()), m_right_image(right_image.impl()),
    m_right_mask(right_mask),
    m_integer_disp( integer_disp.impl() ), m_sub_disp( sub_disp.impl() ),
    m_local_hom(local_hom), m_opt(opt){

    m_upscale_factor = Vector2(double(m_left_image.impl().cols()) / m_sub_disp.cols(),
                               double(m_left_image.impl().rows()) / m_sub_disp.rows());
  }

  // Image View interface
  typedef PixelMask<Vector2f>                  pixel_type;
  typedef pixel_type                           result_type;
  typedef ProceduralPixelAccessor<PerTileRfne> pixel_accessor;

  inline int32 cols  () const { return m_left_image.cols(); }
  inline int32 rows  () const { return m_left_image.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double /*i*/, double /*j*/, int32 /*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "PerTileRfne::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    ImageView<pixel_type> tile_disparity;
    bool verbose = false;
    if (stereo_settings().seed_mode > 0 && stereo_settings().use_local_homography){

      int ts = ASPGlobalOptions::corr_tile_size();
      Matrix<double>  lowres_hom = m_local_hom(bbox.min().x()/ts, bbox.min().y()/ts);
      Vector3 upscale( m_upscale_factor[0],     m_upscale_factor[1],     1 );
      Vector3 dnscale( 1.0/m_upscale_factor[0], 1.0/m_upscale_factor[1], 1 );
      Matrix<double>  fullres_hom = diagonal_matrix(upscale)*lowres_hom*diagonal_matrix(dnscale);

      // Must transform the right image by the local disparity
      // to be in the same conditions as for stereo correlation.
      typedef typename Image2T::pixel_type right_pix_type;
      ImageViewRef< PixelMask<right_pix_type> > right_trans_masked_img
        = transform (copy_mask( m_right_image.impl(), create_mask(m_right_mask) ),
                     HomographyTransform(fullres_hom),
                     m_left_image.impl().cols(), m_left_image.impl().rows());
      ImageViewRef<right_pix_type> right_trans_img = apply_mask(right_trans_masked_img);


      tile_disparity = crop(refine_disparity(m_left_image, right_trans_img,
                                             m_integer_disp, m_opt, verbose), bbox);

      // Must undo the local homography transform
      bool do_round = false; // don't round floating point disparities
      tile_disparity = transform_disparities(do_round, bbox, inverse(fullres_hom),
                                             tile_disparity);

    }else{
      tile_disparity = crop(refine_disparity(m_left_image, m_right_image,
                                             m_integer_disp, m_opt, verbose), bbox);
    }
    
    prerasterize_type disparity = prerasterize_type(tile_disparity,
                                                    -bbox.min().x(), -bbox.min().y(),
                                                    cols(), rows() );
    return disparity;
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class Image1T, class Image2T, class SeedDispT>
PerTileRfne<Image1T, Image2T, SeedDispT>
per_tile_rfne( ImageViewBase<Image1T  > const& left,
               ImageViewBase<Image2T  > const& right,
               ImageViewRef<uint8     > const& right_mask,
               ImageViewBase<SeedDispT> const& integer_disp,
               ImageViewBase<SeedDispT> const& sub_disp,
               ImageView<Matrix3x3    > const& local_hom,
               ASPGlobalOptions const& opt) {
  typedef PerTileRfne<Image1T, Image2T, SeedDispT> return_type;
  return return_type( left.impl(), right.impl(), right_mask,
                      integer_disp.impl(), sub_disp.impl(), local_hom, opt );
}

void stereo_refinement( ASPGlobalOptions const& opt ) {

  ImageViewRef<PixelGray<float>    > left_image, right_image;
  ImageViewRef<uint8               > left_mask,  right_mask;
  ImageViewRef<PixelMask<Vector2f> > integer_disp;
  ImageViewRef<PixelMask<Vector2f> > sub_disp;
  ImageView<Matrix3x3> local_hom;
  string left_image_file  = opt.out_prefix+"-L.tif";
  string right_image_file = opt.out_prefix+"-R.tif";
  string left_mask_file   = opt.out_prefix+"-lMask.tif";
  string right_mask_file  = opt.out_prefix+"-rMask.tif";

  try {
    left_image   = DiskImageView< PixelGray<float> >(left_image_file );
    right_image  = DiskImageView< PixelGray<float> >(right_image_file);
    left_mask    = DiskImageView<uint8>(left_mask_file );
    right_mask   = DiskImageView<uint8>(right_mask_file);

    // Read the correct type of correlation file (float for SGM/MGM, otherwise integer)
    std::string disp_file = opt.out_prefix + "-D.tif";
    boost::shared_ptr<DiskImageResource> rsrc(DiskImageResourcePtr(disp_file));
    ChannelTypeEnum disp_data_type = rsrc->channel_type();
    if (disp_data_type == VW_CHANNEL_INT32)
      integer_disp = pixel_cast<PixelMask<Vector2f> >(
                      DiskImageView< PixelMask<Vector2i> >(disp_file));
    else // File on disk is float
      integer_disp = DiskImageView< PixelMask<Vector2f> >(disp_file);
    
    if ( stereo_settings().seed_mode > 0 &&
         stereo_settings().use_local_homography ){
      sub_disp = DiskImageView<PixelMask<Vector2f> >(opt.out_prefix+"-D_sub.tif");

      string local_hom_file = opt.out_prefix + "-local_hom.txt";
      read_local_homographies(local_hom_file, local_hom);
    }

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at refinement stage -- could not read input files.\n" 
                            << e.what() << "\nExiting.\n\n" );
  }

  bool skip_img_norm = asp::skip_image_normalization(opt);
  if (skip_img_norm && stereo_settings().subpixel_mode == 2){
    // Images were not normalized in pre-processing. Must do so now
    // as bayes_em_subpixel assumes them to be normalized.
    ImageViewRef< PixelMask< PixelGray<float> > > Limg
      = copy_mask(left_image, create_mask(left_mask));
    ImageViewRef< PixelMask< PixelGray<float> > > Rimg
      = copy_mask(right_image, create_mask(right_mask));

    Vector<float32> left_stats, right_stats;
    string left_stats_file  = opt.out_prefix+"-lStats.tif";
    string right_stats_file = opt.out_prefix+"-rStats.tif";
    vw_out() << "Reading: " << left_stats_file << ' ' << right_stats_file << endl;
    read_vector(left_stats,  left_stats_file );
    read_vector(right_stats, right_stats_file);
    normalize_images(stereo_settings().force_use_entire_range,
                     stereo_settings().individually_normalize,
                     false, // Use std stretch
                     left_stats, right_stats, Limg, Rimg);
    left_image  = apply_mask(Limg);
    right_image = apply_mask(Rimg);
  }

  // The whole goal of this block it to go through the motions of
  // refining disparity solely for the purpose of printing
  // the relevant messages.
  bool verbose = true;
  ImageView<PixelGray<float>    > left_dummy(1, 1), right_dummy(1, 1);
  ImageView<PixelMask<Vector2f> > dummy_disp(1, 1);
  refine_disparity(left_dummy, right_dummy, dummy_disp, opt, verbose);

  ImageViewRef< PixelMask<Vector2f> > refined_disp
    = crop(per_tile_rfne(left_image, right_image, right_mask,
                    integer_disp, sub_disp, local_hom, opt), 
           stereo_settings().trans_crop_win);
  
  cartography::GeoReference left_georef;
  bool   has_left_georef = read_georeference(left_georef,  opt.out_prefix + "-L.tif");
  bool   has_nodata      = false;
  double nodata          = -32768.0;

  string rd_file = opt.out_prefix + "-RD.tif";
  vw_out() << "Writing: " << rd_file << "\n";
  vw::cartography::block_write_gdal_image(rd_file, refined_disp,
                              has_left_georef, left_georef,
                              has_nodata, nodata, opt,
                              TerminalProgressCallback("asp", "\t--> Refinement :") );
}

int main(int argc, char* argv[]) {

  try {
    xercesc::XMLPlatformUtils::Initialize();

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : Stage 2 --> REFINEMENT \n";

    stereo_register_sessions();

    bool verbose = false;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, SubpixelDescription(),
                         verbose, output_prefix, opt_vec);
    ASPGlobalOptions opt = opt_vec[0];

    // Subpixel refinement uses smaller tiles.
    //---------------------------------------------------------
    int ts = ASPGlobalOptions::rfne_tile_size();
    opt.raster_tile_size = Vector2i(ts, ts);

    // Internal Processes
    //---------------------------------------------------------
    stereo_refinement( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : REFINEMENT FINISHED \n";

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}

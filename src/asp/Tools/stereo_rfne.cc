// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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
//#define USE_GRAPHICS

#include <asp/Tools/stereo.h>
#include <vw/Stereo/PreFilter.h>
#include <vw/Stereo/CostFunctions.h>
#include <vw/Stereo/SubpixelView.h>
#include <vw/Stereo/EMSubpixelCorrelatorView.h>

using namespace vw;
using namespace asp;

namespace vw {
  template<> struct PixelFormatID<PixelMask<Vector<float, 5> > >   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_6_CHANNEL; };
}

template <class ImageT>
class SelectiveRasterView : public ImageViewBase< SelectiveRasterView<ImageT> > {
  ImageT m_image;
  BBox2i m_left_image_crop_win;

public:
  SelectiveRasterView( ImageViewBase<ImageT> const& image,
                       BBox2i left_image_crop_win ) :
    m_image( image.impl() ), m_left_image_crop_win( left_image_crop_win ) {}

  typedef typename ImageT::pixel_type pixel_type;
  typedef typename ImageT::result_type result_type;
  typedef typename ImageT::pixel_accessor pixel_accessor;

  inline int32 cols() const { return m_image.cols(); }
  inline int32 rows() const { return m_image.rows(); }
  inline int32 planes() const { return m_image.planes(); }

  inline pixel_accessor origin() const { return m_image.origin(); }

  inline result_type operator()( int32 i, int32 j, int32 p = 0 ) const {
    return m_image( i, j, p );
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {
    // We do stereo only in m_left_image_crop_win. Skip the current tile if
    // it does not intersect this region.
    BBox2i intersection = bbox; intersection.crop(m_left_image_crop_win);
    if (intersection.empty()){
      return prerasterize_type(ImageView<pixel_type>(bbox.width(),
                                                     bbox.height()),
                               -bbox.min().x(), -bbox.min().y(),
                               cols(), rows() );
    }

    ImageView<pixel_type> output =
      crop( m_image.prerasterize( bbox ), bbox );
    return prerasterize_type( output, -bbox.min().x(), -bbox.min().y(),
                              cols(), rows() );
  }

  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

template <class ImageT>
SelectiveRasterView<ImageT>
selective_rasterize( ImageViewBase<ImageT> const& image,
                     BBox2i const& left_crop ) {
  return SelectiveRasterView<ImageT>( image.impl(), left_crop );
}

void stereo_refinement( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string() << " ] : Stage 2 --> REFINEMENT \n";

  try {
    std::string filename_L = opt.out_prefix+"-L.tif",
      filename_R  = opt.out_prefix+"-R.tif";
    /*
      if (MEDIAN_FILTER == 1){
      filename_L = out_prefix+"-median-L.tif";
      filename_R = out_prefix+"-median-R.tif";
      } else {
      filename_L = out_prefix+"-L.tif";
      filename_R = out_prefix+"-R.tif";
      }
    */
    typedef DiskImageView<PixelGray<float> > InnerView;
    InnerView left_disk_image(filename_L), right_disk_image(filename_R);
    DiskImageView<PixelMask<Vector2i> > disparity_disk_image(opt.out_prefix + "-D.tif");
    ImageViewRef<PixelMask<Vector2f> > disparity_map =
      pixel_cast<PixelMask<Vector2f> >(disparity_disk_image);

    if (stereo_settings().subpixel_mode == 0) {
      // Do nothing
    } else if (stereo_settings().subpixel_mode == 1) {
      // Parabola
      vw_out() << "\t--> Using parabola subpixel mode.\n";
      if (stereo_settings().pre_filter_mode == 2) {
        vw_out() << "\t--> Using LOG pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
        typedef stereo::LaplacianOfGaussian PreFilter;
        disparity_map =
          parabola_subpixel( disparity_disk_image,
                             left_disk_image, right_disk_image,
                             PreFilter(stereo_settings().slogW),
                             stereo_settings().subpixel_kernel );
      } else if (stereo_settings().pre_filter_mode == 1) {
        vw_out() << "\t--> Using Subtracted Mean pre-processing filter with "
                 << stereo_settings().slogW << " sigma blur.\n";
        typedef stereo::SubtractedMean PreFilter;
        disparity_map =
          parabola_subpixel( disparity_disk_image,
                             left_disk_image, right_disk_image,
                             PreFilter(stereo_settings().slogW),
                             stereo_settings().subpixel_kernel );
      } else {
        vw_out() << "\t--> NO preprocessing" << std::endl;
        typedef stereo::NullOperation PreFilter;
        disparity_map =
          parabola_subpixel( disparity_disk_image,
                             left_disk_image, right_disk_image,
                             PreFilter(),
                             stereo_settings().subpixel_kernel );
      }
    } else if (stereo_settings().subpixel_mode == 2) {
      // Bayes EM
      vw_out() << "\t--> Using affine adaptive subpixel mode\n";
      vw_out() << "\t--> Forcing use of LOG filter with "
               << stereo_settings().slogW << " sigma blur.\n";
      typedef stereo::LaplacianOfGaussian PreFilter;
      disparity_map =
        bayes_em_subpixel( disparity_disk_image,
                           left_disk_image, right_disk_image,
                           PreFilter(stereo_settings().slogW),
                           stereo_settings().subpixel_kernel,
                           stereo_settings().subpixel_max_levels );

    } else if (stereo_settings().subpixel_mode == 3) {
      // Affine and Bayes subpixel refinement always use the
      // LogPreprocessingFilter...
      vw_out() << "\t--> Using EM Subpixel mode "
               << stereo_settings().subpixel_mode << std::endl;
      vw_out() << "\t--> Mode 3 does internal preprocessing;"
               << " settings will be ignored. " << std::endl;

      typedef stereo::EMSubpixelCorrelatorView<float32> EMCorrelator;
      EMCorrelator em_correlator(channels_to_planes(left_disk_image),
                                 channels_to_planes(right_disk_image),
                                 pixel_cast<PixelMask<Vector2f> >(disparity_disk_image), -1);
      em_correlator.set_em_iter_max(stereo_settings().subpixel_em_iter);
      em_correlator.set_inner_iter_max(stereo_settings().subpixel_affine_iter);
      em_correlator.set_kernel_size(stereo_settings().subpixel_kernel);
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

      disparity_map =
        per_pixel_filter(em_disparity_disk_image,
                         EMCorrelator::ExtractDisparityFunctor());
    } else {
      vw_out() << "\t--> Invalid Subpixel mode selection: " << stereo_settings().subpixel_mode << std::endl;
      vw_out() << "\t--> Doing nothing\n";
    }

    asp::block_write_gdal_image( opt.out_prefix + "-RD.tif",
                                 selective_rasterize(disparity_map,
                                                     opt.left_image_crop_win), opt,
                                 TerminalProgressCallback("asp", "\t--> Refinement :") );

  } catch (IOErr const& e) {
    vw_throw( ArgumentErr() << "\nUnable to start at refinement stage -- could not read input files.\n" << e.what() << "\nExiting.\n\n" );
  }
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      SubpixelDescription() );

    // Internal Processes
    //---------------------------------------------------------
    stereo_refinement( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : REFINEMENT FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file PhotometricOutlier.cc
///
/// Warning: This code was written with only the Apollo Metric data in mind

#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Sessions/ISIS/PhotometricOutlier.h>

using namespace vw;

void vw::photometric_outlier_rejection( std::string const& prefix,
                                        std::string const& input_disparity,
                                        std::string & output_disparity,
                                        int kernel_size ) {

  // Projecting right into perspective of left
  DiskImageView<PixelGray<float> > right_disk_image(prefix+"-R.tif");
  DiskImageView<PixelMask<Vector2f> > disparity_disk_image( input_disparity );
  stereo::DisparityTransform trans( disparity_disk_image );
  DiskCacheImageView<PixelGray<float> > right_proj( transform( right_disk_image, trans, ZeroEdgeExtension() ), "tif", TerminalProgressCallback("asp","Projecting R:") );

  // Differencing Left and Projected Right
  DiskImageView<PixelGray<float32> > left_image(prefix+"-L.tif");
  DiskCacheImageView<PixelGray<float> > diff( abs(left_image-right_proj), "tif", TerminalProgressCallback("asp","Diff:") );
  ChannelAccumulator<math::CDFAccumulator<float32> > cdf;
  for_each_pixel(diff, cdf);
  float thresh = cdf.quantile(0.99);

  // Thresholding image and dilating
  ImageView<PixelGray<float> > dust = threshold(apply_mask(copy_mask(diff,create_mask(right_proj))),thresh,1.0,0.0);
  ImageView<PixelGray<float> > grass;
  grassfire(dust,grass);

  ImageViewRef<PixelMask<Vector2f > > cleaned_disparity = intersect_mask(disparity_disk_image,intersect_mask(create_mask(threshold(grass,kernel_size,0.0,1.0)),create_mask(right_proj)));

  output_disparity = prefix+"-FDust.tif";
  DiskImageResourceGDAL disparity_rsrc( output_disparity,
                                        cleaned_disparity.format(),
                                        Vector2i(vw_settings().default_tile_size(),
                                                 vw_settings().default_tile_size()) );
  block_write_image( disparity_rsrc, cleaned_disparity,
                     TerminalProgressCallback("asp","Dust Removal:") );
}

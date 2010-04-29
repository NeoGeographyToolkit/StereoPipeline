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
#include <asp/Core/StereoSettings.h>

using namespace vw;



void vw::photometric_outlier_rejection( std::string const& prefix,
                                        std::string const& input_disparity,
                                        std::string & output_disparity,
                                        int kernel_size ) {

  // Projecting right into perspective of left
  DiskImageView<PixelGray<float> > right_disk_image(prefix+"-R.tif");
  DiskImageView<PixelMask<Vector2f> > disparity_disk_image( input_disparity );
  stereo::DisparityTransform trans( disparity_disk_image );
  DiskCacheImageView<PixelGray<float> > right_proj( transform( right_disk_image, trans, ZeroEdgeExtension() ), "tif", TerminalProgressCallback("asp","Projecting R:"), stereo_settings().cache_dir);

  // Differencing Left and Projected Right
  ImageViewRef<PixelMask<PixelGray<float32> > > right_mask = create_mask(right_proj);
  DiskImageView<PixelGray<float32> > left_image(prefix+"-L.tif");
  DiskCacheImageView<PixelGray<float> > diff( abs(apply_mask(copy_mask(left_image,right_mask))-right_proj), "tif", TerminalProgressCallback("asp","\tDifference:"), stereo_settings().cache_dir);
  ChannelAccumulator<math::CDFAccumulator<float32> > cdf;
  cdf.resize(3000,1001);
  for_each_pixel(diff, cdf);
  float thresh = cdf.quantile(0.9985); // Pulling out last bin of CDF
  vw_out() << "\t  Using threshold: " << thresh << "\n";

  // Thresholding image and dilating
  ImageView<PixelGray<float> > dust = threshold(apply_mask(copy_mask(diff,right_mask)),thresh,1.0,0.0);
  ImageView<PixelGray<float> > grass;
  grassfire(dust,grass);
  dust = gaussian_filter(grass,kernel_size/3);

  ImageViewRef<PixelMask<Vector2f > > cleaned_disparity = intersect_mask(disparity_disk_image,intersect_mask(create_mask(threshold(dust,kernel_size,0.0,1.0)),right_mask));

  output_disparity = prefix+"-FDust.tif";
  DiskImageResourceGDAL disparity_rsrc( output_disparity,
                                        cleaned_disparity.format(),
                                        Vector2i(vw_settings().default_tile_size(),
                                                 vw_settings().default_tile_size()) );
  block_write_image( disparity_rsrc, cleaned_disparity,
                     TerminalProgressCallback("asp","Dust Removal:") );
}

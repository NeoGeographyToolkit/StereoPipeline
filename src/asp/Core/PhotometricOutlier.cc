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


/// \file PhotometricOutlier.cc
///
/// Warning: This code was written with only the Apollo Metric data in mind

#include <vw/Image/AlgorithmFunctions.h>
#include <vw/Image/Algorithms.h>
#include <vw/Image/Filter.h>
#include <vw/Image/Convolution.h>
#include <vw/Image/EdgeExtension.h>
#include <vw/Image/Manipulation.h>
#include <vw/Image/MaskViews.h>
#include <vw/Image/Transform.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/Common.h>
#include <asp/Core/PhotometricOutlier.h>
namespace fs = boost::filesystem;

using namespace vw;
using namespace asp;

void asp::photometric_outlier_rejection( vw::cartography::GdalWriteOptions const& opt,
                                         std::string const& prefix,
                                         std::string const& input_disparity,
                                         std::string & output_disparity,
                                         int kernel_size ) {
  // Projecting right into perspective of left
  DiskImageView<PixelGray<float> > right_disk_image(prefix+"-R.tif");
  DiskImageView<PixelMask<Vector2f> > disparity_disk_image( input_disparity );
  stereo::DisparityTransform trans( disparity_disk_image );

  std::string cache_dir = "/tmp"; // modify here to use a different cache dir if needed
  if (!fs::is_directory(cache_dir)) {
    vw_out() << "\nCreating cache directory: " << cache_dir << std::endl;
    fs::create_directories(cache_dir);
  }

  DiskCacheImageView<PixelGray<float> >
    right_proj( transform( right_disk_image, trans, ZeroEdgeExtension() ), "tif", TerminalProgressCallback("asp","Projecting R:"), cache_dir);

  // Differencing Left and Projected Right
  ImageViewRef<PixelMask<PixelGray<float32> > > right_mask =
    create_mask(right_proj);
  DiskImageView<PixelGray<float32> > left_image(prefix+"-L.tif");
  DiskCacheImageView<PixelGray<float> >
    diff( abs(apply_mask(copy_mask(left_image,right_mask))-right_proj),
          "tif", TerminalProgressCallback("asp","\tDifference:"),
          cache_dir);
  ChannelAccumulator<math::CDFAccumulator<float32> > cdf;
  cdf.resize(8000,2001);
  for_each_pixel(diff, cdf);
  float thresh = cdf.quantile(0.99985); // Pulling out last bin of CDF
  vw_out() << "\t  Using threshold: " << thresh << "\n";

  // Thresholding image and dilating
  ImageView<PixelGray<float> > dust =
    threshold(apply_mask(copy_mask(diff,right_mask)),thresh,1.0,0.0);
  ImageView<PixelGray<float> > grass;
  grassfire(dust,grass);
  dust = gaussian_filter(grass,kernel_size/3);

  ImageViewRef<PixelMask<Vector2f > > cleaned_disparity =
    intersect_mask(disparity_disk_image,
                   intersect_mask(create_mask(threshold(dust,kernel_size,0.0,1.0)),right_mask));

  vw::cartography::block_write_gdal_image( prefix+"-FDust.tif",
                          cleaned_disparity, opt,
                          TerminalProgressCallback("asp","Dust Removal:") );
}

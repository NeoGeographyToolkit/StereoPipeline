#ifndef __SURFACENURBS_H__
#define __SURFACENURBS_H__

#include <vw/Image/ImageView.h>
#include <vw/Image/PixelTypes.h>
#include <vw/Stereo/DisparityMap.h>

// Function prototypes 
vw::ImageView<vw::PixelDisparity<float> > MBASurfaceNURBS(vw::ImageView<vw::PixelDisparity<float> > const& input, 
                                                          int numIterations = 10);

vw::ImageView<vw::PixelGrayA<float> > MBASurfaceNURBS(vw::ImageView<vw::PixelGrayA<float> > const& input, 
                                                      int numIterations = 10);

#endif

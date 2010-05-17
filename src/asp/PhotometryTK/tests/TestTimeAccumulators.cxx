// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <gtest/gtest.h>
#include <vw/Image.h>
#include <asp/PhotometryTK/TimeAccumulators.h>
#include <test/Helpers.h>

using namespace vw;
using namespace asp;
using namespace asp::pho;

template <class ChannelT>
class TimeAccumulatorTest : public ::testing::Test {
protected:
  TimeAccumulatorTest() {}

  virtual void SetUp() {
    image.set_size(2,2);
    reflectance.set_size(2,2);
    albedo.set_size(2,2);
    exposure_t = 0.5;

    image(0,0) = PixelGrayA<ChannelT>( 1, 1 );
    image(0,1) = PixelGrayA<ChannelT>( 0, 1 );
    image(1,0) = PixelGrayA<ChannelT>( 1, 0 );
    image(1,1) = PixelGrayA<ChannelT>( 2, 4 );

    reflectance(0,0) = 1;
    reflectance(0,1) = 3;
    reflectance(1,0) = 0;
    reflectance(1,1) = 2;

    albedo(0,0) = PixelGrayA<ChannelT>( 2, 1 );
    albedo(0,1) = PixelGrayA<ChannelT>( 1, 1 );
    albedo(1,0) = PixelGrayA<ChannelT>( 2, 0 );
    albedo(1,1) = PixelGrayA<ChannelT>( 3, 2 );
  }

  ImageView<PixelGrayA<ChannelT> > image, albedo;
  ImageView<double> reflectance;
  double exposure_t;
};

typedef TimeAccumulatorTest<uint8> TimeAccumulatorTestInt;
typedef TimeAccumulatorTest<float> TimeAccumulatorTestFloat;

TEST_F( TimeAccumulatorTestInt, TimeDelta ) {
  TimeDeltaAccumulator<PixelGrayA<uint8>, uint8> taccum( exposure_t );
  for_each_pixel(image,albedo,reflectance,taccum);
  EXPECT_NEAR( -0.048387, taccum.value(), 1e-6 );
}

TEST_F( TimeAccumulatorTestFloat, TimeDelta ) {
  TimeDeltaAccumulator<PixelGrayA<float>, float> taccum( exposure_t );
  for_each_pixel(image,albedo,reflectance,taccum);
  EXPECT_NEAR( -0.048387, taccum.value(), 1e-6 );
}

TEST_F( TimeAccumulatorTestInt, TimeDeltaNR ) {
  TimeDeltaNRAccumulator<PixelGrayA<uint8> > taccum( exposure_t );
  for_each_pixel(image,albedo,taccum);
  EXPECT_NEAR( 0.03691275, taccum.value(), 1e-8);
}

TEST_F( TimeAccumulatorTestFloat, TimeDeltaNR ) {
  TimeDeltaNRAccumulator<PixelGrayA<float> > taccum( exposure_t );
  for_each_pixel(image,albedo,taccum);
  EXPECT_NEAR( 0.03691275, taccum.value(), 1e-8);
}

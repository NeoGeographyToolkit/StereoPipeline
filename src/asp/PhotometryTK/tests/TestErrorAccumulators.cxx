// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <vw/Image.h>
#include <asp/PhotometryTK/ErrorAccumulators.h>
#include <test/Helpers.h>

using namespace vw;
using namespace asp;
using namespace asp::pho;

template <class ChannelT>
class ErrorAccumulatorTest : public ::testing::Test {
protected:
  ErrorAccumulatorTest() {}

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

typedef ErrorAccumulatorTest<uint8> ErrorAccumulatorTestInt;
typedef ErrorAccumulatorTest<float> ErrorAccumulatorTestFloat;

TEST_F( ErrorAccumulatorTestInt, Error ) {
  ErrorAccumulator eaccum( exposure_t );
  for_each_pixel(image,albedo,reflectance,eaccum);
  EXPECT_EQ( 18.25, eaccum.value() );
}

TEST_F( ErrorAccumulatorTestFloat, Error ) {
  ErrorAccumulator eaccum( exposure_t );
  for_each_pixel(image,albedo,reflectance,eaccum);
  EXPECT_EQ( 18.25, eaccum.value() );
}

TEST_F( ErrorAccumulatorTestInt, ErrorNR ) {
  ErrorNRAccumulator eaccum( exposure_t );
  for_each_pixel(image,albedo,eaccum);
  EXPECT_EQ( 4.25, eaccum.value() );
}

TEST_F( ErrorAccumulatorTestFloat, ErrorNR ) {
  ErrorNRAccumulator eaccum( exposure_t );
  for_each_pixel(image,albedo,eaccum);
  EXPECT_EQ( 4.25, eaccum.value() );
}

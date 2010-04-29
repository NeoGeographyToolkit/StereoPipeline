// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

#include <gtest/gtest.h>
#include <vw/Image.h>
#include <asp/PhotometryTK/AlbedoAccumulators.h>
#include <test/Helpers.h>

using namespace vw;
using namespace asp;
using namespace asp::pho;

template <class ChannelT>
class AlbedoAccumulatorTest : public ::testing::Test {
protected:
  AlbedoAccumulatorTest() {}

  virtual void SetUp() {
    image.set_size(2,2); image2.set_size(2,2);
    reflectance.set_size(2,2); reflectance2.set_size(2,2);
    albedo.set_size(2,2);
    exposure_t = 0.5;
    exposure_t2 = 0.75;

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

    image2(0,0) = PixelGrayA<ChannelT>( 3, 1 );
    image2(0,1) = PixelGrayA<ChannelT>( 2, 3 );
    image2(1,0) = PixelGrayA<ChannelT>( 4, 5 );
    image2(1,1) = PixelGrayA<ChannelT>( 1, 1 );

    reflectance2(0,0) = 2;
    reflectance2(0,1) = 3;
    reflectance2(1,0) = 3;
    reflectance2(1,1) = 4;
  }

  ImageView<PixelGrayA<ChannelT> > image, image2;
  ImageView<double> reflectance, reflectance2;
  ImageView<PixelGrayA<ChannelT> > albedo;
  double exposure_t, exposure_t2;
};

typedef AlbedoAccumulatorTest<uint8> AlbedoAccumulatorTestInt;
typedef AlbedoAccumulatorTest<uint8> AlbedoAccumulatorTestFloat;

#define EXPECT_IMAGE_NULL( p, i ) \
  EXPECT_EQ( p(), i(0,0) );       \
  EXPECT_EQ( p(), i(0,1) );       \
  EXPECT_EQ( p(), i(1,0) );       \
  EXPECT_EQ( p(), i(1,1) );

TEST_F( AlbedoAccumulatorTestInt, AlbedoInit ) {
  typedef PixelGrayA<uint8> Px;
  AlbedoInitAccumulator<Px > accu(2,2);
  typedef AlbedoInitAccumulator<Px >::result_type result_type;
  result_type null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate one image
  accu( image, reflectance, exposure_t );
  result_type result = accu.result();
  EXPECT_PIXEL_EQ( Px(2,255), result(0,0) );
  EXPECT_PIXEL_EQ( Px(0,255), result(0,1) );
  EXPECT_PIXEL_EQ( Px(),      result(1,0) );
  EXPECT_PIXEL_EQ( Px(2,255), result(1,1) );

  // Check that it reset
  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate 2 images
  accu( image, reflectance, exposure_t );
  accu( image2, reflectance2, exposure_t2 );
  result = accu.result();
  EXPECT_PIXEL_EQ( Px(2,255), result(0,0) );
  EXPECT_PIXEL_EQ( Px(0,255), result(0,1) );
  EXPECT_PIXEL_EQ( Px(1,255), result(1,0) );
  EXPECT_PIXEL_EQ( Px(1,255), result(1,1) );

  // Check that it reset
  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );
}

TEST_F( AlbedoAccumulatorTestFloat, AlbedoInit ) {
  typedef PixelGrayA<float> Px;
  AlbedoInitAccumulator<Px > accu(2,2);
  typedef AlbedoInitAccumulator<Px >::result_type result_type;
  result_type null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate one image
  accu( image, reflectance, exposure_t );
  result_type result = accu.result();
  EXPECT_PIXEL_EQ( Px(2,1), result(0,0) );
  EXPECT_PIXEL_EQ( Px(0,1), result(0,1) );
  EXPECT_PIXEL_EQ( Px(),    result(1,0) );
  EXPECT_PIXEL_EQ( Px(2,1), result(1,1) );

  // Check that it reset
  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate 2 images
  accu( image, reflectance, exposure_t );
  accu( image2, reflectance2, exposure_t2 );
  result = accu.result();
  EXPECT_PIXEL_EQ( Px(2,1), result(0,0) );
  EXPECT_PIXEL_NEAR( Px(0.666,1), result(0,1), 1e-2 );
  EXPECT_PIXEL_NEAR( Px(1.777,1), result(1,0), 1e-2 );
  EXPECT_PIXEL_NEAR( Px(1.666,1), result(1,1), 1e-2 );

  // Check that it reset
  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );
}

TEST_F( AlbedoAccumulatorTestFloat, AlbedoInitNR ) {
  typedef PixelGrayA<float> Px;
  AlbedoInitNRAccumulator<Px > accu(2,2);
  typedef AlbedoInitNRAccumulator<Px >::result_type result_type;
  result_type null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate one image
  accu( image, exposure_t );
  result_type result = accu.result();
  EXPECT_PIXEL_EQ( Px(2,1), result(0,0) );
  EXPECT_PIXEL_EQ( Px(0,1), result(0,1) );
  EXPECT_PIXEL_EQ( Px(),      result(1,0) );
  EXPECT_PIXEL_EQ( Px(4,1), result(1,1) );

  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );

  // Accumulate 2 images
  accu( image, exposure_t );
  accu( image2, exposure_t2 );
  result = accu.result();
  EXPECT_PIXEL_EQ( Px(3,1), result(0,0) );
  EXPECT_PIXEL_EQ( Px(2,1), result(0,1) );
  EXPECT_PIXEL_NEAR( Px(5.333,1), result(1,0), 1e-2 );
  EXPECT_PIXEL_NEAR( Px(3.466,1), result(1,1), 1e-2 );

  null = accu.result();
  EXPECT_IMAGE_NULL( Px, null );
}

TEST_F( AlbedoAccumulatorTestInt, AlbedoDelta ) {
}

TEST_F( AlbedoAccumulatorTestFloat, AlbedoDelta ) {
}

TEST_F( AlbedoAccumulatorTestFloat, AlbedoDeltaNR ) {
}

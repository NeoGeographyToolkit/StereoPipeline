// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <asp/PhotometryTK/ErrorAccumulators.h>
#include <asp/PhotometryTK/TimeAccumulators.h>
#include <asp/PhotometryTK/AlbedoAccumulators.h>
#include <test/Helpers.h>

using namespace vw;
using namespace asp;
using namespace asp::pho;

// These test is a repeat of previous tests but uses actually data and
// will run the data into a position where overflow is possible. This
// will stress that the accumulators are of the correct data type.

template <class ChannelT>
class FullIterationTest : public ::testing::Test {
protected:
  FullIterationTest() {}

  typedef PixelGrayA<ChannelT> Px;

  virtual void SetUp() {
    exposure.resize( 4 );
    drgs.resize( 4 );

    // Load outside images and set exposure
    for ( int i = 0; i < 4; i++ ) {
      std::ostringstream ostr;
      ostr << "input_image" << i+1 << ".tif";
      read_image( drgs[i], ostr.str() );
      exposure[i] = 1.0;
    }

    // Build the starting albedo
    AlbedoInitNRAccumulator<Px > albedo_accu(256,256);
    for ( int i = 0; i < 4; i++ )
      albedo_accu( drgs[i], exposure[i] );
    albedo = albedo_accu.result();
  }

  std::vector<double> exposure;
  std::vector<ImageView<Px > > drgs;
  ImageView<Px > albedo;

};

typedef FullIterationTest<uint8> FullU8IterationTest;
typedef FullIterationTest<float> FullF32IterationTest;

TEST_F( FullU8IterationTest, FullCycle ) {
  EXPECT_PIXEL_EQ( Px(129,255), albedo(128,128) );

  // Calculate current error
  double starting_error = 0;
  for ( int i = 0; i < 4; i++ ) {
    ErrorNRAccumulator eaccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, eaccum );
    starting_error += eaccum.value();
  }

  // Perform single iteration
  for ( int i = 0; i < 4; i++ ) {
    TimeDeltaNRAccumulator taccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, taccum );
    exposure[i] += taccum.value();
  }
  {
    AlbedoDeltaNRAccumulator<Px> albedo_accum( 256, 256 );
    for ( uint i = 0; i < 4; i++ )
      albedo_accum(drgs[i],albedo,exposure[i]);
    select_channel(albedo,0) += select_channel(albedo_accum.result(),0);
  }

  // Calculate final error
  double final_error = 0;
  for ( int i = 0; i < 4; i++ ) {
    ErrorNRAccumulator eaccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, eaccum );
    final_error += eaccum.value();
  }
  ASSERT_LT( final_error, starting_error );
  ASSERT_LT( exposure[0], 1.0 );
  ASSERT_GT( exposure[1], 1.0 );
  ASSERT_LT( exposure[2], 1.0 );
  ASSERT_GT( exposure[3], 1.0 );
}

TEST_F( FullF32IterationTest, FullCycle ) {
  EXPECT_PIXEL_NEAR( Px(0.507843,1.0), albedo(128,128), 1e-5 );

  // Calculate current error
  double starting_error = 0;
  for ( int i = 0; i < 4; i++ ) {
    ErrorNRAccumulator eaccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, eaccum );
    starting_error += eaccum.value();
  }

  // Perform single iteration
  for ( int i = 0; i < 4; i++ ) {
    TimeDeltaNRAccumulator taccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, taccum );
    exposure[i] += taccum.value();
  }
  {
    AlbedoDeltaNRAccumulator<Px> albedo_accum( 256, 256 );
    for ( uint i = 0; i < 4; i++ )
      albedo_accum(drgs[i],albedo,exposure[i]);
    select_channel(albedo,0) += select_channel(albedo_accum.result(),0);
  }

  // Calculate final error
  double final_error = 0;
  for ( int i = 0; i < 4; i++ ) {
    ErrorNRAccumulator eaccum( exposure[i] );
    for_each_pixel( drgs[i], albedo, eaccum );
    final_error += eaccum.value();
  }
  ASSERT_LT( final_error, starting_error );
  ASSERT_LT( exposure[0], 1.0 );
  ASSERT_GT( exposure[1], 1.0 );
  ASSERT_LT( exposure[2], 1.0 );
  ASSERT_GT( exposure[3], 1.0 );
}

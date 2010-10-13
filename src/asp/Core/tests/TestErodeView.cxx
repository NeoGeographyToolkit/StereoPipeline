// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>

#include <vw/Image/ImageViewRef.h>
#include <asp/Core/ErodeView.h>

using namespace vw;

TEST(ErodeView, basic_test) {
  ImageView<PixelMask<uint8> > test(3,3);
  EXPECT_EQ( PixelMask<uint8>(), test(0,0) );
  EXPECT_FALSE( is_valid(test(0,0)) );
  test(1,1) = PixelMask<uint8>(100);
  EXPECT_TRUE( is_valid(test(1,1)) );

  BlobIndexThreaded bindex( test, 100 );
  EXPECT_EQ( 1u, bindex.num_blobs() );

  ImageViewRef<PixelMask<uint8> > erode_ref =
    ErodeView<ImageView<PixelMask<uint8> > >( test, bindex );
  EXPECT_FALSE( is_valid(erode_ref(1,1)) );
  EXPECT_FALSE( is_valid(erode_ref(0,0)) );
  ImageView<PixelMask<uint8> > eroded = erode_ref;
  EXPECT_FALSE( is_valid(eroded(0,0) ) );
  EXPECT_FALSE( is_valid(eroded(1,1) ) );
  EXPECT_EQ( 0, eroded(1,1).child() );
}

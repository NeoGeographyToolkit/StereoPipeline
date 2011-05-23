// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <gtest/gtest.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <asp/Core/BlobIndexThreaded.h>

using namespace vw;

TEST(BlobIndexThreaded, TestImage1) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest1.tif");
  EXPECT_EQ( 20, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 10 );
  EXPECT_EQ( 2u, bindex.num_blobs() );
}

TEST(BlobIndexThreaded, TestImage2) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest2.tif");
  EXPECT_EQ( 10, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 5 );
  EXPECT_EQ( 1u, bindex.num_blobs() );
}

TEST(BlobIndexThreaded, TestImage3) {
  DiskImageView<PixelGray<uint8> > input("ThreadTest3.tif");
  EXPECT_EQ( 10, input.cols() );
  BlobIndexThreaded bindex( create_mask(input,255), 1000, 5 );
  EXPECT_EQ( 2u, bindex.num_blobs() );
}

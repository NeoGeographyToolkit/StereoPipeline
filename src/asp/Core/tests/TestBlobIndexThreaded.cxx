// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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


#include <test/Helpers.h>

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

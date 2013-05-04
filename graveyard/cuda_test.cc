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


#include <cuda_code.h>
#include <vw/vw.h>

using namespace vw;

int main(int argc, char** argv) {

  if (argc != 3) {
    vw_out(0) << "Usage: " << argv[0] << " " << argc << " <input image> <output image>\n";
    exit(0);
  }
  
  // Open the image
  ImageView<PixelGray<float> > image;
  read_image(image, argv[1]);

  // Call out to our CUDA code
  invert_image(&(image(0,0)[0]), image.cols(), image.rows());

  // Write the result
  write_image(argv[2], image);
} 


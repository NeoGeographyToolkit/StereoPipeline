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


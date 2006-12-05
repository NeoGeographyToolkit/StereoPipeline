//#define DEANS_INTEREST_POINT

#include "asp_config.h"

#include <iostream>
#include <vw/FileIO.h>
#include <vw/Image/Transform.h>
#include <vw/Image/Manipulation.h>
#include <vw/Math/Matrix.h>
using namespace vw;

#include "ImageAlign.h"
#include "SIFT.h"

using namespace std;

// double align_max(ImageView<float> image, unsigned int &rowMax, unsigned int &colMax) {

//   unsigned int ni = image.cols();	   // cols
//   unsigned int nj = image.rows();	   // rows
//   unsigned int nplanes = image.planes();
//   double value = image(0,0,0);	

//   for (unsigned int p = 0; p < nplanes; p++) {
//     for (unsigned int j = 0; j < nj; j++) {
//       for (unsigned int i = 0; i < ni; i++) {
// 	if (image(i,j,p) > value) {
// 	  value = image(i,j,p);
// 	  colMax = i;
// 	  rowMax = j;
// 	}
//       }
//     }
//   }

//   return value;
// }

// Matrix<double> MOCImageAlign(ImageView<PixelGray<float> > &primary, 
//                              ImageView<PixelGray<float> > &secondary, 
//                              ImageView<PixelGray<float> > &adj_image,
//                              ImageView<PixelGray<float> > &fully_adj_image,
//                              MOCImageMetadata &primary_metadata,
//                              MOCImageMetadata &secondary_metadata,
//                              double kernel_x, double kernel_y,
//                              int kernel_width, int kernel_height, 
//                              std::string out_prefix) {
  
//   // Check to see whether the images have actually been initialized 
//   // with ephemeris information.  Bail if they have not.
//   if (!primary_metadata.initialized() || !secondary_metadata.initialized())
//     throw MOCEphemerisErr() << "No ephemeris information has been loaded for these images.\n";

//   // ASPECT RATIO and SCALING (unitless)
//   //
//   // Convert the aspect ratio and resolution of the secondary_metadata image 
//   // to match the primary image.
//   //
//   // Pixel aspect ratio (pixel height/pixel width)
//   // at center of image.  The image's center pixel is projected on the
//   // ground and its linear extents are ratioed, so any effects of
//   // emission angle and line time are modelled.  To recover an image
//   // with 'square' pixels, the image should be resampled to a height
//   // of aspect_ratio * input height.
//   // T_aspect = [1 0 0 ; 0 1/primary.aspect 0 ; 0 0 1];
//   double sx = secondary_metadata.m_res_x / primary_metadata.m_res_x;
//   double sy = secondary_metadata.m_res_y / primary_metadata.m_res_y;

//   //
//   // ROTATION
//   //
//   // The angle in degrees clockwise from the reference axis
//   // of the image (a line from the center to the right edge of the image)
//   // to the direction to the north pole of the target body.  If the
//   // USAGE_NOTE described previously is 'F', the image should be flipped
//   // prior to applying this angle."
//   double rot = secondary_metadata.m_north_angle - primary_metadata.m_north_angle;

//   // 
//   // SKEW
//   //
//   // The image skew angle is the absolute value, in degrees, of the angle
//   // between the left edge (a line between the lower left and upper left
//   // corners) and the bottom edge (a line between the lower left and lower
//   // right corners) of the image, where the edges are projected on the
//   // target body.  For images whose footprints are exactly rectangular,
//   // this angle will be 90 degrees.  Departures from 90 degrees indicate a
//   // non-rectangular image footprint, caused by slews of the spacecraft
//   // during imaging outside the orbit plane or off-nadir pointing, or
//   // both.
//   double skew = -secondary_metadata.m_skew_angle + primary_metadata.m_skew_angle;
//   double a = 1.0;
//   double c = atan(skew);

//   a = a / (1 + c*c);			   // Normalize
//   c = c / (1 + c*c);

//   //  double sData[] = { sx, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 1.0 };
//   Matrix<double> T_s(3, 3);
//   T_s.set_identity();
//   T_s(0,0) = sx;
//   T_s(1,1) = sy;

//   // Correct for the difference in image orientation relative to north.
//   double cosRot = cos(rot), sinRot = sin(rot);
//   //  double rData[] = {cosRot, sinRot, 0.0, -sinRot, cosRot, 0.0, 0.0, 0.0, 1.0};
//   Matrix<double> T_r(3, 3);
//   T_r.set_identity();
//   T_r(0,0) = cosRot;  T_r(0,1) = sinRot;
//   T_r(1,0) = -sinRot; T_r(1,1) = cosRot;

//   // Correct for the difference in image skew.
//   //  double skData[] = { a, 0.0, 0.0, c, 1.0, 0.0, 0.0, 0.0, 1.0 };
//   Matrix<double> T_sk(3, 3);
//   T_sk.set_identity();
//   T_sk(0,0) = a;
//   T_sk(1,0) = c;

//   // The correction matrix not considering translation
//   Matrix<double> T = T_s * T_r * T_sk;

//   /* For debugging */
//   //   cout << "The scale transform is:\n\n" <<  T_s << endl;
//   //   cout << "The rotation transform is:\n\n" <<  T_r << endl;
//   //   cout << "The skew transform is:\n\n" <<  T_sk << endl;
//   //   cout << "The partial image transform is:\n\n" <<  T << endl;
  
//   // Apply the transformation to the secondary image
//   //  adj_image = FixedTransform(secondary, T, primary.cols(), primary.rows());
//   adj_image = vw::free_homography_transform(secondary, T);

//   // TRANSLATION (in pixels)
//   //
//   // The absolute position of a MOC image is only known to within
//   // roughly half a kilometer, so the ephemeris based information is 
//   // inadequate for alignment.  Instead, we search across the whole 
//   // image using normalized cross correlation.

//   try {
//     // First, attempt to find the image translation and fine tuned
//     // alignment using keypoint homograhpy.  The hope is that keypoints
//     // will work better here because the aspect ratio of the images has
//     // now been corrected.

//     Matrix<double> keypoint_homography = KeypointAlign(primary, adj_image, fully_adj_image);
//     T = keypoint_homography * T;
    
//   } catch (KeypointAlignmentErr &e) {
//     cout << "Keypoint based translation alignment has failed.\n"
//          << "\tFalling back on (slow) normalized cross correlation alignment.\n";
    
//     // If keypoint alignment has failed, we make one final brute for
//     // attempt to align the images.  The user must manually select a
//     // point and kernel size in the -ephem_align.jpg image and place
//     // those values in the stereo.default.  The kernel in the
//     // adj_image so specified will be used for normalized cross
//     // correlation with the primary image.
//     write_image(out_prefix + "-ephem_align.jpg", adj_image);

//     // Pick an image patch for the translation search.
//     double cx = kernel_x;
//     double cy = kernel_y;
//     unsigned w = kernel_width;
//     unsigned h = kernel_height;
    
//     // Create an image of the kernel
//     ImageView<PixelGray<float> > kernel = vw::crop(adj_image, (unsigned)roundf(cx-w/2.0), (unsigned)roundf(cy-h/2.0),w,h);
    
//     // Find the x and y value of the maximum in the correlation image.
//     vil_image_view<float> k = kernel.vil_view();
//     vil_image_view<float> p = primary.vil_view();
    
//     vil_image_view<float> temp =  NormalizedCrossCorrelation2D(k,p);
//     ImageView<float > ccImage = temp;
    
//     // ml::imwrite(kernel, "kernel.tif");
//     // write_image("vwb/test-corr.pgm", ccImage);
    
//     unsigned int ypeak, xpeak;
//     align_max(ccImage, ypeak, xpeak);
    
//     cout << "\n\n(Xpeak, YPeak) = (" << xpeak << ", " << ypeak << ")"
//        << endl << endl;
    
//     /* NOTE : BUG
//      * 
//      * I'm not certain why we have to add an additional offset of 6 to
//      * the horizontal alignment, but this seems to be necessary to bring
//      * the aligned image into agreement with the output of imtransform
//      * on MATLAB.  I suspect that this might be a bug in the VXL image
//      * transformation code, but it's difficult to know for sure.  For
//      * now, here is the workaround. -mbroxton
//      */
//     double dx = xpeak - cx + (double)w/2 + 6;       
//     double dy = ypeak - cy + (double)h/2;
//     //    double tData[] = { 1.0, 0.0, dx, 0.0, 1.0, dy, 0.0, 0.0, 1.0 };
//     Matrix<double> T_t(3, 3);
//     T_t.set_identity();
//     T_t(0,2) = dx;  
//     T_t(1,2) = dy;

//     T = T_t * T;

//     // Take the fully compiled transformation and apply it now to the 
//     // original secondary image.
//     fully_adj_image = vw::fixed_homography_transform(secondary, T, primary.cols(), primary.rows());

//   }
  
//   //  cout << "The translation transform is:\n\n" <<  T_t << endl;
//   cout << "The full image transform is:\n\n" <<  T << endl;
//   return T;
// }

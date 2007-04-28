#ifndef __STEREO_SESSION_KEYPOINT_H__
#define __STEREO_SESSION_KEYPOINT_H__

#include "StereoSession.h"


// This abstract class overrides the default pre-processing behavior
// by adding keypoint alignment of the images.
class StereoSessionKeypoint: public StereoSession {
  
public:

  // Stage 1: Preprocessing
  //
  // Pre file is a pair of images.            ( ImageView<PixelT> )
  // Post file is a grayscale images.         ( ImageView<PixelGray<flaot> > )
  virtual void pre_preprocessing_hook(std::string const& input_file1, std::string const& input_file2,
                                      std::string & output_file1, std::string & output_file2);

};

#endif // __PINHOLE_STEREO_SESSION_H__

#ifndef RMAX_H__
#define RMAX_H__


#include <string>
#include <vw/FileIO/DiskImageResourcePNG.h>
#include <vw/Camera/CAHVORModel.h>
#include <vw/Camera/BundleAdjust.h>
#include <vw/Core.h>

using namespace vw; 

struct ImageInfo {
  std::string filename;
  double easting, northing;
  double heading, pitch, roll;
  double height, tilt;
  enum {
    LEFT=1, RIGHT=2, COLOR=3
  } camera;
};

void read_image_info( std::string const& filename, ImageInfo& info );

vw::camera::CAHVORModel rmax_image_camera_model( ImageInfo const& info,
                                                 vw::Vector3 const& position_correction,
                                                 vw::Vector3 const& pose_correction);

vw::camera::CAHVORModel rmax_image_camera_model( ImageInfo const& info );

vw::camera::CAHVORModel rmax_image_camera_model( std::string const& filename );

bool may_overlap( ImageInfo const& i1, ImageInfo const& i2 );
bool may_overlap( std::string const& file1, std::string const& file2 );

// -----

#endif // __RMAX_H__

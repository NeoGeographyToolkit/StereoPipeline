#ifndef RMAX_H__
#define RMAX_H__

#include <string>
#include <vw/FileIO/DiskImageResourcePNG.h>
#include <vw/Camera/CAHVORModel.h>

#define RMAX_GLOBAL_EASTING (582680)
#define RMAX_GLOBAL_NORTHING (4141480)

struct ImageInfo {
  std::string filename;
  double easting, northing;
  double heading, pitch, roll;
  double height, tilt;
  enum {
    LEFT=1, RIGHT=2, COLOR=3
  } camera;
};

void read_image_info( std::string const& filename, ImageInfo& info ) {
  vw_out(DebugMessage) << "Reading image info from " << filename << std::endl;;
  DiskImageResourcePNG png( filename );
  info.filename = filename;
  for( unsigned i=0; i<png.num_comments(); ++i ) {
    std::string const& key = png.get_comment(i).key;
    std::istringstream value( png.get_comment(i).text );
    if( key == "easting" ) value >> info.easting;
    else if( key == "northing" ) value >> info.northing;
    else if( key == "heading" ) value >> info.heading;
    else if( key == "pitch" ) value >> info.pitch;
    else if( key == "roll" ) value >> info.roll;
    else if( key == "height" ) value >> info.height;
    // Note: The key name says it's in radians, but it's clearly actually in degrees!
    else if( key == "tilt angle in radians" ) value >> info.tilt;
    else if( key == "which camera" ) {
      switch( png.get_comment(i).text[0] ) {
      case 'l': info.camera = ImageInfo::LEFT; break;
      case 'r': info.camera = ImageInfo::RIGHT; break;
      case 'c': info.camera = ImageInfo::COLOR; break;
      }
    }
    //else std::cout << "Unkown key: " << key << std::endl;
  }
}

vw::camera::CAHVORModel rmax_image_camera_model( ImageInfo const& info ) {
  CAHVORModel base;
  if( info.camera == ImageInfo::RIGHT ) {
    base.C = Vector3(    0.495334,   -0.003158,   -0.004600 );
    base.A = Vector3(    0.006871,   -0.000863,    0.999976 );
    base.H = Vector3( 1095.653400,    8.513659,  321.246801 );
    base.V = Vector3(   -7.076027, 1093.871625,  251.343361 );
    base.O = Vector3(    0.004817,    0.000619,    0.999988 );
    base.R = Vector3(    0.000000,   -0.235833,    0.180509 );
  }
  else {
    base.C = Vector3(   -0.495334,    0.003158,    0.004600 );
    base.A = Vector3(   -0.000000,    0.000000,    1.000000 );
    base.H = Vector3( 1095.186165,   -0.000000,  324.865834 );
    base.V = Vector3(    0.000000, 1095.678491,  250.561559 );
    base.O = Vector3(    0.000012,   -0.002199,    0.999998 );
    base.R = Vector3(    0.000000,   -0.230915,    0.128078 );
  }
  double cr=cos(info.roll*M_PI/180), sr=sin(info.roll*M_PI/180);
  double cp=cos(info.pitch*M_PI/180), sp=sin(info.pitch*M_PI/180);
  double cy=cos(info.heading*M_PI/180), sy=sin(info.heading*M_PI/180);
  Matrix3x3 roll, pitch, yaw, flip;
  roll(0,0)=cr; roll(0,1)=0;   roll(0,2)=-sr;
  roll(1,0)=0;  roll(1,1)=1;   roll(1,2)=0;
  roll(2,0)=sr; roll(2,1)=0;   roll(2,2)=cr;
  pitch(0,0)=1; pitch(0,1)=0;  pitch(0,2)=0;
  // XXX Pitch might have the wrong sign here....
  pitch(1,0)=0; pitch(1,1)=cp; pitch(1,2)=-sp;
  pitch(2,0)=0; pitch(2,1)=sp; pitch(2,2)=cp;
  yaw(0,0)=cy;  yaw(0,1)=-sy;  yaw(0,2)=0;
  yaw(1,0)=sy;  yaw(1,1)=cy;   yaw(1,2)=0;
  yaw(2,0)=0;   yaw(2,1)=0;    yaw(2,2)=1;
  flip(0,0)=1;  flip(0,1)=0;   flip(0,2)=0;
  flip(1,0)=0;  flip(1,1)=-1;  flip(1,2)=0;
  flip(2,0)=0;  flip(2,1)=0;   flip(2,2)=-1;
  Matrix3x3 ori = flip*yaw*pitch*roll;
  Vector3 pos( info.easting, info.northing, info.height );
  CAHVORModel cahvor;
  cahvor.C = pos - Vector3(RMAX_GLOBAL_EASTING,RMAX_GLOBAL_NORTHING,0) + ori*base.C;
  cahvor.A = ori*base.A;
  cahvor.H = ori*base.H;
  cahvor.V = ori*base.V;
  cahvor.O = ori*base.O;
  cahvor.R = base.R;
  return cahvor;
}

vw::camera::CAHVORModel rmax_image_camera_model( std::string const& filename ) {
  ImageInfo info;
  read_image_info( filename, info );
  return rmax_image_camera_model( info );
}

bool may_overlap( ImageInfo const& i1, ImageInfo const& i2 ) {
  const double diameter = 6; // approximate diameter in meters
  return hypot( i2.easting-i1.easting, i2.northing-i1.northing ) < diameter;
}

#endif // RMAX_H__

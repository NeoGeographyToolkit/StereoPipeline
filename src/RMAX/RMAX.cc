// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file RMAX.cc
///

#include <RMAX/RMAX.h>

#define RMAX_GLOBAL_EASTING (582680)
#define RMAX_GLOBAL_NORTHING (4141480)

void read_image_info( std::string const& filename, ImageInfo& info ) {
  vw::vw_out(vw::DebugMessage) << "Reading image info from " << filename << std::endl;;
  vw::DiskImageResourcePNG png( filename );
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

vw::camera::CAHVORModel rmax_image_camera_model( ImageInfo const& info,
                                                 vw::Vector3 const& position_correction,
                                                 vw::Vector3 const& pose_correction) {
  // Bundle Adjustment
  double r = info.roll + pose_correction[0];
  double p = info.pitch + pose_correction[1];
  double y = info.heading + pose_correction[2];

  camera::CAHVORModel base;
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
  double cr=cos(r*M_PI/180), sr=sin(r*M_PI/180);
  double cp=cos(p*M_PI/180), sp=sin(p*M_PI/180);
  double cy=cos(y*M_PI/180), sy=sin(y*M_PI/180);
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

  // Bundle Adjustment
  pos += position_correction;

  camera::CAHVORModel cahvor;
  cahvor.C = pos - Vector3(RMAX_GLOBAL_EASTING,RMAX_GLOBAL_NORTHING,0) + ori*base.C;
  cahvor.A = ori*base.A;
  cahvor.H = ori*base.H;
  cahvor.V = ori*base.V;
  cahvor.O = ori*base.O;
  cahvor.R = base.R;
  return cahvor;
}

vw::camera::CAHVORModel rmax_image_camera_model( ImageInfo const& info ) {
  return rmax_image_camera_model(info, Vector3(), Vector3());
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

bool may_overlap( std::string const& file1, std::string const& file2 ) {
  ImageInfo i1, i2;
  read_image_info(file1,i1);
  read_image_info(file2,i2);
  return may_overlap(i1,i2);
}


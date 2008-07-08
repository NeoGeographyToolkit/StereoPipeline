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

/// \file Ephemeris.h
///

#ifndef __MOCEPHEMERIS_H__
#define __MOCEPHEMERIS_H__

#include "MOC/Metadata.h"

#include <string>
#include <vw/Camera/OrbitingPushbroomModel.h>

// Use this routine to read MOC telemetry directly from a *.sup file
// on disk.
void ReadSupplementalEphemerisFile(std::string const& filename, MOCImageMetadata &image_data);

// MOC-specific SPICE functions can also be used to read telemetry
// from an archive of raw MGS spice information.
void load_moc_kernels();
void MOC_state(double begin_time, double end_time, double interval,
               std::vector<vw::Vector3> &position,
               std::vector<vw::Vector3> &velocity, 
               std::vector<vw::Quaternion<double> > &pose);

#endif // __MOCEPHEMERIS_H__

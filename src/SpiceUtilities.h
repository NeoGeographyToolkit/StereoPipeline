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

/// \file SpiceUtilities.h
///

#ifndef __SPICE_H__
#define __SPICE_H__

#include <list>
#include <vector>
#include <string>
#include <vw/Core/Exception.h> 
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>

namespace spice {

  VW_DEFINE_EXCEPTION(SpiceErr, vw::Exception);
  
  //  The NAIF SPICE ID value for various spacecraft and instruments
  const int NAIF_ID_MGS( -94    );    // Mars Global Surveyor
  const int NAIF_ID_MOC( -94031 );    // Mars Orbital Camera (MOC)
  const int NAIF_ID_MRO( -74    );    // Mars Reconnaisance Orbiter
  const int NAIF_ID_CLEMENTINE( -40    );    // Clementine

  // Function prototypes 
  void load_kernels(std::list<std::string> &kernels);
  void load_kernels(std::string const& kernels_file, std::string const& prefix="");
  double sclk_to_et(std::string sclk, int naif_id);
  std::string et_to_utc(double ephemeris_time);
  double utc_to_et(std::string const& utc);

  template <int ElemN>
  void kernel_param(std::string const& key, vw::Vector<double, ElemN> &value);
  void kernel_param(std::string const& key, double &value);

  void body_state(double begin_time, double end_time, double interval,
                  std::vector<vw::Vector3> &position,
                  std::vector<vw::Vector3> &velocity, 
                  std::vector<vw::Quaternion<double> > &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument);

  void body_state(double time,
                  vw::Vector3 &position,
                  vw::Vector3 &velocity, 
                  vw::Quaternion<double> &pose,
                  std::string const& spacecraft,
                  std::string const& reference_frame,
                  std::string const& planet,
                  std::string const& instrument);
  
} // namespace spice 

#endif // __SPICE_H__

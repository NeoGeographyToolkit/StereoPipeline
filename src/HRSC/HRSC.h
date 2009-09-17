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

/// \file HRSC.h
///

#ifndef __HRSC_METADATA_H__
#define __HRSC_METADATA_H__

#include <vector>
#include <string>

#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Math/EulerAngles.h>
#include <vw/Camera/OrbitingPushbroomModel.h>

class HRSCImageMetadata {

 public:
  HRSCImageMetadata(std::string const& filename); 
  virtual ~HRSCImageMetadata() {}
  
  /// Read the line times from an HRSC metadata file
  void read_line_times(std::string const& filename);

  /// Read the *.sup file
  void read_ephemeris_supplement(std::string const& filename);

  /// Read corrected (bundle adjusted) telemetry from the extori file
  void read_extori_file(std::string const& filename, std::string const& scanline);

  /// Returns a newly allocated camera model object of the appropriate
  /// type.  It is the responsibility of the user to later deallocate
  /// this camera model object or to manage it using some sort of smart
  /// pointer.
  vw::camera::CameraModel* camera_model();
  

  // Accessors
  int cols() const { return (int)m_width_pixels; }
  int rows() const { return (int)m_height_pixels; }
  //  double scan_duration() const;
  double crosstrack_summing() const { return m_crosstrack_summing; }
  double downtrack_summing() const { return m_crosstrack_summing; }
  double start_sample() const {return m_start_sample; }

private:

  // Constants 
  double m_focal_length;
  double m_along_scan_pixel_size; 
  double m_across_scan_pixel_size;

  // Parameters
  std::string m_filename; 
  double m_downtrack_summing;
  double m_crosstrack_summing;
  double m_width_pixels;
  double m_height_pixels;
  std::vector<double> m_line_times;
  double m_start_sample;
  double m_first_line_ephem_time;

  // Used when SUP or SPICE is used for ephemeris
  std::vector<vw::Vector3> m_ephem;
  std::vector<vw::Vector3> m_ephem_rate;
  std::vector<vw::Quaternion<double> > m_quat;

  // Used when the extori file is used to supply ephemeris
  std::vector<vw::Vector3> m_extori_ephem;
  std::vector<vw::Quaternion<double> > m_extori_quat;
  std::vector<double> m_extori_ephem_times;

  double m_t0_ephem;
  double m_dt_ephem;
  double m_t0_quat;
  double m_dt_quat;

  void parse_ephemeris_entry(std::vector<std::string> ephemerisData);
};

#endif // __HRSC_METADATA_H__

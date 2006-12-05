#ifndef __HRSC_METADATA_H__
#define __HRSC_METADATA_H__

#include <vector>
#include <string>
#include <vw/Math/Vector.h>
#include <vw/Math/Quaternion.h>
#include <vw/Camera/OrbitingPushbroomModel.h>
class HRSCImageMetadata {

 public:
  HRSCImageMetadata(std::string const& filename); 
  
  /// Read the line times from an HRSC metadata file
  void read_line_times(std::string const& filename);

  /// Read the *.sup file
  void read_ephemeris_supplement(std::string const& filename);
  
  /// Read corrected (bundle adjusted) telemetry from the extori file
  //  void read_extori(std::string const& filename);
  //  void read_spice_data();
  //  vw::camera::LinescanModel camera_model();
  vw::camera::OrbitingPushbroomModel camera_model();
  

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
  std::vector<vw::Vector3> m_ephem;
  std::vector<vw::Vector3> m_ephem_rate;
  std::vector<vw::Quaternion<double> > m_quat;
  double m_t0_ephem;
  double m_dt_ephem;
  double m_t0_quat;
  double m_dt_quat;

  void parse_ephemeris_entry(std::vector<std::string> ephemerisData);
};

#endif // __HRSC_METADATA_H__

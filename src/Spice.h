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
  
  // Function prototypes 
  void load_kernels(std::list<std::string> &kernels);
  double sclk_to_et(std::string sclk, int naif_id);
  std::string et_to_utc(double ephemeris_time);
  
void body_state(double begin_time, double end_time, double interval,
                std::vector<vw::Vector3> &position,
                std::vector<vw::Vector3> &velocity, 
                std::vector<vw::Quaternion<double> > &pose,
                std::string const& spacecraft,
                std::string const& reference_frame,
                std::string const& planet,
                std::string const& instrument);
  
} // namespace spice 

#endif // __SPICE_H__

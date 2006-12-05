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

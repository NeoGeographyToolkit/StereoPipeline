#include <boost/shared_ptr.hpp>

#include "HRSC/StereoSessionHRSC.h"

#include "HRSC/HRSC.h"
#include "stereo.h"
#include "file_lib.h"

using namespace vw;
using namespace vw::camera;

void StereoSessionHRSC::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                      boost::shared_ptr<camera::CameraModel> &cam2) {
    
  // Build the input prefix path by removing the filename suffix
  std::string in_prefix1 = prefix_from_filename(m_left_image_file);
  std::string in_prefix2 = prefix_from_filename(m_right_image_file);
  
  boost::shared_ptr<StereoImageMetadata> metadata1, metadata2;
  
  // Initialize the HRSC metadata object
  HRSCImageMetadata hrsc_metadata1(m_left_image_file);
  HRSCImageMetadata hrsc_metadata2(m_right_image_file);
  try {
    std::cout << "Loading HRSC Metadata.\n";
    hrsc_metadata1.read_line_times(in_prefix1 + ".txt");
    hrsc_metadata2.read_line_times(in_prefix2 + ".txt");
    hrsc_metadata1.read_ephemeris_supplement(in_prefix1 + ".sup");
    hrsc_metadata2.read_ephemeris_supplement(in_prefix2 + ".sup");
    
    if (m_extra_argument1.size() != 0) 
      hrsc_metadata1.read_extori_file(m_extra_argument1,m_extra_argument3);
    if (m_extra_argument2.size() != 0) 
      hrsc_metadata2.read_extori_file(m_extra_argument2,m_extra_argument4);
  } catch (IOErr &e) {
    std::cout << "An error occurred when loading HRSC metadata:\n\t" << e.what();
    std::cout << "\nExiting.\n\n";
    exit(1);
  }

  // Save the recitified camera model in the return value.
  cam1 = boost::shared_ptr<camera::CameraModel>(hrsc_metadata1.camera_model());
  cam2 = boost::shared_ptr<camera::CameraModel>(hrsc_metadata2.camera_model());
}


#include <boost/shared_ptr.hpp>

#include "MOC/StereoSessionMOC.h"

#include "MOC/Ephemeris.h"
#include "MOC/Metadata.h"
#include "MOC/MOLA.h"
#include "Spice.h" 
#include "stereo.h"
#include "file_lib.h"

using namespace vw;
using namespace vw::camera;

void StereoSessionMOC::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                      boost::shared_ptr<camera::CameraModel> &cam2) {
    
  MOCImageMetadata moc_metadata1(m_left_image_file);
  MOCImageMetadata moc_metadata2(m_right_image_file);
  
  // Read in the tabulated description entry
  std::string description_tab_filename = "description.tab"; // FIXME: Needs to get this setting from the command line.
  std::cout << "Attempting to read data from " << description_tab_filename << ".\n";
  moc_metadata1.read_tabulated_description(description_tab_filename);
  moc_metadata2.read_tabulated_description(description_tab_filename);
  moc_metadata1.write_viz_site_frame(m_out_prefix);
  
  try {
    std::cout << "Attempting to read MOC telemetry from supplementary ephemeris files... " << std::flush;
    moc_metadata1.read_ephemeris_supplement(m_left_camera_file);
    moc_metadata2.read_ephemeris_supplement(m_right_camera_file);
  } catch (EphemerisErr &e) {
    std::cout << "Failed to open the supplementary ephemeris file:\n\t";
    std::cout << e.what() << "\n";
    std::cout << "\tWarning: Proceeding without supplementary ephemeris information.\n";
  }
  
  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.  
  try {
    std::cout << "Attempting to read MOC telemetry from SPICE kernels... " << std::flush;
    load_moc_kernels();
    moc_metadata1.read_spice_data();
    moc_metadata2.read_spice_data();
    std::cout << "success.\n";
  } catch (spice::SpiceErr &e) {
    std::cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }

  // Save the recitified camera model in the return value.
  cam1 = boost::shared_ptr<camera::CameraModel>(moc_metadata1.camera_model());
  cam2 = boost::shared_ptr<camera::CameraModel>(moc_metadata2.camera_model());
}


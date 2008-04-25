#include <boost/shared_ptr.hpp>

#include "MOC/StereoSessionMOC.h"

#include "MOC/Ephemeris.h"
#include "MOC/Metadata.h"
#include "MOC/MOLA.h"
#include "SpiceUtilities.h" 
#include "stereo.h"
#include "file_lib.h"

using namespace vw;
using namespace vw::camera;

void StereoSessionMOC::camera_models(boost::shared_ptr<camera::CameraModel> &cam1,
                                      boost::shared_ptr<camera::CameraModel> &cam2) {
    
  MOCImageMetadata moc_metadata1(m_left_image_file);
  MOCImageMetadata moc_metadata2(m_right_image_file);
  
  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.  
  try {
    load_moc_kernels();
  } catch (spice::SpiceErr &e) {
    std::cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }

  // Read in the tabulated description entry
  std::string description_tab_filename = "description.tab"; // FIXME: Needs to get this setting from the command line.
  std::cout << "Reading data from " << description_tab_filename << ".\n";
  moc_metadata1.read_tabulated_description(description_tab_filename);
  moc_metadata2.read_tabulated_description(description_tab_filename);
  moc_metadata1.write_viz_site_frame(m_out_prefix);
  std::cout << "Image 1 TAB ET: " << moc_metadata1.ephemeris_time() << "\n";
  std::cout << "Image 2 TAB ET: " << moc_metadata2.ephemeris_time() << "\n";
  
  try {
    std::cout << "Reading MOC telemetry from supplementary ephemeris files.\n";
    moc_metadata1.read_ephemeris_supplement(m_left_camera_file);
    moc_metadata2.read_ephemeris_supplement(m_right_camera_file);
    std::cout << "Image 1 SUP ET: " << moc_metadata1.ephemeris_time() << "\n";
    std::cout << "Image 2 SUP ET: " << moc_metadata2.ephemeris_time() << "\n";
  } catch (EphemerisErr &e) {
    std::cout << "Failed to open the supplementary ephemeris file:\n\t";
    std::cout << e.what() << "\n";
    std::cout << "\tWarning: Proceeding without supplementary ephemeris information.\n";
  }

  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.  
  try {
    std::cout << "Attempting to read MOC telemetry from SPICE kernels... " << std::flush;
    moc_metadata1.read_spice_data();
    moc_metadata2.read_spice_data();
    std::cout << "Image 1 SPICE ET: " << moc_metadata1.ephemeris_time() << "\n";
    std::cout << "Image 2 SPICE ET: " << moc_metadata2.ephemeris_time() << "\n";
    std::cout << "success.\n";
  } catch (spice::SpiceErr &e) {
    std::cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }
  
  // Save the recitified camera model in the return value.
  cam1 = boost::shared_ptr<camera::CameraModel>(moc_metadata1.camera_model());
  cam2 = boost::shared_ptr<camera::CameraModel>(moc_metadata2.camera_model());
}


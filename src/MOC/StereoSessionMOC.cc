#include <boost/shared_ptr.hpp>

#include "MOC/StereoSessionMOC.h"

#include "MOC/Ephemeris.h"
#include "MOC/Metadata.h"
#include "MOC/MOLA.h"
#include "SpiceUtilities.h" 
#include "stereo.h"

using namespace vw;
using namespace vw::camera;


boost::shared_ptr<vw::camera::CameraModel> StereoSessionMOC::camera_model(std::string image_file, 
                                                                          std::string camera_file) {
  MOCImageMetadata moc_metadata(image_file);
  
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
  moc_metadata.read_tabulated_description(description_tab_filename);
  moc_metadata.write_viz_site_frame(m_out_prefix);
  std::cout << image_file << " TAB ET: " << moc_metadata.ephemeris_time() << "\n";
  
  try {
    std::cout << "Reading MOC telemetry from supplementary ephemeris files.\n";
    moc_metadata.read_ephemeris_supplement(camera_file);
    std::cout << camera_file << " ET: " << moc_metadata.ephemeris_time() << "\n";
  } catch (EphemerisErr &e) {
    std::cout << "Failed to open the supplementary ephemeris file:\n\t";
    std::cout << e.what() << "\n";
    std::cout << "\tWarning: Proceeding without supplementary ephemeris information.\n";
  }

  // If the spice kernels are available, try to use them directly to
  // read in MOC telemetry.  
  try {
    std::cout << "Attempting to read MOC telemetry from SPICE kernels... \n";
    moc_metadata.read_spice_data();
    std::cout << image_file << " SPICE ET: " << moc_metadata.ephemeris_time() << "\n";
  } catch (spice::SpiceErr &e) {
    std::cout << "Warning: an error occurred when reading SPICE information.  Falling back to *.sup files\n";
  }
  
  return boost::shared_ptr<camera::CameraModel>(moc_metadata.camera_model());
}


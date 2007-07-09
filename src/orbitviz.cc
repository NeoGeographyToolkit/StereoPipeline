#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <fstream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Cartography.h>
#include <vw/Math/EulerAngles.h>
using namespace vw;

#include "stereo.h"
#include "nff_terrain.h"
#include "Spice.h"

// Returns: A Vector3 containing the euler angles [phi, omega, kappa]
inline Vector3 rotation_matrix_to_euler_xyz(const Matrix<double,3,3> rotation_matrix) {
  double omega = asin(rotation_matrix(0,2));
  double phi = acos(rotation_matrix(2,2) / cos(omega));
  double kappa = acos(rotation_matrix(0,0) / cos(omega));
  return Vector3(phi, omega, kappa);
}

// Returns: A Vector3 containing the euler angles [phi, omega, kappa]
inline Vector3 rotation_matrix_to_euler_yxz(const Matrix<double,3,3> rotation_matrix) {
  double cos_phi = sqrt (1 - rotation_matrix(2,1) * rotation_matrix(2,1));
  double phi = atan2(rotation_matrix(2,1), cos_phi);
  double omega = atan2(-rotation_matrix(2,0), rotation_matrix(2,2));
  double kappa = atan2(-rotation_matrix(0,1), rotation_matrix(1,1));
  return Vector3(omega, phi, kappa);
}

// Returns: A Vector3 containing the euler angles [phi, omega, kappa]
inline Vector3 rotation_matrix_to_euler_zxy(const Matrix<double,3,3> rotation_matrix) {
  double sin_phi = -rotation_matrix(1,2);
  double cos_phi = sqrt (1 - sin_phi*sin_phi);
  double phi = atan2(sin_phi, cos_phi);
  double omega = atan2(rotation_matrix(0,2), rotation_matrix(2,2));
  double kappa = atan2(rotation_matrix(1,0), rotation_matrix(1,1));
  return Vector3(kappa, phi, omega);
}


void append_model(std::ofstream &output_file, double ephemeris_time, double scale, 
                  std::string spacecraft, std::string reference_frame, std::string planet, std::string instrument, int index) {

  Vector3 position, velocity;
  Quaternion<double> pose;
  spice::body_state(ephemeris_time,
                    position, velocity, pose,
                    spacecraft, reference_frame, planet, instrument);

  cartography::XYZtoLonLatFunctor<double> func;
  Vector3 lon_lat_alt = func(position);

  Matrix3x3 correction_rot = math::euler_to_rotation_matrix((90-lon_lat_alt(1))*M_PI/180, (90+lon_lat_alt(0))*M_PI/180, 0, "xzy");
  Vector3 angles = rotation_matrix_to_euler_zxy(pose.rotation_matrix()*correction_rot);
  std::cout << "Angles: " << angles << "\n";
  double heading = angles(0)*180/M_PI, tilt = angles(1)*180/M_PI, roll = angles(2)*180/M_PI;

  // The master KML document needs a lookat directive.  We point it at
  // the location of the first target we are tracking.
  if (index == 0) {
    output_file << "<LookAt>\n" 
                << "  <longitude>" << lon_lat_alt(0) << "</longitude>\n"
                << "  <latitude> " << lon_lat_alt(1) << "</latitude>\n"
                << "  <altitude> " << lon_lat_alt(2)-1737400 << "</altitude>\n" // 1737400 is the lunar radius
                << "  <range> " << 1e6 << "</range>\n"
                << "  <tilt>" << 0 << "</tilt>\n"
                << "  <heading>" << 0 << "</heading>\n"
                << "</LookAt>\n";
  }

  output_file << "<Placemark>\n"
              << "<name>Ephemeris Time: " << ephemeris_time << "</name>\n"
              << "<LookAt>\n" 
              << "  <longitude>" << lon_lat_alt(0) << "</longitude>\n"
              << "  <latitude> " << lon_lat_alt(1) << "</latitude>\n"
              << "  <altitude> " << lon_lat_alt(2)-1737400 << "</altitude>\n" // 1737400 is the lunar radius
              << "  <range> " << 1e6 << "</range>\n"
              << "  <tilt>" << 0 << "</tilt>\n"
              << "  <heading>" << 0 << "</heading>\n"
              << "</LookAt>\n"
              << "<Model id=\"model_" << ephemeris_time << "\">\n"
              << "  <altitudeMode>absolute</altitudeMode>\n"
              << "  <Location>\n"
              << "     <longitude>" << lon_lat_alt(0) << "</longitude>\n" 
              << "     <latitude> " << lon_lat_alt(1) << "</latitude>\n"
              << "     <altitude> " << lon_lat_alt(2)-1737400 << "</altitude>\n"
              << "  </Location>\n"
              << "  <Orientation>\n"
              << "		 <heading>" << heading << "</heading>\n"
              << "     <tilt>" << tilt << "</tilt>\n"
              << "     <roll>" << roll << "</roll>\n"
              << "  </Orientation>\n"
              << "  <Scale> \n"
              << "    <x>" << 3000*scale << "</x>\n"
              << "    <y>" << 3000*scale << "</y>\n"
              << "    <z>" << 3000*scale << "</z>\n"
              << "  </Scale>\n"
              << "  <Link>\n"
              << "    <href>/Users/mbroxton/Desktop/models/axis.dae</href>\n"
              << "  </Link>\n"
              << "</Model>\n"
              << "</Placemark>\n";
}

void write_orbital_reference_model(std::string filename,
                                   std::vector<double> ephemeris_times,
                                   double scale,
                                   std::string spacecraft, std::string reference_frame, std::string planet, std::string instrument) {

  std::cout << "Writing Orbital Visualization VRML file to \"" << filename << "\".\n";

  // Open the file and verify that everything is ok.
  std::ofstream output_file;
  output_file.open(filename.c_str(), std::ios::out);
  if (!output_file.good()) 
    vw_throw(IOErr() << "An error occured while opening the Orbital Reference VRML file for writing.");

  output_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" 
              << "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n" 
              << "<Document>\n" 
              << "<name>KmlFile</name>\n";

  for (int i = 0; i < ephemeris_times.size(); ++i) 
    append_model(output_file, ephemeris_times[i], scale, spacecraft, reference_frame, planet, instrument,i);

  output_file << "</Document>\n"
              << "</kml>\n\n";

  output_file.close();
}

int main( int argc, char *argv[] ) {
  set_debug_level(VerboseDebugMessage+11);
  
  int debug_level;
  std::vector<double> ephem_times;
  std::vector<std::string> utc_times;
  std::string output_file;
  std::string kernels_file;
  std::string kernels_prefix;
  double scale;
  std::string spacecraft;
  std::string reference_frame;
  std::string planet;
  std::string instrument;
  
  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("kernels-file,k", po::value<std::string>(&kernels_file)->default_value("kernels.txt"), "Supply a file containing a list of spice kernels to load")
    ("scale", po::value<double>(&scale)->default_value(1.0), "Scale the size of the coordinate axes by this amount")
    ("kernels-prefix,p", po::value<std::string>(&kernels_prefix)->default_value(""), "Supply a path to prepend to the paths to kernels listed in the supplied kernel list file")
    ("output-file,o", po::value<std::string>(&output_file)->default_value("orbitviz.kml"), "Explicitly specify the output file")
    ("utc-times,u", po::value<std::vector<std::string> >(&utc_times), "Specify a set of UTC times to plot in KML")
    ("ephemeris-times,e", po::value<std::vector<double> >(&ephem_times), "Specify a set of UTC times to plot in KML")
    ("spacecraft", po::value<std::string>(&spacecraft)->default_value(""), "")
    ("reference-frame", po::value<std::string>(&reference_frame)->default_value(""), "")
    ("planet", po::value<std::string>(&planet)->default_value(""), "")
    ("instrument", po::value<std::string>(&instrument)->default_value(""), "");

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).run(), vm );
  po::notify( vm );

  // Set the Vision Workbench debug level and cache
  set_debug_level(debug_level);

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  }

  if ( vm.count("kernels-file") )
    spice::load_kernels(kernels_file, kernels_prefix);
  else {
    std::cout << "You must supply a file containing a list of SPICE kernels to load!\n";
    std::cout << "Exiting\n\n";
    exit(0);
  }
  const char* resource_path = "/irg/projects/MOC/resources/OrbitViz";

  if ( vm.count("ephemeris-times") )
    for (int i = 0; i < ephem_times.size(); ++i) 
      std::cout << "EPHEM TIME: " << ephem_times[i] << "\n";

  if ( vm.count("utc-times") )
    for (int i = 0; i < utc_times.size(); ++i) {
      std::cout << "UTC TIME: " << utc_times[i] << "\n";
      ephem_times.push_back(spice::utc_to_et(utc_times[i]));
      std::cout << "      ET: " << spice::utc_to_et(utc_times[i]) << "\n";
    }

  if (ephem_times.size() == 0) {
    std::cout << "You did not specify any times.  \nExiting.\n\n";
    exit(0);
  }

  write_orbital_reference_model(output_file, ephem_times, scale, spacecraft, reference_frame, planet, instrument);

  return 0;
}

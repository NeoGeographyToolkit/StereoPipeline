#include <vw/Math/EulerAngles.h>
#include <vw/Cartography/PointImageManipulation.h>
using namespace vw;

#include "KML.h"
#include "stereo.h"
#include "nff_terrain.h"

// Returns: A Vector3 containing the euler angles [phi, omega, kappa] inline
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




void KMLStateVectorViz::append_body_state(std::string name, Vector3 position, Quaternion<double> pose) {

  cartography::XYZtoLonLatRadFunctor func;
  Vector3 lon_lat_alt = func(position);
  
  // Converts from GE's default rotation which is oriented over the
  // site frame to a standard planetocentric rotation.
  Matrix3x3 correction_rot = vw::math::euler_to_rotation_matrix((90-lon_lat_alt(1))*M_PI/180, (90+lon_lat_alt(0))*M_PI/180, 0, "xzy");

  std::cout << "Adding to KML: " << pose.rotation_matrix() << "\n";

  Vector3 angles = rotation_matrix_to_euler_zxy(pose.rotation_matrix()*correction_rot);
  double heading = angles(0)*180/M_PI, tilt = angles(1)*180/M_PI, roll = angles(2)*180/M_PI;

  // The master KML document needs a lookat directive.  We point it at
  // the location of the first target we are tracking.
  if (index == 0) {
    m_output_file << "<LookAt>\n" 
                  << "  <longitude>" << lon_lat_alt(0) << "</longitude>\n"
                  << "  <latitude> " << lon_lat_alt(1) << "</latitude>\n"
                  << "  <altitude> " << lon_lat_alt(2)-1737400 << "</altitude>\n" // 1737400 is the lunar radius
                  << "  <range> " << 1e6 << "</range>\n"
                  << "  <tilt>" << 0 << "</tilt>\n"
                  << "  <heading>" << 0 << "</heading>\n"
                  << "</LookAt>\n";
  }

  m_output_file << "<Placemark>\n"
                << "<name>" << name << "</name>\n"
                << "<LookAt>\n" 
                << "  <longitude>" << lon_lat_alt(0) << "</longitude>\n"
                << "  <latitude> " << lon_lat_alt(1) << "</latitude>\n"
                << "  <altitude> " << lon_lat_alt(2)-1737400 << "</altitude>\n" // 1737400 is the lunar radius
                << "  <range> " << 1e6 << "</range>\n"
                << "  <tilt>" << 0 << "</tilt>\n"
                << "  <heading>" << 0 << "</heading>\n"
                << "</LookAt>\n"
                << "<Model id=\"model_" << name << "\">\n"
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
                << "    <x>" << 3000*m_scale << "</x>\n"
                << "    <y>" << 3000*m_scale << "</y>\n"
                << "    <z>" << 3000*m_scale << "</z>\n"
                << "  </Scale>\n"
                << "  <Link>\n"
                << "    <href>/Users/mbroxton/Desktop/models/axis.dae</href>\n"
                << "  </Link>\n"
                << "</Model>\n"
                << "</Placemark>\n";
}

KMLStateVectorViz::KMLStateVectorViz(std::string filename, std::string name, double scale) {
  
  std::cout << "Writing Orbital Visualization VRML file to \"" << filename << "\".\n";
  
  // Open the file and verify that everything is ok.
  m_output_file.open(filename.c_str(), std::ios::out);
  if (!m_output_file.good()) 
    vw_throw(IOErr() << "An error occured while opening the Orbital Reference VRML file for writing.");
  
  m_output_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" 
                << "<kml xmlns=\"http://earth.google.com/kml/2.1\">\n" 
                << "<Document>\n" 
                << "<name>" << name << "</name>\n";

  m_scale = scale;
}

KMLStateVectorViz::~KMLStateVectorViz() {
  this->close();
}

void KMLStateVectorViz::close() {
  if (m_output_file.good()) {
    m_output_file << "</Document>\n"
                  << "</kml>\n\n";
    m_output_file.close();
  }
}


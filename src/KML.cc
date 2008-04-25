#include <vw/Math/EulerAngles.h>
#include <vw/Cartography/PointImageManipulation.h>
using namespace vw;

#include "KML.h"
#include "stereo.h"
#include "nff_terrain.h"

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
                  << "  <altitude> " << lon_lat_alt(2)-3396200 << "</altitude>\n" // 1737400 is the lunar radius
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
                << "  <altitude> " << lon_lat_alt(2)-3396200 << "</altitude>\n" // 1737400 is the lunar radius
                << "  <range> " << 1e6 << "</range>\n"
                << "  <tilt>" << 0 << "</tilt>\n"
                << "  <heading>" << 0 << "</heading>\n"
                << "</LookAt>\n"
                << "<Model id=\"model_" << name << "\">\n"
                << "  <altitudeMode>absolute</altitudeMode>\n"
                << "  <Location>\n"
                << "     <longitude>" << lon_lat_alt(0) << "</longitude>\n" 
                << "     <latitude> " << lon_lat_alt(1) << "</latitude>\n"
                << "     <altitude> " << lon_lat_alt(2)-3396200 << "</altitude>\n"
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
                << "    <href>/Users/mbroxton/projects/axis.dae</href>\n"
                << "  </Link>\n"
                << "</Model>\n"
                << "</Placemark>\n";
}

KMLStateVectorViz::KMLStateVectorViz(std::string filename, std::string name, double scale) {
  
  std::cout << "Writing Orbital Visualization file to \"" << filename << "\".\n";
  
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


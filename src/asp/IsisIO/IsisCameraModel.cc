// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <vw/Cartography/PointImageManipulation.h>

#include <asp/IsisIO/IsisCameraModel.h>

// Isis Headers
#include <Cube.h>
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <Pvl.h>
#include <SerialNumber.h>

using namespace vw;
using namespace vw::camera;


IsisCameraModel::IsisCameraModel(std::string cube_filename) {
  Isis::Cube* cube_ptr = new Isis::Cube;
  m_isis_cube = cube_ptr;

  cube_ptr->Open(cube_filename);

  if ( !(cube_ptr->IsOpen()) )
      vw_throw(IOErr() << "IsisCameraModel: Could not open cube file: \"" << cube_filename << "\".");

  try {
    // Generate a camera model for this Cube
    m_isis_camera_ptr = cube_ptr->Camera();
    Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

    // Determine the start and end time for the image.
    this->set_image(0,0);
    double start_time = cam->EphemerisTime();
    this->set_image(0,cam->Lines());
    double end_time = cam->EphemerisTime();
    vw_out(VerboseDebugMessage,"stereo") << "\t     Time range: [" << start_time << " " << end_time << "]  " << (end_time-start_time) << "\n";
    vw_out(VerboseDebugMessage,"stereo") << "\t     Existing range : [" << cam->CacheStartTime() << " " << cam->CacheEndTime() << "]  " << ( cam->CacheEndTime() - cam->CacheStartTime() ) << "\n";
    if (end_time > start_time) {
      m_max_ephemeris = end_time;
      m_min_ephemeris = start_time;
    } else {
      m_max_ephemeris = start_time;
      m_min_ephemeris = end_time;
    }
    // Cache important values (such as sun angles, orientations, etc)
    //     try {
    //       std::cout << "\tCaching SPICE information..." << std::flush;
    //       cam->CreateCache(start_time, end_time, cam->Lines());
    //       std::cout << " done.\n";
    //     } catch (Isis::iException &e) {
    //       std::cout << " failed.\n\t" << e.what() << "\n";
    //     }

  } catch (Isis::iException &e) {
    m_isis_camera_ptr = 0;
    vw_throw(IOErr() << "IsisCameraModel: failed to instantiate a camera model from " << cube_filename << ". " << e.what());
  }
}

IsisCameraModel::~IsisCameraModel() {
  if (m_isis_cube) {
    static_cast<Isis::Cube*>(m_isis_cube)->Close();
    delete static_cast<Isis::Cube*>(m_isis_cube);
  }
}

// Calling cam->SetImage() is actually very expensive, so we avoid
// calling it twice in a row with the same line/sample pair.
void IsisCameraModel::set_image(double sample, double line) const {
  if (m_current_line != line || m_current_sample != sample) {
    Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);
    // ISIS indexes the top left as (1,1) while VW uses (0,0)
    cam->SetImage(sample+1, line+1);
    m_current_line = line;
    m_current_sample = sample;
  }
}


Vector2 IsisCameraModel::point_to_pixel(Vector3 const& point) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  // Convert into "Universal Ground" coordinates
  Vector3 lon_lat_radius = cartography::xyz_to_lon_lat_radius(point);

  // Set the current "active" pixel for the upcoming computation
  cam->SetUniversalGround(lon_lat_radius[1], lon_lat_radius[0], lon_lat_radius[2]);

  //   std::cout << "LON LAT RAD: " << lon_lat_radius << " ---> ";
  //   std::cout << cam->Line() << " " << cam->Sample() << "\n"
  // ISIS indexes the top left as (1,1) while VW uses (0,0)
  return Vector2(cam->Sample()-1, cam->Line()-1);
}

Vector3 IsisCameraModel::pixel_to_vector (Vector2 const& pix) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  // This is the foolproof method for setting the current pixel of
  // interest.  However, this method is very slow because it check for
  // an intersection with the planetary datum on every call.
  this->set_image(pix[0],pix[1]);

  // Compute ground point
  double p[3];
  cam->InstrumentPosition(p);
  Vector3 instrument_pt(p[0],p[1],p[2]);

  // Compute vector from the spacecraft to the ground intersection point
  cam->Coordinate(p);
  Vector3 ground_pt(p[0],p[1],p[2]);
  return normalize(ground_pt - instrument_pt);


//   Quaternion<double> look_transform = this->camera_pose(pix);
//   return inverse(look_transform).rotate(Vector3(0,0,1));
}

Vector3 IsisCameraModel::camera_center(Vector2 const& pix ) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);

  // This is the foolproof method for setting the current pixel of
  // interest.  However, this method is very slow because it check for
  // an intersection with the planetary datum on every call.
  this->set_image(pix[0],pix[1]);

  // On the other hand, this call is *much* faster because it directly
  // updates the DetectorMap object and the ephemeris time without
  // engaging any of the other "heavy machinery" involved with a call
  // to cam->SetImage().  However, this is violating some abstraction
  // barriers, and it may not be correct for all camera models.
  //  cam->DetectorMap()->SetParent(pix[0],pix[1]);
  double pos[3];
  cam->InstrumentPosition(pos);
  return Vector3(pos[0]*1000,pos[1]*1000,pos[2]*1000);
}

Quaternion<double> IsisCameraModel::camera_pose(Vector2 const& pix ) const {
  Isis::Camera* cam  = static_cast<Isis::Camera*>(m_isis_camera_ptr);
  this->set_image(pix[0],pix[1]);

  // Convert from instrument frame --> J2000 frame --> Mars body-fixed frame
  std::vector<double> rot_inst = cam->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = cam->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));

  return Quaternion<double>(R_inst*inverse(R_body));
}


int IsisCameraModel::getLines( void ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>(m_isis_camera_ptr);
  return cam->Lines();
}

int IsisCameraModel::getSamples( void ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>(m_isis_camera_ptr);
  return cam->Samples();
}

std::string IsisCameraModel::serial_number( void ) const {
  Isis::Cube* cube = static_cast<Isis::Cube*>(m_isis_cube);
  Isis::Pvl* label = cube->Label();
  std::string serial = Isis::SerialNumber::Compose(*label,true);
  return serial;
}

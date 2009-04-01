#include <vw/Cartography/PointImageManipulation.h>
#include <Isis/IsisAdjustCameraModel.h>

// Isis Headers
#include <Cube.h>
#include <Camera.h>
#include <CameraDetectorMap.h>
#include <CameraDistortionMap.h>
#include <CameraFocalPlaneMap.h>
#include <CameraGroundMap.h>

using namespace vw;
using namespace vw::camera;

//-------------------------------------------------------------------------
//  Constructors / Deconstructor
//-------------------------------------------------------------------------
IsisAdjustCameraModel::IsisAdjustCameraModel( std::string cube_filename,
                                              boost::shared_ptr<VectorEquation> position_func,
                                              boost::shared_ptr<QuaternionEquation> pose_func ) : 
  IsisCameraModel( cube_filename ),
  m_position_func( position_func ),
  m_pose_func( pose_func ) {

  m_position_func->set_time_offset( (m_max_ephemeris + m_min_ephemeris)/2 );
  m_pose_func->set_time_offset( (m_max_ephemeris + m_min_ephemeris)/2 );

  Isis::Cube* cube = static_cast<Isis::Cube*>(m_isis_cube);
  Isis::AlphaCube* alpha_cube_ptr = new Isis::AlphaCube(*(cube->Label()));
  m_isis_alpha_cube = alpha_cube_ptr;
}

IsisAdjustCameraModel::~IsisAdjustCameraModel() {
  if (m_isis_alpha_cube) {
    delete static_cast<Isis::AlphaCube*>(m_isis_alpha_cube);
  }
}

//-------------------------------------------------------------------------
//  Traditional Camera Routines
//-------------------------------------------------------------------------

Vector2 IsisAdjustCameraModel::point_to_pixel( Vector3 const& point) const {
  
  VW_ASSERT(m_min_ephemeris == m_max_ephemeris, 
            NoImplErr() << "IsisAdjustCameraModel::point_to_pixel() does not yet support linescan imagers.\n");

  // point_to_pixel() is not yet able to handle linescan cameras,
  // however for framing cameras we can pick a fixed ephemeris time
  // and feed it into the point_to_mm_time() routine.
  //
  // We haven't worked out how to do this properly for linescan
  // imagers, but a least-squares method seems inevitable. 
  Vector3 fixed_mm_time(0,0,m_min_ephemeris);
  Vector3 mm_time = this->point_to_mm_time( fixed_mm_time, point );  
  return this->mm_time_to_pixel(mm_time);
}

Vector3 IsisAdjustCameraModel::pixel_to_vector( Vector2 const& pix ) const {
  Vector3 mm_time = this->pixel_to_mm_time( pix );  
  return this->mm_time_to_vector( mm_time );
}

Vector3 IsisAdjustCameraModel::camera_center( Vector2 const& pix )  const {
  Vector3 mm_time = this->pixel_to_mm_time( pix );
  return this->camera_center( mm_time );
}

Quaternion<double> IsisAdjustCameraModel::camera_pose( Vector2 const& pix ) const {
  Vector3 mm_time = this->pixel_to_mm_time( pix );
  return this->camera_pose( mm_time );
}

int IsisAdjustCameraModel::getLines( void ) const {
  return IsisCameraModel::getLines();
}

int IsisAdjustCameraModel::getSamples( void ) const {
  return IsisCameraModel::getSamples();
}

std::string IsisAdjustCameraModel::serial_number(void) const {
  return IsisCameraModel::serial_number();
}

//-------------------------------------------------------------------------
//  Non-Traditional Camera Routines
//-------------------------------------------------------------------------

Vector3 IsisAdjustCameraModel::pixel_to_mm_time( Vector2 const& pix ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_image( pix[0], pix[1] );
  
  // Now finding the undistorted mms that make up the pixels
  Isis::CameraDistortionMap* distortMap = cam->DistortionMap();
  return Vector3( distortMap->UndistortedFocalPlaneX(),
		  distortMap->UndistortedFocalPlaneY(),
		  cam->EphemerisTime() );
}

Vector2 IsisAdjustCameraModel::mm_time_to_pixel( Vector3 const& mm_time ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  Isis::AlphaCube* alpha_cube = static_cast<Isis::AlphaCube*>(m_isis_alpha_cube);

  this->set_time( mm_time[2] );  
  Isis::CameraDistortionMap* distortMap = cam->DistortionMap();
  distortMap->SetUndistortedFocalPlane(mm_time[0],mm_time[1]);

  // This code (commented out here) is a little bit more robust to
  // changes in the ISIS API and variation in different types of ISIS
  // cubes (map projected cubes, for example...).  However, it's MUCH
  // SLOWER, so I've hard coded the functionality to a certain extent
  // below.  We should take a careful look at this if we ever want to
  // start playing with map projected ISIS cubes. 
  //
  //   double x = distortMap->UndistortedFocalPlaneX();
  //   double y = distortMap->UndistortedFocalPlaneY();
  //   double z = distortMap->UndistortedFocalPlaneZ();
  //   // This ugly bit of code is necessary to kick the ISIS camera into
  //   // propagating our changes to the focal plane position up through
  //   // the chain of groundmap->distortionmap->detectormap->etc...
  //   cam->GroundMap()->SetFocalPlane(x,y,z);
  //   cam->SetUniversalGround(cam->UniversalLatitude(), cam->UniversalLongitude());

  // This is the much faster code that is less robust...
  double focalPlaneX = distortMap->FocalPlaneX();
  double focalPlaneY = distortMap->FocalPlaneY();

  Isis::CameraFocalPlaneMap* focalMap = cam->FocalPlaneMap();
  focalMap->SetFocalPlane(focalPlaneX,focalPlaneY);
  double detectorSample = focalMap->DetectorSample();
  double detectorLine = focalMap->DetectorLine();

  Isis::CameraDetectorMap* detectorMap = cam->DetectorMap();
  detectorMap->SetDetector(detectorSample,detectorLine);
  double parentSample = detectorMap->ParentSample();
  double parentLine = detectorMap->ParentLine();

  double childSample = alpha_cube->BetaSample(parentSample);
  double childLine = alpha_cube->BetaLine(parentLine);
  // Remember: ISIS indexes top left as (1,1) while VW uses (0,0)
  return Vector2( childSample-1, childLine-1);
}

Vector3 IsisAdjustCameraModel::point_to_mm_time( Vector3 const& mm_time, Vector3 const& point ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_time( mm_time[2] );

  // Finding the focal length for the camera in millimeters
  Isis::CameraDistortionMap* distortMap = cam->DistortionMap();
  double focal_length_mm = distortMap->UndistortedFocalPlaneZ();

  // Now building a pinhole camera model
  Vector3 center = this->camera_center( mm_time );
  Quaternion<double> orientation = this->camera_pose( mm_time );

  // Performing the forward projection.  The following code is
  // equivalent to forward projection with a pinhole camera
  // constructed as follows:
  //
  //     PinholeModel pin_cam( center, transpose(orientation.rotation_matrix()), 
  //   			focal_length_mm, focal_length_mm,
  //   			0, 0 );
  //     pin_cam.set_coordinate_frame( Vector3( 1, 0, 0 ),
  //   				Vector3( 0, 1, 0 ),
  //   				Vector3( 0, 0, 1 ) );
  //     Vector2 forward_projection = pin_cam.point_to_pixel( point );
  //
  // However, the following code runs quite a bit faster since it
  // doesn't do all of the heavy-weight matrix math in the pinhole's
  // rebuild_camera_matrix() method.
  Vector3 forward_projection = orientation.rotate(point - center);
  forward_projection(0) *= focal_length_mm;
  forward_projection(1) *= focal_length_mm;
  forward_projection = forward_projection / forward_projection[2];
  return Vector3( forward_projection[0], forward_projection[1], mm_time[2] );
}

Vector3 IsisAdjustCameraModel::mm_time_to_vector( Vector3 const& mm_time ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_time( mm_time[2] );

  // Now looking up the focal length
  Isis::CameraDistortionMap* distortMap = cam->DistortionMap();
  Vector3 pointing( mm_time[0], mm_time[1], distortMap->UndistortedFocalPlaneZ() );

  // Normalize
  pointing = normalize( pointing );

  Quaternion<double> look_transform = this->camera_pose( mm_time );
  return inverse( look_transform ).rotate( pointing );
}

Vector3 IsisAdjustCameraModel::camera_center( Vector3 const& mm_time ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_time( mm_time[2] );

  // Remember that Isis performs globe centered globe fixed
  // calculations in kilometers.
  double pos[3];
  cam->InstrumentPosition( pos );
  return Vector3( pos[0]*1000, pos[1]*1000, pos[2]*1000 ) + m_position_func->evaluate( mm_time[2] );
}

Quaternion<double> IsisAdjustCameraModel::camera_pose( Vector3 const& mm_time ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_time( mm_time[2] );
  
  // Convert from instrument frame-> J2000 frame --> Globe centered-fixed
  std::vector<double> rot_inst = cam->InstrumentRotation()->Matrix();
  std::vector<double> rot_body = cam->BodyRotation()->Matrix();
  MatrixProxy<double,3,3> R_inst(&(rot_inst[0]));
  MatrixProxy<double,3,3> R_body(&(rot_body[0]));

  Quaternion<double> quat (R_inst*transpose(R_body)*m_pose_func->evaluate( mm_time[2] ).rotation_matrix());
  quat = quat / norm_2(quat);
  return quat;
    //return Quaternion<double>(R_inst*transpose(R_body)*m_pose_func->evaluate( mm_time[2] ).rotation_matrix() );
}

double IsisAdjustCameraModel::undistorted_focal( Vector3 const& mm_time ) const {
  Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
  this->set_time( mm_time[2] );

  // Looking up the focal length
  Isis::CameraDistortionMap* distortMap = cam->DistortionMap();
  return distortMap->UndistortedFocalPlaneZ();
}

//-------------------------------------------------------------------------
// Protected
//-------------------------------------------------------------------------

void IsisAdjustCameraModel::set_image( double const& sample, double const& line) const {
  if (m_current_line != line || m_current_sample != sample ) {
    Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
    // ISIS indexes the top left corner (1,1) while VW uses(0,0)
    cam->SetImage( sample+1, line+1 );
    m_current_line = cam->Line()-1;
    m_current_sample = cam->Sample()-1;
    m_current_time = cam->EphemerisTime();
  }
}

void IsisAdjustCameraModel::set_time( double const& time ) const {
  VW_DEBUG_ASSERT( time <= m_max_ephemeris && time >= m_min_ephemeris, vw::ArgumentErr() << "Incorrect ephemeris time given in IsisAdjustCameraModel::set_time." );
  if (m_current_time != time ) {
    Isis::Camera* cam = static_cast<Isis::Camera*>( m_isis_camera_ptr );
    cam->SetEphemerisTime( time );
    m_current_line = cam->Line()-1;
    m_current_sample = cam->Sample()-1;
    m_current_time = cam->EphemerisTime();
  }
}

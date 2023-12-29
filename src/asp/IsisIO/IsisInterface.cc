// __BEGIN_LICENSE__
//  Copyright (c) 2009-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__

#include <asp/IsisIO/IsisInterface.h>
#include <asp/IsisIO/IsisInterfaceMapFrame.h>
#include <asp/IsisIO/IsisInterfaceFrame.h>
#include <asp/IsisIO/IsisInterfaceMapLineScan.h>
#include <asp/IsisIO/IsisInterfaceLineScan.h>
#include <asp/IsisIO/IsisInterfaceSAR.h>

#include <isis/Cube.h>
#include <isis/Distance.h>
#include <isis/Pvl.h>
#include <isis/Camera.h>
#include <isis/Target.h>
#include <isis/FileName.h>
#include <isis/CameraFactory.h>
#include <isis/SerialNumber.h>
#include <isis/iTime.h>
#include <isis/Blob.h>
#include <isis/Process.h>
#include <isis/CubeAttribute.h>

#include <vw/Core/Exception.h>
#include <vw/Math/Vector.h>
#include <vw/Core/Log.h>

#include <boost/filesystem.hpp>

#include <iomanip>
#include <ostream>

using namespace vw;

namespace asp {namespace isis { 
                               
IsisInterface::IsisInterface(std::string const& file) {
  // Opening labels and camera
  Isis::FileName ifilename(QString::fromStdString(file));
  m_label.reset(new Isis::Pvl());
  m_label->read(ifilename.expanded());

  // Opening Isis::Camera
  m_cube.reset(new Isis::Cube(QString::fromStdString(file)));
  m_camera.reset(Isis::CameraFactory::Create(*m_cube));

  // Set the datum
  // TODO(oalexan1): This is fragile. Need to find the right internal ISIS
  // function to use to convert ECEF to lon-lat-height and vice-versa.
  bool use_sphere_for_non_earth = true;
  m_datum = this->get_datum(use_sphere_for_non_earth);
}

IsisInterface::~IsisInterface() {}

IsisInterface* IsisInterface::open(std::string const& filename) {
  // Opening Labels (This should be done somehow though labels)
  Isis::FileName ifilename(QString::fromStdString(filename));
  Isis::Pvl label;
  label.read(ifilename.expanded());

  Isis::Cube tempCube(QString::fromStdString(filename));
  Isis::Camera* camera = Isis::CameraFactory::Create(tempCube);

  IsisInterface* result;

  // Instantiate the correct class type
  switch (camera->GetCameraType()) {
  case 0:
    // Framing camera
    if (camera->HasProjection())
      result = new IsisInterfaceMapFrame(filename);
    else
      result = new IsisInterfaceFrame(filename);
    break;
  case 2:
    // Linescan camera
    if (camera->HasProjection())
      result = new IsisInterfaceMapLineScan(filename);
    else
      result = new IsisInterfaceLineScan(filename);
    break;
  case 3:
    // SAR camera (such as MiniRF)
    // The same interface handles both projected and unprojected images,
    // since the ISIS functions take care of the details.
    // TODO(oalexan1): that cam2map-ed images are handled correctly.
    result = new IsisInterfaceSAR(filename);
    break;
  default:
    // LRO WAC comes here
    vw_throw(NoImplErr() << "Unusual input file: " << filename
             << ". Seems to have Isis camera type " << camera->GetCameraType() 
             << ". Check your data. Maybe it will work with CSM.");
  }

  return result;
}

int IsisInterface::lines() const {
  return m_camera->Lines();
}

int IsisInterface::samples() const {
  return m_camera->Samples();
}

std::string IsisInterface::serial_number() const {
  Isis::Pvl copy(*m_label);
  return Isis::SerialNumber::Compose(copy, true).toStdString();
}

double IsisInterface::ephemeris_time(vw::Vector2 const& pix) const {
  m_camera->SetImage(pix[0]+1, pix[1]+1);
  return m_camera->time().Et();
}

vw::Vector3 IsisInterface::sun_position(vw::Vector2 const& pix) const {
  m_camera->SetImage(pix[0]+1, pix[1]+1);
  Vector3 sun;
  m_camera->sunPosition(&sun[0]);
  return sun * 1000;
}

vw::Vector3 IsisInterface::target_radii() const {
  Isis::Distance radii[3];
  m_camera->radii(radii);
  return Vector3(radii[0].meters(),
                  radii[1].meters(),
                  radii[2].meters());
}

std::string IsisInterface::target_name() const {
  return m_camera->target()->name().toStdString();
}

// Manufacture a datum
vw::cartography::Datum IsisInterface::get_datum(bool use_sphere_for_non_earth) const {
      
  vw::Vector3 radii = this->target_radii();
  double radius1 = (radii[0] + radii[1]) / 2; // average the x and y axes (semi-major) 
  double radius2 = radius1;
  if (!use_sphere_for_non_earth) {
    radius2 = radii[2]; // the z radius (semi-minor axis)
  }
  
  vw::cartography::Datum datum("D_" + this->target_name(), this->target_name(),
                               "Reference Meridian", radius1, radius2, 0);
  return datum;
}

std::ostream& operator<<(std::ostream& os, IsisInterface* i) {
  os << "IsisInterface" << i->type()
       << "(Serial=" << i->serial_number()
       << std::setprecision(9)
       << ", f=" << i->m_camera->FocalLength()
       << " mm, pitch=" << i->m_camera->PixelPitch()
       << " mm/px," << std::setprecision(6)
       << "Center=" << i->camera_center() << ")";
    return os;
}

// Check if ISISROOT and ISISDATA was set
bool IsisEnv() {
  char * isisroot_ptr = getenv("ISISROOT");
  char * isisdata_ptr = getenv("ISISDATA");

  if (isisroot_ptr == NULL || isisdata_ptr == NULL ||
      std::string(isisroot_ptr) == "" ||
      std::string(isisdata_ptr) == "" ||
      !boost::filesystem::exists(std::string(isisroot_ptr) + "/IsisPreferences"))
    return false;
  return true;
}

// TODO(oalexan1): This must be integrated with ISIS
void deleteKeywords(Isis::Cube *cube) {
  
  Isis::PvlGroup &kernelsGroup = cube->group("Kernels");

  // Get rid of keywords from spiceinit
  if (kernelsGroup.hasKeyword("LeapSecond")) {
    kernelsGroup.deleteKeyword("LeapSecond");
  }
  if (kernelsGroup.hasKeyword("TargetAttitudeShape")) {
    kernelsGroup.deleteKeyword("TargetAttitudeShape");
  }
  if (kernelsGroup.hasKeyword("TargetPosition")) {
    kernelsGroup.deleteKeyword("TargetPosition");
  }
  if (kernelsGroup.hasKeyword("InstrumentPointing")) {
    kernelsGroup.deleteKeyword("InstrumentPointing");
  }
  if (kernelsGroup.hasKeyword("InstrumentPointingQuality")) {
    kernelsGroup.deleteKeyword("InstrumentPointingQuality");
  }
  if (kernelsGroup.hasKeyword("Instrument")) {
    kernelsGroup.deleteKeyword("Instrument");
  }
  if (kernelsGroup.hasKeyword("SpacecraftClock")) {
    kernelsGroup.deleteKeyword("SpacecraftClock");
  }
  if (kernelsGroup.hasKeyword("InstrumentPositionQuality")) {
    kernelsGroup.deleteKeyword("InstrumentPositionQuality");
  }
  if (kernelsGroup.hasKeyword("InstrumentPosition")) {
    kernelsGroup.deleteKeyword("InstrumentPosition");
  }
  if (kernelsGroup.hasKeyword("InstrumentAddendum")) {
    kernelsGroup.deleteKeyword("InstrumentAddendum");
  }
  if (kernelsGroup.hasKeyword("EXTRA")) {
    kernelsGroup.deleteKeyword("EXTRA");
  }
  if (kernelsGroup.hasKeyword("Source")) {
    kernelsGroup.deleteKeyword("Source");
  }
  if (kernelsGroup.hasKeyword("SpacecraftPointing")) {
    kernelsGroup.deleteKeyword("SpacecraftPointing");
  }
  if (kernelsGroup.hasKeyword("SpacecraftPosition")) {
    kernelsGroup.deleteKeyword("SpacecraftPosition");
  }
  if (kernelsGroup.hasKeyword("CameraVersion")) {
    kernelsGroup.deleteKeyword("CameraVersion");
  }
  if (kernelsGroup.hasKeyword("ElevationModel")) {
    kernelsGroup.deleteKeyword("ElevationModel");
  }
  if (kernelsGroup.hasKeyword("Frame")) {
    kernelsGroup.deleteKeyword("Frame");
  }
  if (kernelsGroup.hasKeyword("StartPadding")) {
    kernelsGroup.deleteKeyword("StartPadding");
  }
  if (kernelsGroup.hasKeyword("EndPadding")) {
    kernelsGroup.deleteKeyword("EndPadding");
  }
  if (kernelsGroup.hasKeyword("RayTraceEngine")) {
    kernelsGroup.deleteKeyword("RayTraceEngine");
  }
  if (kernelsGroup.hasKeyword("OnError")) {
    kernelsGroup.deleteKeyword("OnError");
  }
  if (kernelsGroup.hasKeyword("Tolerance")) {
    kernelsGroup.deleteKeyword("Tolerance");
  }

  if (cube->label()->hasObject("NaifKeywords")) {
    cube->label()->deleteObject("NaifKeywords");
  }
  
  // Delete any existing polygon (per jigsaw)
  if (cube->label()->hasObject("Polygon")) {
    cube->label()->deleteObject("Polygon");
  }

  // Delete any CameraStatistics table (per jigsaw)
  for (int iobj = 0; iobj < cube->label()->objects(); iobj++) {
    Isis::PvlObject obj = cube->label()->object(iobj);
    if (obj.name() != "Table") continue;
    if (obj["Name"][0] != QString("CameraStatistics")) continue;
    cube->label()->deleteObject(iobj);
    break;
  }

  // Remove tables from spiceinit before writing to the cube (per csminit)
  cube->deleteBlob("InstrumentPointing", "Table");
  cube->deleteBlob("InstrumentPosition", "Table");
  cube->deleteBlob("BodyRotation", "Table");
  cube->deleteBlob("SunPosition", "Table");
  cube->deleteBlob("CameraStatistics", "Table");
  cube->deleteBlob("Footprint", "Polygon");
  
  // Wipe any existing blob (per csminit)
  cube->deleteBlob("CSMState", "String");
}

// Peek inside a cube file to see if it has a CSM blob. This needs ISIS
// logic, rather than any CSM-specific info.  
bool IsisCubeHasCsmBlob(std::string const& cubeFile) {
  
  QString qCubeFile = QString::fromStdString(cubeFile);
  Isis::Process p;
  Isis::CubeAttributeInput inAtt;
  Isis::Cube *cube = p.SetInputCube(qCubeFile, inAtt, Isis::ReadWrite);
  
  // Hopefully the memory of cube is freed when p goes out of scope.  
  return cube->hasBlob("CSMState", "String");
}

// Read the CSM state (a string) from a cube file. Throw an exception if
// missing. Careful not to copy junk from the blob.
std::string csmStateFromIsisCube(std::string const& cubeFile) {
    
  QString qCubeFile = QString::fromStdString(cubeFile);
  Isis::Process p;
  Isis::CubeAttributeInput inAtt;
  Isis::Cube *cube = p.SetInputCube(qCubeFile, inAtt, Isis::ReadWrite);
  
  if (!cube->hasBlob("CSMState", "String"))
    vw::vw_throw( vw::ArgumentErr() << "Cannot find the CSM state in: "
                  << cubeFile << "\n");

  Isis::Blob csmStateBlob("CSMState", "String");
  cube->read(csmStateBlob);

  // Copy precisely the number of characters in the blob. This prevents copying junk,
  // which can result in a parse error later.
  int len = csmStateBlob.Size();
  std::string buf(len, ' ');
  memcpy(&buf[0], csmStateBlob.getBuffer(), len);
  return buf;
}

// Save a CSM state to an ISIS Cube file. Wipe any spice info.
// This may throw if the file cannot be saved.
void saveCsmStateToIsisCube(std::string const& cubeFile, 
                            std::string const& pluginName, 
                            std::string const& modelName,
                            std::string const& modelState) {
 
 if (modelState.empty())
   vw::vw_throw( vw::ArgumentErr() << "Cannot save empty CSM state to ISIS cube file "
                 << cubeFile << ".\n");

  QString qCubeFile = QString::fromStdString(cubeFile);
  Isis::Process p;
  Isis::CubeAttributeInput inAtt;
  Isis::Cube *cube = p.SetInputCube(qCubeFile, inAtt, Isis::ReadWrite);
  
  // Delete any spice and other obsolete keywords (per csminit)
  deleteKeywords(cube);

  // Add the CSM state to to a blob
  Isis::Blob csmStateBlob("CSMState", "String");
  csmStateBlob.setData(modelState.c_str(), modelState.size());

  // Save the state and other info to the cube
  QString jigComment = "Jigged = " + Isis::iTime::CurrentLocalTime();
  QString qModelName = QString::fromStdString(modelName);
  QString qPluginName = QString::fromStdString(pluginName);
  Isis::PvlObject &blobLabel = csmStateBlob.Label();
  blobLabel += Isis::PvlKeyword("ModelName", qModelName);
  blobLabel += Isis::PvlKeyword("PluginName", qPluginName);
  blobLabel.addComment(jigComment);

  // Write the cube  
  cube->write(csmStateBlob);
  Isis::CameraFactory::Create(*cube);
  p.WriteHistory(*cube);
}

}} // end namespace asp::isis


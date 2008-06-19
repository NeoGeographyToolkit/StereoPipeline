#ifndef _BUNDLEVIS_H_
#define _BUNDLEVIS_H_

//The general libraries that I need for this project
//BOOST files
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//OpenSceneGraph files
#include <osg/Geode>
#include <osg/Point>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/Sequence>
#include <osgUtil/Optimizer>
#include <osg/Node>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgSim/DOFTransform>
#include <osgSim/MultiSwitch>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgText/Text>

//Standard
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>

//VisionWorkbench
#include <vw/Camera/ControlNetwork.h>

//POINT_IN_TIME CLASS
//This is a class that holds the position and string tag for a point
//being tracked through time. It's important to have the string as it
//will be used during the event that the user clicks on that
//point. This also contains an integeter representing what iteration
//this point is from.
class PointInTime {
 public:
  //Constructor & Deconstructor
  PointInTime (osg::Vec3f center, const int& pnt_num, const int& iter_num);
  ~PointInTime();
  //Useful for drawing
  osg::Geode* getSelectCube(const float& size);
  const osg::Vec3f* getCenter();
  const std::string getDescription(){
    return description_;
  }
 protected:
  osg::Vec3f location_;
  std::string description_;
  int pointID_;
  int iteration_;
};

//CAMERA_IN_TIME CLASS
//This is a class that holds the CAHVOR model of the camera along with
//a descriptor string. The string is the most imporant as this allows
//for the user to determine what camera is being looked at. This also
//contains an integer representing what iteration this camera is from.
class CameraInTime {
 public:
  //Constructor & Deconstructor
  CameraInTime(osg::Vec3Array* model_param,const int& cam_num, const int& iter_num);
  ~CameraInTime();
  //Useful for drawing
  osg::Geode* get3Axis(const float& size, const float& opacity);
  osg::Geode* getSelectCube(const float& size);
  const osg::Vec3f* getCenter();
  const std::string getDescription(){
    return description_;
  }
 protected:
  osg::Vec3f c_;
  osg::Vec3f a_;
  osg::Vec3f h_;
  osg::Vec3f v_;
  osg::Vec3f o_;
  osg::Vec3f r_;
  std::string description_;
  int cameraNum_;
  int iteration_;
};

//This function will draw an entire group using the points in time
//provide, to create selectable points with fading lines connecting all
//of them.
osg::Group* PointString (std::vector<PointInTime* >* pointData);

//This function will draw an entire group using the cameras in time
//provided. This will create a string of connected 3 axes that can be
//picked to reveal their label
osg::Group* CameraString (std::vector<CameraInTime* >* cameraData);

//This will build a vector containing vectors for all the points that
//existed in the bundle adjustment problem. The second level vector
//contains a specific point's data across all iterations of BA. The
//lowest level is the PointInTime, which contains a specific points time
//instance data.
std::vector<std::vector<PointInTime*>*>* loadPointsData(std::string pntFile);

//This will build a vector containing a vector for each camera in the
//bundle adjustment. The second level vector contains that camera's data
//for each time iteration. The lowest level is the CameraInTime class
//which contains that instance's data.
std::vector<std::vector<CameraInTime*>*>* loadCamerasData(std::string camFile);

//This function creates the HUD of the screen. The passed pointer to
//text, is the text label that keeps changing based on what the user has
//selected.
osg::Node* createHUD(osgText::Text* updateText);

//This will create a sequence, which contains at the first level, the
//point shown at all locations like in Ver1.0 of this code. All the
//sequences after the first frame contain an animation sequence that
//show the camera at it's first location, and ghost of where the camera
//was before in the previous 4 frames. Also this is over loaded to
//accept both vectors of PointInTime and CameraInTime classes
osg::Sequence* createSeq(std::vector<CameraInTime*>* cameraData);

//This will create a sequence, which contains at the first level, the
//point shown at all locations like in Ver1.0 of this code. All the
//sequences after the first frame contain an animation sequence that
//show the camera at it's first location, and ghost of where the point
//was before in the previous 4 frames. Also this is over loaded to
//accept both vectors of PointInTime and CameraInTime classes
osg::Sequence* createSeq(std::vector<PointInTime*>* pointData);

//Copied this from the 'net
std::vector<std::string> tokenize(const std::string & str, const std::string & delim)
{
  using namespace std;
  vector<string> tokens;

  size_t p0 = 0, p1 = string::npos;
  while(p0 != string::npos)
  {
    p1 = str.find_first_of(delim, p0);
    if(p1 != p0)
    {
      string token = str.substr(p0, p1 - p0);
      tokens.push_back(token);
    }
    p0 = str.find_first_not_of(delim, p1);
  }

  return tokens;
}


//An event handler for all
class AllEventHandler : public osgGA::GUIEventHandler{
 public:
  AllEventHandler(std::vector<osg::Sequence*>* sequences, osgText::Text* updateText, osgGA::TrackballManipulator* camera, std::vector<std::vector<PointInTime*>*>* pointData, std::vector<std::vector<CameraInTime*>*>* cameraData, vw::camera::ControlNetwork* cnet)
    {
      sequences_ = sequences;
      updateText_ = updateText;
      camera_ = camera;
      pointData_ = pointData;
      cameraData_ = cameraData;
      cnet_ = cnet;
    }
  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
  void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
  void setLabel(const std::string& name)
  {
    //If update text still exits
    if (updateText_.get())
      updateText_->setText(name);
  }
 private:
  std::vector<osg::Sequence*>* sequences_;
  osg::ref_ptr<osgText::Text> updateText_;
  osgGA::TrackballManipulator* camera_;
  std::vector<std::vector<PointInTime*>*>* pointData_;
  std::vector<std::vector<CameraInTime*>*>* cameraData_;
  vw::camera::ControlNetwork* cnet_;
};

#endif

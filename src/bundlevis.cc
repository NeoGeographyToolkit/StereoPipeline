// __BEGIN_LICENSE__
// 
// Copyright (C) 2008 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2008 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
//
// __END_LICENSE__

/// \file bundlevis.cc
///

//Hopefully cut down in some of the organizational mess
#include "bundlevis.h"

#define LINE_LENGTH 1.0f
#define PROGRAM_NAME "Bundlevis v1.1"

/************************************************
* Constructor for PointInTime                   *
************************************************/
PointInTime::PointInTime(osg::Vec3f center, const int& pnt_num, const int& iter_num){
  location_ = center;
  pointID_ = pnt_num;
  iteration_ = iter_num;

  std::ostringstream os;
  os << "Point: " << pointID_ << " @Time: " << iteration_;
  
  description_ = os.str();
}

/*************************************************
* PointInTime build Select Cube function         *
*************************************************/
osg::Geode* PointInTime::getSelectCube(const float& size){
  osg::Geode* selectCube = new osg::Geode();

  osg::ShapeDrawable* box = new osg::ShapeDrawable(new osg::Box(location_,size));
  box->setColor(osg::Vec4f(1.0f,1.0f,1.0f,0.0f));

  selectCube->addDrawable(box);

  //State sets for now, will not be set on an individual level but as
  //globally. This supposedly is a speed advantage.

  selectCube->setName(description_);
  
  //Finishing up my work here
  return selectCube;
}

/**************************************************************************
* Function to return the location of the point, this is really just to    *
* protect the data inside the class, so that it can not be changed.       *
**************************************************************************/
const osg::Vec3f* PointInTime::getCenter(){
  return &location_;
}

/**************************************************************************
* This function will create a floating text object above the point        *
* indicating which point this is                                          *
**************************************************************************/
osg::Node* PointInTime::getTextGraphic(){
  
  std::ostringstream os;
  os << pointID_;
  
  osgText::Text* text = new osgText::Text;
  text->setCharacterSize(1.0f);
  text->setText(os.str());
  text->setFont("fonts/arial.ttf");
  text->setAlignment(osgText::Text::CENTER_CENTER);

  osg::Geode* geode = new osg::Geode;
  geode->addDrawable(text);
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

  osg::AutoTransform* at = new osg::AutoTransform;
  at->addChild(geode);

  at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
  at->setAutoScaleToScreen(true);
  at->setMinimumScale(0.0f);
  at->setMaximumScale(2.0f);
  at->setPosition(location_ + osg::Vec3f(0,0,0.5f));

  return at;

}

/********************************************************************
* addImageMeasure                                                   *
*  This will add a image measurement for this point in time. The    *
*  first input variable j is defining which camera measured this    *
*  point. The second and third is the x and y of the image measure. *
********************************************************************/
void PointInTime::addImageMeasure(const int& j, const float& x, const float& y){
  
  //Is image_measures even big enough for camera j?
  if ( j >= image_measures_.size() ){
    image_measures_.resize(j+1,NULL);

    //DEBUG
    //std::cout << "resized\n";
  }

  //DEBUG
  //std::cout << "inst pnt: " << j << " size: " << image_measures_.size() << std::endl;

  image_measures_.at(j) = new osg::Vec2f(x,y);
}

/********************************************************************
* getImageMeasures                                                  *
*  This just pulls that vector safely out of the class.             *
********************************************************************/
const std::vector<osg::Vec2f*>& PointInTime::getImageMeasures(){
  return image_measures_;
}

/*******************************************
* Constructor for CameraInTime (is CAHVOR  *
*******************************************/
CameraInTime::CameraInTime(osg::Vec3Array* model_param,const int& cam_num, const int& iter_num){
  //It assumed that I have been given a good Vec3Array which is arranged in CAHVOR
  c_ = (*model_param)[0];
  a_ = (*model_param)[1];
  v_ = (*model_param)[2];
  h_ = (*model_param)[3];
  o_ = (*model_param)[4];
  r_ = (*model_param)[5];
  cameraNum_ = cam_num;
  iteration_ = iter_num;

  std::ostringstream os;
  os << "Camera: " << cameraNum_ << " @Time: " << iteration_;
  description_ = os.str();

  imageData_ = NULL;

  isCAHVOR_ = true;
}

/********************************************
* Constructor for CameraInTime (is PinHole  *
********************************************/ 
CameraInTime::CameraInTime(const osg::Vec3f& model_center, const osg::Vec3f& euler, const int& cam_num, const int& iter_num){
  c_ = model_center;
  euler_ = euler;
  cameraNum_ = cam_num;
  iteration_ = iter_num;
  
  std::ostringstream os;
  os << "Camera: " << cameraNum_ << " @Time: " << iteration_;
  description_ = os.str();
  
  imageData_ = NULL;

  isCAHVOR_ = false;
}

/***************************************************************************
* get3Axis, will build the little red green blue reference frame that      *
* represents the camera. It is made of only thick lines                    *
***************************************************************************/
osg::Geode* CameraInTime::get3Axis(const float& size, const float& opacity){
  osg::Geode* the3Axis = new osg::Geode();
  osg::Geometry* the3AxisGeo = new osg::Geometry();
  
  //This is a temporary value, which allows for processing of camera
  //data, while still keeping the original data.
  osg::Vec3f normal;

  if (isCAHVOR_){
    //I'm now going to build the vertices required for the 3 axis
    osg::Vec3Array* lineData = new osg::Vec3Array();
    lineData->push_back(c_);
    normal = h_;
    normal.normalize();
    lineData->push_back(c_ + normal*size);
    lineData->push_back(c_);
    normal = v_;
    normal.normalize();
    lineData->push_back(c_ + normal*size);
    lineData->push_back(c_);
    normal = a_;
    normal.normalize();
    lineData->push_back(c_ + normal*size);
    the3AxisGeo->setVertexArray(lineData);
    
    //Setting up the color
    osg::Vec4Array* colours = new osg::Vec4Array();
    colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
    the3AxisGeo->setColorArray(colours);
    the3AxisGeo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    //Add the primitve to draw the lines using the vertice array
    the3AxisGeo->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,lineData->getNumElements()));
  } else { //Sounds like we have a pin hole model
    
    //I need to make my rotational matrix
    double ca =  cos(euler_[0]), sa = sin(euler_[0]);
    double cb =  cos(euler_[1]), sb = sin(euler_[1]);
    double cc =  cos(euler_[2]), sc = sin(euler_[2]);
    
    //These euler angles are a little weird in that they describe how
    //to go from the world frame into the camera's frame. This is to
    //make since for say a pilot flying an airplane?

    vw::Matrix<double, 3, 3> rotational;
    rotational(0,0) = cb*cc;
    rotational(0,1) = -cb*sc;
    rotational(0,2) = sb;
    rotational(1,0) = sa*sb*cc+ca*sc;
    rotational(1,1) = -sa*sb*sc+ca*cc;
    rotational(1,2) = -sa*cb;
    rotational(2,0) = -ca*sb*cc+sa*sc;
    rotational(2,1) = ca*sb*sc+sa*cc;
    rotational(2,2) = ca*cb;
    
    rotational = inverse(rotational);

    //This matrix now take thing in the camera frame and describes
    //them in the world frame. The rotation matrix these euler angles
    //are expected to relate to is the Rx'y'z' which is found in
    //Appendix B of Craig's Introduction to Robotics.

    vw::Vector3 center;
    center[0] = c_[0];
    center[1] = c_[1];
    center[2] = c_[2];

    vw::Vector3 x_axis = center + rotational*vw::Vector3(size, 0, 0);
    vw::Vector3 y_axis = center + rotational*vw::Vector3(0, size, 0);
    vw::Vector3 z_axis = center + rotational*vw::Vector3(0, 0, size);

    //Now drawing data
    osg::Vec3Array* lineData = new osg::Vec3Array();
    lineData->push_back(osg::Vec3f(center[0],center[1],center[2]));
    lineData->push_back(osg::Vec3f(x_axis[0],x_axis[1],x_axis[2]));
    lineData->push_back(osg::Vec3f(center[0],center[1],center[2]));
    lineData->push_back(osg::Vec3f(y_axis[0],y_axis[1],y_axis[2]));
    lineData->push_back(osg::Vec3f(center[0],center[1],center[2]));
    lineData->push_back(osg::Vec3f(z_axis[0],z_axis[1],z_axis[2]));
    the3AxisGeo->setVertexArray(lineData);
    
    //Setting up the color
    osg::Vec4Array* colours = new osg::Vec4Array();
    colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
    colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
    the3AxisGeo->setColorArray(colours);
    the3AxisGeo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    
    //Add the primitve to draw the lines using the vertice array
    the3AxisGeo->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,lineData->getNumElements()));
  }

  //Setting up some of the state properties, so it will look correct
  osg::StateSet* stateSet = new osg::StateSet();
  stateSet->setAttribute(new osg::LineWidth(2));

  //Now setting up the Geode
  the3Axis->setStateSet(stateSet);
  the3Axis->addDrawable(the3AxisGeo);
  the3Axis->setName(description_);
  
  //Finishing up my work here
  return the3Axis;
}

/****************************************************************************
* getSelectCube, this will building an invisble cube around the center      *
* of the camera, which allow for the selection by mouse picking to          *
* happen.                                                                   *
****************************************************************************/
osg::Geode* CameraInTime::getSelectCube(const float& size){
  osg::Geode* selectCube = new osg::Geode();

  osg::ShapeDrawable* box = new osg::ShapeDrawable(new osg::Box(c_,size));
  box->setColor(osg::Vec4f(1.0f,1.0f,1.0f,0.0f));
  
  selectCube->addDrawable(box);

  selectCube->setName(description_);

  //Finishing up my work here
  return selectCube;
}

/**********************************************************************
* This is just returning the center of the camera                     *
**********************************************************************/
const osg::Vec3f* CameraInTime::getCenter(){
  return &c_;
}

/**********************************************************************
* This function will create a floating text object above the camera   *
* indicating which camera this is.                                    *
**********************************************************************/
osg::Node* CameraInTime::getTextGraphic(){
  
  std::ostringstream os;
  os << cameraNum_;

  osgText::Text* text = new osgText::Text;
  text->setCharacterSize(20.0f);
  text->setText(os.str());
  text->setFont("fonts/arial.ttf");
  text->setAlignment(osgText::Text::CENTER_CENTER);

  osg::Geode* geode = new osg::Geode;
  geode->addDrawable(text);
  geode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);

  osg::AutoTransform* at = new osg::AutoTransform;
  at->addChild(geode);

  at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
  at->setAutoScaleToScreen(true);
  at->setMinimumScale(0.0f);
  at->setMaximumScale(0.05f);
  at->setPosition(c_ + osg::Vec3f(0,0,0.1f));

  return at;

}

//This just allows the use to pull the image belonging to this camera node
osg::Image* CameraInTime::getImage( void ) {
  return imageData_.get();
}

//This allows use to set the image associated with the camera
void CameraInTime::setImage(osg::Image* imageData) {
  imageData_ = imageData;
}

/**************************************************************************
 * PointString                                                            *
* This function will draw an entire group using the points in time        *
* provide, to create selectable points with fading lines connecting all   *
* of them.                                                                *
**************************************************************************/
osg::Group* PointString(std::vector<PointInTime* >* pointData){
  
  osg::Group* entire_pointString = new osg::Group();

  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec4Array* colours = new osg::Vec4Array();

  //I'm going to working through all the point given to me to draw in
  //the point string.
  for (int i = 0; i < pointData->size(); ++i){
    //Adding the current point to the vertices
    vertices->push_back(*(pointData->at(i)->getCenter()));

    //Now going to add the color for the segment. It is assumed the
    //lower indices are the points further back in time, therefore
    //they must be slightly transparent as well.
    colours->push_back(osg::Vec4f(1.0f, ((float)i+1)/(float)pointData->size(), ((float)i+1)/(float)pointData->size(), ((float)i+1)/(float)pointData->size()));

    //TODO: Someday in the future I'd like to have line length be an
    //option that can be brought up and set from the command line.
  }

  //I'm going to put a select cube only on the last point drawn
  entire_pointString->addChild(pointData->at(pointData->size() - 1)->getSelectCube(LINE_LENGTH/4.0f));

  osg::Geode* bumpyLine = new osg::Geode();
  bumpyLine->setName("BumpyLine for cool effect");
  osg::Geometry* bumpyLineGeo = new osg::Geometry();
  
  //Drawing the fading lines, along with points.
  bumpyLineGeo->setVertexArray(vertices);
  bumpyLineGeo->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,vertices->getNumElements()));
  bumpyLineGeo->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,0,vertices->getNumElements()));

  //Setting up the colors
  bumpyLineGeo->setColorArray(colours);
  bumpyLineGeo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
  
  //Setting up some state parameters, so the graphics will come out right
  osg::StateSet* stateSet = new osg::StateSet();
  osg::Point* point = new osg::Point();
  point->setSize(5.0f);
  stateSet->setAttribute(point);

  //Setting up geode for the bumpy line;
  bumpyLine->setStateSet(stateSet);
  bumpyLine->addDrawable(bumpyLineGeo);
  entire_pointString->addChild(bumpyLine);
  
  //Finishing up
  return entire_pointString;
}

/************************************************************************
* CameraString                                                          *
* This function will draw an entire group using the cameras in time     *
* provided. This will create a string of connected 3 axes that can be   *
* picked to reveal their label                                          *
************************************************************************/
osg::Group* CameraString (std::vector<CameraInTime* >* cameraData){
  
  osg::Group* entire_cameraString = new osg::Group();

  osg::Vec3Array* vertices = new osg::Vec3Array();
  osg::Vec4Array* colours = new osg::Vec4Array();

  //I'm going to work through all the camera data given to me to draw
  //in the camera string
  for (int j = 0; j < cameraData->size(); ++j){
    //Adding the current camera center to the vertices
    vertices->push_back(*(cameraData->at(j)->getCenter()));

    //Now I'm going to add the color segment. It is assumed the lower
    //numbers are the cameras further back in time, therefore they
    //must be slightly transparent as well. I'm making this black
    colours->push_back(osg::Vec4f( ((float)j + 1)/(float)cameraData->size() , ((float)j + 1)/(float)cameraData->size() , 1.0f, ((float)j + 1)/(float)cameraData->size()));

    //I'm also going to put on the reference frame as well
    entire_cameraString->addChild(cameraData->at(j)->get3Axis(LINE_LENGTH, ((float)j + 1)/(float)cameraData->size()));

    //TODO: Someday in the future I'd like to have line length be an
    //option that can be brought up and set from the command line.
  }

  //I'm going to put a select cube only on the last camera instance
  //drawn. Which should be the latest time plotted.
  entire_cameraString->addChild(cameraData->at(cameraData->size() - 1)->getSelectCube(LINE_LENGTH/2.0f));

  //Drawing a text spot at the end
  entire_cameraString->addChild(cameraData->at(cameraData->size() - 1)->getTextGraphic());

  osg::Geode* bumpyLine = new osg::Geode();
  bumpyLine->setName("BumpyLine for cool effect");
  osg::Geometry* bumpyLineGeo = new osg::Geometry();

  //Drawing the fading lines
  bumpyLineGeo->setVertexArray(vertices);
  bumpyLineGeo->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,vertices->getNumElements()));

  //Setting up the colors
  bumpyLineGeo->setColorArray(colours);
  bumpyLineGeo->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  //Setting up the geode for the bumpy line;
  bumpyLine->addDrawable(bumpyLineGeo);
  entire_cameraString->addChild(bumpyLine);

  //Finishing up
  return entire_cameraString;
}

/*************************************************************************
* loadPointsData                                                         *
* This will build a vector containing vectors for all the points that    *
* existed in the bundle adjustment problem. The second level vector      *
* contains a specific point's data across all iterations of BA. The      *
* lowest level is the PointInTime, which contains a specific points time *
* instance data.                                                         *
*************************************************************************/
std::vector<std::vector<PointInTime*>*>* loadPointsData(std::string pntFile){
  
  //First step is to determine the number of lines in the file
  std::ifstream iFile(pntFile.c_str(), std::ios::in);
  int numLines = 0;          //Number of lines in the file
  int numPoints = 0;         //Number of points in the file
  int numTimeIter = 0;       //Number of time iterations
  char c;
  while (!iFile.eof()){
    c=iFile.get();
    if (c == '\n'){
      int buffer;
      numLines++;
      iFile >> buffer;
      if (buffer > numPoints)
	numPoints = buffer;
    }
  }
  numPoints++;
  numTimeIter=numLines/numPoints;

  std::cout << "The number of lines: " << numLines << " Num of Points: " << numPoints << std::endl;

  //Reset the file reading to the front
  iFile.clear();
  iFile.seekg(0);

  //Making the memory organization to be known as pointData
  std::vector<std::vector<PointInTime*>*>* pointData = new std::vector<std::vector<PointInTime*>*>(numPoints);
  for (int i = 0; i < numPoints; ++i){
    pointData->at(i) = new std::vector<PointInTime*>(numTimeIter);
  }

  //Now filing in pointData, with actual information. This code is
  //laid out in the fointersectionsrm of the organization found in the input file
  
  //For every time iteration
  for(int t = 0; t < numTimeIter; ++t){
    for(int i = 0; i < numPoints; ++i){
      float float_fill_buffer;
      osg::Vec3f vec_fill_buffer;
      
      iFile >> float_fill_buffer;   //This first one is the point number
      if (float_fill_buffer != i) {
	std::cout << "Error reading " << i << " " << float_fill_buffer << " in " << pntFile << std::endl;
	break;
      }

      //Reading in the data now.
      iFile >> float_fill_buffer;
      vec_fill_buffer[0] = float_fill_buffer;
      iFile >> float_fill_buffer;
      vec_fill_buffer[1] = float_fill_buffer;
      iFile >> float_fill_buffer;
      vec_fill_buffer[2] = float_fill_buffer;

      //Saving the point data in the massive pointData
      pointData->at(i)->at(t) =  new PointInTime(vec_fill_buffer, i,t);
    }
  }

  iFile.close();
  
  return pointData;
}

/*****************************************************************************
* loadCameraData                                                             *
*  This will build a vector containing a vector for each camera in the       *
*  bundle adjustment. The second level vector contains that camera's data    *
*  for each time iteration. The lowest level is the CameraInTime class       *
*  which contains that instance's data.                                      *
*****************************************************************************/
std::vector<std::vector<CameraInTime*>*>* loadCamerasData(std::string camFile){
  
  //First step is to determine the number of lines in the file
  std::ifstream iFile(camFile.c_str(), std::ios::in);
  int numLines = 0;           //Number of lines in the file
  int numCameras = 0;         //Number of cameras per iteration
  int numTimeIter = 0;        //Number of iterations
  int numCameraParam = 0;     //Number of lines given to describe a camera
  char c;
  int counts = 0;
  int last_camera = 2000;
  while (!iFile.eof()){
    c=iFile.get();
    if (c == '\n'){
      int buffer;
      numLines++;
      iFile >> buffer;
      if (buffer > numCameras)
	numCameras = buffer;
      if (buffer != last_camera){
	last_camera = buffer;
	counts = 0;
      } else {
	counts++;
      }
      if (counts > numCameraParam)
	numCameraParam = counts;
    }
  }
  numCameras++;
  numTimeIter=numLines/(numCameras*numCameraParam);

  std::cout << "The number of lines: " << numLines << " Num of Cameras: " << numCameras << " Num Lines per Camera: " << numCameraParam << std::endl;

  //Reset the file reading to the front
  iFile.clear();
  iFile.seekg(0);

  //Making the memory organization to be known as cameraData
  std::vector<std::vector<CameraInTime*>*>* cameraData = new std::vector<std::vector<CameraInTime*>*>(numCameras);
  for (int j = 0; j < numCameras; ++j){
    cameraData->at(j) = new std::vector<CameraInTime*>(numTimeIter);
  }

  //Now filling in the cameraData, with actual information. This code
  //is laid out in the form of the organization found in the input
  //file.

  //For every time iteration
  for(int t = 0; t < numTimeIter; ++t){
    for(int j = 0; j < numCameras; ++j){
      float float_fill_buffer;
      osg::Vec3f vec_fill_buffer;
      osg::Vec3Array* array_fill_buffer = new osg::Vec3Array();
      
      if (numCameraParam == 6) {      //THIS IS A CAHVOR Camera Parameters that we are loading in
	
	//Working through loading up the CAHVOR model
	for (int p = 0; p < 6; ++p){
	  
	  iFile >> float_fill_buffer;     //This first one is the camera
	                                  //number
	  if (float_fill_buffer != j){
	    std::cout << "Error reading " << j << " " << float_fill_buffer << " in " << camFile << std::endl;
	    break;
	  }
	  
	  //Reading in the data data now.
	  iFile >> float_fill_buffer;
	  vec_fill_buffer[0] = float_fill_buffer;
	  iFile >> float_fill_buffer;
	  vec_fill_buffer[1] = float_fill_buffer;
	  iFile >> float_fill_buffer;
	  vec_fill_buffer[2] = float_fill_buffer;
	  
	  //Pushing it on the stack.
	  array_fill_buffer->push_back(vec_fill_buffer);
	}
      
	//Now saving the camera data into the large memory organization
	cameraData->at(j)->at(t) = new CameraInTime(array_fill_buffer, j, t);
	
	//Clearing it on the back end so I can fill it again
	array_fill_buffer->clear();
      } else if (numCameraParam == 1) { //This is a simplified camera
					//parameters that only has a
					//center listed and
					//roll_pitch_yaw

	//This first one is the camera number
	iFile >> float_fill_buffer;
	
	if (float_fill_buffer != j){
	  std::cout << "Error reading (non CAHVOR) " << j << " " << float_fill_buffer << " in " << camFile << std::endl;
	}

	osg::Vec3f euler_fill_buffer;
	iFile >> float_fill_buffer;
	vec_fill_buffer[0] = float_fill_buffer;
	iFile >> float_fill_buffer;
	vec_fill_buffer[1] = float_fill_buffer;
	iFile >> float_fill_buffer;
	vec_fill_buffer[2] = float_fill_buffer;
	iFile >> float_fill_buffer;
	euler_fill_buffer[0] = float_fill_buffer;
	iFile >> float_fill_buffer;
	euler_fill_buffer[1] = float_fill_buffer;
	iFile >> float_fill_buffer;
	euler_fill_buffer[2] = float_fill_buffer;
	
	//Now saving the camera data into the large memory organization
	cameraData->at(j)->at(t) = new CameraInTime(vec_fill_buffer, euler_fill_buffer, j, t);
	
      }
    }
  }

  iFile.close();

  return cameraData;
}

/****************************************************************************
* loadPixelData                                                             *
*  This will load pixel information data. It's not stored in it's own unique*
*  spot, it is instead appended to the points data. Hopefully this is more  *
*  organized in a way. So, becuase of the previous statement, the pointData *
*  is required as an input requirement.                                     *
****************************************************************************/
void loadPixelData(std::string pxlFile, std::vector<std::vector<PointInTime*>*>* pointData){
  
  //Like the rest, the first step is to determine how many lines in the file
  std::ifstream iFile(pxlFile.c_str(), std::ios::in);
  int numLines = 0;
  int numCameras = 0;
  int numPoints = 0;
  char c;
  while (!iFile.eof()){
    c=iFile.get();
    if (c == '\n'){
      int buffer;
      numLines++;
      iFile >> buffer;
      if (buffer > numPoints)
	numPoints = buffer;
      iFile >> buffer;
      if (buffer > numCameras)
	numCameras = buffer;
    }
  }
  numPoints++;
  numCameras++;

  std::cout << "The number of lines: " << numLines << " Num of Cameras: " << numCameras << " Num of Points: " << numPoints << std::endl;

  //Now some checks to see if this data really belongs with the pointData
  if (numPoints != pointData->size()){
    std::cout << "Pixel Iteration Data does not match Point Iteration Data\n";
    return;
  } else if (numLines%pointData->at(0)->size()){
    std::cout << "Pixel Iteration Data does not have the same number of time iterations as Point Iteration Data\n";
    return;
  }

  //Reset the file reading to the front
  iFile.clear();
  iFile.seekg(0);

  //Now I'm actually going to parse the data
  {
    std::vector<int> location;
    std::vector<float> data;
    float buffer;
    int previous_point = 0;
    int time_iteration = 0;
    
    //Priming the read
    iFile >> buffer;

    while (!iFile.eof()){

      //Reading in one line at a time
      location.push_back((int)buffer);
      iFile >> buffer;
      location.push_back((int)buffer);
      iFile >> buffer;
      data.push_back(buffer);
      iFile >> buffer;
      data.push_back(buffer);

      //Setting up for the next read around
      iFile >> buffer;

      //Checking which time iteration this is
      if (previous_point > location[0])
	time_iteration++;

      //DEBUGGING
      //std::cout << location[0] << " " << location[1] << " " << data[0] << " " << data[1] << " " << time_iteration << std::endl;
      //std::cout << "pointData size: " << pointData->size() << std::endl;
      //std::cout << "pointData[] size: " << pointData->at(location[0])->size() << std::endl;

      //Saving the data
      pointData->at(location[0])->at(time_iteration)->addImageMeasure(location[1],data[0],data[1]);
      
      //Closing changes
      previous_point = location[0];
      data.clear();
      location.clear();
    }
  }

}

/*********************************************************************
* loadImageData                                                      *
*  This will attach image data to every camera. It makes             *
*  large assumptions, like that a file exist with the same number as *
*  the camera.                                                       *
*********************************************************************/
void loadImageData(std::vector<std::vector<CameraInTime*>*>* cameraData, const std::string& prefix, const std::string& postfix){
  //Loading up image data
  int numCameras = cameraData->size();
  std::vector<osg::ref_ptr<osg::Image> > ImageList;
  for (int j = 0; j < numCameras; ++j){
    std::ostringstream os;
    os << prefix << std::setw(2) << std::setfill('0') << (j+1) << postfix;
    std::cout << "Loading: " << os.str() << std::endl;

    ImageList.push_back(osgDB::readImageFile(os.str()));

    ImageList.push_back(osgDB::readImageFile(os.str()));

    if (!ImageList.end()->valid()){
      std::cout << "This is invalid" << std::endl;
    }
  }
  
  //A test check
  if (ImageList.size() != numCameras){
    std::cout << "Didn't load enough images!\n";
    return;
  }

  //Attaching the images to the data
  for (int j = 0; j < numCameras; ++j){
    for (int t = 0; t < cameraData->at(j)->size(); ++t){
      cameraData->at(j)->at(t)->setImage(ImageList[j].get());
    }
  }
}

/********************************************************************
* createHUD                                                         *
*  This function creates the HUD of the screen. The passed          *
*  pointer to text, is the text label that keeps changing based on  *
*  what the user has selected.                                      *
********************************************************************/
osg::Node* createHUD(osgText::Text* updateText, osg::Texture2D* texture){

  osg::Camera* hudCamera = new osg::Camera;
  hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  hudCamera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
  hudCamera->setViewMatrix(osg::Matrix::identity());
  //  hudCamera->setRenderOrder(osg::Camera::NESTED_RENDER,1);
  hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);

  //Traditional lower_left.
  {
    std::string arialFont("fonts/arial.ttf");
    
    std::cout << "Running that spot in HUD\n";

    //Setting up label at bottom of the screen
    osg::Geode* geode = new osg::Geode();
    geode->setName("HUD: The text label of selected part");
    geode->addDrawable(updateText);
    hudCamera->addChild(geode);

    //Setting up some of the text properties
    updateText->setCharacterSize(20.0);
    updateText->setFont(arialFont);
    updateText->setColor(osg::Vec4(1.0f,1.0f,1.0f,0.0f));
    updateText->setText(PROGRAM_NAME);
    updateText->setPosition(osg::Vec3(50.0f,100.0f,0.0f));
    updateText->setDataVariance(osg::Object::DYNAMIC);
  }

  //Viewer, top right
  if (texture){
    osg::Geode* geode = new osg::Geode();
    geode->setName("HUD: Top Right Viewer");
    
    osg::Vec3Array* vertices = new osg::Vec3Array();
    vertices->push_back(osg::Vec3f(1280,1024,0));
    vertices->push_back(osg::Vec3f(1280,724,0));
    vertices->push_back(osg::Vec3f(980,724,0));
    vertices->push_back(osg::Vec3f(980,1024,0));

    osg::Vec4Array* colour = new osg::Vec4Array();
    colour->push_back(osg::Vec4f(1.0f,1.0f,1.0f,1.0f));

    osg::Vec2Array* texcoords = new osg::Vec2Array();
    texcoords->push_back(osg::Vec2f(1.0f,1.0f));
    texcoords->push_back(osg::Vec2f(1.0f,0.0f));
    texcoords->push_back(osg::Vec2f(0.0f,0.0f));
    texcoords->push_back(osg::Vec2f(0.0f,1.0f));

    osg::Geometry* geometry = new osg::Geometry();
    geometry->setVertexArray(vertices);
    geometry->setColorArray(colour);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
    geometry->setTexCoordArray(0,texcoords);
    geometry->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
        
    // setting up the texture state
    texture->setDataVariance(osg::Object::DYNAMIC);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    //State sets
    osg::StateSet* stateset = geometry->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);

    

    geode->addDrawable(geometry);
    hudCamera->addChild(geode);
  }

  return hudCamera;
}

/******************************************************************************
* CreateScene                                                                 *
*  This will build the entire scene, which revolves around a single           *
*  Sequences which contains all frames of the animation sequences.            *
******************************************************************************/
osg::Sequence* createScene(std::vector<std::vector<PointInTime*>*>* pointData, std::vector<std::vector<CameraInTime*>*>* cameraData, vw::camera::ControlNetwork* cnet ){

  osg::Sequence* seq = new osg::Sequence;

  #define num_ghost 2

  //Creating the first frame which shows all points at all time, along with cameras.
  osg::ref_ptr<osg::Group> firstFrame = new osg::Group();

  if (pointData){
    for (unsigned int i = 0; i < pointData->size(); ++i){            //Drawing points
      firstFrame->addChild(PointString(pointData->at(i)));
    }
  }

  if (cameraData){
    for (unsigned int j = 0; j < cameraData->size(); ++j){           //Drawing cameras
      firstFrame->addChild(CameraString(cameraData->at(j)));
    }
  }

  if (cnet){
    osg::ref_ptr<osg::Switch> tieSwitch = new osg::Switch();
    tieSwitch->setDataVariance(osg::Object::DYNAMIC);
    for (unsigned int p = 0; p < cnet->size(); ++p){            //Drawing connecting lines
      //DEBUG
      //std::cout << "Drawing lines for point: " << p << std::endl;
      for (unsigned int m = 0; m < (*cnet)[p].size(); ++m){
	//DEBUG
	//std::cout << "Found point " << p << " in image " << m << std::endl;
	//Hopefully at this time I have found a relations ship between point p, and camera m
	if ((*cnet)[p][m].image_id() < (signed)cameraData->size()){
	  //Seems legit

	  int camera_id = (*cnet)[p][m].image_id();

	  osg::Geode* geode = new osg::Geode();
	  osg::Geometry* geometry = new osg::Geometry();
	  
	  osg::Vec3Array* vertices = new osg::Vec3Array();
	  vertices->push_back(*(pointData->at(p)->at(pointData->at(p)->size() - 1)->getCenter()));
	  vertices->push_back(*(cameraData->at(camera_id)->at(pointData->at(camera_id)->size() - 1)->getCenter()));
	  
	  osg::Vec4Array* colour = new osg::Vec4Array();
	  colour->push_back(osg::Vec4f(1.0,0.65f,0.0f,0.4f));

	  geometry->setVertexArray(vertices);
	  geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	  geometry->setColorArray(colour);
	  geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	  std::ostringstream os;
	  
	  os << "LINE CAM " << camera_id << " PNT " << p << std::endl;

	  geode->addDrawable(geometry);
	  geode->setName(os.str());

	  tieSwitch->addChild(geode, false);
	  
	}

      }
    }
    firstFrame->addChild(tieSwitch.get());
  }

  seq->addChild(firstFrame.get());
  seq->setTime(seq->getNumChildren()-1, 0.2f);

  //Finding out the number of time instances
  int amountTime = 0;
  if (pointData){
    amountTime = pointData->at(0)->size();
  } else {
    amountTime = cameraData->at(0)->size();
  }

  //Now I am going to draw the rest of the scenes.
  for (int t = 0; t < (amountTime + num_ghost); ++t){

    osg::ref_ptr<osg::Group> frame = new osg::Group();
    
    //Finding out what the stop and start times that will be rendered
    signed int latest_time = t;
    if (latest_time >= amountTime)
      latest_time = amountTime - 1;
    //std::cout << "Latest time is: " << latest_time << std::endl;

    signed int earliest_time = t - num_ghost;
    if (earliest_time < 0)
      earliest_time = 0;
    //std::cout << "Earliest Time is: " << earliest_time << std::endl;

    //Drawing point data
    if (pointData){
      std::vector<PointInTime*>* modelsRendered = new std::vector<PointInTime*>;
      for (unsigned int i = 0; i < pointData->size(); ++i){
	modelsRendered->clear();
	
	for (int k = earliest_time; k <= latest_time; ++k){
	  //std::cout << "K is " << k << std::endl;
	  modelsRendered->push_back(pointData->at(i)->at(k));
	}

	frame->addChild(PointString(modelsRendered));

      }

    }

    //Drawing camera data
    if (cameraData){
      std::vector<CameraInTime*>* modelsRendered = new std::vector<CameraInTime*>;
      for (unsigned int j = 0; j < cameraData->size(); ++j){
	modelsRendered->clear();

	for (int k = earliest_time; k <= latest_time; ++k){
	  modelsRendered->push_back(cameraData->at(j)->at(k));
	}

	frame->addChild(CameraString(modelsRendered));

      }

    }

    //Drawing control network data
    if (cnet){
      //In this section we only draw the latest time stamp. Otherwise
      //the screen would be full of lines.
      osg::ref_ptr<osg::Switch> tieSwitch = new osg::Switch();
      tieSwitch->setDataVariance(osg::Object::DYNAMIC);
      for (unsigned int p = 0; p < cnet->size(); ++p){
	for(unsigned int m = 0; m < (*cnet)[p].size(); ++m){
	  
	  if ((*cnet)[p][m].image_id() < (signed)cameraData->size()){
	    
	    int camera_id = (*cnet)[p][m].image_id();

	    osg::Geode* geode = new osg::Geode();
	    osg::Geometry* geometry = new osg::Geometry();

	    osg::Vec3Array* vertices = new osg::Vec3Array();
	    vertices->push_back(*(pointData->at(p)->at(latest_time)->getCenter()));
	    vertices->push_back(*(cameraData->at(camera_id)->at(latest_time)->getCenter()));

	    osg::Vec4Array* colour = new osg::Vec4Array();
	    colour->push_back(osg::Vec4f(1.0, 0.65f, 0.0f, 0.4f));

	    geometry->setVertexArray(vertices);
	    geometry->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));
	    geometry->setColorArray(colour);
	    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

	    std::ostringstream os;

	    os << "LINE CAM " << camera_id << " PNT " << p << std::endl;

	    geode->addDrawable(geometry);
	    geode->setName(os.str());

	    tieSwitch->addChild(geode,false);

	  }

	}
      }
      frame->addChild(tieSwitch.get());
    }

    //So now we have an entire frame drawn
    seq->addChild(frame.get());
    seq->setTime(seq->getNumChildren()-1, 0.2f);

  }

  //Now settings for the overall sequence
  seq->setInterval(osg::Sequence::LOOP, 1, seq->getNumFrames() - 1);
  seq->setDuration(1.0f, -1);
  seq->setMode(osg::Sequence::START);
  
  return seq;
}

/***********************************************
*All event handler                             *
* This handles all keyboard commands and mouse *
* commands from the user. This will also update*
* the approiate parameters.                    *
***********************************************/
bool AllEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
  //Note: for some reason, you should only perform getEventType()
  //once. Otherwise the whole system freezes
  if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN){
    switch (ea.getKey()){
    case 'z':   //Step Backward
      {
	signed int value = seq_->getValue();
	value--;
	if (value < 1)
	  value = 1;

	seq_->setValue(value);
       
	break;
      }
    case 'x':   //Play
      {
	if (seq_->getMode() == osg::Sequence::STOP){

	  seq_->setMode(osg::Sequence::START);

	} else {
	  seq_->setMode(osg::Sequence::RESUME);
	}
	break;
      }
    case 'c':   //Pause
      {

	seq_->setMode(osg::Sequence::PAUSE);

	break;
      }
    case 'v':   //Stop
      {

	  seq_->setMode(osg::Sequence::STOP);
	  seq_->setValue(1);

	  imageTexture_->setImage(NULL);

	break;
      }
    case 'b':   //Step Forward
      {
	signed int value = seq_->getValue();
	value++;
	if (value >= seq_->getNumFrames())
	  value = seq_->getNumFrames() - 1;

	seq_->setValue(value);

	break;
      }
    case '0':   //Show all steps
      {

	  seq_->setValue(0);

	break;
      }
    case '1':   //Move to the first frame
      {

	seq_->setValue(1);

	break;
      }
    case '2':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*2/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '3':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*3/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '4':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*4/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '5':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*5/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '6':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*6/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '7':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*7/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '8':   //Move to a place somewhere in between
      {
	int value = (seq_->getNumFrames()-2)*8/9 + 1;

	seq_->setValue(value);

	break;
      }
    case '9':   //Move to the last frame
      {
	int value = seq_->getNumFrames()-1;

	seq_->setValue(value);

	break;
      }
    }
  } else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE){    //Going to just update the label
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (view)
      pick(view,ea);
  } else if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH){    //Going to move the camera
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (view) 
      pick(view,ea);
  }

  return false;
}

/*********************************************************************
* Pick                                                               *
*  This actually performs the calculation to determine if the ray    *
*  from the user's mouse click actually intersects with any          *
*  objects. The first object found that the ray intersects, is       *
*  determind to be what the user was looking for.                    *
*********************************************************************/
void AllEventHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea){
  osgUtil::LineSegmentIntersector::Intersections intersections;

  std::string itemFound="";
  float x = ea.getX();
  float y = ea.getY();
  
  if (view->computeIntersections(x,y,intersections)){

    //I only care about the first interestion
    osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
    
    std::ostringstream os;
    std::ostringstream debug;
    if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty())){
      

      //I'm going to move the center
      if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH){
	std::string temp = hitr->nodePath.back()->getName();
	std::vector<std::string> data = tokenize(temp," ");

	int item_identity = std::atoi(data[1].c_str());
	int time_stamp = std::atoi(data[3].c_str());

	if (data[0] == "Camera:"){
	  
	  os << hitr->nodePath.back()->getName() << std::endl;

	  camera_->setCenter(*(cameraData_->at(item_identity)->at(time_stamp)->getCenter()));

	  std::cout << "Searching ...\n";


	  if (cnet_){
	    //Also when they click we'll set what lines are currently on
	    SwitchNamedNodes finder;
	    for (int p = 0; p < (*cnet_).size(); ++p){
	      for (int m = 0; m < (*cnet_)[p].size(); ++m){
		if ((*cnet_)[p][m].image_id() == item_identity){  //We've found a point that was seen by this camera

		  std::ostringstream os;

		  os << "LINE CAM " << item_identity << " PNT " << p << std::endl;

		  finder.addNameToFind(os.str());

		  //Additional test code:
		  //std::cout << "Image description: " << (*cnet_)[p][m].description() << std::endl;

		}
	      }
	    }

	    finder.apply(*(root_.get()->getChild(0)));

	    std::cout << "Done\n";
	  }
	  
	  
	} else if (data[0] == "Point:"){
	  
	  os << hitr->nodePath.back()->getName() << std::endl;

	  camera_->setCenter(*(pointData_->at(item_identity)->at(time_stamp)->getCenter()));

	  if (cnet_){

	    //Also when they click we'll set what lines are currently on
	    SwitchNamedNodes finder;
	    for (int m = 0; m < (*cnet_)[item_identity].size(); ++m){
	      
	      std::ostringstream os;
	      
	      os << "LINE CAM " << (*cnet_)[item_identity][m].image_id() << " PNT " << item_identity << std::endl;
	      
	      finder.addNameToFind(os.str());
	      
	    }
	    finder.apply(*(root_.get()->getChild(0))); 
	  }

     	}

      } else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE) {
	std::string temp = hitr->nodePath.back()->getName();
	std::vector<std::string> data = tokenize(temp," ");

	int item_identity = std::atoi(data[1].c_str());
	int time_stamp = std::atoi(data[3].c_str());	

	if (data[0] == "Point:"){

	  os << hitr->nodePath.back()->getName() << std::endl;
	
	  if (cnet_){
	    debug << "In Images: ";
	    for (int i = 0; i < (*cnet_)[item_identity].size(); ++i){
	      debug << (*cnet_)[item_identity][i].image_id() << " ";
	    }
	    debug << std::endl;
	  }

	} else if (data[0] == "Camera:"){
	  //Also if this thing is a camera... change the texture to be it's image
	  if (imageTexture_.get()){
	    if (cameraData_->at(item_identity)->at(time_stamp)->getImage()){
	      imageTexture_->setImage(cameraData_->at(item_identity)->at(time_stamp)->getImage());
	    } else {
	      std::cout << "There was no image data attached to this camera";
	    }
	  }
	} 
      }
    }

    itemFound += os.str();
    itemFound += debug.str();
        
  }
  
  if (itemFound != ""){
    //itemFound = PROGRAM_NAME;
    setLabel(itemFound);
  } 
}

/************************************************
* Main                                          *
*  This is the main execution of the program    *
************************************************/
int main(int argc, char* argv[]){

  //Variables to be used later in this section
  std::string camera_iter_file;
  std::string points_iter_file;
  std::string pixel_iter_file;
  std::string control_net_file;
  std::string file_prefix;
  std::string file_postfix;
  std::vector<std::vector<PointInTime*>*>* pointData = NULL;
  std::vector<std::vector<CameraInTime*>*>* cameraData = NULL;
  vw::camera::ControlNetwork* cnet = NULL;;

  //OpenSceneGraph Variable which are important overall
  osgViewer::Viewer viewer;
  osg::ref_ptr<osg::Group> root = new osg::Group();
  osg::ref_ptr<osgText::Text> updateText = new osgText::Text;
  osg::ref_ptr<osg::Texture2D> imageTexture;

  //This first half of the program is wholly devoted to setting up boost program options
  //Boost program options code
  po::options_description general_options("Your most awesome Options");
  general_options.add_options()
    ("camera-iteration-file,c",po::value<std::string>(&camera_iter_file),"Load the camera parameters for each iteration from a file")
    ("points-iteration-file,p",po::value<std::string>(&points_iter_file),"Load the 3d points parameters for each iteration from a file")
    ("pixel-iteration-file,x",po::value<std::string>(&pixel_iter_file),"Loads the pixel information data. Allowing for an illustration of the pixel data over time")
    ("control-network-file,n",po::value<std::string>(&control_net_file),"Loads the control network for point and camera relationship status. Camera and Point Iteration data is need before a control network file can be used.")
    ("windowed","Sets the Bundlevis to be rendered in a window")
    ("prefix",po::value<std::string>(&file_prefix)->default_value("left_"),"Sets the prefix for the image files to be loaded")
    ("postfix",po::value<std::string>(&file_postfix)->default_value(".png"),"Sets the file extension for the image files to be loaded")
    ("help","Display this help message");

  po::variables_map vm;
  po::store(po::parse_command_line(argc,argv,general_options),vm);
  po::notify(vm);

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <filename> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  if (vm.count("help")){
    std::cout<<usage.str();
    return 1;
  }

  //////////////////////////////////////////////////////////
  //Did the User mess up?
  if (!vm.count("points-iteration-file") && !vm.count("camera-iteration-file")){

    std::cout << usage.str();
    return 1;

  }

  /////////////////////////////////////////////////////////
  //Loading up camera iteration data
  if (vm.count("camera-iteration-file")){
    
    std::cout << "Loading Camera Iteration File: " << camera_iter_file << "\n";

    //Storing camera data into a large vector
    cameraData = loadCamerasData( camera_iter_file );

  }

  /////////////////////////////////////////////////////////
  //Loading up point iteration data
  if (vm.count("points-iteration-file")){

    std::cout << "Loading Points Iteration File: " << points_iter_file << "\n";

    //Storing point data in large vector
    pointData = loadPointsData( points_iter_file );

  }

  //////////////////////////////////////////////////////////
  //Loading up pixel iteration data
  if (vm.count("pixel-iteration-file") && vm.count("points-iteration-file") && vm.count("camera-iteration-file") ){
    
    imageTexture = new osg::Texture2D;
    osg::Image* nasaImage = osgDB::readImageFile("nasa-logo.gif");
    if (!nasaImage->valid())
      std::cout << "Error loading NASA.gif" << std::endl;
    imageTexture->setImage(nasaImage);

    std::cout << "Loading Pixel Iteration File: " << pixel_iter_file << "\n";

    //Storing pixel data inside camera data
    loadPixelData( pixel_iter_file, pointData );

    //Also attaching image data... if there is any
    loadImageData( cameraData , file_prefix , file_postfix );
  }

  //////////////////////////////////////////////////////////
  //Loading the control network
  if (vm.count("control-network-file") && (pointData->size()) && (cameraData->size())){

    cnet = new vw::camera::ControlNetwork("Bundlevis");
    
    std::cout << "Loading Control Network File: " << control_net_file << "\n";
    
    cnet->read_control_network(control_net_file);

    if (pointData){
      if (cnet->size() != pointData->size()){
	std::cout << "ERROR: Control Network doesn't seem to match point data!\n";
	cnet = NULL;
      }
    }

  }

  //////////////////////////////////////////////////////////
  //Performing some checks on data integrity

  // ... rewrite some time?

  //////////////////////////////////////////////////////////
  //Building the scene
  std::cout << "Building Scene\n";
  osg::ref_ptr<osg::Sequence> sceneSeq = createScene(pointData, cameraData, cnet);
  root->addChild(sceneSeq.get());

  //////////////////////////////////////////////////////////
  //Setting up window if requested
  if ( vm.count( "windowed" ) ){
    std::cout << "Building Window\n";
    
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 100;
    traits->y = 100;
    traits->width = 1280;
    traits->height = 1024;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->windowName = PROGRAM_NAME;
    traits->vsync = true;
    traits->useMultiThreadedOpenGLEngine = false;
    traits->supportsResize = true;
    traits->useCursor = true;

    osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());

    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setGraphicsContext(gc.get());
    camera->setViewport(new osg::Viewport(0,0, traits->width, traits->height));
    GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
    camera->setDrawBuffer(buffer);
    camera->setReadBuffer(buffer);

    viewer.addSlave(camera.get());
  }  

  //////////////////////////////////////////////////////////
  //Adding HUD and setting up to draw the whole scene
  std::cout << "Building HUD\n";
  root->addChild(createHUD(updateText.get(),imageTexture.get()));
  osgGA::TrackballManipulator* camera = new osgGA::TrackballManipulator();
  viewer.setCameraManipulator(camera);

  //////////////////////////////////////////////////////////
  //Optimizing and attaching scene
  osgUtil::Optimizer optimizer;
  optimizer.optimize(root.get());
  viewer.setSceneData(root.get());
  viewer.addEventHandler(new AllEventHandler(sceneSeq.get(),updateText.get(),imageTexture.get(),camera,pointData,cameraData,cnet,root.get()));

  //Setting up render options for the entire group, as they are not done on a node basis anymore
  osg::StateSet* stateSet = new osg::StateSet();
  stateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
  stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
  stateSet->setBinNumber(11);
  root->setStateSet(stateSet);
  
  ///////////////////////////////////////////////////////////
  //A fix so small features don't disappear
  osg::CullStack::CullingMode cullingMode = viewer.getCamera()->getCullingMode();
  cullingMode &= ~(osg::CullStack::SMALL_FEATURE_CULLING);
  viewer.getCamera()->setCullingMode( cullingMode );
  std::cout << "Drawing Scene\n";
  viewer.realize();

  viewer.run();

  std::cout << "Ending\n";

  return 0;
}

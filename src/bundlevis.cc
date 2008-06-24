//NOTE: Sorry this program is currently under some rapid
//developement. So this code is really hard to read. In the future I
//will break this program up and make it more readable with some well
//written discriptions.

//Hopefully cut down in some of the organizational mess
#include "bundlevis.h"

#define line_length 1.0f

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

/*******************************************
* Constructor for CameraInTime             *
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
  entire_pointString->addChild(pointData->at(pointData->size() - 1)->getSelectCube(line_length/4.0f));

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
    entire_cameraString->addChild(cameraData->at(j)->get3Axis(line_length, ((float)j + 1)/(float)cameraData->size()));

    //TODO: Someday in the future I'd like to have line length be an
    //option that can be brought up and set from the command line.
  }

  //I'm going to put a select cube only on the last camera instance
  //drawn. Which should be the latest time plotted.
  entire_cameraString->addChild(cameraData->at(cameraData->size() - 1)->getSelectCube(line_length/2.0f));

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
  char c;
  while (!iFile.eof()){
    c=iFile.get();
    if (c == '\n'){
      int buffer;
      numLines++;
      iFile >> buffer;
      if (buffer > numCameras)
	numCameras = buffer;
    }
  }
  numCameras++;
  numTimeIter=numLines/(numCameras*6);

  std::cout << "The number of lines: " << numLines << " Num of Cameras: " << numCameras << std::endl;

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
    }
  }

  iFile.close();

  return cameraData;
}

/********************************************************************
* createHUD                                                         *
*  This function creates the HUD of the screen. The passed          *
*  pointer to text, is the text label that keeps changing based on  *
*  what the user has selected.                                      *
********************************************************************/
osg::Node* createHUD(osgText::Text* updateText){
  osg::Camera* hudCamera = new osg::Camera;
  hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  hudCamera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
  hudCamera->setViewMatrix(osg::Matrix::identity());
  hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
  hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);

  std::string arialFont("fonts/arial.ttf");

  //Turning off lighting and depth testing so the text is always in view

  //Setting up label at bottom of the screen
  osg::Geode* geode = new osg::Geode();
  osg::StateSet* stateSet = new osg::StateSet();
  stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
  geode->setName("The text label of selected part");
  geode->addDrawable(updateText);
  geode->setStateSet(stateSet);
  hudCamera->addChild(geode);

  //Setting up some of the text properties
  updateText->setCharacterSize(20.0);
  updateText->setFont(arialFont);
  updateText->setColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
  updateText->setText("Bundlevis v1.1");
  updateText->setPosition(osg::Vec3(50.0f,200.0f,0.0f));
  updateText->setDataVariance(osg::Object::DYNAMIC);

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
    for (int i = 0; i < pointData->size(); ++i){            //Drawing points
      firstFrame->addChild(PointString(pointData->at(i)));
    }
  }

  if (cameraData){
    for (int j = 0; j < cameraData->size(); ++j){           //Drawing cameras
      firstFrame->addChild(CameraString(cameraData->at(j)));
    }
  }

  if (cnet){
    osg::ref_ptr<osg::Switch> tieSwitch = new osg::Switch();
    for (int p = 0; p < cnet->size(); ++p){            //Drawing connecting lines
      //DEBUG
      //std::cout << "Drawing lines for point: " << p << std::endl;
      for (int m = 0; m < (*cnet)[p].size(); ++m){
	//DEBUG
	//std::cout << "Found point " << p << " in image " << m << std::endl;
	//Hopefully at this time I have found a relations ship between point p, and camera m
	if ((*cnet)[p][m].image_id() < cameraData->size()){
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
      for (int i = 0; i < pointData->size(); ++i){
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
      for (int j = 0; j < cameraData->size(); ++j){
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
      for (int p = 0; p < cnet->size(); ++p){
	for(int m = 0; m < (*cnet)[p].size(); ++m){
	  
	  if ((*cnet)[p][m].image_id() < cameraData->size()){
	    
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
      os << hitr->nodePath.back()->getName() << std::endl;

      //I'm going to move the center
      if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH){
	std::string temp = hitr->nodePath.back()->getName();
	std::vector<std::string> data = tokenize(temp," ");

	int item_identity = std::atoi(data[1].c_str());
	int time_stamp = std::atoi(data[3].c_str());

	if (data[0] == "Camera:"){

	  camera_->setCenter(*(cameraData_->at(item_identity)->at(time_stamp)->getCenter()));

	  //Also when they click we'll set what lines are currently on
	  SwitchNamedNodes finder;
	  for (int p = 0; p < (*cnet_).size(); ++p){
	    for (int m = 0; m < (*cnet_)[p].size(); ++m){
	      if ((*cnet_)[p][m].image_id() == item_identity){  //We've found a point that was seen by this camera

		std::ostringstream os;

		os << "LINE CAM " << item_identity << " PNT " << p << std::endl;

		finder.addNameToFind(os.str());

	      }
	    }
	  }

	  finder.apply(*(root_.get()->getChild(0)));

	} else if (data[0] == "Point:"){

	  camera_->setCenter(*(pointData_->at(item_identity)->at(time_stamp)->getCenter()));

	  //Also when they click we'll set what lines are currently on
	  SwitchNamedNodes finder;
	  for (int m = 0; m < (*cnet_)[item_identity].size(); ++m){

	    std::ostringstream os;

	    os << "LINE CAM " << (*cnet_)[item_identity][m].image_id() << " PNT " << item_identity << std::endl;

	    finder.addNameToFind(os.str());

	  }

	  finder.apply(*(root_.get()->getChild(0))); 

	}

      } else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE) {
	std::string temp = hitr->nodePath.back()->getName();
	std::vector<std::string> data = tokenize(temp," ");

	int item_identity = std::atoi(data[1].c_str());
	int time_stamp = std::atoi(data[3].c_str());

	if (item_identity >= cameraData_->size()){         //It must be a point

	  debug << "In Images: ";
	  for (int i = 0; i < (*cnet_)[item_identity].size(); ++i){
	    debug << (*cnet_)[item_identity][i].image_id() << " ";
	  }
	  debug << std::endl;

	} 
      }
    }

    itemFound += os.str();
    itemFound += debug.str();
        
  }
  
  if (itemFound != ""){
    //itemFound = "Bundlevis v1.1";
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
  std::string control_net_file;
  std::vector<std::vector<PointInTime*>*>* pointData;
  std::vector<std::vector<CameraInTime*>*>* cameraData;
  vw::camera::ControlNetwork* cnet;

  //OpenSceneGraph Variable which are important overall
  osgViewer::Viewer viewer;
  osg::ref_ptr<osg::Group> root = new osg::Group();
  osg::ref_ptr<osgText::Text> updateText = new osgText::Text;

  //This first half of the program is wholly devoted to setting up boost program options
  //Boost program options code
  po::options_description general_options("Your most awesome Options");
  general_options.add_options()
    ("camera-iteration-file,c",po::value<std::string>(&camera_iter_file),"Load the camera parameters for each iteration from a file")
    ("points-iteration-file,p",po::value<std::string>(&points_iter_file),"Load the 3d points parameters for each iteration from a file")
    ("control-network-file,n",po::value<std::string>(&control_net_file),"Loads the control network for point and camera relationship status. Camera and Point Iteration data is need before a control network file can be used.")
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
  //Loading the control network
  if (vm.count("control-network-file") && (pointData) && (cameraData)){

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
  //Adding HUD and setting up to draw the whole scene
  root->addChild(createHUD(updateText.get()));
  osgGA::TrackballManipulator* camera = new osgGA::TrackballManipulator();
  viewer.setCameraManipulator(camera);
  viewer.setSceneData(root.get());
  viewer.addEventHandler(new AllEventHandler(sceneSeq.get(),updateText.get(),camera,pointData,cameraData,cnet,root.get()));

  //Setting up render options for the entire group, as they are not done on a node basis anymore
  osg::StateSet* stateSet = new osg::StateSet();
  stateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
  stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
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

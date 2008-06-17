//NOTE: Sorry this program is currently under some rapid
//developement. So this code is really hard to read. In the future I
//will break this program up and make it more readable with some well
//written discriptions.

//BOOST files
#include <boost/program_options.hpp>
namespace po = boost::program_options;

//OpenSceneGraph files
#include <osg/Geode>
#include <osg/Point>
#include <osg/Group>
#include <osg/Geometry>
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

#define line_length 1.0f

// class to handle events for clicking on items
class PickHandler : public osgGA::GUIEventHandler {
protected:
  osg::ref_ptr<osgText::Text> _updateText;
public:
  PickHandler(osgText::Text* updateText):
    _updateText(updateText){}

  ~PickHandler() {}

  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

  virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

  void setLabel(const std::string& name)
  {
    //if update text still exists
    if (_updateText.get())
      _updateText->setText(name);
  }
};

//General handler is my guess
bool PickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  switch(ea.getEventType())
  {
  case(osgGA::GUIEventAdapter::PUSH):
    {
      osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
      if (view) pick(view,ea);
      return false;
    }
  default:
    return false;
  }
}

//Actually performs the math, is called by PickHandler::pick
void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  std::string itemFound="";
  float x = ea.getX();
  float y = ea.getY();

  if (view->computeIntersections(x,y,intersections)){
    // I'm just going to list off the top most intersection. I don't
    // care about the things below the top item.

    for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
	hitr != intersections.end();
	++hitr){
      
      std::ostringstream os;
      if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
      {
	std::cout << "Hit1" << std::endl;
	os << "Object1 " << hitr->nodePath.back()->getName().empty() << std::endl;
      } else if (hitr->drawable.valid()) {
	std::cout << "Hit2" << std::endl;
	os << "Object2 " << hitr->drawable->className() << std::endl;
      }
      

      itemFound += os.str();

    }
  }
  setLabel(itemFound);
}

//Function to create a HUD element
osg::Node* createHUD(osgText::Text* updateText)
{
  osg::Camera* hudCamera = new osg::Camera;
  hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  hudCamera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
  hudCamera->setViewMatrix(osg::Matrix::identity());
  hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
  hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);

  std::string arialFont("fonts/arial.ttf");

  //Turning off lighting and depth test so the text is always in view

  //Setting up label at bottom of the screen
  osg::Geode* geode = new osg::Geode();
  osg::StateSet* stateset = geode->getOrCreateStateSet();
  stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
  geode->setName("The text label of selected part");
  geode->addDrawable(updateText);
  hudCamera->addChild(geode);
  
  updateText->setCharacterSize(20.0);
  updateText->setFont(arialFont);
  updateText->setColor(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
  updateText->setText("visCamPoints v1.0");
  updateText->setPosition(osg::Vec3(50.0f,50.0f,0.0f));
  updateText->setDataVariance(osg::Object::DYNAMIC);

  return hudCamera;
}

//Function to create selection point QUAD, feed this into a billboard
osg::Drawable* buildSelectQuad(const float & scale,const osg::Vec4f& color){
  float edge = 1.0f;
  edge *= scale;

  osg::Geometry* selectQuad = new osg::Geometry;

  osg::Vec3Array* selectVerts = new osg::Vec3Array(4);
  (*selectVerts)[0] = osg::Vec3(-edge/2.0f,0,-edge/2.0f);
  (*selectVerts)[1] = osg::Vec3(edge/2.0f,0,-edge/2.0f);
  (*selectVerts)[2] = osg::Vec3(edge/2.0f,0,edge/2.0f);
  (*selectVerts)[3] = osg::Vec3(-edge/2.0f,0,edge/2.0f);

  selectQuad->setVertexArray(selectVerts);
  
  selectQuad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

  osg::Vec4Array* colorArray = new osg::Vec4Array;
  colorArray->push_back(color);
  selectQuad->setColorArray(colorArray);
  selectQuad->setColorBinding(osg::Geometry::BIND_OVERALL);

  osg::StateSet* setState = new osg::StateSet;

  setState->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
  setState->setMode(GL_BLEND, osg::StateAttribute::ON);  

  selectQuad->setStateSet(setState);

  return selectQuad;
}

//Function to load up point data
std::vector<osg::Vec3Array* >* loadPointsData(std::string pntFile){
  //I'm going to determine the number of lines in the file
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

  //The big one!
  std::vector<osg::Vec3Array*> * pointData = new std::vector<osg::Vec3Array*>(numPoints);
  for (int i = 0; i < numPoints; ++i){
    pointData->at(i) = new osg::Vec3Array(numTimeIter);
  }

  //For every time iteration
  for(int t = 0; t < numTimeIter; ++t){
    //For every point
    for(int i = 0; i < numPoints; ++i){
      float fill_buffer;
      iFile >> fill_buffer;            //This first one should be the camera number
      if (fill_buffer != i) {
	std::cout << "Error reading " << i << " " << fill_buffer << std::endl;
	break;
      }
      iFile >> fill_buffer;
      pointData->at(i)->at(t)[0] = fill_buffer;
      iFile >> fill_buffer;
      pointData->at(i)->at(t)[1] = fill_buffer;
      iFile >> fill_buffer;
      pointData->at(i)->at(t)[2] = fill_buffer;
  
      //std::cout << pointData->at(i)->at(t).x() << " " << pointData->at(i)->at(t).y() << " " << pointData->at(i)->at(t).z() << std::endl;
    }
  }

  iFile.close();
  

  return pointData;
}

//Function to load up camera data
std::vector<std::vector<osg::Vec3Array*>*>* loadCameraData(std::string camFile){
  //I'm going to detemine the number of lines in the file
  std::ifstream iFile(camFile.c_str(), std::ios::in);
  int numLines = 0;   //Number of lines in the file
  int numCameras = 0;  //Number of camera in file
  int numTimeIter = 0; //Number of time iterations
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
  numTimeIter=numLines/(6*numCameras);

  //Reset the file reading to the front
  iFile.clear();
  iFile.seekg(0);
  
  std::cout << "The number of lines: " << numLines << " Num of Cameras: " << numCameras << std::endl;

  //Building the giant cluster of data types, declaring?
  std::vector<std::vector<osg::Vec3Array*>* >* cameraData = new std::vector<std::vector<osg::Vec3Array*>* >(numCameras);
  for (int j = 0; j < numCameras; ++j){        //Working through all the cameras
    cameraData->at(j) = new std::vector<osg::Vec3Array*>(6);   //For 6 parameters
    for (int p = 0; p < 6; ++p){               //Working through the 6 parameters CAHVOR
      cameraData->at(j)->at(p) = new osg::Vec3Array(numTimeIter);
    }
  }

  //Actually filling in the data type with the values;
  for(int t = 0; t < numTimeIter; ++t){
    for(int j = 0; j < numCameras; ++j){
      for(int p = 0; p < 6; ++p){
	float fill_buffer;
	iFile >> fill_buffer;    //This first one is the camera number
	if (fill_buffer != j) {
	  std::cout << "Error reading!" << std::endl;
	  break;
	}
	iFile >> fill_buffer;
	cameraData->at(j)->at(p)->at(t)[0] = fill_buffer;
	iFile >> fill_buffer;
	cameraData->at(j)->at(p)->at(t)[1] = fill_buffer;
	iFile >> fill_buffer;
	cameraData->at(j)->at(p)->at(t)[2] = fill_buffer;
      }
    }
  }

  iFile.close();

  return cameraData;
}

osg::Group* buildNew3Axis(const osg::Vec3f* c, const osg::Vec3f* a, const osg::Vec3f* h, const osg::Vec3f* v, const float& opacity){
  osg::Group* theWholeSchbang = new osg::Group();
  osg::Geometry* the3Axis = new osg::Geometry();
  osg::Vec3f temp;

  //Going to build the vertices require for the 3 axis
  osg::Vec3Array* lineData = new osg::Vec3Array();
  lineData->push_back((*c));
  temp = (*h);
  temp.normalize();
  lineData->push_back((*c) + temp*line_length);
  lineData->push_back((*c));
  temp = (*v);
  temp.normalize();
  lineData->push_back((*c) + temp*line_length);
  lineData->push_back((*c));
  temp = (*a);
  temp.normalize();
  lineData->push_back((*c) + temp*line_length);

  //Apply them to the new geometry
  the3Axis->setVertexArray(lineData);
  
  //Setting up the colors
  osg::Vec4Array* colours = new osg::Vec4Array();
  colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
  colours->push_back(osg::Vec4(1.0f,0.0f,0.0f,opacity));
  colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
  colours->push_back(osg::Vec4(0.0f,1.0f,0.0f,opacity));
  colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
  colours->push_back(osg::Vec4(0.0f,0.0f,1.0f,opacity));
  the3Axis->setColorArray(colours);
  the3Axis->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  //Add the primitive to draw the lines using the vertice array
  the3Axis->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,lineData->getNumElements()));

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(the3Axis);
  geode->setName("Camera");

  //Setting up some state parameters
  osg::StateSet* stateSet = new osg::StateSet();
  geode->setStateSet(stateSet);
  stateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
  stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  stateSet->setAttribute(new osg::LineWidth(2));

  //I'm also going to add a select point in the middle of the camera
  //so I can click it

  osg::Billboard* selectPoint = new osg::Billboard();
  theWholeSchbang->addChild(selectPoint);
  selectPoint->setName("Camera test2");

  selectPoint->setMode(osg::Billboard::POINT_ROT_EYE);
  selectPoint->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));   //I think this
							//is point at
							//the camera
  osg::Drawable* selectDrawable = buildSelectQuad(.5f,osg::Vec4f(1.0f,1.0f,1.0f,opacity));

  selectPoint->addDrawable(selectDrawable,(*c));

  //Must organize code
  theWholeSchbang->addChild(geode);

  return theWholeSchbang;
}

osg::Geode* buildNewPointPlot(osg::Vec3Array* vertices){
  osg::Geometry* theWholeLine = new osg::Geometry();
  
  theWholeLine->setVertexArray(vertices);

  //Setting up the colors
  osg::Vec4Array* colours = new osg::Vec4Array(vertices->getNumElements());
  theWholeLine->setColorArray(colours);
  for (unsigned int i =0; i < vertices->getNumElements(); i++){
    (*colours)[i].set(1.0f,((float)i+1)/(float)vertices->getNumElements(),((float)i+1)/(float)vertices->getNumElements(),((float)i+1)/(float)vertices->getNumElements());
  }
  theWholeLine->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

  //Adding the primitive set to draw lines
  theWholeLine->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,vertices->getNumElements()));

  //Adding the primitive to draw points as well
  theWholeLine->addPrimitiveSet(new osg::DrawArrays(GL_POINTS,0,vertices->getNumElements()));

  osg::Geode* geode = new osg::Geode();
  geode->addDrawable(theWholeLine);
  geode->setName("Point Plot");

  //Setting up some state parameters
  osg::StateSet* stateSet = new osg::StateSet();
  geode->setStateSet(stateSet);
  stateSet->setMode(GL_BLEND,osg::StateAttribute::ON);
  stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
  //stateSet->setAttribute(new osg::LineWidth(5));

  //Going to make the point size larger
  osg::Point* point = new osg::Point();
  point->setSize(5.0f);
  stateSet->setAttribute(point);

  return geode;
}

int main(int argc, char* argv[]){
  //Variable Initialization
  std::string camera_iter_file;
  std::string points_iter_file;

  //OpenSceneGraph variables  osg::Group* root = new osg::Group();
  osgViewer::Viewer viewer;
  osg::Group* root = new osg::Group();
  osg::ref_ptr<osgText::Text> updateText = new osgText::Text;

  //Boost program options code
  po::options_description general_options("Your most awesome Options");
  general_options.add_options()
    ("camera-iteration-file,c",po::value<std::string>(&camera_iter_file),"Load the camera parameters for each iteration from a file")
    ("points-iteration-file,p",po::value<std::string>(&points_iter_file),"Load the 3d points parameters for each iteration from a file")
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

  //Loading up camera iteration data
  if(vm.count("camera-iteration-file")){
    std::cout << "Loading Camera Iteration File: " << camera_iter_file << "\n"; 

    //Storing camera data in large vector^3
    std::vector<std::vector<osg::Vec3Array*>*>* cameraData;
    cameraData = loadCameraData(camera_iter_file);

    //Plotting all of time
    for (unsigned int t = 0; t < cameraData->at(0)->at(0)->getNumElements(); ++t){
      float opacity = .1;
      if (t == cameraData->at(0)->at(0)->getNumElements())
	opacity = 1.0;
      for (unsigned int j = 0; j < cameraData->size() ; ++j){
	root->addChild(buildNew3Axis(&cameraData->at(j)->at(0)->at(t),&cameraData->at(j)->at(1)->at(t),&cameraData->at(j)->at(2)->at(t),&cameraData->at(j)->at(3)->at(t), opacity));
      }
    }
    
  }

  //Loading up point iteration data
  if(vm.count("points-iteration-file")){
    std::cout << "Loading Points Iteration File: " << points_iter_file << "\n";

    //Storing point data in large vector^3
    std::vector<osg::Vec3Array* >* pointData;
    pointData= loadPointsData(points_iter_file);

    //Plot them all ....AHAHAHAHA!
    for (unsigned int i = 0; i < pointData->size(); ++i){
      root->addChild(buildNewPointPlot(pointData->at(i)));
    }
  }

  //User failed to input anything
  if(!vm.count("points-iteration-file") && !vm.count("camera-iteration-file")){
    std::cout << usage.str();
    return 1;
  }

  //Adding the HUD
  root->addChild(createHUD(updateText.get()));

  viewer.setCameraManipulator(new osgGA::TrackballManipulator());
  viewer.setSceneData(root);
  viewer.addEventHandler(new PickHandler(updateText.get()));
  viewer.realize();

  while (!viewer.done()){
    viewer.frame();
  }

  return 0;
}

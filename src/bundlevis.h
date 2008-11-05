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

/// \file bundlevis.h
///

#ifndef _BUNDLEVIS_H_
#define _BUNDLEVIS_H_

#include <boost/algorithm/string.hpp>
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
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <osg/LineWidth>
#include <osgText/Text>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osgUtil/Optimizer>
#include <osg/Texture2D>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>

//Standard
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>

//VisionWorkbench
#include <vw/Camera/ControlNetwork.h>
#include <vw/Math.h>

// PointIter, the lowest quantum of points
class PointIter : public osg::Referenced {
 public:
  PointIter (const int& ID, int* step) {
    _ID = ID;
    _step = step;
    _drawConnLines = false;
    _isGCP = false;

    std::ostringstream os;
    os << "Point: " << (_ID+1);
    _description = os.str();
  }
  int getStep(void) { return (*_step); }
  unsigned size(void) { return _position.size(); }
  void addIteration( osg::Vec3f& newPos, float const& newError ) {
    _position.push_back( newPos );
    _error.push_back( newError );
  }
  const osg::Vec3f getPosition( const int& step ) {
    return _position.at( step );
  }
  const float getError( const int& step ) {
    return _error.at( step );
  }
  void setDrawConnLines( bool value ) {
    _drawConnLines = value;
  }
  bool getDrawConnLines( void ) {
    return _drawConnLines;
  }
  void setGCP( bool value ) {
    _isGCP = value;
  }
  bool getGCP(void ) {
    return _isGCP;
  }
  const std::string getDescription( void ) {
    return _description;
  }
 private:
  int* _step;
  int _ID;
  bool _drawConnLines;
  bool _isGCP;
  std::string _description;
  std::vector<osg::Vec3f> _position;
  std::vector<float> _error;
  vw::camera::ControlPoint* _controlPoint;
};

// CameraIter, the lowest quantum of cameras
class CameraIter : public osg::Referenced  {
 public:
  CameraIter (const int& ID, int* step) {
    _ID = ID;
    _step = step;
    _vertices = 1;
    _drawConnLines = false;
    _isPushbroom = false;

    std::ostringstream os;
    os << "Camera: " << (_ID+1);
    _description = os.str();
  }
  CameraIter (const int& ID, int* step, const int& vertices) {
    _ID = ID;
    _step = step;
    _vertices = vertices;
    _drawConnLines = false;
    _isPushbroom = true;

    std::ostringstream os;
    os << "Camera: " << (_ID+1);
    _description = os.str();
  }
  int getStep(void) { return (*_step); }
  unsigned size(void) { return _position.size()/_vertices; }
  void addIteration( const osg::Vec3f& newPos, const osg::Vec3f& newEuler ) {
    if (!_isPushbroom) {
      _position.push_back( newPos );
      _euler.push_back( newEuler );
    } else {
      std::cout << "ERROR CAMERA: Trying to add frame like data to non-frame camera" << std::endl;
    }
  }
  void addIteration( const std::vector<osg::Vec3f>& newPos,
		     const std::vector<osg::Vec3f>& newEuler ) {
    if (_isPushbroom){
      if ( int(newPos.size()) == _vertices ) {
	for (int i = 0; i < _vertices; ++i) {
	  _position.push_back( newPos[i] );
	  _euler.push_back( newEuler[i] );
	}
      } else {
	std::cout << "ERROR PUSHBROOM CAMERA: This data doesn't look like it belongs to me" << std::endl;
      }
    } else {
      std::cout << "ERROR CAMERA: Trying to add pushbroom like data to non-pushbroom camera" << std::endl;
    }
  }
  const osg::Vec3f getPosition( const int& step ) {
    return _position.at( step*_vertices );
  }
  const osg::Vec3f getEuler( const int& step ) {
    return _euler.at( step*_vertices );
  }
  const osg::Vec3f getPosition( const int& step, const int& vert ) {
    return _position.at( step*_vertices + vert );
  }
  const osg::Vec3f getEuler( const int& step, const int& vert ) {
    return _euler.at( step*_vertices + vert );
  }
  const std::string getDescription( void ) {
    return _description;
  }
  void setDrawConnLines( bool value ) {
    _drawConnLines = value;
  }
  bool getDrawConnLines( void ) {
    return _drawConnLines;
  }
  bool getIsPushbroom( void ) {
    return _isPushbroom;
  }
  int getVertices( void ) {
    return _vertices;
  }
  osg::MatrixTransform* buildMatrixTransform( const int& step, const int& vert );

 private:
  int* _step;
  int _ID;
  int _vertices;
  bool _drawConnLines;
  bool _isPushbroom;
  std::string _description;
  std::vector<osg::Vec3f> _position;
  std::vector<osg::Vec3f> _euler;
};

// ConnLinesIter, the quanta of connecting lines between point and cameras
class ConnLineIter {
 public:
  ConnLineIter (PointIter* point, CameraIter* camera, int* step) {
    _point = point;
    _camera = camera;
    _step = step;
    colour.set(0.5f,0.5f,0.5f,1.0f);
  }
  int getStep(void) { return (*_step); }
  bool getToDraw( void ) {
    return _point->getDrawConnLines() || _camera->getDrawConnLines();
  }
  PointIter* getPoint(void) { return _point; };
  CameraIter* getCamera(void) { return _camera; };
 private:
  int* _step;
  PointIter* _point;
  CameraIter* _camera;
  osg::Vec4f colour;
};

// This will load a point data file
std::vector<PointIter*> loadPointData( std::string pntFile,
				       int* step );

// This will load a camera data file
std::vector<CameraIter*> loadCameraData( std::string camFile,
					 int* step );

// This will load a control network file
std::vector<ConnLineIter*> loadControlNet( std::string cnetFile ,
					   std::vector<PointIter*>& points,
					   std::vector<CameraIter*>& cameras,
					   vw::camera::ControlNetwork* cnet,
					   int* step );

// This builds the entire scene
osg::Node* createScene( std::vector<PointIter*>& points,
			std::vector<CameraIter*>& cameras,
			std::vector<ConnLineIter*>& connLines );

// Playback control
class PlaybackControl {
 public:
  PlaybackControl(int* step){
    _step = step;
    _play = true;
    _stop = false;
    _pause = false;
  }
  void setPlay(){ _play = true; _stop = false; _pause = false; }
  void setPause(){ _play = false; _stop = false; _pause = true; }
  void setStop(){ _play = false; _stop = true; _pause = false; (*_step) = 1; }
  bool getPlay(){ return _play; }
 private:
  int* _step;
  bool _play;
  bool _pause;
  bool _stop;
};

// Event Handler for Mouse and Keyboard
class AllEventHandler : public osgGA::GUIEventHandler{
 public:
  AllEventHandler( int* step, const int& numIter, PlaybackControl* playControl ) {
    _step = step;
    _numIter = numIter;
    _playControl = playControl;
  }
  bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );
  void pick( osgViewer::View* view, const osgGA::GUIEventAdapter& ea );
 private:
  int* _step;
  int _numIter;
  PlaybackControl* _playControl;
};

// This is a draw back for a collection of points in a single geometry
struct pointsDrawCallback : public osg::Drawable::DrawCallback
{
  pointsDrawCallback( std::vector<PointIter*>* points ) {
    _points = points;
    _previousStep = 0;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
				   const osg::Drawable* drawable ) const
  {
    //osg::State& state = *renderInfo.getState();
    
    int buffer = (*_points)[0]->getStep();

    if ( _previousStep != buffer ) {
      // Time to change vector data

      osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      if ( buffer > 1 ) {
	// Here we will draw a line to the just the last step
	
	for ( unsigned i = 0; i < (*_points).size()*2; i+=2 ){
	  (*vertices)[i] = (*_points)[i/2]->getPosition( buffer - 1 );
	  (*vertices)[i+1] = (*_points)[i/2]->getPosition( buffer - 2 );
	}

      } else if ( buffer == 1 ) {
	// There will be no lines, but only points

	for ( unsigned i = 0; i < (*_points).size()*2; i+=2 ){
	  (*vertices)[i] = (*_points)[i/2]->getPosition( 0 );
	  (*vertices)[i+1] = (*vertices)[i];
	}

      } else if ( buffer == 0 ) {
	// Here the line goes between the first and last iteration

	for ( unsigned i = 0; i < (*_points).size()*2; i+=2 ){
	  (*vertices)[i] = (*_points)[i/2]->getPosition( (*_points)[0]->size() - 1 );
	  (*vertices)[i+1] = (*_points)[i/2]->getPosition( 0 );
	}
      }

    }

    _previousStep = buffer;

    drawable->drawImplementation( renderInfo );
  }

  mutable std::vector<PointIter*>* _points;
  mutable int _previousStep;
};

// This is a drawback for the lines representing the path of a pushbroom
struct pushbroomDrawCallback : public osg::Drawable::DrawCallback
{
  pushbroomDrawCallback( CameraIter* camera ) {
    _camera = camera;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
				   const osg::Drawable* drawable ) const
  {
    int buffer = (*_camera).getStep();

    if ( _previousStep != buffer ) {
      // Time to change vector data

      osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      if ( buffer == 0 ) { // Draw the last instance

	for ( int i = 0; i < (*_camera).getVertices(); ++i )
	  (*vertices)[i] = (*_camera).getPosition( _camera->size() - 1, i);

      } else {             // normal
	
	for ( int i = 0; i < (*_camera).getVertices(); ++i )
	  (*vertices)[i] = (*_camera).getPosition( buffer-1, i );

      }
    }

    _previousStep = buffer;

    drawable->drawImplementation( renderInfo );
  }

  mutable CameraIter* _camera;
  mutable int _previousStep;
};

// This is a draw back for the lines connecting cameras and points
struct linesDrawCallback : public osg::Drawable::DrawCallback
{
  linesDrawCallback( ConnLineIter* connLine ) {
    _connLine = connLine;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
				   const osg::Drawable* drawable ) const
  {
    int buffer = _connLine->getStep();

    if ( buffer == 0 )
      buffer = _connLine->getPoint()->size();

    if ( _previousStep != buffer ) {
      osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      (*vertices)[0] = _connLine->getPoint()->getPosition( buffer - 1 );
      (*vertices)[1] = _connLine->getCamera()->getPosition( buffer - 1 );
    }

    _previousStep = buffer;

    if ( _connLine->getToDraw() )
      drawable->drawImplementation( renderInfo );
  }

  mutable ConnLineIter* _connLine;
  mutable int _previousStep;
};

// This is a update callback, it's meant just to work with playback control
class playbackNodeCallback : public osg::NodeCallback
{
 public:
  playbackNodeCallback( int* step , int maxIter, PlaybackControl* playback ){
    _playback = playback;
    _maxIter = maxIter;
    _step = step;
    _delayCount = 0;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv )
  {
    _delayCount++;
    _delayCount&=0x01;
    
    if ( _playback->getPlay() && !_delayCount ) {
      int buffer = (*_step);
      buffer++;
      if ( buffer > _maxIter )
	buffer = 1;
      (*_step) = buffer;
    }
    traverse( node, nv );
  }
 private:
  PlaybackControl* _playback;
  int* _step;
  int _maxIter;
  unsigned _delayCount;
};

// This is an update callback for the matrix that transforms the 3
// axis representing the camera
class cameraMatrixCallback : public osg::NodeCallback
{
 public:
  cameraMatrixCallback( CameraIter* camera, const int& vertice = 0 ) {
    _camera = camera;
    _vertice = vertice;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv )
  {
    int buffer = _camera->getStep();
    
    //When displaying all... just display end
    if ( buffer == 0 )
      buffer = _camera->size();

    if ( buffer != _previousStep ) {
      
      //Moving the transform to reflect the current step
      osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);
     
      osg::Vec3f euler = _camera->getEuler( buffer - 1, _vertice );
      osg::Vec3f position = _camera->getPosition( buffer - 1, _vertice );
      
      vw::Matrix3x3 temp = vw::math::euler_to_rotation_matrix(euler[0],
							      euler[1],
							      euler[2],
							      "xyz" );

      osg::Matrix rot( temp(0,0), temp(0,1), temp(0,2), 0,
		       temp(1,0), temp(1,1), temp(1,2), 0,
		       temp(2,0), temp(2,1), temp(2,2), 0,
		       0, 0, 0, 1);

      osg::Matrix trans( 1,   0,   0,   0,
			 0,   1,   0,   0,
			 0,   0,   1,   0,
			 position[0],   position[1],   position[2],   1 );

      osg::Matrix result( rot*trans );

      mt->setMatrix( result );

    }

    _previousStep = buffer;
    traverse( node, nv );
  }
 private:
  int _previousStep;
  int _vertice;
  CameraIter* _camera;
};

// This is an update callback for the auto matrix that transforms the
// hit square and text for the camera
class cameraAutoMatrixCallback : public osg::NodeCallback
{
 public:
  cameraAutoMatrixCallback( CameraIter* camera ) {
    _camera = camera;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv )
  {
    int buffer = _camera->getStep();
    
    //When display all... just display end
    if ( buffer == 0 )
      buffer = _camera->size();
    
    if ( buffer != _previousStep ) {
      osg::AutoTransform* autoT = dynamic_cast< osg::AutoTransform* >(node);

      autoT->setPosition( _camera->getPosition( buffer - 1 ) );
    }

    _previousStep = buffer;
    traverse( node, nv );
  }
 private:
  int _previousStep;
  CameraIter* _camera;
};

// This is an update callback for the automatrix that transforms the
// hit square and text for the points
class pointAutoMatrixCallback : public osg::NodeCallback
{
 public:
  pointAutoMatrixCallback( PointIter* point ) {
    _point = point;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv )
  {
    int buffer = _point->getStep();
    
    //When display all... just display end
    if ( buffer == 0 )
      buffer = _point->size();

    if ( buffer != _previousStep ) {
      osg::AutoTransform* autoT = dynamic_cast< osg::AutoTransform* >(node);
      
      autoT->setPosition( _point->getPosition( buffer - 1 ) );
    }
  }
 private:
  int _previousStep;
  PointIter* _point;
};

// This just builds the 3 Axis that represents the camera
osg::Geode* build3Axis( void );

#endif

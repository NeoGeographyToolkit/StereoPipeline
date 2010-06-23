// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
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
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <vw/Math.h>

// PointIter, the lowest quantum of points
class PointIter : public osg::Referenced {
    int* m_step;
    int m_ID;
    bool m_drawConnLines;
    bool m_isGCP;
    std::string m_description;
    std::vector<osg::Vec3f> m_position;
    std::vector<float> m_error;
    vw::ba::ControlPoint* m_controlPoint;
 public:
  PointIter (const int& ID, int* step) {
    m_ID = ID;
    m_step = step;
    m_drawConnLines = false;
    m_isGCP = false;

    std::ostringstream os;
    os << "Point: " << m_ID+1;
    m_description = os.str();
  }
  int getStep(void) { return *m_step; }
  unsigned size(void) { return m_position.size(); }
  void addIteration( osg::Vec3f& newPos, float const& newError ) {
    m_position.push_back( newPos );
    m_error.push_back( newError );
  }
  const osg::Vec3f getPosition( const int& step ) {
    return m_position.at( step );
  }
  const float getError( const int& step ) {
    return m_error.at( step );
  }
  void setDrawConnLines( bool value ) {
    m_drawConnLines = value;
  }
  bool getDrawConnLines( void ) {
    return m_drawConnLines;
  }
  void setGCP( bool value ) {
    m_isGCP = value;
  }
  bool getGCP(void ) {
    return m_isGCP;
  }
  const std::string getDescription( void ) {
    return m_description;
  }
};

// CameraIter, the lowest quantum of cameras
class CameraIter : public osg::Referenced  {
    int* m_step;
    int m_ID;
    int m_vertices;
    bool m_drawConnLines;
    bool m_isPushbroom;
    std::string m_description;
    std::vector<osg::Vec3f> m_position;
    std::vector<osg::Vec3f> m_euler;
 public:
    CameraIter (const int& ID, int* step, int const& vertices = 1) {
    m_ID = ID;
    m_step = step;
    m_vertices = vertices;
    m_drawConnLines = false;
    m_isPushbroom = vertices > 1;

    std::ostringstream os;
    os << "Camera: " << m_ID+1;
    m_description = os.str();
  }
  int getStep(void) { return *m_step; }
  unsigned size(void) { return m_position.size()/m_vertices; }
  void addIteration( const osg::Vec3f& newPos,
                     const osg::Vec3f& newEuler ) {
    if (!m_isPushbroom) {
      m_position.push_back( newPos );
      m_euler.push_back( newEuler );
    } else {
      std::cout << "ERROR CAMERA: Trying to add frame like data to non-frame camera" << std::endl;
    }
  }
  void addIteration( const std::vector<osg::Vec3f>& newPos,
                     const std::vector<osg::Vec3f>& newEuler ) {
    if (m_isPushbroom){
      if ( int(newPos.size()) == m_vertices ) {
        for (int i = 0; i < m_vertices; ++i) {
          m_position.push_back( newPos[i] );
          m_euler.push_back( newEuler[i] );
        }
      } else {
        std::cout << "ERROR PUSHBROOM CAMERA: This data doesn't look like it belongs to me" << std::endl;
      }
    } else {
      std::cout << "ERROR CAMERA: Trying to add pushbroom like data to non-pushbroom camera" << std::endl;
    }
  }
  const osg::Vec3f getPosition( const int& step ) {
    return m_position.at( step*m_vertices ); }
  const osg::Vec3f getEuler( const int& step ) {
    return m_euler.at( step*m_vertices ); }
  const osg::Vec3f getPosition( const int& step, const int& vert ) {
    return m_position.at( step*m_vertices + vert ); }
  const osg::Vec3f getEuler( const int& step, const int& vert ) {
    return m_euler.at( step*m_vertices + vert ); }
  const std::string getDescription( void ) {
    return m_description; }
  void setDrawConnLines( bool value ) {
    m_drawConnLines = value; }
  bool getDrawConnLines( void ) {
    return m_drawConnLines; }
  bool getIsPushbroom( void ) {
    return m_isPushbroom; }
  int getVertices( void ) {
    return m_vertices; }
  osg::MatrixTransform* buildMatrixTransform( const int& step, const int& vert );

};

// ConnLinesIter, the quanta of connecting lines between point and cameras
class ConnLineIter {
  int* m_step;
  PointIter* m_point;
  CameraIter* m_camera;
  osg::Vec4f colour;
 public:
  ConnLineIter (PointIter* point, CameraIter* camera, int* step) {
    m_point = point;
    m_camera = camera;
    m_step = step;
    colour.set(0.5f,0.5f,0.5f,1.0f);
  }
  int getStep(void) { return (*m_step); }
  bool getToDraw( void ) {
    return m_point->getDrawConnLines() || m_camera->getDrawConnLines();
  }
  PointIter* getPoint(void) { return m_point; };
  CameraIter* getCamera(void) { return m_camera; };
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
                                           vw::ba::ControlNetwork* cnet,
                                           int* step );

// This builds the entire scene
osg::Node* createScene( std::vector<PointIter*>& points,
                        std::vector<CameraIter*>& cameras,
                        std::vector<ConnLineIter*>& connLines );

// Playback control
class PlaybackControl {
  int* m_step;
  bool m_play;
  bool m_pause;
  bool m_stop;
 public:
  PlaybackControl(int* step){
    m_step = step;
    m_play = true;
    m_stop = false;
    m_pause = false;
  }
  void setPlay(){ m_play = true; m_stop = false; m_pause = false; }
  void setPause(){ m_play = false; m_stop = false; m_pause = true; }
  void setStop(){ m_play = false; m_stop = true; m_pause = false; *m_step = 1; }
  bool getPlay(){ return m_play; }
};

// Event Handler for Mouse and Keyboard
class AllEventHandler : public osgGA::GUIEventHandler{
  int* m_step;
  int m_numIter;
  PlaybackControl* m_playControl;
 public:
  AllEventHandler( int* step, const int& numIter, PlaybackControl* playControl ) {
    m_step = step;
    m_numIter = numIter;
    m_playControl = playControl;
  }
  bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa );
  void pick( osgViewer::View* view, const osgGA::GUIEventAdapter& ea );
};

// This is a draw back for a collection of points in a single geometry
struct pointsDrawCallback : public osg::Drawable::DrawCallback {
  pointsDrawCallback( std::vector<PointIter*>* points ) {
    m_points = points;
    m_previousStep = 0;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
                                   const osg::Drawable* drawable ) const
  {
    //osg::State& state = *renderInfo.getState();

    int buffer = (*m_points)[0]->getStep();

    if ( m_previousStep != buffer ) {
      // Time to change vector data

      osg::Geometry* geometry =
        dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices =
        dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      if ( buffer > 1 ) {
        // Here we will draw a line to the just the last step
        for ( unsigned i = 0; i < (*m_points).size()*2; i+=2 ){
          (*vertices)[i] =
            (*m_points)[i/2]->getPosition( buffer - 1 );
          (*vertices)[i+1] =
            (*m_points)[i/2]->getPosition( buffer - 2 );
        }

      } else if ( buffer == 1 ) {
        // There will be no lines, but only points
        for ( unsigned i = 0; i < (*m_points).size()*2; i+=2 ){
          (*vertices)[i] =
            (*m_points)[i/2]->getPosition( 0 );
          (*vertices)[i+1] =
            (*vertices)[i];
        }

      } else if ( buffer == 0 ) {
        // Here the line goes between the first and last iteration
        for ( unsigned i = 0; i < (*m_points).size()*2; i+=2 ){
          (*vertices)[i] =
            (*m_points)[i/2]->getPosition( (*m_points)[0]->size() - 1 );
          (*vertices)[i+1] =
            (*m_points)[i/2]->getPosition( 0 );
        }
      }

    }

    m_previousStep = buffer;

    drawable->drawImplementation( renderInfo );
  }

  mutable std::vector<PointIter*>* m_points;
  mutable int m_previousStep;
};

// This is a drawback for the lines representing the path of a pushbroom
struct pushbroomDrawCallback : public osg::Drawable::DrawCallback {
  pushbroomDrawCallback( CameraIter* camera ) {
    m_camera = camera;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
                                   const osg::Drawable* drawable ) const
  {
    int buffer = m_camera->getStep();

    if ( m_previousStep != buffer ) {
      // Time to change vector data

      osg::Geometry* geometry =
        dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices =
        dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      if ( buffer == 0 ) { // Draw the last instance
        for ( int i = 0; i < m_camera->getVertices(); ++i )
          (*vertices)[i] = m_camera->getPosition( m_camera->size() - 1, i);

      } else {             // normal

        for ( int i = 0; i < m_camera->getVertices(); ++i )
          (*vertices)[i] = m_camera->getPosition( buffer-1, i );

      }
    }

    m_previousStep = buffer;

    drawable->drawImplementation( renderInfo );
  }

  mutable CameraIter* m_camera;
  mutable int m_previousStep;
};

// This is a draw back for the lines connecting cameras and points
struct linesDrawCallback : public osg::Drawable::DrawCallback {
  linesDrawCallback( ConnLineIter* connLine ) {
    m_connLine = connLine;
  }

  virtual void drawImplementation( osg::RenderInfo& renderInfo,
                                   const osg::Drawable* drawable ) const {
    int buffer = m_connLine->getStep();

    if ( buffer == 0 )
      buffer = m_connLine->getPoint()->size();

    if ( m_previousStep != buffer ) {
      osg::Geometry* geometry =
        dynamic_cast<osg::Geometry*>(const_cast<osg::Drawable*>(drawable));
      osg::Vec3Array* vertices =
        dynamic_cast<osg::Vec3Array*>(geometry->getVertexArray());

      (*vertices)[0] = m_connLine->getPoint()->getPosition( buffer - 1 );
      (*vertices)[1] = m_connLine->getCamera()->getPosition( buffer - 1 );
    }

    m_previousStep = buffer;

    if ( m_connLine->getToDraw() )
      drawable->drawImplementation( renderInfo );
  }

  mutable ConnLineIter* m_connLine;
  mutable int m_previousStep;
};

// This is a update callback, it's meant just to work with playback control
class playbackNodeCallback : public osg::NodeCallback {
  PlaybackControl* m_playback;
  int* m_step;
  int m_maxIter;
  unsigned m_delayCount;
 public:
  playbackNodeCallback( int* step , int maxIter, PlaybackControl* playback ){
    m_playback = playback;
    m_maxIter = maxIter;
    m_step = step;
    m_delayCount = 0;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv )
  {
    m_delayCount++;
    m_delayCount &= 0x01;

    if ( m_playback->getPlay() && !m_delayCount ) {
      int buffer = *m_step;
      buffer++;
      if ( buffer > m_maxIter )
        buffer = 1;
      *m_step = buffer;
    }
    traverse( node, nv );
  }
};

// This is an update callback for the matrix that transforms the 3
// axis representing the camera
class cameraMatrixCallback : public osg::NodeCallback {
  int m_previousStep;
  int m_vertice;
  CameraIter* m_camera;
 public:
  cameraMatrixCallback( CameraIter* camera, const int& vertice = 0 ) {
    m_camera = camera;
    m_vertice = vertice;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv ) {
    int buffer = m_camera->getStep();

    //When displaying all... just display end
    if ( buffer == 0 )
      buffer = m_camera->size();

    if ( buffer != m_previousStep ) {
      //Moving the transform to reflect the current step
      osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(node);

      osg::Vec3f euler = m_camera->getEuler( buffer - 1, m_vertice );
      osg::Vec3f position = m_camera->getPosition( buffer - 1, m_vertice );

      vw::Matrix3x3 temp =
        vw::math::euler_to_rotation_matrix(euler[0],
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

    m_previousStep = buffer;
    traverse( node, nv );
  }
};

// This is an update callback for the auto matrix that transforms the
// hit square and text for the camera
class cameraAutoMatrixCallback : public osg::NodeCallback {
  int m_previousStep;
  CameraIter* m_camera;
 public:
  cameraAutoMatrixCallback( CameraIter* camera ) {
    m_camera = camera;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* nv ) {
    int buffer = m_camera->getStep();

    //When display all... just display end
    if ( buffer == 0 )
      buffer = m_camera->size();

    if ( buffer != m_previousStep ) {
      osg::AutoTransform* autoT = dynamic_cast< osg::AutoTransform* >(node);

      autoT->setPosition( m_camera->getPosition( buffer - 1 ) );
    }

    m_previousStep = buffer;
    traverse( node, nv );
  }
};

// This is an update callback for the automatrix that transforms the
// hit square and text for the points
class pointAutoMatrixCallback : public osg::NodeCallback {
  int m_previousStep;
  PointIter* m_point;
 public:
  pointAutoMatrixCallback( PointIter* point ) {
    m_point = point;
  }
  virtual void operator() ( osg::Node* node, osg::NodeVisitor* /*nv*/ ) {
    int buffer = m_point->getStep();

    //When display all... just display end
    if ( buffer == 0 )
      buffer = m_point->size();

    if ( buffer != m_previousStep ) {
      osg::AutoTransform* autoT = dynamic_cast< osg::AutoTransform* >(node);

      autoT->setPosition( m_point->getPosition( buffer - 1 ) );
    }
  }
};

// This just builds the 3 Axis that represents the camera
osg::Geode* build3Axis( void );

#endif

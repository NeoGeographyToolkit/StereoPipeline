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


/// \file bundlevis.cc
///

#include <vw/Core/ProgressCallback.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Statistics.h>
#include <asp/Tools/bundlevis.h>

#include <boost/foreach.hpp>

using namespace vw;

// This builds the 3 Axis that represents the camera
osg::Geode* build3Axis( float const& line_length ){
  osg::Geode* axisGeode = new osg::Geode();
  osg::Geometry* axis = new osg::Geometry();

  osg::Vec3Array* vertices = new osg::Vec3Array(6);
  (*vertices)[0].set( 0.0f, 0.0f, 0.0f );
  (*vertices)[1].set( line_length, 0.0f, 0.0f );
  (*vertices)[2].set( 0.0f, 0.0f, 0.0f );
  (*vertices)[3].set( 0.0f, line_length, 0.0f );
  (*vertices)[4].set( 0.0f, 0.0f, 0.0f );
  (*vertices)[5].set( 0.0f, 0.0f, line_length );
  axis->setVertexArray( vertices );

  osg::Vec4Array* colours = new osg::Vec4Array(6);
  (*colours)[0].set( 1.0f, 0.0f, 0.0f, 1.0f );
  (*colours)[1].set( 1.0f, 0.0f, 0.0f, 1.0f );
  (*colours)[2].set( 0.0f, 1.0f, 0.0f, 1.0f );
  (*colours)[3].set( 0.0f, 1.0f, 0.0f, 1.0f );
  (*colours)[4].set( 0.0f, 0.0f, 1.0f, 1.0f );
  (*colours)[5].set( 0.0f, 0.0f, 1.0f, 1.0f );
  axis->setColorArray( colours );
  axis->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

  axis->addPrimitiveSet( new osg::DrawArrays( GL_LINES, 0, vertices->getNumElements()));

  osg::StateSet* stateSet = new osg::StateSet();
  stateSet->setAttribute( new osg::LineWidth(2) );

  axisGeode->setStateSet( stateSet );
  axisGeode->addDrawable( axis );

  return axisGeode;
}

// This builds the matrix transform for a camera
osg::MatrixTransform*
CameraIter::matrix_transform( int const& step, int const& vertice = 0 ) {
  osg::MatrixTransform* mt = new osg::MatrixTransform;

  osg::Matrixf rot, trans;
  rot.makeRotate( this->pose( step, vertice ) );
  trans.makeTranslate( this->position( step, vertice ) );
  mt->setMatrix( rot*trans );
  mt->setUpdateCallback( new cameraMatrixCallback( this, vertice ));

  return mt;
}

// This will load a points iteration file
std::vector<PointIter*> loadPointData( std::string pntFile,
                                       int* step ){
  std::cout << "Loading Points Iteration Data : " << pntFile << std::endl;

  // Determing the number of lines in the file
  std::ifstream file(pntFile.c_str(), std::ios::in);
  if ( !file.is_open() )
    vw_throw( ArgumentErr() << "Unable to open: " << pntFile << "!\n" );
  int numLines = 0, numPoints = 0, numTimeIter = 0;
  char c;
  while (!file.eof()){
    c=file.get();
    if (c == '\n'){
      int buffer;
      numLines++;
      file >> buffer;
      if ( buffer > numPoints )
        numPoints = buffer;
    }
  }
  numPoints++;
  numTimeIter=numLines/numPoints;

  std::cout << "Number of Points found: " << numPoints << std::endl;

  // Starting from the top again
  file.clear();
  file.seekg(0);

  // Building the point organizational structure
  std::vector<PointIter*> pointData;
  for ( int i = 0; i < numPoints; ++i )
    pointData.push_back( new PointIter(i, step) );

  // Loading up the data finally

  // For every time iteration
  for ( int t = 0; t < numTimeIter; ++t ){
    for ( int i = 0; i < numPoints; ++i ){
      float float_fill_buffer;
      osg::Vec3f vec_fill_buffer;

      // The first one is just the point number
      file >> float_fill_buffer;
      if (float_fill_buffer != i)
        std::cout << "Reading number mismatch. Reading pnt " << i
                 << " , found it to be " << float_fill_buffer << std::endl;

      // Filling the vector
      file >> vec_fill_buffer[0];
      file >> vec_fill_buffer[1];
      file >> vec_fill_buffer[2];

      // Now attaching the data
      pointData[i]->add_iteration( vec_fill_buffer,
                                  0.0f);

    }
  }

  if ( pointData[0]->size() != (unsigned) numTimeIter )
    std::cout << "Number of Time Iterations found, " << numTimeIter
             << ", not equal loaded, " << pointData[0]->size()
             << std::endl;

  return pointData;
}

// This will load a camera data file
std::vector<CameraIter*> loadCameraData( std::string camFile,
                                         int* step ){

  std::cout << "Loading Cameras Iteration Data : " << camFile << std::endl;

  // Determing the number of lines in the file
  std::ifstream file(camFile.c_str(), std::ifstream::in);
  if ( !file.is_open() )
    vw_throw( ArgumentErr() << "Unable to open: " << camFile << "!\n" );
  int32 numLines = 0, numCameras = 0, numTimeIter = 0, numCameraParam = 1;
  int32 last_camera = -1, buffer;
  while (file.good()){
    file >> buffer;
    if (file.eof())
      break;

    if ( numCameras == 0 ) {
      if ( buffer == last_camera )
        numCameraParam++;
      last_camera = buffer;
    }
    if ( buffer > numCameras )
      numCameras = buffer;

    numLines++;
    char c = file.get();
    while ( c != '\n' && !file.eof() )
      c = file.get();
  }
  numCameras++;
  numTimeIter=numLines/(numCameras*numCameraParam);

  std::cout << "Number of Cameras found: " << numCameras << "\n"
            << "Number of Camera Parameters found: " << numCameraParam << "\n"
            << "Number of Time Iterations found: " << numTimeIter << "\n";

  // Starting from the top again
  file.clear();
  file.seekg(0);

  // Building the camera organizational structure
  std::vector<CameraIter*> cameraData;
  for ( int j = 0; j < numCameras; ++j ) {
    if ( numCameraParam == 1 || numCameraParam == 6 )
      cameraData.push_back( new CameraIter(j, step) );
    else
      cameraData.push_back( new CameraIter(j, step, numCameraParam) );
  }

  // Loading up the data finally

  // For every time iteration
  std::string throwAway;
  for ( int t = 0; t < numTimeIter; ++t ){
    for ( int j = 0; j < numCameras; ++j ){
      float float_fill_buffer;

      if ( numCameraParam == 1 ) {
        // Simple Camera

        // First one is just the Camera ID
        file >> float_fill_buffer;
        if (float_fill_buffer != j)
          std::cout << "Reading number mismatch. Reading camera " << j
                   << ", found it to be " << float_fill_buffer << std::endl;

        osg::Vec3f position_buffer;
        osg::Quat  pose_buffer;

        // This whole conditional mess is to handle NAN, at least I
        // hope it works!
        for ( unsigned i = 0; i < 3; i++ ) {
          if (!(file >> position_buffer[i])) {
            file >> throwAway;
            position_buffer[i] = 0;
          }
        }
        // Reason for this .. is that the Quat is written in VW format
        if (!(file >> pose_buffer[3])) {
          file >> throwAway;
          pose_buffer[3] = 0;
        }
        for ( unsigned i = 0; i < 3; i++ ) {
          if (!(file >> pose_buffer[i])) {
            file >> throwAway;
            pose_buffer[i] = 0;
          }
        }

        // Now attaching the data
        cameraData[j]->add_iteration( position_buffer,
                                      pose_buffer );
      } else {
        // Linescan Camera
        std::vector< osg::Vec3f> position_buffer;
        std::vector< osg::Quat> pose_buffer;

        // Going through all the vertice data which was given
        for ( int p = 0; p < numCameraParam; ++p ) {
          // The first on is just the Camera ID
          file >> float_fill_buffer;
          if ( float_fill_buffer != j)
            std::cout << "Reading number mismatch. Reading camera " << j
                      << ", found it to be " << float_fill_buffer << std::endl;

          position_buffer.push_back( osg::Vec3f() );
          for ( unsigned i = 0; i < 3; i++ ) {
            if (!(file >> position_buffer.back()[i])) {
              file >> throwAway;
              position_buffer.back()[i] = 0;
            }
          }
          pose_buffer.push_back( osg::Quat() );
          if (!(file >> pose_buffer.back()[3])) { // Files are written
                                                  // in VW notation
            file >> throwAway;
            pose_buffer.back()[3] = 0;
          }
          for ( unsigned i = 0; i < 3; i++ ) {
            if (!(file >> pose_buffer.back()[i])) {
              file >> throwAway;
              pose_buffer.back()[i] = 0;
            }
          }
        }

        // Cool.. now I'm going to attach this data
        cameraData[j]->add_iteration( position_buffer, pose_buffer );
      }
    }
  }

  if ( (cameraData[0]->size())/(cameraData[0]->num_vertices()) != (unsigned) numTimeIter )
    std::cout << "Number of Time Iterations found, " << numTimeIter
              << ", not equal loaded, " << cameraData[0]->size()/cameraData[0]->num_vertices()
             << std::endl;
  return cameraData;
}

// This will load a control network file
std::vector<ConnLineIter*> loadControlNet( std::string cnetFile,
                                           std::vector<PointIter*>& points,
                                           std::vector<CameraIter*>& cameras,
                                           ba::ControlNetwork* cnet,
                                           int* step ) {
  std::cout << "Loading Control Network : " << cnetFile << std::endl;

  cnet = new ba::ControlNetwork("Bundlevis");
  cnet->read_binary( cnetFile );

  if ( cnet->size() != points.size() ) {
    std::cout << "Control network doesn't seem to match loaded points data. Number of points: "
             << points.size() << " Number of points in Control Network: "
             << cnet->size() << std::endl;
    delete cnet;
    std::vector<ConnLineIter*> blank;
    return blank;
  }

  // Building the Connecting Line organizational structure
  std::vector<ConnLineIter*> connLineData;
  // For every point
  for ( unsigned p = 0; p < cnet->size(); ++p ) {
    points[p]->set_gcp( (*cnet)[p].type() == ba::ControlPoint::GroundControlPoint  );

    if (cameras.size()) {
      // For every measure
      for ( unsigned m = 0; m < (*cnet)[p].size(); ++m ){
        // Is this a valid camera ?
        if ( (*cnet)[p][m].image_id() < cameras.size() ) {
          // Now popping on a new connection
          connLineData.push_back( new ConnLineIter( points[p],
                                                    cameras[(*cnet)[p][m].image_id()],
                                                    step ) );
        }
      } //end for
    } // end if
  } // end for

  std::cout << "Number of Connections found: " << connLineData.size()
            << std::endl;

  return connLineData;
}

// This will build the entire scene
osg::Node* createScene( std::vector<PointIter*>& points,
                        std::vector<CameraIter*>& cameras,
                        std::vector<ConnLineIter*>& connLines,
                        std::vector< std::vector<PointIter*> >& addPoints,
                        BBox3f const* point_cloud, double cam_distance ) {

  osg::Group* scene = new osg::Group();

  // First, setting the back drop to be dark
  {
    osg::ClearNode* backdrop = new osg::ClearNode;
    backdrop->setClearColor( osg::Vec4f( 0.1f, 0.1f, 0.1f, 1.0f ) );
    scene->addChild(backdrop);
  }

  // Second, drawing the points
  if ( points.size() ){
    osg::Geometry* geometry = new osg::Geometry;

    // Setting up vertices
    osg::Vec3Array* vertices = new osg::Vec3Array( points.size()*2 );
    geometry->setVertexArray( vertices );

    for ( unsigned i = 0; i < points.size()*2; i+=2 ) {
      (*vertices)[i] = points[i/2]->position(0);
      (*vertices)[i+1] = (*vertices)[i] + osg::Vec3f(0.0f, 0.0f, 0.1f);
    }

    // Setting up color
    osg::Vec4Array* colours = new osg::Vec4Array( points.size()*2 );
    (*colours)[0].set(1.0f, 0.5f, 1.0f, 1.0f);  // Generic Tie Points (PURPLE)
    (*colours)[1].set(0.5f, 1.0f, 0.5f, 1.0f);  // Ground Control Points (GREEN)
    for ( unsigned i = 0; i < points.size(); i++ ) {
      if ( points[i/2]->is_gcp() ) {
        (*colours)[2*i].set(0.5f,1.0f,0.5f,1.0f);
        (*colours)[2*i+1].set(0.5f,1.0f,0.5f,1.0f);
      } else {
        (*colours)[2*i].set(1.0f,0.5f,1.0f,1.0f);
        (*colours)[2*i+1].set(1.0f,0.5f,1.0f,1.0f);
      }
    }
    geometry->setColorArray( colours );

    // Setting up primitive to draw line to last location
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINES, 0, vertices->size() ));

    // Setting up to draw points at the front end of the line
    osg::DrawElementsUInt* pointsArray = new osg::DrawElementsUInt(GL_POINTS, points.size());
    geometry->addPrimitiveSet( pointsArray );
    for (unsigned i = 0; i < points.size(); ++i)
      (*pointsArray)[i] = i*2;

    geometry->setUseDisplayList(false);
    geometry->setDrawCallback( new pointsDrawCallback( &points ) );

    // Making geode
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );


    // Making the points large
    osg::StateSet* stateSet = new osg::StateSet();
    osg::Point* point = new osg::Point();
    point->setSize( 2.0f );
    stateSet->setAttribute( point );
    geode->setStateSet( stateSet );

    scene->addChild( geode );
  }

  // 2.b Adding additional points
  for ( size_t n = 0; n < addPoints.size(); ++n ) {
    osg::Geometry* geometry = new osg::Geometry;

    // Setting up vertices
    osg::Vec3Array* vertices = new osg::Vec3Array( addPoints[n].size()*2 );
    geometry->setVertexArray( vertices );

    for ( unsigned i = 0; i < addPoints[n].size()*2; i+=2 ) {
      (*vertices)[i] = addPoints[n][i/2]->position(0);
      (*vertices)[i+1] = (*vertices)[i] + osg::Vec3f(0.0f, 0.0f, 0.1f);
    }

    // Setting up color
    osg::Vec4Array* colours = new osg::Vec4Array( 1 );
    geometry->setColorArray( colours );
    geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
    (*colours)[0].set(1.0f, 0.25f, 0.5f, 0.5f);

    // Setting up primitive to draw line to last location
    geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINES, 0, vertices->size() ));

    // Setting up to draw points at the front end of the line
    osg::DrawElementsUInt* pointsArray = new osg::DrawElementsUInt(GL_POINTS, addPoints[n].size());
    geometry->addPrimitiveSet( pointsArray );
    for (unsigned i = 0; i < addPoints[n].size(); ++i)
      (*pointsArray)[i] = i*2;

    geometry->setUseDisplayList(false);
    geometry->setDrawCallback( new pointsDrawCallback( &(addPoints[n]) ) );

    // Making geode
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable( geometry );


    // Making the points large
    osg::StateSet* stateSet = new osg::StateSet();
    osg::Point* point = new osg::Point();
    point->setSize( 2.0f );
    stateSet->setAttribute( point );
    geode->setStateSet( stateSet );

    scene->addChild( geode );
  }

  // Thirdly, draw cameras
  if ( cameras.size() ){

    //osg::Geode* the3Axis = build3Axis();
    osg::Geode* the3Axis;

    if ( point_cloud ) {
      the3Axis = build3Axis( cam_distance*0.8 );
    } else {
      the3Axis = build3Axis( 1.0 );
    }

    osg::Group* camerasGroup = new osg::Group;

    for (unsigned j = 0; j < cameras.size(); ++j){
      osg::MatrixTransform* mt = cameras[j]->matrix_transform( 0 );
      mt->addChild( the3Axis );
      camerasGroup->addChild( mt );
    }

    // 3b.) Draw potential pushbroom camera lines
    osg::Group* pushbroomGroup = new osg::Group;
    {
      vw::TerminalProgressCallback tpc("asp","Loading Camera: ");
      tpc.report_progress(0);
      for (unsigned j = 0; j < cameras.size(); ++j){
        tpc.report_progress(float(j)/float(cameras.size()));
        if ( cameras[j]->is_pushbroom() ) {
          osg::Geometry* geometry = new osg::Geometry;

          // Setting up vertices
          osg::Vec3Array* vertices = new osg::Vec3Array( cameras[j]->num_vertices() );
          geometry->setVertexArray( vertices );

          geometry->setUseDisplayList( false );
          geometry->setDrawCallback( new pushbroomDrawCallback( cameras[j] ) );

          for ( unsigned i = 0; i < vertices->size(); ++i ){
            (*vertices)[i] = cameras[j]->position( 0, i );
          }

          // Setting up color
          osg::Vec4Array* colours = new osg::Vec4Array( 1 );
          geometry->setColorArray( colours );
          geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
          (*colours)[0].set(1.0f, 1.0f, 0.0f, 1.0f);

          // Setting up primitive to draw lines
          geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINE_STRIP, 0, vertices->size() ));

          // Making geode
          osg::Geode* geode = new osg::Geode;
          geode->addDrawable( geometry );

          // Setting the line width
          osg::StateSet* stateSet = new osg::StateSet();
          stateSet->setAttribute( new osg::LineWidth(2) );
          geode->setStateSet( stateSet );
          pushbroomGroup->addChild( geode );

          // Also throwing in another 3Axis at the end of the line
          osg::MatrixTransform* mt = cameras[j]->matrix_transform(0, cameras[j]->num_vertices()-1);
          mt->addChild( the3Axis );
          pushbroomGroup->addChild( mt );
        }
      }
      camerasGroup->addChild( pushbroomGroup );
      tpc.report_finished();
    }

    scene->addChild( camerasGroup );
  }

  // 4.) Draw Connecting Lines
  if ( connLines.size() ){
    osg::Group* linesGroup = new osg::Group;

    for ( unsigned i = 0; i < connLines.size(); i+= 1 ){
      osg::Geometry* geometry = new osg::Geometry;

      osg::Vec3Array* vertices = new osg::Vec3Array( 2 );
      (*vertices)[0] = connLines[i]->point()->position( 0 );
      (*vertices)[1] = connLines[i]->camera()->position( 0 );
      geometry->setVertexArray( vertices );

      osg::Vec4Array* colours = new osg::Vec4Array( 1 );
      geometry->setColorArray( colours );
      geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
      (*colours)[0].set(0.8f, 0.8f, 0.8f, 1.0f);

      geometry->addPrimitiveSet( new osg::DrawArrays(GL_LINES, 0, 2) );

      geometry->setUseDisplayList( false );
      geometry->setDrawCallback( new linesDrawCallback( connLines[i] ));

      osg::Geode* geode = new osg::Geode;
      geode->addDrawable( geometry );
      linesGroup->addChild( geode );
    }

    scene->addChild( linesGroup );

  }

  // 5.) Build hit points, surfaces that the mouse intersector can use
  {
    double scale = cam_distance * 0.5;
    osg::Group* hitTargetGroup = new osg::Group;

    // Targets for cameras
    if ( cameras.size() ) {
      // Building the target that all will use
      osg::Vec3Array* vertices = new osg::Vec3Array(4);
      (*vertices)[0].set( 1.0f, -1.0f, -0.1f );
      (*vertices)[1].set( 1.0f, 1.0f, -0.1f );
      (*vertices)[2].set( -1.0f, 1.0f, -0.1f );
      (*vertices)[3].set( -1.0f, -1.0f, -0.1f );
      osg::Vec4Array* colours = new osg::Vec4Array(1);
      (*colours)[0].set( 1.0f, 1.0f, 1.0f, 0.0f );

      for ( size_t j = 0; j < cameras.size(); ++j ) {

        // Geode that is the target
        osg::Geometry* geometry = new osg::Geometry;
        geometry->setVertexArray( vertices );
        geometry->setColorArray( colours );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
        geometry->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, 4 ) );
        osg::Geode* geode = new osg::Geode;
        geode->setName( cameras[j]->description() );
        geode->setUserData( cameras[j] );
        geode->addDrawable( geometry );

        // Add some text, cause it's cool
        osgText::Text* text = new osgText::Text;
        std::ostringstream os;
        os << (j + 1);
        text->setCharacterSize( 70.0 );
        //text->setFontResolution(40,40);
        text->setText( os.str() );
        text->setAlignment( osgText::Text::CENTER_CENTER );
        //text->setCharacterSizeMode( osgText::Text::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );
        geode->addDrawable( text );

        // Transform that does my bidding
        osg::AutoTransform* autoT = new osg::AutoTransform;
        autoT->addChild( geode );
        autoT->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        autoT->setAutoScaleToScreen( true );
        autoT->setMinimumScale( 0.0f );
        autoT->setMaximumScale( 0.1f * scale );
        autoT->setPosition( cameras[j]->position( 0 ) );
        autoT->setUpdateCallback( new cameraAutoMatrixCallback( cameras[j] ));

        hitTargetGroup->addChild( autoT );
      }
    }

    // Targets for points
    if ( points.size() ) {
      // Building the target that all will use
      osg::Vec3Array* vertices = new osg::Vec3Array(4);
      (*vertices)[0].set( 10.0f, -10.0f, -0.5f );
      (*vertices)[1].set( 10.0f, 10.0f, -0.5f );
      (*vertices)[2].set( -10.0f, 10.0f, -0.5f );
      (*vertices)[3].set( -10.0f, -10.0f, -0.5f );
      osg::Vec4Array* colours = new osg::Vec4Array(1);
      (*colours)[0].set( 0.0f, 0.0f, 0.0f, 0.0f );

      for ( size_t i = 0; i < points.size(); ++i ) {

        // Geode that is the target
        osg::Geometry* geometry = new osg::Geometry;
        geometry->setVertexArray( vertices );
        geometry->setColorArray( colours );
        geometry->setColorBinding( osg::Geometry::BIND_OVERALL );
        geometry->addPrimitiveSet( new osg::DrawArrays( osg::PrimitiveSet::QUADS,
                                                       0,
                                                       4 ) );

        // Add some text
        osgText::Text* text = new osgText::Text;
        std::ostringstream os;
        os << (i + 1);
        text->setCharacterSize( 25.0 );
        //text->setFontResolution(40,40);
        text->setText( os.str() );
        text->setAlignment( osgText::Text::CENTER_CENTER );
        //text->setCharacterSizeMode( osgText::Text::OBJECT_COORDS_WITH_MAXIMUM_SCREEN_SIZE_CAPPED_BY_FONT_HEIGHT );

        osg::Geode* geode =new osg::Geode;
        geode->addDrawable( text );
        geode->addDrawable( text );
        geode->setName( points[i]->description() );
        geode->addDrawable( geometry );
        geode->setUserData( points[i] );

        // Building an LOD so it shuts off when too far away
        osg::LOD* lod = new osg::LOD;
        lod->addChild( geode, 10.0f, 1000.0f );
        lod->setRangeMode( osg::LOD::PIXEL_SIZE_ON_SCREEN );
        lod->setRadius( scale/10.0 );

        // Transform that does my bidding
        osg::AutoTransform* autoT = new osg::AutoTransform;
        autoT->addChild(lod);
        autoT->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
        autoT->setAutoScaleToScreen( true );
        autoT->setMinimumScale( 0.0f );
        autoT->setMaximumScale( 0.0005f * scale );
        autoT->setPosition( points[i]->position( 0 ) );
        autoT->setUpdateCallback( new pointAutoMatrixCallback( points[i] ));

        hitTargetGroup->addChild( autoT );
      }
    }

    scene->addChild ( hitTargetGroup );
  }

  return scene;
}

// This is the event handler, mouse, and keyboard.
bool AllEventHandler::handle( const osgGA::GUIEventAdapter& ea,
                              osgGA::GUIActionAdapter& aa ) {
  // Note: for some reason, you should only perform getEventType()
  // once.
  osgGA::GUIEventAdapter::EventType event = ea.getEventType();

  if ( event == osgGA::GUIEventAdapter::KEYDOWN ) {
    switch (ea.getKey()){
    case 'z':     //Step Backwards
      if ( *m_step - 1 < 1 )
        *m_step = m_numIter;
      else
        (*m_step)--;
      break;
    case 'x':     //Play
      m_playControl->set_play();
      break;
    case 'c':     //Pause
      m_playControl->set_pause();
      break;
    case 'v':     //Stop
      m_playControl->set_stop();
      break;
    case 'b':     //Step Forward
      if ( *m_step + 1 > m_numIter )
        *m_step = 1;
      else
        (*m_step)++;
      break;
    case '0':     //Show all steps
      *m_step = 0;
      break;
    case '1':     //Move to the first frame
      *m_step = 1;
      break;
    case '2':     //Move ro a place somewhere in between
      *m_step = (m_numIter * 2) / 9;
      break;
    case '3':
      *m_step = (m_numIter * 3) / 9;
      break;
    case '4':
      *m_step = (m_numIter * 4) / 9;
      break;
    case '5':
      *m_step = (m_numIter * 5) / 9;
      break;
    case '6':
      *m_step = (m_numIter * 6) / 9;
      break;
    case '7':
      *m_step = (m_numIter * 7) / 9;
      break;
    case '8':
      *m_step = (m_numIter * 8) / 9;
      break;
    case '9':     //Move to the last frame
      *m_step = m_numIter;
      break;
    default:
      break;
    }
  } else if ( event == osgGA::GUIEventAdapter::PUSH ) {
    osgViewer::View* view = dynamic_cast< osgViewer::View* >(&aa);
    if (view)
      pick( view, ea );
  }
  return false;
}

void AllEventHandler::pick( osgViewer::View* view, const osgGA::GUIEventAdapter& ea ) {
  osgUtil::LineSegmentIntersector::Intersections intersections;

  float x = ea.getX();
  float y = ea.getY();

  if (view->computeIntersections( x, y, intersections )){
    //I only care about the first intersection event
    osgUtil::LineSegmentIntersector::Intersections::iterator hit =
      intersections.begin();
    if (!hit->nodePath.empty() && !(hit->nodePath.back()->getName().empty())){
      std::cout << hit->nodePath.back()->getName() << std::endl;

      // Now I'll determine if it's a point
      PointIter* point =
        dynamic_cast< PointIter* >( hit->nodePath.back()->getUserData() );
      if ( point ) {
        point->toggle_drawconnlines();
        return;
      }

      // Well... snap, is it a camera?
      CameraIter* camera =
        dynamic_cast< CameraIter* >( hit->nodePath.back()->getUserData() );
      if ( camera ) {
        camera->toggle_drawconnlines();
        return;
      }
    }
  }
}

// Main Execution
int main(int argc, char* argv[]){

  //Variables to be used later in this section
  int* controlStep = new int(0);
  std::string camera_iter_file, points_iter_file;
  std::string pixel_iter_file, control_net_file;
  std::vector<std::string> additional_pnt_files;
  ba::ControlNetwork* cnet = NULL;
  std::vector<PointIter*> pointData;
  std::vector<CameraIter*> cameraData;
  std::vector<ConnLineIter*> connLineData;
  std::vector<std::vector<PointIter*> > addPointData;
  PlaybackControl* playControl = new PlaybackControl(controlStep);
  boost::scoped_ptr<BBox3f> point_cloud;        // World size of the objects
  double q25_camera_distance; // Average distance between cameras

  //OpenSceneGraph Variable which are important overall
  osgViewer::Viewer viewer;
  osg::ref_ptr<osg::Group> root = new osg::Group();

  //This first half of the program is wholly devoted to setting up boost program options
  //Boost program options code
  po::options_description general_options("Your most awesome Options");
  general_options.add_options()
    ("camera-iteration-file,c",po::value<std::string>(&camera_iter_file),"Load the camera parameters for each iteration from a file.")
    ("points-iteration-file,p",po::value<std::string>(&points_iter_file),"Load the 3d points parameters for each iteration from a file.")
    ("pixel-iteration-file,x",po::value<std::string>(&pixel_iter_file),"Load the pixel information data. Allowing for an illustration of the pixel data over time.")
    ("control-network-file,n",po::value<std::string>(&control_net_file),"Load the control network for point and camera relationship status. Camera and Point Iteration data is needed before a control network file can be used.")
    ("additional-pnt-files",po::value< std::vector<std::string> >(&additional_pnt_files),"For additional iterPoint files that can be plotted simultaneously.")
    ("fullscreen","Set bundlevis to render with the entire screen, does not work so well with dual screens.")
    ("stereo","Set bundlevis to display in anagylph mode.")
    ("show-moon","Add a transparent Moon to the display.")
    ("show-mars","Add a transparent Mars to the display.")
    ("show-earth","Add a transparent Earth to the display.")
    ("help,h","Display this help message.");

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << "[options] <filename> ... " << std::endl << std::endl;
  usage << general_options << std::endl;

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc,argv,general_options),vm);
    po::notify(vm);
  } catch (po::error const& e) {
    std::cout << "An error occured while parsing command line arguments.\n";
    std::cout << "\t" << e.what() << "\n\n";
    std::cout << usage.str();
    return 1;
  }

  if (vm.count("help")){
    std::cout<<usage.str();
    return 1;
  }

  //////////////////////////////////////////////////////////
  //Stereo?
  if (vm.count("stereo")){
    osg::DisplaySettings::instance()->setStereo(true);
  }

  //////////////////////////////////////////////////////////
  //Did the User mess up?
  if (!vm.count("points-iteration-file") && !vm.count("camera-iteration-file")){
    std::cout << usage.str();
    return 1;
  }

  //////////////////////////////////////////////////////////
  //Loading up point iteration data
  if (vm.count("points-iteration-file")){
    pointData = loadPointData( points_iter_file,
                               controlStep );
  }

  //////////////////////////////////////////////////////////
  //Loading up camera iteration data
  if (vm.count("camera-iteration-file")){
    cameraData = loadCameraData( camera_iter_file,
                                 controlStep );
  }

  //////////////////////////////////////////////////////////
  //Gathering Statistics about point cloud
  {
    math::CDFAccumulator<double> xcdf, ycdf, zcdf, dcdf;

    if (vm.count("points-iteration-file")) {
      BOOST_FOREACH( PointIter* pitr, pointData ) {
        xcdf( pitr->position(0)[0] );
        ycdf( pitr->position(0)[1] );
        zcdf( pitr->position(0)[2] );
      }
    }

    Vector3 before;
    if (vm.count("camera-iteration-file")) {
      BOOST_FOREACH( CameraIter* citr, cameraData ) {
        xcdf( citr->position(0)[0] );
        ycdf( citr->position(0)[1] );
        zcdf( citr->position(0)[2] );
        if ( before == Vector3() ) {
          before = Vector3( citr->position(0)[0],
                            citr->position(0)[1],
                            citr->position(0)[2] );
        } else {
          Vector3 current( citr->position(0)[0],
                           citr->position(0)[1],
                           citr->position(0)[2] );
          dcdf( norm_2(current-before) );
          before = current;
        }
      }
    }

    xcdf.update(); ycdf.update(); zcdf.update(); dcdf.update();
    point_cloud.reset( new BBox3f(Vector3(xcdf.quantile(.1),
                                          ycdf.quantile(.1),
                                          zcdf.quantile(.1)),
                                  Vector3(xcdf.quantile(.9),
                                          ycdf.quantile(.9),
                                          zcdf.quantile(.9))) );
    q25_camera_distance = dcdf.quantile(.25);
    if (q25_camera_distance < 1 ) q25_camera_distance = 1;

    Vector3 center = point_cloud->center();
    Vector3 size = point_cloud->size();

    std::cout << "Point Cloud:\n\t> Center " << center << "\n\t> Size " << size;
    std::cout << "\n\t> CSize " << q25_camera_distance << std::endl;
  }

  //////////////////////////////////////////////////////////
  //Loading up additional point iteration data
  if (vm.count("additional-pnt-files")) {
    std::cout << "Checking to see if there is additional:" << std::endl;
    for (unsigned n = 0; n < additional_pnt_files.size(); ++n) {
      std::cout << " > " << additional_pnt_files[n] << std::endl;
      addPointData.push_back( loadPointData( additional_pnt_files[n],
                                             controlStep ) );
    }
  }

  //////////////////////////////////////////////////////////
  //Loading the control network
  if (vm.count("control-network-file") && (pointData.size()) ){
    connLineData = loadControlNet( control_net_file,
                                   pointData,
                                   cameraData,
                                   cnet,
                                   controlStep );
  }

  //////////////////////////////////////////////////////////
  //Setting up window if requested
  if ( !vm.count( "fullscreen" ) ){
    std::cout << "Building Window\n";

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 100;
    traits->y = 100;
    traits->width = 1280;
    traits->height = 1024;
    traits->windowDecoration = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    traits->windowName = "Bundlevis 2.1";
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
  // Adding user Requested data
  {
    // I can haz moon?
    if (vm.count("show-moon")){

      osg::ref_ptr<osg::Geode> moon_geode = new osg::Geode();

      osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3(0,0,0), 1737100)); // meters
      sphere->setColor( osg::Vec4f(0.5,0.5,0.5,0.25)); // Gray 25% Opacity
      osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
      sphere->getOrCreateStateSet()->setAttributeAndModes( pm.get(), osg::StateAttribute::ON); // Wireframe
      moon_geode->addDrawable( sphere.get() );

      root->addChild( moon_geode.get() );
    }

    if (vm.count("show-mars")){

      osg::ref_ptr<osg::Geode> mars_geode = new osg::Geode();

      osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3(0,0,0), 3396200)); // meters
      sphere->setColor( osg::Vec4f(1.0,0.0,0.0,0.25)); // Red 25% Opacity
      osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
      sphere->getOrCreateStateSet()->setAttributeAndModes( pm.get(), osg::StateAttribute::ON); // Wireframe
      mars_geode->addDrawable( sphere.get() );

      root->addChild( mars_geode.get() );
    }

    if (vm.count("show-earth")){

      osg::ref_ptr<osg::Geode> earth_geode = new osg::Geode();

      osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable( new osg::Sphere(osg::Vec3(0,0,0), 6371000)); // meters
      sphere->setColor( osg::Vec4f(0.2,0.2,1.0,0.25)); // Blue 25% Opacity
      osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode( osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE );
      sphere->getOrCreateStateSet()->setAttributeAndModes( pm.get(), osg::StateAttribute::ON); // Wireframe
      earth_geode->addDrawable( sphere.get() );

      root->addChild( earth_geode.get() );
    }
  }

  //////////////////////////////////////////////////////////
  //Building Scene
  {
    root->addChild( createScene( pointData,    cameraData,
                                 connLineData, addPointData,
                                 point_cloud.get(),  q25_camera_distance ) );

    // Optimizing the Scene (seems like good practice in OSG)
    osgUtil::Optimizer optimizer;
    optimizer.optimize( root.get() );

    osgGA::TrackballManipulator* trackball = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator( trackball );
    viewer.setSceneData( root.get() );

    int numIter;
    if ( pointData.size() > 0 )
      numIter = pointData[0]->size();
    else
      numIter = cameraData[0]->size();
    viewer.addEventHandler( new AllEventHandler( controlStep,
                                                 numIter,
                                                 playControl) );

    root->setUpdateCallback( new playbackNodeCallback ( controlStep,
                                                        numIter,
                                                        playControl) );

    trackball->setDistance(norm_2(point_cloud->size()));
    trackball->setCenter( osg::Vec3(point_cloud->center()[0],
                                    point_cloud->center()[1],
                                    point_cloud->center()[2]) );
  }

  //////////////////////////////////////////////////////////
  //Setting some global scene parameters
  {
    osg::StateSet* stateSet = new osg::StateSet();
    stateSet->setMode( GL_LIGHTING, osg::StateAttribute::OFF );
    stateSet->setMode( GL_BLEND, osg::StateAttribute::ON );
    root->setStateSet( stateSet );
  }

  viewer.realize();
  viewer.run();
  std::cout << "Ending\n";

  return 0;
}

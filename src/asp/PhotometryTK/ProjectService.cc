// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#include <vw/Core/Log.h>
#include <vw/Core/Exception.h>
#include <vw/Plate/Exception.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <asp/PhotometryTK/ProjectFileIO.h>

using namespace vw;
using namespace vw::platefile;

using namespace asp::pho;

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Search Function
/*
std::vector<std::string> inline
glob_ptk_filenames( std::string const& root_directory ) {
  std::vector<std::string> result;

  if ( !fs::exists( root_directory ) )
    vw_throw( IOErr() << "Could not access the root directory: \"" <<
              root_directory << "\"" );

  boost::regex re;
  re.assign(".*\\.ptk", boost::regex_constants::icase);

  fs::directory_iterator end_itr; // eof

  for ( fs::directory_iterator itr( root_directory ); itr != end_itr; ++itr ) {
    if (boost::regex_match(itr->leaf(), re))
      result.push_back(itr->leaf());
  }

  return result;
}
*/

#define METHOD_IMPL(Name, Input, Output) \
  void ProjectServiceImpl::Name(::google::protobuf::RpcController*, const Input* request, Output* response, ::google::protobuf::Closure* done)


#define CHECK_CAMERA_RANGE()                                \
  if ( request->camera_id() < 0 ||                          \
       request->camera_id() >=                              \
       int32(m_camera_metas.size()) ) { \
    response->set_project_id( request->project_id() );      \
    response->set_camera_id( -1 );                          \
    return;                                                 \
  }

// Public Methods
// ---------------------------------------------

ProjectServiceImpl::ProjectServiceImpl( std::string root_directory ) :
  m_root_directory( fs::system_complete(root_directory).string() ) {

  // Search for all the project files within a given root directory.
  // Project files end in a ptk
  //std::vector<std::string> ptkfiles = glob_ptk_filenames( root_directory );
  //if ( ptkfiles.size() < 1 )
  //vw_throw( IOErr() << "There are no project files where the server started.\n" );

  read_pho_project( root_directory,
		    m_project_meta,
		    m_camera_metas );
  m_project_meta.set_name( fs::path( root_directory ).relative_path().string() );

  /*
  for (size_t i = 0; i < ptkfiles.size(); ++i ) {
    int32 index = m_ptk_lookup.size();
    m_ptk_lookup[fs::path(ptkfiles[i]).relative_path().string()] =
      index;
    m_project_metas.push_back( ProjectMeta() );
    m_camera_metas.push_back( std::vector<CameraMeta>() );
    read_pho_project( root_directory+"/"+ptkfiles[i],
                      m_project_metas.back(),
                      m_camera_metas.back() );
    m_project_metas.back().set_name( fs::path(ptkfiles[i]).relative_path().string() );
  }
  */
}

void ProjectServiceImpl::sync() {

  Mutex::Lock iulock(m_iterUpMutex);
  Mutex::Lock cclock(m_camCreateMutex);
  Mutex::Lock cwlock(m_camWriteMutex);
  Mutex::Lock alock(m_pixAddMutex);
  Mutex::Lock rlock(m_pixResetMutex);

  vw_out() << "\t--> Syncing project files for " << m_project_meta.name() << " to disk.\n";

  //  for ( size_t i = 0; i < m_project_metas.size(); i++ ) {
  /*
    vw_out() << "\t--> Syncing project files for "
             << m_project_metas[i].name() <<  " to disk.\n";
  */

  std::string ptk_file = m_root_directory; //+"/"+m_project_metas[i].name();

  // Delete bak
  if ( fs::exists(ptk_file+"/photometrytk.bak") )
    fs::remove(ptk_file+"/photometrytk.bak");

  // Move current ptk to bak
  fs::rename(ptk_file+"/photometrytk.dat",
	     ptk_file+"/photometrytk.bak");

  // Save ptk
  write_pho_project( ptk_file,
		     m_project_meta,
		     m_camera_metas.begin(),
		     m_camera_metas.end() );
}

METHOD_IMPL(OpenRequest, ::asp::pho::ProjectOpenRequest, ::asp::pho::ProjectOpenReply) {
  detail::RequireCall call(done);

  std::string request_ptk = request->name();

  response->set_project_id( 0 );
  *(response->mutable_meta()) = m_project_meta;

  /*
  std::map<std::string,int>::iterator it =
    m_ptk_lookup.find( request_ptk );
  if ( it != m_ptk_lookup.end() ) {
    // Success
    response->set_project_id( it->second );
    *(response->mutable_meta()) = m_project_metas[ it->second ];
  } else {
    // Fail
    response->set_project_id( -1 );
    *(response->mutable_meta()) = ProjectMeta();
    response->mutable_meta()->set_reflectance( ProjectMeta::NONE );
    response->mutable_meta()->set_num_cameras( 0 );
  }
  */
}

METHOD_IMPL(IterationUpdate, ::asp::pho::IterationUpdateRequest, ::asp::pho::IterationUpdateReply) {
  detail::RequireCall call(done);

  Mutex::Lock syncLock(m_iterUpMutex);

  m_project_meta.set_current_iteration(request->iteration());
  response->set_project_id( 0 );
}

METHOD_IMPL(CameraCreate, ::asp::pho::CameraCreateRequest, ::asp::pho::CameraCreateReply) {
  detail::RequireCall call(done);

  Mutex::Lock syncLock(m_camCreateMutex);

  // Adding camera meta
  response->set_project_id( 0 );
  response->set_camera_id( m_camera_metas.size() );
  m_camera_metas.push_back( request->meta() );
  m_project_meta.set_num_cameras( response->camera_id()+1 );
  // Set Base Transaction ID
  m_camera_metas.back().set_base_transaction_id( m_project_meta.max_iterations()*(m_camera_metas.size()-1) );
}

METHOD_IMPL(CameraRead, ::asp::pho::CameraReadRequest, ::asp::pho::CameraReadReply) {
  detail::RequireCall call(done);

  // Set empty meta .. for the error checking
  *(response->mutable_meta()) = CameraMeta();
  CHECK_CAMERA_RANGE();

  // echo correct
  response->set_project_id( 0 );
  response->set_camera_id( request->camera_id() );

  *(response->mutable_meta()) =
    m_camera_metas[request->camera_id()];
}

METHOD_IMPL(CameraWrite, ::asp::pho::CameraWriteRequest, ::asp::pho::CameraWriteReply) {
  detail::RequireCall call(done);

  Mutex::Lock syncLock(m_camWriteMutex);

  CHECK_CAMERA_RANGE();

  // echo correct
  response->set_project_id( 0 );
  response->set_camera_id( request->camera_id() );
  m_camera_metas[request->camera_id()] = request->meta();
}

METHOD_IMPL(PixvalAdd, ::asp::pho::PixvalAddRequest, ::asp::pho::PixvalAddReply) {
  detail::RequireCall call(done);

  Mutex::Lock syncLock(m_pixAddMutex);

  m_project_meta.add_min_pixval_vec( request->min_pixval() );
  m_project_meta.add_max_pixval_vec( request->max_pixval() );
  response->set_project_id( 0 );
}

METHOD_IMPL(PixvalGetAndReset, ::asp::pho::PixvalGetAndResetRequest, ::asp::pho::PixvalGetAndResetReply) {
  detail::RequireCall call(done);

  Mutex::Lock syncLock(m_pixResetMutex);

  if (m_project_meta.min_pixval_vec_size() == 0 ||
      m_project_meta.max_pixval_vec_size() == 0) {
    response->set_project_id( 0 );
    response->set_min_pixval( m_project_meta.min_pixval() );
    response->set_max_pixval( m_project_meta.max_pixval() );    
    return;
  }

  double min = 0;
  for(int i = 0; i < m_project_meta.min_pixval_vec_size(); i++) {
    double newmin = m_project_meta.min_pixval_vec(i);
    //std::cout << "newmin=" << newmin << "\n";
    if (0 == i) min = newmin;
    else {
      if (newmin < min) min = newmin;
    }
  }
  m_project_meta.set_min_pixval(min);

  double max = 0;
  for(int i = 0; i < m_project_meta.max_pixval_vec_size(); i++) {
    double newmax = m_project_meta.max_pixval_vec(i);
    //std::cout << "newmax=" << newmax << "\n";
    if (0 == i) max = newmax;
    else {
      if (newmax > max) max = newmax;
    }
  }
  m_project_meta.set_max_pixval(max);
  
  m_project_meta.clear_min_pixval_vec();
  m_project_meta.clear_max_pixval_vec();

  response->set_project_id( 0 );
  response->set_min_pixval(min);
  response->set_max_pixval(max);
 }


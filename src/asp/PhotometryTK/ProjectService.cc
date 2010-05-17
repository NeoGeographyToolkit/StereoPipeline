#include <vw/Core/Log.h>
#include <vw/Core/Exception.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <asp/PhotometryTK/ProjectFileIO.h>
using namespace vw;
using namespace asp::pho;

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

// Search Function
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

// Range checking macros
#define CHECK_PROJECT_RANGE()                               \
  if ( request->project_id() < 0 ||                         \
       request->project_id() >= int32(m_project_metas.size()) ) { \
    response->set_project_id( -1 );                         \
    response->set_camera_id( -1 );                          \
    done->Run();                                            \
    return;                                                 \
  }


#define CHECK_CAMERA_RANGE()                                \
  if ( request->camera_id() < 0 ||                          \
       request->camera_id() >=                              \
       int32(m_camera_metas[ request->project_id() ].size()) ) { \
    response->set_project_id( request->project_id() );      \
    response->set_camera_id( -1 );                          \
    done->Run();                                            \
    return;                                                 \
  }

// Public Methods
// ---------------------------------------------

ProjectServiceImpl::ProjectServiceImpl( std::string root_directory ) :
  m_root_directory( fs::system_complete(root_directory).string() ) {

  // Search for all the project files within a given root directory.
  // Project files end in a ptk
  std::vector<std::string> ptkfiles = glob_ptk_filenames( root_directory );
  if ( ptkfiles.size() < 1 )
    vw_throw( IOErr() << "There are no project files where the server started.\n" );

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
}

void ProjectServiceImpl::sync() {
  for ( size_t i = 0; i < m_project_metas.size(); i++ ) {
    vw_out() << "\t--> Syncing project files for "
             << m_project_metas[i].name() <<  " to disk.\n";

    std::string ptk_file = m_root_directory+"/"+m_project_metas[i].name();

    // Delete bak
    if ( fs::exists(ptk_file+".bak") )
      fs::remove(ptk_file+".bak");

    // Move current ptk to bak
    fs::rename(ptk_file,
               ptk_file+".bak");

    // Save ptk
    write_pho_project( ptk_file,
                       m_project_metas[i],
                       m_camera_metas[i].begin(),
                       m_camera_metas[i].end() );
  }
}

void
ProjectServiceImpl::OpenRequest(::google::protobuf::RpcController* /*controller*/,
                                const ::asp::pho::ProjectOpenRequest* request,
                                ::asp::pho::ProjectOpenReply* response,
                                ::google::protobuf::Closure* done) {
  std::string request_ptk = request->name();
  {
    size_t findx = request_ptk.find("ptk/");
    if ( findx != std::string::npos )
      request_ptk = request_ptk.substr(findx+4);
  }

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
  done->Run();
}

void
ProjectServiceImpl::ProjectUpdate(::google::protobuf::RpcController* /*controller*/,
                                  const ::asp::pho::ProjectUpdateRequest* request,
                                  ::asp::pho::ProjectUpdateReply* response,
                                  ::google::protobuf::Closure* done) {
  if ( request->project_id() < 0 ||
       request->project_id() >= int32(m_project_metas.size()) ) {
    response->set_project_id( -1 );
    done->Run();
    return;
  }

  // Setting iteration
  m_project_metas[ request->project_id() ].set_current_iteration(request->iteration());
  response->set_project_id( request->project_id() );
  done->Run();
}

void
ProjectServiceImpl::CameraCreate(::google::protobuf::RpcController* /*controller*/,
                                 const ::asp::pho::CameraCreateRequest* request,
                                 ::asp::pho::CameraCreateReply* response,
                                 ::google::protobuf::Closure* done) {
  CHECK_PROJECT_RANGE();

  // Adding camera meta
  response->set_project_id( request->project_id() );
  response->set_camera_id( m_camera_metas[request->project_id()].size() );
  m_camera_metas[request->project_id()].push_back( request->meta() );
  m_project_metas[request->project_id()].set_num_cameras( response->camera_id()+1 );
  // Set Base Transaction ID
  m_camera_metas[request->project_id()].back().set_base_transaction_id( m_project_metas[request->project_id()].max_iterations()*(m_camera_metas[request->project_id()].size()-1) );
  done->Run();
}

void
ProjectServiceImpl::CameraRead(::google::protobuf::RpcController* /*controller*/,
                               const ::asp::pho::CameraReadRequest* request,
                               ::asp::pho::CameraReadReply* response,
                               ::google::protobuf::Closure* done) {
  // Set empty meta .. for the error checking
  *(response->mutable_meta()) = CameraMeta();
  CHECK_PROJECT_RANGE();
  CHECK_CAMERA_RANGE();

  // echo correct
  response->set_project_id( request->project_id() );
  response->set_camera_id( request->camera_id() );

  *(response->mutable_meta()) =
    m_camera_metas[request->project_id()][request->camera_id()];
}

void
ProjectServiceImpl::CameraWrite(::google::protobuf::RpcController* /*controller*/,
                                const ::asp::pho::CameraWriteRequest* request,
                                ::asp::pho::CameraWriteReply* response,
                                ::google::protobuf::Closure* done) {
  CHECK_PROJECT_RANGE();
  CHECK_CAMERA_RANGE();

  // echo correct
  response->set_project_id( request->project_id() );
  response->set_camera_id( request->camera_id() );
  m_camera_metas[request->project_id()][request->camera_id()] =
    request->meta();
}

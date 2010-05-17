#ifndef __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__
#define __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__

#include <vw/Plate/RpcServices.h>
#include <vw/Plate/AmqpConnection.h>
#include <asp/PhotometryTK/ProjectService.h>

namespace asp {
namespace pho {

  class RemoteProjectFile {
    boost::shared_ptr<vw::platefile::AmqpRpcClient> m_rpc_controller;
    boost::shared_ptr<ProjectService> m_ptk_service;
    vw::int32 m_project_id;
    std::string m_name_short;
  public:
    RemoteProjectFile( std::string const& url );

    void OpenProjectMeta( ProjectMeta& meta );
    void UpdateIteration( vw::int32 const& i );
    // Returns it's index
    // --- User should read back camera meta
    vw::int32 CreateCameraMeta( CameraMeta const& meta );
    void ReadCameraMeta( vw::int32 i, CameraMeta& meta );
    void WriteCameraMeta( vw::int32 i, CameraMeta const& meta );
  };

}} //end namespace asp::pho

#endif//__ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__

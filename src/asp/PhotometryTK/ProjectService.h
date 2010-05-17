#ifndef __ASP_PHOTOMETRYTK_PROJECT_SERVICE_H__
#define __ASP_PHOTOMETRYTK_PROJECT_SERVICE_H__

#include <vw/Core/FundamentalTypes.h>
#include <asp/PhotometryTK/ProjectFile.pb.h>

#include <google/protobuf/service.h>

namespace asp {
namespace pho {

  // Implementation
  //  This is what the server interacts with. Clients just call ProjectService.
  // This server implementation does not ever deal with the creation of project
  // files, but only dishes out their information and allows for modification
  // of their camera information.
  class ProjectServiceImpl : public ProjectService {
    std::string m_root_directory;

    std::map<std::string,vw::int32> m_ptk_lookup;
    std::vector<ProjectMeta> m_project_metas;
    std::vector<std::vector<CameraMeta> > m_camera_metas;

  public:
    ProjectServiceImpl(std::string root_directory);

    void sync(); // Write to disk

    virtual void OpenRequest(::google::protobuf::RpcController* controller,
                             const ::asp::pho::ProjectOpenRequest* request,
                             ::asp::pho::ProjectOpenReply* response,
                             ::google::protobuf::Closure* done);
    virtual void ProjectUpdate(::google::protobuf::RpcController* controller,
                               const ::asp::pho::ProjectUpdateRequest* request,
                               ::asp::pho::ProjectUpdateReply* response,
                               ::google::protobuf::Closure* done);
    virtual void CameraCreate(::google::protobuf::RpcController* controller,
                              const ::asp::pho::CameraCreateRequest* request,
                              ::asp::pho::CameraCreateReply* response,
                              ::google::protobuf::Closure* done);
    virtual void CameraRead(::google::protobuf::RpcController* controller,
                            const ::asp::pho::CameraReadRequest* request,
                            ::asp::pho::CameraReadReply* response,
                            ::google::protobuf::Closure* done);
    virtual void CameraWrite(::google::protobuf::RpcController* controller,
                             const ::asp::pho::CameraWriteRequest* request,
                             ::asp::pho::CameraWriteReply* response,
                             ::google::protobuf::Closure* done);

  };

}} //end namespace asp::pho

#endif//__ASP_PHOTOMETRYTK_PROJECT_SERVICE_H__

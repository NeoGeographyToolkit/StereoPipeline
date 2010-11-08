// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__
#define __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__

#include <vw/Plate/Rpc.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <boost/lexical_cast.hpp>
#include <vw/Plate/HTTPUtils.h>

namespace vw {
namespace platefile {
  class PlateFile;
}}

namespace asp {
namespace pho {

  class RemoteProjectFile {
    boost::shared_ptr<vw::platefile::RpcClient<ProjectService> > m_client;
    vw::int32 m_project_id;
    std::string m_projectname;
    vw::platefile::Url m_url;
  public:
    RemoteProjectFile( vw::platefile::Url const& url );

    void get_project( ProjectMeta& meta ) const;
    void set_iteration( vw::int32 const& i );
    // Returns it's index
    // --- User should read back camera meta
    vw::int32 add_camera( CameraMeta const& meta );
    void get_camera( vw::int32 i, CameraMeta& meta ) const;
    void set_camera( vw::int32 i, CameraMeta const& meta );

    void get_platefiles( boost::shared_ptr<vw::platefile::PlateFile>& drg,
                         boost::shared_ptr<vw::platefile::PlateFile>& albedo,
                         boost::shared_ptr<vw::platefile::PlateFile>& reflect ) const;
  };

}} //end namespace asp::pho

#endif//__ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__

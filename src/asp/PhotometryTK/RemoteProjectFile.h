#ifndef __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__
#define __ASP_PHOTOMETRYTK_REMOTE_PROJECT_FILE_H__

#include <vw/Plate/RpcServices.h>
#include <vw/Plate/AmqpConnection.h>
#include <asp/PhotometryTK/ProjectService.h>
#include <boost/lexical_cast.hpp>

namespace asp {
namespace pho {

  // Parse a URL with the format: pf://<exchange>/<platefile name>.plate
  void parse_url(std::string const& url, std::string &hostname, int &port, 
                 std::string &exchange, std::string &file_name) {

    if (url.find("pf://") != 0) {
      vw_throw(vw::ArgumentErr() << "RemoteProjectFile::parse_url() -- this does not appear to be a well-formed URL: " << url);
    } else {
      std::string substr = url.substr(5);

      std::vector<std::string> split_vec;
      boost::split( split_vec, substr, boost::is_any_of("/") );

      // No hostname was specified: pf://<exchange>/<platefilename>.plate
      if (split_vec.size() == 2) {
        hostname = "localhost"; // default to localhost
        port = 5672;            // default rabbitmq port

        exchange = split_vec[0];
        file_name = split_vec[1];

        // No hostname was specified: pf://<ip address>:<port>/<exchange>/<platefilename>.plate
      } else if (split_vec.size() == 3) {

        exchange = split_vec[1];
        file_name = split_vec[2];

        std::vector<std::string> host_port_vec;
        boost::split( host_port_vec, split_vec[0], boost::is_any_of(":") );

        if (host_port_vec.size() == 1) {

          hostname = host_port_vec[0];
          port = 5672;            // default rabbitmq port

        } else if (host_port_vec.size() == 2) {

          hostname = host_port_vec[0];
          port = boost::lexical_cast<int>(host_port_vec[1]);

        } else {
          vw_throw(vw::ArgumentErr() << "RemoteProjectFile::parse_url() -- " 
                   << "could not parse hostname and port from URL string: " << url);
        }

      } else {
        vw_throw(vw::ArgumentErr() << "RemoteProjectFile::parse_url() -- "
                 << "could not parse URL string: " << url);
      }
    }
  }

  class RemoteProjectFile {
    boost::shared_ptr<vw::platefile::AmqpRpcClient> m_rpc_controller;
    boost::shared_ptr<ProjectService> m_ptk_service;
    vw::int32 m_project_id;
    std::string m_projectname;
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

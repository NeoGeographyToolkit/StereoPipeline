// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


#ifndef __PROJECT_FILE_IO_H__
#define __PROJECT_FILE_IO_H__

#include <vw/Core/FundamentalTypes.h>
#include <vw/Core/Exception.h>
#include <asp/PhotometryTK/ProjectFile.pb.h>
#include <algorithm>
#include <fstream>

#include <boost/filesystem.hpp>

namespace asp {
namespace pho {

  // Functors to help me
  namespace hidden {
    template <class StreamT>
    struct WriteCameraMeta {
      StreamT* m_stream;
      WriteCameraMeta( StreamT* stream ) : m_stream(stream) {}

      void operator()( CameraMeta& cam ) {
        (*m_stream) << vw::int32(cam.ByteSize());
        cam.SerializeToOstream( m_stream );
      }
    };

    template <class StreamT>
    struct ReadCameraMeta {
      StreamT* m_stream;
      ReadCameraMeta( StreamT* stream ) : m_stream(stream) {}

      void operator()( CameraMeta& cam ) {
        vw::int32 bytes;
        (*m_stream) >> bytes;
        char* i = new char[bytes];
        m_stream->read(i,bytes);
        cam.ParseFromArray(i,bytes);
        delete [] i;
      }
    };
  }

  // User interfaces
  // -------------------------------------------------------
  template <class IterT>
  void write_pho_project( std::string const& file,
                          ProjectMeta const& prj,
                          IterT cam_start, IterT cam_end ) {
    namespace fs = boost::filesystem;

    // Forcing file extension type
    std::string directory = file.substr(0,file.rfind("."));
    directory += ".ptk";

    // Check that the directory is there
    fs::create_directory( directory );

    // Actually start writing
    std::string prj_file = directory + "/photometrytk.dat";
    std::fstream output( prj_file.c_str(),
                         std::ios::out | std::ios::trunc | std::ios::binary );

    output << vw::int32( prj.ByteSize() );
    prj.SerializeToOstream(&output);
    std::for_each( cam_start, cam_end,
                   hidden::WriteCameraMeta<std::fstream>( &output ) );

    output.close();
  }

  template <class ContainerT>
  void read_pho_project( std::string const& file,
                         ProjectMeta& prj,
                         ContainerT& cams ) {
    namespace fs = boost::filesystem;
    cams.clear(); prj.Clear();

    BOOST_ASSERT( fs::is_directory(file) );
    std::string prj_file = file + "/photometrytk.dat";
    std::fstream input( prj_file.c_str(), std::ios::in | std::ios::binary );
    if ( !input )
      vw::vw_throw( vw::IOErr() << "Unable to open \"" << file << "\"." );
    {
      vw::int32 bytes;
      input >> bytes;
      char* i = new char[bytes];
      input.read(i,bytes);
      prj.ParseFromArray(i,bytes);
      delete i;
    }
    cams.resize( prj.num_cameras() );
    std::for_each( cams.begin(), cams.end(),
                   hidden::ReadCameraMeta<std::fstream>( &input ) );

    input.close();
  }

}} // namespace asp::pho

#endif//__PROJECT_FILE_IO_H__

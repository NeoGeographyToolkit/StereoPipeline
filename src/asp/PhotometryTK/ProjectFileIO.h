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
    struct WriteCameraMeta {
      std::ostream* m_stream;
      WriteCameraMeta( std::ostream* stream ) : m_stream(stream) {}

      void operator()( CameraMeta& cam ) {
        // ByteSize is just 'int' in protobuf. We typecast to be
        // consistent across platforms.
        vw::int32 bytes = cam.ByteSize();
        m_stream->write( (char*)&bytes, 4);
        if ( !cam.SerializeToOstream( m_stream ) )
          vw::vw_throw( vw::IOErr() << "Failed writing camera meta!\n" );
      }
    };

    struct ReadCameraMeta {
      std::istream* m_stream;
      ReadCameraMeta( std::istream* stream ) : m_stream(stream) {}

      void operator()( CameraMeta& cam ) {
        vw::int32 bytes;
        m_stream->read( (char*)&(bytes), 4 );
        char* i = new char[bytes];
        m_stream->read(i,bytes);
        if ( !cam.ParseFromArray(i,bytes) )
          vw::vw_throw( vw::IOErr() << "Failed reading camera meta!\n" );
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

    std::string prj_file = directory + "/photometrytk.dat";
    try { // Actually start writing
      std::fstream output( prj_file.c_str(),
                           std::ios::out | std::ios::trunc | std::ios::binary );
      output.exceptions(std::ofstream::failbit | std::ofstream::badbit);

      vw::int32 bytes = prj.ByteSize();
      output.write( (char*)&bytes, 4 );
      if ( !prj.SerializeToOstream(&output) )
        vw::vw_throw( vw::IOErr() << "Failed writing project meta!\n" );
      std::for_each( cam_start, cam_end,
                     hidden::WriteCameraMeta( &output ) );

    } catch ( const std::ofstream::failure& e ) {
      vw::vw_throw( vw::IOErr() << "write_pho_project: Could not write file: " << file << " (" << e.what() << ")" );
    } catch ( const vw::IOErr& e ) {
      vw::vw_throw( vw::IOErr() << "write_pho_project: Could not write file: " << file << " (" << e.what() << ")" );
    }
  }

  template <class ContainerT>
  void read_pho_project( std::string const& file,
                         ProjectMeta& prj,
                         ContainerT& cams ) {
    namespace fs = boost::filesystem;
    cams.clear(); prj.Clear();

    BOOST_ASSERT( fs::is_directory(file) );
    std::string prj_file = file + "/photometrytk.dat";

    try {
      std::fstream input( prj_file.c_str(), std::ios::in | std::ios::binary );
      input.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      {
        vw::int32 bytes;
        input.read( (char*)&(bytes), 4 );
        char* i = new char[bytes];
        input.read(i,bytes);
        if ( !prj.ParseFromArray(i,bytes) )
          vw::vw_throw( vw::IOErr() << "Failed reading project meta!\n" );
        delete i;
      }
      cams.resize( prj.num_cameras() );
      std::for_each( cams.begin(), cams.end(),
                     hidden::ReadCameraMeta( &input ) );
    } catch ( const std::ifstream::failure& e ) {
      vw::vw_throw( vw::IOErr() << "read_pho_project: Could not read file: " << file << " (" << e.what() << ")" );
    } catch ( const vw::IOErr& e ) {
      vw::vw_throw( vw::IOErr() << "read_pho_project: Could not read file: " << file << " (" << e.what() << ")" );
    }
  }

}} // namespace asp::pho

#endif//__PROJECT_FILE_IO_H__

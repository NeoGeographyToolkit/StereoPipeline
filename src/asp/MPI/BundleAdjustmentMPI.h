// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file BundleAdjustmentMPI.h
///
/// Header to define MPI interactions

#ifndef __ASP_MPI_BUNDLE_ADJUSTMENT_MPI_H__
#define __ASP_MPI_BUNDLE_ADJUSTMENT_MPI_H__

#include <boost/mpi.hpp>
#include <iostream>
#include <boost/serialization/string.hpp>
namespace mpi = boost::mpi;
#include <boost/foreach.hpp>

#include <asp/IsisIO/IsisAdjustCameraModel.h>

enum MPISlaveTask {
  LoadCameraModels,
  LoadControlNetwork,
  SolveJacobian,
  SolveUpdateError,
  Finish
};

namespace vw {
namespace camera {

  class MPISlave {
    mpi::communicator & m_world;
    std::vector<boost::shared_ptr<CameraModel> > m_cameras;
  public:
    MPISlave( mpi::communicator & world) : m_world(world) {
      vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                       << " : Starting\n";
      int task;
      while (true) {
        broadcast(m_world, task, 0);
        if ( task == LoadCameraModels ) {
          vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                           << " : Loading Camera Models\n";
          m_cameras.resize(0);
          std::vector<std::string> camera_files;
          broadcast(m_world, camera_files, 0);
          BOOST_FOREACH( std::string camera_file, camera_files ) {
            boost::shared_ptr<asp::BaseEquation> posF( new asp::PolyEquation(0) );
            boost::shared_ptr<asp::BaseEquation> poseF( new asp::PolyEquation(0) );
            m_cameras.push_back( boost::shared_ptr<IsisAdjustCameraModel>( new IsisAdjustCameraModel( camera_file,
                                                                                                      posF, poseF )));
          }
        } else if ( task == LoadControlNetwork ) {
          vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                           << " : Loading Control Network\n";
        } else if ( task == SolveJacobian ) {
          vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                           << " : Solving Jacobian\n";
        } else if ( task == SolveUpdateError ) {
          vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                           << " : Solving Update Error\n";
        } else if ( task == Finish ) {
          vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                           << " : Finished\n";
          break;
        }
      }
    }
  };

}}

#endif//__ASP_MPI_BUNDLE_ADJUSTMENT_MPI_H__

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
    unsigned m_camera_start;

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
          m_camera_start = (m_world.rank()-1)*float(camera_files.size())/float(m_world.size()-1);
          unsigned cam_e = m_world.rank()*float(camera_files.size())/float(m_world.size()-1);
          for ( unsigned i = m_camera_start; i < cam_e; i++ ) {
            vw_out(DebugMessage,"mpi_slave") << "\tSlave " << m_world.rank()
                                             << " : Loading "
                                             << camera_files[i] << "\n";
            boost::shared_ptr<asp::BaseEquation> posF( new asp::PolyEquation(0) );
            boost::shared_ptr<asp::BaseEquation> poseF( new asp::PolyEquation(0) );
            m_cameras.push_back( boost::shared_ptr<IsisAdjustCameraModel>( new IsisAdjustCameraModel( camera_files[i],
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

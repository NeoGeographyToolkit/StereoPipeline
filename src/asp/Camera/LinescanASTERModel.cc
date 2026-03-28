// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

#include <asp/Camera/ASTER_XML.h>
#include <asp/Camera/CsmModelFit.h>
#include <asp/Camera/LinescanASTERModel.h>

namespace asp {

ASTERCameraModel::ASTERCameraModel(
           std::vector<std::vector<vw::Vector2>> const& lattice_mat,
				   std::vector<std::vector<vw::Vector3>> const& sight_mat,
				   std::vector<std::vector<vw::Vector3>> const& world_sight_mat,
				   std::vector<vw::Vector3>              const& sat_pos,
				   vw::Vector2i                          const& image_size) {

  if (lattice_mat.empty() || lattice_mat[0].empty())
    vw::vw_throw(vw::ArgumentErr() << "Empty matrix of lattice points.\n");

  int min_col = lattice_mat.front().front().x();
  int min_row = lattice_mat.front().front().y();

  int max_col = lattice_mat.back().back().x();
  int max_row = lattice_mat.back().back().y();

  int num_rows = lattice_mat.size();
  int num_cols = lattice_mat.front().size();

  // The spacing between rows must be integer
  double tol = 1e-10;
  double d_row = double(max_row - min_row)/(num_rows - 1.0);
  if (std::abs(d_row - round(d_row)) > tol)
    vw::vw_throw(vw::ArgumentErr()
                   << "The spacing between lattice points must be integer.\n");
  d_row = round(d_row);

  // The spacing between columns must be integer
  double d_col = double(max_col - min_col)/(num_cols - 1.0);
  if (std::abs(d_col - round(d_col)) > tol)
    vw::vw_throw(vw::ArgumentErr()
                   << "The spacing between lattice points must be integer.\n");
  d_col = round(d_col);

  if ((int)sat_pos.size() != num_rows)
    vw::vw_throw(vw::ArgumentErr()
                   << "The number of rows of lattice points does not "
                   << "agree with the number of satellite positions.\n");

  if (num_rows != (int)world_sight_mat.size() ||
      num_cols != (int)world_sight_mat[0].size())
    vw::vw_throw(vw::ArgumentErr()
                   << "The number of rows or columns of lattice points does not "
                   << "agree with the number of sight vectors.\n");

  // CSM fitting assumes the detector starts at column 0
  if (min_col != 0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot use the CSM model with ASTER cameras "
                 << "if the first column index of the lattice matrix is not 0.\n");

  vw::cartography::Datum datum("WGS84"); // ASTER is for Earth
  std::string sensor_id = "ASTER";
  vw::vw_out() << "Using the CSM model with " << sensor_id << " cameras.\n";

  // Fit a CSM sensor with distortion to given tabulated sight directions
  bool fit_distortion = true;
  fitCsmLinescan(sensor_id, datum, image_size, sat_pos, world_sight_mat,
              min_col, min_row, d_col, d_row, fit_distortion, *this);
}

boost::shared_ptr<ASTERCameraModel>
load_ASTER_camera_model_from_xml(std::string const& path) {

  // XYZ coordinates are in the ITRF coordinate frame which means GCC coordinates.
  // - The velocities are in the same coordinate frame, not in some local frame.

  vw::vw_out(vw::DebugMessage, "asp") << "Loading ASTER camera file: "
                                      << path << "\n";

  // Parse the ASTER XML file
  ASTERXML xml_reader;
  xml_reader.read_xml(path);

  // Feed everything into a new camera model.
  return boost::shared_ptr<ASTERCameraModel>
    (new ASTERCameraModel(xml_reader.m_lattice_mat,
                          xml_reader.m_sight_mat,
                          xml_reader.m_world_sight_mat,
                          xml_reader.m_sat_pos,
                          xml_reader.m_image_size));
}

} // end namespace asp


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


#include <asp/Camera/ASTER_XML.h>
#include <vw/Camera/CameraSolve.h>
#include <asp/Camera/LinescanASTERModel.h>
namespace asp {

using namespace vw;

ASTERCameraModel::ASTERCameraModel(std::vector< std::vector<vw::Vector2> > const& lattice_mat,
				   std::vector< std::vector<vw::Vector3> > const& sight_mat,
				   std::vector< std::vector<vw::Vector3> > const& world_sight_mat,
				   std::vector<vw::Vector3>                const& sat_pos,
				   vw::Vector2                             const& image_size,
				   boost::shared_ptr<vw::camera::CameraModel> rpc_model):
  m_lattice_mat(lattice_mat), m_sight_mat(sight_mat),
  m_world_sight_mat(world_sight_mat),
  m_sat_pos(sat_pos), m_image_size(image_size), m_rpc_model(rpc_model){
  
  if (m_lattice_mat.empty() || m_lattice_mat[0].empty()) 
    vw::vw_throw( vw::ArgumentErr() << "Empty matrix of lattice points.\n" );
  
  int min_col = m_lattice_mat.front().front().x();
  int min_row = m_lattice_mat.front().front().y();
  
  int max_col = m_lattice_mat.back().back().x();
  int max_row = m_lattice_mat.back().back().y();
  
  int num_rows = m_lattice_mat.size();
  int num_cols = m_lattice_mat.front().size();
      
  // The spacing between rows must be integer
  double tol = 1e-10;
  double d_row = double(max_row - min_row)/(num_rows - 1.0);
  if (std::abs(d_row - round(d_row)) > tol) {
    vw::vw_throw( vw::ArgumentErr()
                  << "The spacing between lattice points must be integer.\n" );
  }
  d_row = round(d_row);

  // The spacing between columns must be integer
  double d_col = double(max_col - min_col)/(num_cols - 1.0);
  if (std::abs(d_col - round(d_col)) > tol) {
    vw::vw_throw( vw::ArgumentErr()
                  << "The spacing between lattice points must be integer.\n" );
  }
  d_col = round(d_col);

  if ((int)m_sat_pos.size() != num_rows) {
    vw::vw_throw( vw::ArgumentErr()
                  << "The number of rows of lattice points does not "
                  << "agree with the number of satellite positions.\n" );
  }

  m_interp_sat_pos
    = vw::camera::LinearPiecewisePositionInterpolation(m_sat_pos, min_row, d_row);

  m_interp_sight_mat
    = vw::camera::SlerpGridPointingInterpolation(m_world_sight_mat, min_row, d_row, min_col, d_col);

#if 0
  // This is useful in testing how well point_to_pixel() works for given point and pixel.
  double spacing = 9.5655;
  double max_err = 0;
  for (double col = 0; col < m_image_size[0]; col+= spacing) {
    for (double row = 0; row < m_image_size[1]; row+= spacing) {
      Vector2 pix(col, row);

      double datum_h = 6378137;
      double h = datum_h + 1000; // 1000 m above the datum
      Vector3 C = this->camera_center(pix);
      Vector3 D = this->pixel_to_vector(pix);

      double Delta = dot_prod(C, D)*dot_prod(C, D) - dot_prod(D, D)*(dot_prod(C, C) - h*h);
      double t = ( -dot_prod(C, D) - sqrt(Delta) ) / dot_prod(D, D);
      Vector3 P = this->camera_center(pix) + t * this->pixel_to_vector(pix);

      std::cout << "cam ctr is in km: " << norm_2(this->camera_center(pix))/1000 << std::endl;
      std::cout << "point height and diff in km: "  << norm_2(P)/1000  << ' '
		<< (norm_2(P) - 6378137)/1000 << std::endl;

      Vector2 pix2 = this->point_to_pixel(P);
      double err = norm_2(pix-pix2);
      std::cout << "Pixel error: " << pix << ' ' << pix2 << ' ' << err << std::endl;
      max_err = std::max(err, max_err);
    }
  }
  std::cout << "max err is " << max_err << std::endl;
#endif
}

// Project the point onto the camera. Sometimes, but not always, seeding with the RPC
// model is beneficial.  
vw::Vector2 ASTERCameraModel::point_to_pixel(Vector3 const& point, Vector2 const& start_in) const {

  // - This method will be slower but works for more complicated geometries
  vw::camera::CameraGenericLMA model( this, point );
  int status;
  vw::Vector2 start = m_image_size / 2.0; // Use the center as the initial guess

  bool has_guess = false;
  
  // If the user provided a column number guess.
  if (start_in[0] >= 0) {
    start[0] = start_in[0];
    has_guess = true;
  }
  
  // If the user provided a row number guess.
  if (start_in[1] >= 0) {
    start[1] = start_in[1];
    has_guess = true;
  }

  if (!has_guess) {
    double min_err = norm_2(model(start));
    // No good initial guess. The method will fail to converge.
    // Iterate through the lattice to find a good initial guess.
    for (int row = 0; row < int(m_lattice_mat.size())-1; row++) {
      // TODO: Experiment more with the number below.
      int T = 100; // This way we'll sample about every 4-th pixel since dcol = 400
      int col = m_lattice_mat.front().size()/2;
      for (int r = 0; r < T; r++) {
	double wr = double(r)/(T-1.0);
	Vector2 pt
	  = wr*m_lattice_mat[row+1][col]
	  + (1-wr)*m_lattice_mat[row][col];
	double err = norm_2(model(pt));
	if (err < min_err) {
	  min_err = err;
	  start = pt;
	}
      }
    }
  }

#if 0
  // This exhaustive search for an initial guess is an overkill
  if (!has_guess) {
    double min_err = norm_2(model(start));
    // No good initial guess. The method will fail to converge.
    // Iterate through the lattice to find a good initial guess.
    for (int row = 0; row < int(m_lattice_mat.size())-1; row++) {
      for (int col = 0; col < int(m_lattice_mat.front().size())-1; col++) {
	int T = 100;
	for (int r = 0; r < T; r++) {
	  for (int c = 0; c < T; c++) {
	    double wr = double(r)/(T-1.0);
	    double wc = double(c)/(T-1.0);
	    Vector2 pt
	      = wr*wc*m_lattice_mat[row+1][col+1]
	      + (1-wr)*wc*m_lattice_mat[row][col+1]
	      + wr*(1-wc)*m_lattice_mat[row+1][col]
	      + (1-wr)*(1-wc)*m_lattice_mat[row][col];
	    double err = norm_2(model(pt));
	    if (err < min_err) {
	      min_err = err;
	      start = pt;
	    }
	  }
	}
      }
    }
  }
#endif

  // Solver constants
  const double ABS_TOL = 1e-16;
  const double REL_TOL = 1e-16;
  const int    MAX_ITERATIONS = 1e+5;
  const double MAX_ERROR = 1e-2;

  // Try two initial guesses. TODO: Study this in more detail.
  
  // Solution with user-provided initial guess
  Vector3 objective(0, 0, 0);
  vw::Vector2 solution1 = vw::math::levenberg_marquardt(model, start, objective, status,
							ABS_TOL, REL_TOL, MAX_ITERATIONS);
  

  // Solution with the RPC initial guess
  Vector2 start_rpc = this->m_rpc_model->point_to_pixel(point);
  vw::Vector2 solution2 = vw::math::levenberg_marquardt(model, start_rpc, objective, status,
							ABS_TOL, REL_TOL, MAX_ITERATIONS);
  
  
  double error1 = norm_2(model(solution1));
  double error2 = norm_2(model(solution2));
  double error  = std::min(error1, error2);
  
  vw::Vector2 solution;
  if (error1 < error2) {
    solution = solution1;
  } else if (error1 > error2) {
    solution = solution2;
  }
  
  // Check the error - If it is too high then the solver probably got
  // stuck at the edge of the image.
  VW_ASSERT( (status > 0) && (error < MAX_ERROR),
             vw::camera::PointToPixelErr() << "Unable to project point into LinescanASTER model" );
  
  return solution;
}

vw::Vector2 ASTERCameraModel::point_to_pixel(Vector3 const& point, double starty) const {
  return this->point_to_pixel(point, Vector2(-1.0, starty));
}
  
vw::Vector3 ASTERCameraModel::camera_center(vw::Vector2 const& pix) const {
  return m_interp_sat_pos(pix.y());
}
    
vw::Vector3 ASTERCameraModel::pixel_to_vector(vw::Vector2 const& pixel) const{
  try {
    return m_interp_sight_mat(pixel);
  } catch(const vw::Exception &e) {
    // Repackage any of our exceptions thrown below this point as a 
    //  pixel to ray exception that other code will be able to handle.
    vw_throw(vw::camera::PixelToRayErr() << e.what());
  }
  return vw::Vector3(); // Never reached
}
    
  boost::shared_ptr<ASTERCameraModel>
  load_ASTER_camera_model_from_xml(std::string const& path,
				   boost::shared_ptr<vw::camera::CameraModel> rpc_model){

  // XYZ coordinates are in the ITRF coordinate frame which means GCC coordinates.
  // - The velocities are in the same coordinate frame, not in some local frame.

  vw_out(vw::DebugMessage,"asp") << "Loading ASTER camera file: " << path << std::endl;

  // Parse the ASTER XML file
  ASTERXML xml_reader;
  xml_reader.read_xml(path);
  
  // Feed everything into a new camera model.
  return boost::shared_ptr<ASTERCameraModel>(new ASTERCameraModel(xml_reader.m_lattice_mat,
								  xml_reader.m_sight_mat,
								  xml_reader.m_world_sight_mat,
								  xml_reader.m_sat_pos,
								  xml_reader.m_image_size,
								  rpc_model));
  
} // End function load_ASTER_camera_model()


} // end namespace asp


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


/// \file bathy_plane_calc.cc
///

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Math/RANSAC.h>
#include <vw/Camera/CameraModel.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Core/ThreadPool.h>
#include <vw/Math/RandomSet.h>

#include <Eigen/Dense>

#include <random>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>

using namespace vw;
using namespace vw::cartography;
using namespace vw::camera;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Add a polygon made up of just one point to given set of polygons.
// Later that set will be saved as a shapefile made up of points.
void addPointToPoly(vw::geometry::dPoly & poly, Vector2 const& p) {
  std::vector<double> vx, vy;
  vx.push_back(p.x());
  vy.push_back(p.y());
  bool isPolyClosed = true;
  std::string poly_color = "green";
  std::string layer = "";
  poly.appendPolygon(vx.size(), vw::geometry::vecPtr(vx), vw::geometry::vecPtr(vy),
                     isPolyClosed, poly_color, layer);
}

// Estimate the projection and convert point_vec to projected coordinates
void find_projection(// Inputs
                     vw::cartography::GeoReference const& dem_georef,
                     std::vector<vw::Vector3> const& llh_vec,
                     // Outputs
                     double & proj_lat, double & proj_lon,
                     vw::cartography::GeoReference & stereographic_georef,
                     std::vector<Eigen::Vector3d> & point_vec) {

  // Initialize the outputs
  proj_lat = -1.0;
  proj_lon = -1.0;
  point_vec.clear();
  
  // Find the mean water height
  Vector3 mean_llh;
  for (size_t it = 0; it < llh_vec.size(); it++) 
    mean_llh += llh_vec[it];
  
  mean_llh /= llh_vec.size();

  stereographic_georef.set_datum(dem_georef.datum());
  double scale = 1.0;
  proj_lat = mean_llh[1];
  proj_lon = mean_llh[0];
  stereographic_georef.set_stereographic(proj_lat, proj_lon, scale);
  
  // Convert the points to projected coordinates with this georeference
  for (size_t it = 0; it < llh_vec.size(); it++) {
    Vector3 point = stereographic_georef.geodetic_to_point(llh_vec[it]);
    
    Eigen::Vector3d eigen_point;
    for (size_t coord = 0; coord < 3; coord++) 
      eigen_point[coord] = point[coord];

    point_vec.push_back(eigen_point);
  }
}

// This is not used for now, it could be used to make the logic
// of processing mask boundary points multi-threaded.
class MaskBoundaryTask : public vw::Task, private boost::noncopyable {
  vw::BBox2i m_bbox; // Region of image we're working in

  ImageViewRef<float>                   m_mask;
  float                                 m_mask_nodata_val;
  boost::shared_ptr<CameraModel>        m_camera_model;
  vw::cartography::GeoReference         m_shape_georef;
  vw::cartography::GeoReference         m_dem_georef;
  ImageViewRef<PixelMask<float>>        m_masked_dem;
  
  // Note how all of these are aliases
  Mutex                        & m_mutex;
  std::vector<Eigen::Vector3d> & m_point_vec;
  std::vector<vw::Vector3>     & m_llh_vec;
  std::vector<vw::Vector2>     & m_used_vertices;
  
public:
  MaskBoundaryTask(vw::BBox2i                            bbox,
                   ImageViewRef<float>                   mask,
                   float                                 mask_nodata_val,
                   boost::shared_ptr<CameraModel>        camera_model,
                   vw::cartography::GeoReference const & shape_georef,
                   vw::cartography::GeoReference const & dem_georef,
                   ImageViewRef<PixelMask<float>>        masked_dem,
                   Mutex                               & mutex,
                   std::vector<Eigen::Vector3d>        & point_vec,
                   std::vector<vw::Vector3>            & llh_vec,
                   std::vector<vw::Vector2>            & used_vertices):
    m_bbox(bbox), m_mask(mask), m_mask_nodata_val(mask_nodata_val),
    m_camera_model(camera_model), m_shape_georef(shape_georef),
    m_dem_georef(dem_georef), m_masked_dem(masked_dem),
    m_mutex(mutex), m_point_vec(point_vec),
    m_llh_vec(llh_vec), m_used_vertices(used_vertices) {}
  
  void operator()() {

    // Grow the box by 1 pixel as we need to look at the immediate neighbors
    BBox2i extra_box = m_bbox;
    extra_box.expand(1); 
    extra_box.crop(bounding_box(m_mask));

    // Make a local copy of the tile
    ImageView<float> mask_tile = crop(m_mask, extra_box);

    for (int col = 0; col < mask_tile.cols(); col++) {
      for (int row = 0; row < mask_tile.rows(); row++) {

        // Look at pixels above threshold which have neighbors <= threshold
        if (mask_tile(col, row) <= m_mask_nodata_val) 
          continue;

        // The four neighbors
        int col_vals[4] = {-1, 0, 0, 1};
        int row_vals[4] = {0, -1, 1, 0};
          
        bool border_pix = false;
        for (int it = 0; it < 4; it++) {
            
          int icol = col + col_vals[it];
          int irow = row + row_vals[it];

          if (icol < 0 || irow < 0 || icol >= mask_tile.cols() || irow >= mask_tile.rows()) 
            continue;
            
          if (mask_tile(icol, irow) <= m_mask_nodata_val) {
            border_pix = true;
            break;
          }
        }
          
        if (!border_pix) 
          continue;

        // Create the pixel in the full image coordinates
        Vector2 pix = Vector2(col, row) + extra_box.min();

        // Only work on pixels in the current box (earlier had a
        // bigger box to be able to examine neighbors).
        if (!m_bbox.contains(pix))
          continue;
        
        // The ray going to the ground
        // Here we assume that the camera model is thread-safe, which is
        // true for all cameras except ISIS, and this code will be used
        // on Earth only.
        Vector3 cam_ctr = m_camera_model->camera_center(pix);
        Vector3 cam_dir = m_camera_model->pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM.
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double height_error_tol = 0.001; // in meters
        double max_abs_tol = 1e-14;
        double max_rel_tol = 1e-14;
        int num_max_iter = 100;
        Vector3 xyz_guess(0, 0, 0);
        Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, m_masked_dem,
           m_dem_georef, treat_nodata_as_zero,
           has_intersection, height_error_tol, max_abs_tol, max_rel_tol, 
           num_max_iter, xyz_guess);
          
        if (!has_intersection) 
          continue;

        Vector3 llh = m_dem_georef.datum().cartesian_to_geodetic(xyz);

        Eigen::Vector3d eigen_xyz;
        for (size_t coord = 0; coord < 3; coord++) 
          eigen_xyz[coord] = xyz[coord];

        // TODO(oalexan1): This is fragile due to the 360 degree
        // uncertainty in latitude
        Vector2 proj_pt = m_shape_georef.lonlat_to_point(Vector2(llh[0], llh[1]));

        {
          // Need to make sure to lock the shared resource
          Mutex::Lock lock(m_mutex);
          m_point_vec.push_back(eigen_xyz);
          m_used_vertices.push_back(proj_pt);
          m_llh_vec.push_back(llh);
        }
      }
      
    }
    
  }
  
};

// Find the mask boundary (points where the points in the mask have
// neighbors not in the mask), shoot points from there onto the DEM,
// and return the obtained points.
void find_points_at_mask_boundary(ImageViewRef<float> mask,
                                  float mask_nodata_val,
                                  boost::shared_ptr<CameraModel> camera_model,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  ImageViewRef<PixelMask<float>> masked_dem,
                                  int num_samples,
                                  std::vector<Eigen::Vector3d> & point_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & used_vertices) {

  // Ensure that the outputs are initialized
  point_vec.clear();
  llh_vec.clear();
  used_vertices.clear();

#if 0

  // For some reason parallel processing is slower than serial processing.
  // Likely it is because the DEM is shared. Need extra heuristics to cut
  // that one as appropriate for each task.
  
  // Subdivide the box for parallel processing
  int block_size = vw::vw_settings().default_tile_size();
  FifoWorkQueue queue(vw_settings().default_num_threads());
  std::vector<BBox2i> bboxes = subdivide_bbox(mask, block_size, block_size);
  
  for (size_t it = 0; it < bboxes.size(); it++) {
    std::cout << "Box is " << bboxes[it] << std::endl;
  }

  Mutex mutex;
  for (size_t it = 0; it < bboxes.size(); it++) {
    boost::shared_ptr<MaskBoundaryTask>
      task(new MaskBoundaryTask(bboxes[it],  mask, mask_nodata_val, camera_model,
                                shape_georef, dem_georef, masked_dem, mutex,
                                point_vec, llh_vec, used_vertices));
    queue.add_task(task);
  }
  
  queue.join_all();
  
#else

  // Let the mask boundary be the mask pixels whose value
  // is above threshold and which border pixels whose values
  // is not above threshold.

  vw_out() << "Processing points at mask boundary.\n";
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / mask.cols();
  tpc.report_progress(0);

  for (int col = 0; col < mask.cols(); col++) {
    for (int row = 0; row < mask.rows(); row++) {

      // Look at pixels above threshold which have neighbors <= threshold
      if (mask(col, row) <= mask_nodata_val) 
        continue;

      // The four neighbors
      int col_vals[4] = {-1, 0, 0, 1};
      int row_vals[4] = {0, -1, 1, 0};
          
      bool border_pix = false;
      for (int it = 0; it < 4; it++) {
            
        int icol = col + col_vals[it];
        int irow = row + row_vals[it];

        if (icol < 0 || irow < 0 || icol >= mask.cols() || irow >= mask.rows()) 
          continue;
            
        if (mask(icol, irow) <= mask_nodata_val) {
          border_pix = true;
          break;
        }
      }
          
      if (!border_pix) 
        continue;

      // The ray going to the ground
      Vector2 pix(col, row);
      Vector3 cam_ctr = camera_model->camera_center(pix);
      Vector3 cam_dir = camera_model->pixel_to_vector(pix);

      // Intersect the ray going from the given camera pixel with a DEM.
      bool treat_nodata_as_zero = false;
      bool has_intersection = false;
      double height_error_tol = 0.001; // in meters
      double max_abs_tol = 1e-14;
      double max_rel_tol = 1e-14;
      int num_max_iter = 100;
      Vector3 xyz_guess(0, 0, 0);
      Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
        (cam_ctr, cam_dir, masked_dem,
         dem_georef, treat_nodata_as_zero,
         has_intersection, height_error_tol, max_abs_tol, max_rel_tol, 
         num_max_iter, xyz_guess);
          
      if (!has_intersection) 
        continue;

      Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

      Eigen::Vector3d eigen_xyz;
      for (size_t coord = 0; coord < 3; coord++) 
        eigen_xyz[coord] = xyz[coord];

      // TODO(oalexan1): This is fragile due to the 360 degree
      // uncertainty in latitude
      Vector2 proj_pt = shape_georef.lonlat_to_point(Vector2(llh[0], llh[1]));
          
      point_vec.push_back(eigen_xyz);
      used_vertices.push_back(proj_pt);
      llh_vec.push_back(llh);
    }

    tpc.report_incremental_progress(inc_amount);
  }

  tpc.report_finished();
  
#endif
  
  int num_pts = point_vec.size();
  if (num_pts > num_samples) {
    vw_out() << "Found " << num_pts << " samples at mask boundary, points but only "
             << num_samples << " samples are desired. Picking a random subset "
             << "of this size.\n";

    // Select a subset
    std::vector<int> w;
    vw::math::pick_random_indices_in_range(num_pts, num_samples, w);
    std::vector<Eigen::Vector3d> big_point_vec     = point_vec;     point_vec.clear();
    std::vector<vw::Vector3>     big_llh_vec       = llh_vec;       llh_vec.clear();
    std::vector<vw::Vector2>     big_used_vertices = used_vertices; used_vertices.clear();
    for (size_t it = 0; it < w.size(); it++) {
      int random_index = w[it];
      point_vec.push_back(big_point_vec[random_index]);
      llh_vec.push_back(big_llh_vec[random_index]);
      used_vertices.push_back(big_used_vertices[random_index]);
    }
  }
  
  return;
}

// Compute the 3D locations at the shape corners based on interpolating
// into the DEM and converting either to ECEF or to local projected
// stereographic coordinates.
// If using a curved water surface, compute the stereographic georeference
// with the projection center being the mean lon and lat, and make the 3D locations
// in reference to this projection
void find_points_at_shape_corners(std::vector<vw::geometry::dPoly> const& polyVec,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  ImageViewRef< PixelMask<float> > interp_dem,
                                  std::vector<Eigen::Vector3d> & point_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & used_vertices) {
  
  // Ensure that the outputs are initialized
  point_vec.clear();
  llh_vec.clear();
  used_vertices.clear();

  int total_num_pts = 0;
  
  for (size_t p = 0; p < polyVec.size(); p++){
    vw::geometry::dPoly const& poly = polyVec[p];
      
    const double * xv       = poly.get_xv();
    const double * yv       = poly.get_yv();
    const int    * numVerts = poly.get_numVerts();
    int numPolys            = poly.get_numPolys();

    int start = 0;
    for (int pIter = 0; pIter < numPolys; pIter++){
        
      if (pIter > 0) start += numVerts[pIter - 1];

      int numV = numVerts[pIter];
      for (int vIter = 0; vIter < numV; vIter++) {

        total_num_pts++;
        
        Vector2 proj_pt(xv[start + vIter], yv[start + vIter]);

        // Convert from projected coordinates to lonlat
        Vector2 lonlat = shape_georef.point_to_lonlat(proj_pt);

        // Convert to DEM pixel
        Vector2 pix = dem_georef.lonlat_to_pixel(lonlat);

        PixelMask<float> h = interp_dem(pix.x(), pix.y());

        if (!is_valid(h)) 
          continue;

        Vector3 llh;
        llh[0] = lonlat[0];
        llh[1] = lonlat[1];
        llh[2] = h.child();

        Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);
        Eigen::Vector3d eigen_xyz;
        for (size_t coord = 0; coord < 3; coord++) 
          eigen_xyz[coord] = xyz[coord];

        point_vec.push_back(eigen_xyz);
        used_vertices.push_back(proj_pt);
        llh_vec.push_back(llh);
      }
    }
  }

  vw_out() << "Read " << total_num_pts << " vertices, with " << llh_vec.size()
            << " of them having a valid DEM height value."  << std::endl;
}

// Read a set of measurements in CSV format, to use later to fit the water surface
void find_points_from_meas_csv(std::string const& water_height_measurements,
                               std::string const& csv_format_str,
                               vw::cartography::GeoReference const& shape_georef,
                               // Outputs
                               std::vector<vw::Vector3> & llh_vec,
                               std::vector<vw::Vector2> & used_vertices) {
  // Wipe the outputs
  llh_vec.clear();
  used_vertices.clear();

  // Read the CSV file
  asp::CsvConv csv_conv;
  std::string csv_proj4_str = ""; // not needed
  try {
    csv_conv.parse_csv_format(csv_format_str, csv_proj4_str);
  }catch (...) {
    // Give a more specific error message
    vw_throw(ArgumentErr() << "Could not parse --csv-format. Was given: "
             << csv_format_str << ".\n");
  }
  std::list<asp::CsvConv::CsvRecord> pos_records;
  typedef std::list<asp::CsvConv::CsvRecord>::const_iterator RecordIter;
  csv_conv.read_csv_file(water_height_measurements, pos_records);
  
  // Create llh and vertices
  for (RecordIter iter = pos_records.begin(); iter != pos_records.end(); iter++) {
    Vector3 llh = csv_conv.csv_to_geodetic(*iter, shape_georef);
    llh_vec.push_back(llh);
    Vector2 proj_pt = shape_georef.lonlat_to_point(Vector2(llh[0], llh[1]));
    used_vertices.push_back(proj_pt);
  }

  return;
}

void sample_mask_boundary(std::vector<Eigen::Vector3d> const& point_vec,
                          std::string const& mask_boundary_shapefile) {
  
  vw::cartography::GeoReference llh_georef; // create a new explicit longlat WGS84 georef
  llh_georef.set_geographic();
  vw::cartography::Datum datum("WGS_1984");
  llh_georef.set_datum(datum);
  vw::geometry::dPoly samplePoly;
  for (size_t ptIter = 0; ptIter < point_vec.size(); ptIter++) {
    vw::Vector3 xyz; // convert Eigen:Vector3 to vw::Vector3
    for (int c = 0; c < 3; c++) xyz[c] = point_vec[ptIter][c];
    vw::Vector3 llh = llh_georef.datum().cartesian_to_geodetic(xyz);
    addPointToPoly(samplePoly, subvector(llh, 0, 2));
  }

  std::vector<vw::geometry::dPoly> samplePolyVec;
  samplePolyVec.push_back(samplePoly);
  vw_out() << "Writing shapefile of samples at mask boundary: "
           << mask_boundary_shapefile << "\n";
  bool has_llh_georef = true;
  write_shapefile(mask_boundary_shapefile, has_llh_georef, llh_georef,
                  samplePolyVec);
}

// Best fit plane without outlier removal
std::pair<Eigen::Vector3d, Eigen::Vector3d>
best_plane_from_points(const std::vector<Eigen::Vector3d> & c) {
  
  // Copy coordinates to a matrix in Eigen format
  size_t num_points = c.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_points);
  for (size_t i = 0; i < num_points; i++)
    coord.col(i) = c[i];
  
  // calculate centroid
  Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());
  
  // subtract centroid
  for (size_t i = 0; i < 3; i++) 
    coord.row(i).array() -= centroid(i);
  
  // We only need the left-singular matrix here
  // http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
  return std::make_pair(centroid, plane_normal);
}

// A functor which returns the best fit plane a*x + b*y + c*z + d = 0
// as the vector (a, b, c, d) with a*a + b*b + c*c = 1 to be used
// with RANSAC to remove outliers.

struct BestFitPlaneFunctor {

  BestFitPlaneFunctor(bool use_proj_water_surface):
    m_use_proj_water_surface(use_proj_water_surface) {}
  
  typedef vw::Matrix<double, 1, 4> result_type;

  bool m_use_proj_water_surface;
  
  /// A best fit plane requires pairs of data points to make a fit.
  template <class ContainerT>
  size_t min_elements_needed_for_fit(ContainerT const& /*example*/) const { return 3; }
  
  /// This function can match points in any container that supports
  /// the size() and operator[] methods.  The container is usually a
  /// vw::Vector<>, but you could substitute other classes here as
  /// well.
  template <class ContainerT>
  vw::Matrix<double> operator() (std::vector<ContainerT> const& p1,
                                 std::vector<ContainerT> const& p2,
                                 vw::Matrix<double> const& /*seed_input*/
                                 = vw::Matrix<double>() ) const {
    
    // check consistency
    VW_ASSERT( p1.size() == p2.size(),
               vw::ArgumentErr() << "Cannot compute best fit plane. "
               << "p1 and p2 are not the same size." );
    VW_ASSERT( !p1.empty() && p1.size() >= min_elements_needed_for_fit(p1[0]),
               vw::ArgumentErr() << "Cannot compute best fit plane. "
               << "Insufficient data.\n");
    
    std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = best_plane_from_points(p1);
    
    Eigen::Vector3d & centroid = plane.first;
    Eigen::Vector3d & normal = plane.second;
    
    Matrix<double> result(1, 4);
    for (int col = 0; col < 3; col++)
      result(0, col) = normal[col];
    
    result(0, 3) = -normal.dot(centroid);

    if (!m_use_proj_water_surface) {
      // Make the normal always point "up", away from the Earth origin,
      // which means that the free term must be negative.
      if (result(0, 3) > 0) {
        for (int col = 0; col < 4; col++) 
          result(0, col) *= -1.0;
      }
    }else {
      // Make the z coefficient positive, which will make the normal
      // point "up" in the projected coordinate system.
      if (result(0, 2) < 0) {
        for (int col = 0; col < 4; col++) 
          result(0, col) *= -1.0;
      }
    }
    
    return result;
  }
};

// Given a 1x4 matrix named 'plane', with values (a, b, c, d),
// determining the plane a * x + b * y + c * z + d = 0, find the
// distance to this plane from a given point.

template<class Vec3>
double dist_to_plane(vw::Matrix<double, 1, 4> const& plane, Vec3 const& point) {

  double ans = 0.0;
  for (unsigned col = 0; col < 3; col++) {
      ans += plane(0, col) * point[col];
  }
  ans += plane(0, 3);
  
  return std::abs(ans);
}

// The value p2 is needed by the interface but we don't use it
struct BestFitPlaneErrorMetric {
  template <class RelationT, class ContainerT>
  double operator() (RelationT const& plane, ContainerT const& p1, ContainerT const& p2) const {
    return dist_to_plane(plane, p1);
  }
};

void calc_plane_properties(bool use_proj_water_surface,
                           std::vector<Eigen::Vector3d> const& point_vec,
                           std::vector<size_t> const& inlier_indices,
                           vw::cartography::GeoReference & dem_georef,
                           vw::Matrix<double> const& plane) {

  double max_error = - 1.0, max_inlier_error = -1.0;
  for (size_t it = 0; it < point_vec.size(); it++)
    max_error = std::max(max_error, dist_to_plane(plane, point_vec[it]));
  
  // Do estimates for the mean height and angle of the plane
  Vector3 mean_point(0, 0, 0);
  double mean_height = 0.0;
  int num = 0;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    Eigen::Vector3d p = point_vec[inlier_indices[it]];
    Vector3 point(p[0], p[1], p[2]);
    max_inlier_error = std::max(max_inlier_error, dist_to_plane(plane, point));
    
    if (!use_proj_water_surface) {
      // the point is xyz in ecef
      Vector3 llh = dem_georef.datum().cartesian_to_geodetic(point);
      mean_height += llh[2];
    } else {
      // the point is in the stereographic projection
      mean_height += point[2];
    }
    
    num++;
    
    if (!use_proj_water_surface) 
      mean_point += point;
  }
  
  mean_height /= num;
  
  if (!use_proj_water_surface) 
    mean_point /= num;
  
  vw_out() << "Max distance to the plane (meters): " << max_error << std::endl;
  vw_out() << "Max inlier distance to the plane (meters): " << max_inlier_error << std::endl;
  vw_out() << "Mean plane height above datum (meters): " << mean_height << std::endl;
  
  if (!use_proj_water_surface) {
    // This does not make sense for a curved surface
    Vector3 plane_normal(plane(0, 0), plane(0, 1), plane(0, 2));
    Vector3 surface_normal = mean_point / norm_2(mean_point); // ignore the datum flattening
    double plane_angle = (180.0 / M_PI) * acos(dot_prod(plane_normal, surface_normal));
    vw_out() << "Plane inclination (degrees): " << plane_angle << std::endl;
  }
}

void save_plane(bool use_proj_water_surface, double proj_lat, double proj_lon,
                vw::Matrix<double> const& plane, std::string const& plane_file) {

  vw_out() << "Writing: " << plane_file << std::endl;
  vw::create_out_dir(plane_file);
  std::ofstream bp(plane_file.c_str());
  bp.precision(17);
  for (int col = 0; col < plane.cols(); col++) {
    bp << plane(0, col);
    if (col < plane.cols() - 1)
      bp << " ";
    else
      bp << "\n";
  }
  if (use_proj_water_surface) {
    bp << "# Latitude and longitude of the local stereographic projection with the WGS_1984 datum:\n";
    bp << proj_lat << " " << proj_lon << "\n";
  }
  bp.close();
}

// Given a DEM surface and a planar surface (with potentially some
// inclination) subtract from each DEM height the height at that
// plane. The height of the plane is obtained by considering the
// current DEM grid point, and another point at same lon-lat but with
// a height 100 meters less, converting these to the given projection
// (if m_use_proj_water_surface is true or to ECEF otherwise), tracing a ray
// through them, seeing where it intersects the plane, converting that
// point to geodetic, and taking the height difference.
class DemMinusPlaneView: public ImageViewBase<DemMinusPlaneView>{
  ImageViewRef<float> m_dem;
  vw::cartography::GeoReference m_dem_georef;
  vw::Matrix<double> m_plane;
  double m_dem_nodata_val;
  bool m_use_proj_water_surface;
  vw::cartography::GeoReference m_stereographic_georef;

  typedef float PixelT;

public:
  DemMinusPlaneView(ImageViewRef<float> const& dem,
                    vw::cartography::GeoReference const& dem_georef,
                    vw::Matrix<double> const& plane, 
                    double dem_nodata_val,
                    bool use_proj_water_surface,
                    vw::cartography::GeoReference const& stereographic_georef): 
    m_dem(dem), m_dem_georef(dem_georef), m_plane(plane), 
    m_dem_nodata_val(dem_nodata_val),
    m_use_proj_water_surface(use_proj_water_surface),
    m_stereographic_georef(stereographic_georef){}
  
  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<DemMinusPlaneView> pixel_accessor;

  inline int32 cols() const { return m_dem.cols(); }
  inline int32 rows() const { return m_dem.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "DemMinusPlaneView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView< ImageView<pixel_type> > prerasterize_type;
  
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    // Bring this portion in memory
    ImageView<result_type> cropped_dem = crop(m_dem, bbox);
    
    ImageView<result_type> tile(bbox.width(), bbox.height());
    
    for (int col = 0; col < bbox.width(); col++){
      for (int row = 0; row < bbox.height(); row++){
        
        // Handle the case when the DEM is not valid
        if (cropped_dem(col, row) == m_dem_nodata_val) {
          tile(col, row) = m_dem_nodata_val;
          continue;
        }

        Vector2 pix(col + bbox.min().x(), row + bbox.min().y());
        Vector2 lon_lat = m_dem_georef.pixel_to_lonlat(pix);

        Vector3 llh;
        subvector(llh, 0, 2) = lon_lat;
        llh[2] = cropped_dem(col, row);

        // Find the DEM point in local projection or ECEF.
        // Find another point on the same ray.
        Vector3 point1, point2;

        if (m_use_proj_water_surface) {
          point1 = m_stereographic_georef.geodetic_to_point(llh);
          llh[2] -= 100.0;
          point2 = m_stereographic_georef.geodetic_to_point(llh);
        } else {
          point1 = m_dem_georef.datum().geodetic_to_cartesian(llh);
          llh[2] -= 100.0;
          point2 = m_dem_georef.datum().geodetic_to_cartesian(llh);
        }

        // The ray is P = point1 + t * (point2 - point1), where t is real.
        // The plane is a*x + b*y + c*z + d = 0, where plane = (a, b, c, d).
        // Then, dot(P, plane_normal) + plane_intercept = 0.
        // Use that to find t.
        Vector3 plane_normal(m_plane(0, 0), m_plane(0, 1), m_plane(0, 2));
        double plane_intercept = m_plane(0, 3);

        double t = -(dot_prod(point1, plane_normal) + plane_intercept) /
          dot_prod(point2 - point1, plane_normal);

        Vector3 P = point1 + t * (point2 - point1);

        // Go back to llh
        if (m_use_proj_water_surface) {
          llh = m_stereographic_georef.point_to_geodetic(P);
        } else{ 
          llh = m_dem_georef.datum().cartesian_to_geodetic(P);
        }
        
        tile(col, row) = cropped_dem(col, row) - llh[2];
      }
    }
    
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

DemMinusPlaneView dem_minus_plane(ImageViewRef<float> const& dem,
                                  vw::cartography::GeoReference const& dem_georef,
                                  vw::Matrix<double> plane, 
                                  double dem_nodata_val,
                                  bool use_proj_water_surface,
                                  vw::cartography::GeoReference const& stereographic_georef) {
  return DemMinusPlaneView(dem, dem_georef, plane, dem_nodata_val,
                           use_proj_water_surface, stereographic_georef);
}

struct Options : vw::GdalWriteOptions {
  std::string shapefile, dem, mask, camera, stereo_session, bathy_plane,
    water_height_measurements, csv_format_str, output_inlier_shapefile,
    bundle_adjust_prefix, output_outlier_shapefile, mask_boundary_shapefile,
    dem_minus_plane;
  
  double outlier_threshold;
  int num_ransac_iterations, num_samples;
  bool save_shapefiles_as_polygons, use_ecef_water_surface;
  Options(): outlier_threshold(0.5), num_ransac_iterations(1000) {}
};

// Given a set of polygons stored in dPoly, create a single polygon with all those vertices
void formSinglePoly(vw::geometry::dPoly const& inPoly,
                    vw::geometry::dPoly & outPoly) {

  int num_pts = inPoly.get_totalNumVerts();
  const double * xv = inPoly.get_xv();
  const double * yv = inPoly.get_yv();
  
  bool isPolyClosed = true;
  std::string layer = "";
  std::string poly_color = "green";
  outPoly.setPolygon(num_pts, xv, yv, isPolyClosed, poly_color, layer);
}

void handle_arguments(int argc, char *argv[], Options& opt) {
  
  po::options_description general_options("General Options");
  general_options.add_options()
    ("shapefile",   po::value(&opt.shapefile),
     "The shapefile with vertices whose coordinates will be looked up in the DEM.")
    ("dem",   po::value(&opt.dem),
     "The DEM to use.")
    ("mask",   po::value(&opt.mask),
     "A input mask, created from a raw camera image and hence having the same dimensions, "
     "with values of 1 on land and 0 on water, or positive values on land and no-data "
     "values on water.")
    ("camera",   po::value(&opt.camera),
     "The camera file to use with the mask.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment at this output prefix, if the cameras changed "
     "based on bundle adjustment or alignment.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select "
     "this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("bathy-plane",   po::value(&opt.bathy_plane),
     "The output file storing the computed plane as four coefficients a, b, c, d, "
     "with the plane being a*x + b*y + c*z + d = 0.")
    ("outlier-threshold",
     po::value(&opt.outlier_threshold)->default_value(0.5),
     "A value, in meters, to determine the distance from a sampled point on the DEM to the "
     "best-fit plane to determine if it will be marked as outlier and not "
     "included in the calculation of that plane. Its value should be roughly the expected "
     "vertical uncertainty of the DEM.")
    ("num-ransac-iterations", 
     po::value(&opt.num_ransac_iterations)->default_value(1000),
     "Number of RANSAC iterations to use to find the best-fitting plane.")
    ("output-inlier-shapefile", po::value(&opt.output_inlier_shapefile)->default_value(""),
     "If specified, save at this location the shape file with the inlier vertices.")
    ("output-outlier-shapefile", po::value(&opt.output_outlier_shapefile)->default_value(""),
     "If specified, save at this location the shape file with the outlier vertices.")
    ("mask-boundary-shapefile", po::value(&opt.mask_boundary_shapefile)->default_value(""),
     "If specified together with a mask, camera, and DEM, save a random "
     "sample of points (their number given by ``--num-samples``) at the "
     "mask boundary (water-land interface) to this shapefile and exit.")
    ("num-samples", 
     po::value(&opt.num_samples)->default_value(10000),
     "Number of samples to pick at the water-land interface if using a mask.")
    ("save-shapefiles-as-polygons", 
     po::bool_switch(&opt.save_shapefiles_as_polygons)->default_value(false),
     "Save the inlier and outlier shapefiles as polygons, rather than "
     "discrete vertices. May be more convenient for processing in a GIS tool.")
    ("water-height-measurements", po::value(&opt.water_height_measurements)->default_value(""),
     "Use this CSV file having longitude, latitude, and height measurements "
     "for the water surface, in degrees and meters, respectively, relative to the WGS84 datum. "
     "The option --csv-format must be used.")
     ("csv-format", po::value(&opt.csv_format_str)->default_value(""),
      "Specify the format of the CSV file having water height measurements. "
      "The format should have a list of entries with syntax column_index:column_type "
      "(indices start from 1). Example: '2:lon 3:lat 4:height_above_datum'.")
    ("dem-minus-plane",
     po::value(&opt.dem_minus_plane),
     "If specified, subtract from the input DEM the best-fit plane and save the "
     "obtained DEM to this GeoTiff file.")
    ("use-ecef-water-surface",
     po::bool_switch(&opt.use_ecef_water_surface)->default_value(false),
     "Compute the best fit plane in ECEF coordinates rather than in a local stereographic "
     "projection. Hence don't model the Earth curvature. Not recommended.");
  
  general_options.add( vw::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  //positional.add_options()
  //   ("input-files", po::value< std::vector<std::string> >(), "Input files");

  po::positional_options_description positional_desc;
  //positional_desc.add("input-files", -1);

  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // Need this to be able to load adjusted camera models. This must be set
  // before loading the cameras. 
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;
  
  bool use_shapefile = !opt.shapefile.empty();
  bool use_mask      = !opt.mask.empty();
  bool use_meas      = !opt.water_height_measurements.empty();

  if (use_mask && opt.camera.empty()) 
    vw_throw(ArgumentErr() << "If using a mask, must specify a camera.\n"
             << usage << general_options);
    
  if (use_shapefile + use_mask + use_meas != 1)
    vw_throw(ArgumentErr() << "Must use either a mask and camera, a shapefile, or water "
              << "height measurements, and just one of these.\n"
              << usage << general_options);

  if (!use_meas && opt.dem == "")
    vw_throw(ArgumentErr() << "Missing the input dem.\n" << usage << general_options );

  if (opt.mask_boundary_shapefile.empty() && opt.bathy_plane == "")
    vw_throw(ArgumentErr() << "Missing the output bathy plane file.\n"
             << usage << general_options );
  
  if (use_meas && opt.csv_format_str == "") 
    vw_throw(ArgumentErr() << "Must set the option --csv-format.\n"
             << usage << general_options );
  
  if (opt.use_ecef_water_surface && use_meas) {
    vw_throw(ArgumentErr() << "Cannot use use-ecef-water-surface with "
             << "--water-height-measurements.\n"
             << usage << general_options);
    
    if (!opt.mask_boundary_shapefile.empty() && 
        (opt.mask.empty() || opt.camera.empty() || opt.dem.empty()))
      vw_throw(ArgumentErr() << "If using --mask-boundary-shapefile, then "
               << "must specify a mask, a camera, and a DEM.\n"
               << usage << general_options);

    if (opt.num_samples <= 0) 
      vw_throw(ArgumentErr() << "A positive number of samples must be specified.\n"
               << usage << general_options);
  }
}

typedef boost::shared_ptr<asp::StereoSession> SessionPtr;

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Load the camera if we use the mask and the camera
    bool use_shapefile = !opt.shapefile.empty();
    bool use_mask      = !opt.mask.empty();
    bool use_meas      = !opt.water_height_measurements.empty();

    boost::shared_ptr<CameraModel> camera_model;
    if (use_mask) {
      std::string out_prefix;
      SessionPtr session(asp::StereoSessionFactory::create(opt.stereo_session, // may change
                                                           opt,
                                                           opt.mask, opt.mask,
                                                           opt.camera, opt.camera,
                                                           out_prefix));
      camera_model = session->camera_model(opt.mask, opt.camera);
    }

    // Only WGS84 is supported. Note that dem_georef and shape_georef
    // may or may not be overwritten below depending on the case.
    vw::cartography::GeoReference dem_georef;
    vw::cartography::Datum datum("WGS_1984");
    dem_georef.set_datum(datum);
    dem_georef.set_geographic();
    bool has_shape_georef = true;
    vw::cartography::GeoReference shape_georef = dem_georef;

    float dem_nodata_val = -std::numeric_limits<float>::max();
    ImageViewRef<float> dem;
    ImageViewRef<PixelMask<float>> masked_dem;
    ImageViewRef< PixelMask<float>> interp_dem;

    if (use_shapefile || use_mask) {
      // Read the DEM and its associated data
      // TODO(oalexan1): Think more about the interpolation method
      vw_out() << "Reading the DEM: " << opt.dem << std::endl;
      if (!read_georeference(dem_georef, opt.dem))
        vw_throw(ArgumentErr() << "The input DEM has no georeference.\n" );
      
      // We assume the WGS_1984 datum
      if (dem_georef.datum().name() != "WGS_1984")
        vw_throw(ArgumentErr() << "Only an input DEM with the "
                  << "WGS_1984 datum is supported.\n"
                  << "Got: " << dem_georef.datum().name() << ".\n");
      
      // Note we use a float nodata
      if (!vw::read_nodata_val(opt.dem, dem_nodata_val))
        vw_out() << "Warning: Could not read the DEM nodata value. "
                 << "Using: " << dem_nodata_val << ".\n";
      else
        vw_out() << "Read DEM nodata value: " << dem_nodata_val << ".\n";
      
      // Read the DEM
      dem = DiskImageView<float>(opt.dem);
      masked_dem = create_mask(dem, dem_nodata_val);
      interp_dem = interpolate(masked_dem, BilinearInterpolation(), ConstantEdgeExtension());
    }
    
    bool use_proj_water_surface = !opt.use_ecef_water_surface;
    std::vector<Eigen::Vector3d> point_vec;
    std::vector<vw::Vector3> llh_vec;
    double proj_lat = -1.0, proj_lon = -1.0; // only for curved water surface
    std::vector<vw::Vector2> used_vertices;
    vw::cartography::GeoReference stereographic_georef;
    std::string poly_color = "green";

    if (use_mask) {
      // Read the mask. The no-data value is the largest of what
      // is read from the mask file and the value 0, as pixels
      // over land are supposed to be positive and be valid data.
      vw_out() << "Reading the mask: " << opt.mask << std::endl;
      float mask_nodata_val = -std::numeric_limits<float>::max();
      if (vw::read_nodata_val(opt.mask, mask_nodata_val))
        vw_out() << "Read mask nodata value: " << mask_nodata_val << ".\n";
      mask_nodata_val = std::max(0.0f, mask_nodata_val);
      vw_out() << "Pixels with values no more than " << mask_nodata_val
               << " are classified as water.\n"; 
      DiskImageView<float> mask(opt.mask);
      
      shape_georef = dem_georef;
      find_points_at_mask_boundary(mask, mask_nodata_val,  
                                   camera_model, shape_georef,  
                                   dem_georef, masked_dem,
                                   opt.num_samples,
                                   point_vec, llh_vec,  
                                   used_vertices);

      if (!opt.mask_boundary_shapefile.empty()) {
        // Just sample the mask boundary and return
        sample_mask_boundary(point_vec, opt.mask_boundary_shapefile);
        return 0;
      }
      
    } else if (use_shapefile) { 
    
      // Read the shapefile, overwriting shape_georef
      vw_out() << "Reading the shapefile: " << opt.shapefile << std::endl;
      std::vector<vw::geometry::dPoly> polyVec;
      read_shapefile(opt.shapefile, poly_color, has_shape_georef, shape_georef, polyVec);
      
      if (!has_shape_georef) 
        vw_throw(ArgumentErr() << "The input shapefile has no georeference.\n" );
      
      // Find the ECEF coordinates of the shape corners
      find_points_at_shape_corners(polyVec, shape_georef, dem_georef, interp_dem, point_vec,
                                   llh_vec, used_vertices);
    } else if (use_meas) {
      find_points_from_meas_csv(opt.water_height_measurements, opt.csv_format_str, 
                               shape_georef,  
                               // Outputs
                               llh_vec, used_vertices);
    }
    
    // See if to convert to local stereographic projection
    if (use_proj_water_surface)
      find_projection(// Inputs
                      dem_georef, llh_vec,
                      // Outputs
                      proj_lat, proj_lon,  
                      stereographic_georef,  
                      point_vec);

    // Compute the water surface using RANSAC
    std::vector<Eigen::Vector3d> dummy_vec(point_vec.size()); // Required by the interface
    std::vector<size_t> inlier_indices;
    double inlier_threshold = opt.outlier_threshold;
    int    min_num_output_inliers = std::max(point_vec.size()/2, size_t(3));
    bool   reduce_min_num_output_inliers_if_no_fit = true;
    vw::Matrix<double> plane;
    vw_out() << "Starting RANSAC.\n";
    try {
      // Must first create the functor and metric, then pass these to ransac. If
      // created as inline arguments to ransac, these may go go out
      // of scope prematurely, which will result in incorrect behavior.
      BestFitPlaneFunctor func(use_proj_water_surface);
      BestFitPlaneErrorMetric error_metric;
      math::RandomSampleConsensus<BestFitPlaneFunctor, BestFitPlaneErrorMetric> 
        ransac(func, error_metric,
               opt.num_ransac_iterations, inlier_threshold,
               min_num_output_inliers, reduce_min_num_output_inliers_if_no_fit);
    
      plane = ransac(point_vec, dummy_vec);
      
      inlier_indices = ransac.inlier_indices(plane, point_vec, dummy_vec);
    } catch (const vw::math::RANSACErr& e ) {
      vw_out() << "RANSAC failed: " << e.what() << "\n";
    }
    vw_out() << "Found " << inlier_indices.size() << " / " << point_vec.size() << " inliers.\n";
    
    calc_plane_properties(use_proj_water_surface, point_vec, inlier_indices,  
                           dem_georef, plane);

    save_plane(use_proj_water_surface, proj_lat, proj_lon,  
               plane, opt.bathy_plane);

    // Save the shape having the inliers.
    if (opt.output_inlier_shapefile != "") {
      vw::geometry::dPoly inlierPoly;
      for (size_t inlier_it = 0; inlier_it < inlier_indices.size(); inlier_it++) 
        addPointToPoly(inlierPoly, used_vertices[inlier_indices[inlier_it]]);
      
      if (opt.save_shapefiles_as_polygons) {
        vw::geometry::dPoly localPoly; 
        formSinglePoly(inlierPoly, localPoly);
        inlierPoly = localPoly;
      }

      std::vector<vw::geometry::dPoly> inlierPolyVec;
      inlierPolyVec.push_back(inlierPoly);
      vw_out() << "Writing inlier shapefile: " << opt.output_inlier_shapefile << "\n";
      write_shapefile(opt.output_inlier_shapefile, has_shape_georef, shape_georef,
                      inlierPolyVec);
    }

    // Save the shape having the outliers.
    if (opt.output_outlier_shapefile != "") {

      // First put the inliers in a set so we can exclude them
      std::set<int> inlier_set;
      for (size_t inlier_it = 0; inlier_it < inlier_indices.size(); inlier_it++)
        inlier_set.insert(inlier_indices[inlier_it]);
      
      vw::geometry::dPoly outlierPoly;
      for (size_t it = 0; it < used_vertices.size(); it++) {
        
        if (inlier_set.find(it) != inlier_set.end())
          continue; // an inlier, skip it

        addPointToPoly(outlierPoly, used_vertices[it]);
      }

      if (opt.save_shapefiles_as_polygons) {
        vw::geometry::dPoly localPoly; 
        formSinglePoly(outlierPoly, localPoly);
        outlierPoly = localPoly;
      }
      
      std::vector<vw::geometry::dPoly> outlierPolyVec;
      outlierPolyVec.push_back(outlierPoly);
      vw_out() << "Writing outlier shapefile: " << opt.output_outlier_shapefile << "\n";
      write_shapefile(opt.output_outlier_shapefile, has_shape_georef, shape_georef,
                      outlierPolyVec);
    }

    if (opt.dem_minus_plane != "") {
      bool has_nodata = true, has_georef = true;
      vw_out() << "Writing: " << opt.dem_minus_plane << std::endl;
      TerminalProgressCallback tpc("asp", ": ");
      block_write_gdal_image(opt.dem_minus_plane,
                             dem_minus_plane(dem, dem_georef, plane,
                                             dem_nodata_val, use_proj_water_surface,
                                             stereographic_georef),
                             has_georef, dem_georef, has_nodata, dem_nodata_val, opt, tpc);
    }

  } ASP_STANDARD_CATCHES;
  
  return 0;
}

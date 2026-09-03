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

#include <asp/Core/BathyPlaneCalc.h>
#include <asp/Core/PointUtils.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/BathyData.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Math/RANSAC.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Core/ThreadPool.h>
#include <vw/Math/RandomSet.h>
#include <vw/FileIO/FileUtils.h>

#include <Eigen/Dense>

#include <random>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>
#include <stack>

namespace asp {

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

// Add a polygon made up of just one point to given set of polygons.
// Later that set will be saved as a shapefile made up of points.
void addPointToPoly(vw::geometry::dPoly & poly, vw::Vector2 const& p) {
  std::vector<double> vx, vy;
  vx.push_back(p.x());
  vy.push_back(p.y());
  bool isPolyClosed = true;
  std::string poly_color = "green";
  std::string layer = "";
  poly.appendPolygon(vx.size(), vw::geometry::vecPtr(vx), vw::geometry::vecPtr(vy),
                     isPolyClosed, poly_color, layer);
}

// Add a point to the ecef_vec, llh_vec, and shape_xy_vec if it is valid
void addPoint(vw::cartography::GeoReference const& dem_georef,
              vw::ImageViewRef<vw::PixelMask<float>> const& interp_dem,
              vw::Vector2 const& lonlat,
              vw::Vector2 const& shape_xy,
              // Append
              std::vector<Eigen::Vector3d> & ecef_vec,
              std::vector<vw::Vector3> & llh_vec,
              std::vector<vw::Vector2> & shape_xy_vec) {

  // Convert to DEM pixel
  vw::Vector2 pix = dem_georef.lonlat_to_pixel(lonlat);
  if (!vw::bounding_box(interp_dem).contains(pix))
    return;

  vw::PixelMask<float> h = interp_dem(pix.x(), pix.y());

  if (!is_valid(h))
    return;

  vw::Vector3 llh;
  llh[0] = lonlat[0];
  llh[1] = lonlat[1];
  llh[2] = h.child();

  vw::Vector3 ecef = dem_georef.datum().geodetic_to_cartesian(llh);
  Eigen::Vector3d eigen_ecef;
  for (size_t coord = 0; coord < 3; coord++)
    eigen_ecef[coord] = ecef[coord];

  ecef_vec.push_back(eigen_ecef);
  shape_xy_vec.push_back(shape_xy);
  llh_vec.push_back(llh);
}

// Estimate the projection and convert llh_vec to projected coordinates
void find_projection(// Inputs
                     vw::cartography::GeoReference const& dem_georef,
                     std::vector<vw::Vector3> const& llh_vec,
                     // Outputs
                     double & proj_lat, double & proj_lon,
                     vw::cartography::GeoReference & stereographic_georef,
                     std::vector<Eigen::Vector3d> & proj_vec) {

  // Must have positive number of points
  if (llh_vec.size() == 0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot find projection with zero input points.\n");

  // Initialize the outputs
  proj_lat = -1.0;
  proj_lon = -1.0;
  proj_vec.clear();

  // Find the mean water height
  vw::Vector3 mean_llh;
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
    vw::Vector3 point = stereographic_georef.geodetic_to_point(llh_vec[it]);

    Eigen::Vector3d eigen_point;
    for (size_t coord = 0; coord < 3; coord++)
      eigen_point[coord] = point[coord];

    proj_vec.push_back(eigen_point);
  }
}

// A function that will find the mean of a std::vector<vw::Vector2> mid_pts
vw::Vector2 meanOfPoints(std::vector<vw::Vector2> const& points) {

  // Check that must have at least one input
  if (points.size() == 0)
    vw::vw_throw(vw::ArgumentErr() << "Cannot find mean of zero input points.\n");
    
  vw::Vector2 mean_point(0.0, 0.0);
  for (size_t it = 0; it < points.size(); it++)
    mean_point += points[it];
    
  mean_point /= points.size();
  return mean_point;
}

// Attempts to find the sub-pixel boundary interface for a specific pixel.
// Returns true if a boundary point was calculated, false otherwise.
bool calcSubpixBdPoint(vw::ImageViewRef<float> const& mask, 
                       float mask_thresh, int col, int row, 
                       vw::Vector2 & pix) {

  std::vector<vw::Vector2> mid_pts;
  
  // The four neighbors
  const int col_vals[4] = {-1, 0, 0, 1};
  const int row_vals[4] = {0, -1, 1, 0};

  for (int it = 0; it < 4; it++) {
    int icol = col + col_vals[it];
    int irow = row + row_vals[it];

    // Check image bounds
    if (icol < 0 || irow < 0 || icol >= mask.cols() || irow >= mask.rows())
      continue;

    // Check if neighbor is on the other side of threshold
    if (mask(icol, irow) <= mask_thresh)
      mid_pts.push_back(vw::Vector2(0.5 * (col + icol), 0.5 * (row + irow)));
  }

  if (mid_pts.empty())
    return false;

  pix = meanOfPoints(mid_pts);
  return true;
}

// This is not used for now, it could be used to make the logic
// of processing mask boundary points multi-threaded.
class MaskBoundaryTask: public vw::Task, private boost::noncopyable {
  vw::BBox2i m_bbox; // Region of image we're working in

  vw::ImageViewRef<float> m_mask;
  float m_mask_nodata_val;
  vw::CamPtr m_camera_model;
  vw::cartography::GeoReference m_shape_georef;
  vw::cartography::GeoReference m_dem_georef;
  vw::ImageViewRef<vw::PixelMask<float>> m_masked_dem;

  // Note how all of these are aliases
  vw::Mutex                    & m_mutex;
  std::vector<Eigen::Vector3d> & m_ecef_vec;
  std::vector<vw::Vector3>     & m_llh_vec;
  std::vector<vw::Vector2>     & m_shape_xy_vec;

public:
  MaskBoundaryTask(vw::BBox2i bbox,
                   vw::ImageViewRef<float> mask,
                   float mask_nodata_val,
                   vw::CamPtr camera_model,
                   vw::cartography::GeoReference   const & shape_georef,
                   vw::cartography::GeoReference   const & dem_georef,
                   vw::ImageViewRef<vw::PixelMask<float>>  masked_dem,
                   vw::Mutex                             & mutex,
                   std::vector<Eigen::Vector3d>          & ecef_vec,
                   std::vector<vw::Vector3>              & llh_vec,
                   std::vector<vw::Vector2>              & shape_xy_vec):
    m_bbox(bbox), m_mask(mask), m_mask_nodata_val(mask_nodata_val),
    m_camera_model(camera_model), m_shape_georef(shape_georef),
    m_dem_georef(dem_georef), m_masked_dem(masked_dem),
    m_mutex(mutex), m_ecef_vec(ecef_vec),
    m_llh_vec(llh_vec), m_shape_xy_vec(shape_xy_vec) {}

  void operator()() {

    // Grow the box by 1 pixel as we need to look at the immediate neighbors
    vw::BBox2i extra_box = m_bbox;
    extra_box.expand(1);
    extra_box.crop(vw::bounding_box(m_mask));

    // Make a local copy of the tile
    vw::ImageView<float> mask_tile = vw::crop(m_mask, extra_box);

    for (int col = 0; col < mask_tile.cols(); col++) {
      for (int row = 0; row < mask_tile.rows(); row++) {

        // Look at pixels above threshold which have neighbors <= threshold
        if (mask_tile(col, row) <= m_mask_nodata_val)
          continue;
        
        // Estimate the sub-pixel boundary point
        vw::Vector2 local_pix;  
        if (!calcSubpixBdPoint(mask_tile, m_mask_nodata_val, col, row, local_pix))
          continue;

        // Create the pixel in the full image coordinates
        vw::Vector2 pix = local_pix + extra_box.min();

        // Only work on pixels in the current box (earlier had a
        // bigger box to be able to examine neighbors).
        if (!m_bbox.contains(pix))
          continue;

        // The ray going to the ground
        // Here we assume that the camera model is thread-safe, which is
        // true for all cameras except ISIS, and this code will be used
        // on Earth only.
        vw::Vector3 cam_ctr = m_camera_model->camera_center(pix);
        vw::Vector3 cam_dir = m_camera_model->pixel_to_vector(pix);

        // Intersect the ray going from the given camera pixel with a DEM.
        bool treat_nodata_as_zero = false;
        bool has_intersection = false;
        double height_error_tol = 0.001; // in meters
        vw::Vector3 xyz_guess(0, 0, 0);
        vw::Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
          (cam_ctr, cam_dir, m_masked_dem,
           m_dem_georef, treat_nodata_as_zero,
           has_intersection, height_error_tol, xyz_guess);

        if (!has_intersection)
          continue;

        vw::Vector3 llh = m_dem_georef.datum().cartesian_to_geodetic(xyz);

        Eigen::Vector3d eigen_xyz;
        for (size_t coord = 0; coord < 3; coord++)
          eigen_xyz[coord] = xyz[coord];

        // TODO(oalexan1): This is fragile due to the 360 degree
        // uncertainty in latitude
        vw::Vector2 shape_xy = m_shape_georef.lonlat_to_point(vw::Vector2(llh[0], llh[1]));

        {
          // Need to make sure to lock the shared resource
          vw::Mutex::Lock lock(m_mutex);
          m_ecef_vec.push_back(eigen_xyz);
          m_shape_xy_vec.push_back(shape_xy);
          m_llh_vec.push_back(llh);
        }
      }

    }

  }

};

// Sample the mask boundary (points where the points in the mask have neighbors
// not in the mask), shoot points from there onto the DEM, and return the
// obtained points.
void sampleMaskBd(vw::ImageViewRef<float> mask,
                  float mask_nodata_val,
                  vw::CamPtr camera_model,
                  vw::cartography::GeoReference const& shape_georef,
                  vw::cartography::GeoReference const& dem_georef,
                  vw::ImageViewRef<vw::PixelMask<float>> masked_dem,
                  int num_samples,
                  std::vector<Eigen::Vector3d> & ecef_vec,
                  std::vector<vw::Vector3> & llh_vec,
                  std::vector<vw::Vector2> & shape_xy_vec) {

  // num_samples must be positive
  if (num_samples <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The value of --num-samples must be positive.\n");

  // We assume the WGS_1984 datum in the shape. For the DEM we checked already.
  if (shape_georef.datum().name() != "WGS_1984")
    vw::vw_throw(vw::ArgumentErr() << "The input shape must be relative to the "
                 << "WGS_1984 datum.\n" << "Got: " << dem_georef.datum().name() << ".\n");

  // Ensure that the outputs are initialized
  ecef_vec.clear();
  llh_vec.clear();
  shape_xy_vec.clear();

#if 0

  // For some reason parallel processing is slower than serial processing.
  // Likely it is because the DEM is shared. Need extra heuristics to cut
  // that one as appropriate for each task.
  
  // TODO(oalexan1): Must review now that the lock issue got fixed

  // Subdivide the box for parallel processing
  int block_size = vw::vw_settings().default_tile_size();
  FifoWorkQueue queue(vw_settings().default_num_threads());
  std::vector<BBox2i> bboxes = subdivide_bbox(mask, block_size, block_size);

  for (size_t it = 0; it < bboxes.size(); it++) {
    std::cout << "Box is " << bboxes[it] << "\n";
  }

  Mutex mutex;
  for (size_t it = 0; it < bboxes.size(); it++) {
    boost::shared_ptr<MaskBoundaryTask>
      task(new MaskBoundaryTask(bboxes[it],  mask, mask_nodata_val, camera_model,
                                shape_georef, dem_georef, masked_dem, mutex,
                                ecef_vec, llh_vec, shape_xy_vec));
    queue.add_task(task);
  }

  queue.join_all();

#else

  // Let the mask boundary be the mask pixels whose value is above threshold and
  // which border pixels whose values is not above threshold.

  vw::vw_out() << "Processing points at mask boundary.\n";
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / mask.cols();
  tpc.report_progress(0);

  for (int col = 0; col < mask.cols(); col++) {
    for (int row = 0; row < mask.rows(); row++) {

      // Look at pixels above threshold which have neighbors <= threshold
      if (mask(col, row) <= mask_nodata_val)
        continue;

      // Estimate the location of the interface  
      vw::Vector2 pix;
      if (!calcSubpixBdPoint(mask, mask_nodata_val, col, row, pix))
        continue;
        
      // The ray going to the ground
      vw::Vector3 cam_ctr = camera_model->camera_center(pix);
      vw::Vector3 cam_dir = camera_model->pixel_to_vector(pix);

      // Intersect the ray going from the given camera pixel with a DEM.
      bool treat_nodata_as_zero = false;
      bool has_intersection = false;
      double height_error_tol = 0.001; // in meters
      vw::Vector3 xyz_guess(0, 0, 0);
      vw::Vector3 xyz = vw::cartography::camera_pixel_to_dem_xyz
        (cam_ctr, cam_dir, masked_dem,
         dem_georef, treat_nodata_as_zero,
         has_intersection, height_error_tol, xyz_guess);

      if (!has_intersection)
        continue;

      vw::Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);

      Eigen::Vector3d eigen_xyz;
      for (size_t coord = 0; coord < 3; coord++)
        eigen_xyz[coord] = xyz[coord];

      vw::Vector2 shape_xy = shape_georef.lonlat_to_point(vw::Vector2(llh[0], llh[1]));

      ecef_vec.push_back(eigen_xyz);
      shape_xy_vec.push_back(shape_xy);
      llh_vec.push_back(llh);
    }

    tpc.report_incremental_progress(inc_amount);
  }

  tpc.report_finished();

#endif

  int num_pts = ecef_vec.size();
  if (num_pts > num_samples) {
    vw::vw_out() << "Found " << num_pts << " samples at mask boundary, points but only "
             << num_samples << " samples are desired. Picking a random subset "
             << "of this size.\n";

    // Select a subset
    std::vector<int> w;
    vw::math::pick_random_indices_in_range(num_pts, num_samples, w);
    std::vector<Eigen::Vector3d> big_ecef_vec     = ecef_vec;     ecef_vec.clear();
    std::vector<vw::Vector3>     big_llh_vec       = llh_vec;       llh_vec.clear();
    std::vector<vw::Vector2>     big_shape_xy_vec = shape_xy_vec; shape_xy_vec.clear();
    for (size_t it = 0; it < w.size(); it++) {
      int random_index = w[it];
      ecef_vec.push_back(big_ecef_vec[random_index]);
      llh_vec.push_back(big_llh_vec[random_index]);
      shape_xy_vec.push_back(big_shape_xy_vec[random_index]);
    }
  }

  return;
}

// Find the mask boundary (points where the points in the mask have
// neighbors not in the mask), and look up the height in the DEM.
void sampleOrthoMaskBd(std::string const& mask_file,
                       vw::cartography::GeoReference const& mask_georef,
                       vw::cartography::GeoReference const& dem_georef,
                       vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                       int num_samples,
                       std::vector<Eigen::Vector3d> & ecef_vec,
                       std::vector<vw::Vector3> & llh_vec,
                       std::vector<vw::Vector2> & shape_xy_vec) {

  // We assume the WGS_1984 datum in the mask. For the DEM we checked already.
  if (mask_georef.datum().name() != "WGS_1984")
    vw::vw_throw(vw::ArgumentErr() << "The input mask must be relative to the "
                 << "WGS_1984 datum.\n" << "Got: " << dem_georef.datum().name() << ".\n");

  // Use read_bathy_mask() which invalidates both nodata and non-positive
  // pixels. Then apply_mask() converts to a plain float image with 0 for
  // water, so the existing threshold logic works with threshold = 0.
  vw::vw_out() << "Reading the ortho mask: " << mask_file << "\n";
  float mask_thresh = 0.0f;
  {
    float file_nodata = -std::numeric_limits<float>::max();
    if (vw::read_nodata_val(mask_file, file_nodata))
      vw::vw_out() << "Read ortho mask nodata value: " << file_nodata << ".\n";
  }
  auto masked_mask = vw::read_bathy_mask(mask_file, mask_thresh);
  auto mask = vw::apply_mask(masked_mask, 0.0f);
  mask_thresh = 0.0f;
  vw::vw_out() << "Non-positive or nodata pixels are classified as water.\n";

  // num_samples must be positive
  if (num_samples <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The value of --num-samples must be positive.\n");

  // Ensure that the outputs are initialized
  ecef_vec.clear();
  llh_vec.clear();
  shape_xy_vec.clear();

  vw::vw_out() << "Processing points at ortho mask boundary.\n";
  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / mask.cols();
  tpc.report_progress(0);

  for (int col = 0; col < mask.cols(); col++) {
    for (int row = 0; row < mask.rows(); row++) {

      // Look at pixels above threshold which have neighbors <= threshold
      if (mask(col, row) <= mask_thresh)
        continue;
      
      // Estimate the location of the interface  
      vw::Vector2 pix;
      if (!calcSubpixBdPoint(mask, mask_thresh, col, row, pix))
        continue;
      
      vw::Vector2 shape_xy = mask_georef.pixel_to_point(pix);
      vw::Vector2 lonlat = mask_georef.point_to_lonlat(shape_xy);

      addPoint(dem_georef, interp_dem, lonlat, shape_xy,
               ecef_vec, llh_vec, shape_xy_vec);
    }

    tpc.report_incremental_progress(inc_amount);
  }

  tpc.report_finished();

  int num_pts = ecef_vec.size();
  if (num_pts > num_samples) {
    vw::vw_out() << "Found " << num_pts << " samples at mask boundary, points but only "
             << num_samples << " samples are desired. Picking a random subset "
             << "of this size.\n";

    // Select a subset
    std::vector<int> w;
    vw::math::pick_random_indices_in_range(num_pts, num_samples, w);
    std::vector<Eigen::Vector3d> big_ecef_vec     = ecef_vec;     ecef_vec.clear();
    std::vector<vw::Vector3>     big_llh_vec      = llh_vec;      llh_vec.clear();
    std::vector<vw::Vector2>     big_shape_xy_vec = shape_xy_vec; shape_xy_vec.clear();
    for (size_t it = 0; it < w.size(); it++) {
      int random_index = w[it];
      ecef_vec.push_back(big_ecef_vec[random_index]);
      llh_vec.push_back(big_llh_vec[random_index]);
      shape_xy_vec.push_back(big_shape_xy_vec[random_index]);
    }
  }

  return;
}

// Compute the 3D locations at the shape corners based on interpolating
// into the DEM and converting to ECEF and to local projected
// stereographic coordinates.
// Compute the stereographic georeference with the projection center 
// being the mean lon and lat, and make the 3D locations in reference 
// to this projection.
void find_points_at_shape_corners(std::vector<vw::geometry::dPoly> const& polyVec,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                                  std::vector<Eigen::Vector3d> & ecef_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & shape_xy_vec) {

  // Ensure that the outputs are initialized
  ecef_vec.clear();
  llh_vec.clear();
  shape_xy_vec.clear();

  int total_num_pts = 0;

  for (size_t p = 0; p < polyVec.size(); p++) {
    vw::geometry::dPoly const& poly = polyVec[p];

    const double * xv       = poly.get_xv();
    const double * yv       = poly.get_yv();
    const int    * numVerts = poly.get_numVerts();
    int numPolys            = poly.get_numPolys();

    int start = 0;
    for (int pIter = 0; pIter < numPolys; pIter++) {

      if (pIter > 0) start += numVerts[pIter - 1];

      int numV = numVerts[pIter];
      for (int vIter = 0; vIter < numV; vIter++) {

        total_num_pts++;

        vw::Vector2 shape_xy(xv[start + vIter], yv[start + vIter]);

        // Convert from projected coordinates to lonlat
        vw::Vector2 lonlat = shape_georef.point_to_lonlat(shape_xy);

        addPoint(dem_georef, interp_dem, lonlat, shape_xy,
                 ecef_vec, llh_vec, shape_xy_vec);
      }
    }
  }

  vw::vw_out() << "Read " << total_num_pts << " vertices, with " << llh_vec.size()
            << " of them having a valid DEM height value."  << "\n";
}

// Read a set of measurements in CSV format, to use later to fit the water surface
void find_points_from_meas_csv(std::string const& water_height_measurements,
                               std::string const& csv_format_str,
                               vw::cartography::GeoReference const& shape_georef,
                               // Outputs
                               std::vector<Eigen::Vector3d> & ecef_vec,
                               std::vector<vw::Vector3> & llh_vec,
                               std::vector<vw::Vector2> & shape_xy_vec) {
  // Wipe the outputs
  ecef_vec.clear();
  llh_vec.clear();
  shape_xy_vec.clear();

  // Read the CSV file
  asp::CsvConv csv_conv;
  std::string csv_srs = ""; // not needed
  try {
    csv_conv.parse_csv_format(csv_format_str, csv_srs);
  } catch (...) {
    // Give a more specific error message
    vw::vw_throw(vw::ArgumentErr() << "Could not parse --csv-format. Was given: "
             << csv_format_str << ".\n");
  }
  std::list<asp::CsvConv::CsvRecord> pos_records;
  csv_conv.read_csv_file(water_height_measurements, pos_records);

  // Create llh and vertices
  for (auto iter = pos_records.begin(); iter != pos_records.end(); iter++) {
    vw::Vector3 llh = csv_conv.csv_to_geodetic(*iter, shape_georef);
    llh_vec.push_back(llh);
    vw::Vector2 shape_xy = shape_georef.lonlat_to_point(vw::Vector2(llh[0], llh[1]));
    shape_xy_vec.push_back(shape_xy);

    vw::Vector3 ecef = shape_georef.datum().geodetic_to_cartesian(llh);
    Eigen::Vector3d eigen_ecef;
    for (size_t coord = 0; coord < 3; coord++)
      eigen_ecef[coord] = ecef[coord];
    ecef_vec.push_back(eigen_ecef);
  }

  return;
}

// Read a set of lon-lat measurements in CSV format, then interpolate into the DEM
// with bilinear interpolation to find the height.
void find_points_from_lon_lat_csv(std::string const& lon_lat_measurements,
                                  std::string const& csv_format_str,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                                  // Outputs
                                  std::vector<Eigen::Vector3d> & ecef_vec,
                                  std::vector<vw::Vector3> & llh_vec,
                                  std::vector<vw::Vector2> & shape_xy_vec) {

  // Wipe the outputs
  ecef_vec.clear();
  llh_vec.clear();
  shape_xy_vec.clear();

  // Read the CSV file
  asp::CsvConv csv_conv;
  std::string csv_srs = ""; // not needed
  try {
    int min_num_fields = 2; // only lon and lat are needed
    csv_conv.parse_csv_format(csv_format_str, csv_srs, min_num_fields);
  } catch (...) {
    // Give a more specific error message
    vw::vw_throw(vw::ArgumentErr() << "Could not parse --csv-format. Was given: "
             << csv_format_str << ".\n");
  }
  std::list<asp::CsvConv::CsvRecord> pos_records;
  csv_conv.read_csv_file(lon_lat_measurements, pos_records);

  // Create llh and vertices
  int total_num_pts = 0;
  for (auto iter = pos_records.begin(); iter != pos_records.end(); iter++) {
    total_num_pts++;
    vw::Vector2 lonlat = csv_conv.csv_to_lonlat(*iter, shape_georef);
    vw::Vector2 shape_xy = shape_georef.lonlat_to_point(lonlat);
    addPoint(dem_georef, interp_dem, lonlat, shape_xy,
             ecef_vec, llh_vec, shape_xy_vec);
  }

  vw::vw_out() << "Read " << total_num_pts << " vertices from CSV, with " << llh_vec.size()
            << " of them having a valid DEM height value."  << "\n";

  return;
}

// Save the points as a shapefile
void saveShape(std::vector<Eigen::Vector3d> const& ecef_vec,
               std::string const& mask_boundary_shapefile) {

  vw::cartography::GeoReference llh_georef; // create a new explicit longlat WGS84 georef
  llh_georef.set_geographic();
  vw::cartography::Datum datum("WGS_1984");
  llh_georef.set_datum(datum);
  vw::geometry::dPoly samplePoly;
  for (size_t ptIter = 0; ptIter < ecef_vec.size(); ptIter++) {
    vw::Vector3 xyz; // convert Eigen:Vector3 to vw::Vector3
    for (int c = 0; c < 3; c++) xyz[c] = ecef_vec[ptIter][c];
    vw::Vector3 llh = llh_georef.datum().cartesian_to_geodetic(xyz);
    addPointToPoly(samplePoly, vw::math::subvector(llh, 0, 2));
  }

  std::vector<vw::geometry::dPoly> samplePolyVec;
  samplePolyVec.push_back(samplePoly);
  vw::vw_out() << "Writing shapefile of samples at mask boundary: "
           << mask_boundary_shapefile << "\n";
  bool has_llh_georef = true;
  vw::geometry::write_shapefile(mask_boundary_shapefile, has_llh_georef, llh_georef,
                                samplePolyVec);
}

// Best fit plane without outlier removal. The input points are in 3D coordinates.
// In practice the points are in a local projected coordinate system, but the 
// math works without this assumption.
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

  BestFitPlaneFunctor() {}

  typedef vw::Matrix<double, 1, 4> result_type;

  /// A best fit plane requires pairs of data points to make a fit.
  template <class ContainerT>
  size_t min_elements_needed_for_fit(ContainerT const& /*example*/) const { return 3; }

  /// This function can match points in any container that supports
  /// the size() and operator[] methods.  The container is usually a
  /// vw::Vector<>, but you could substitute other classes here as
  /// well.
  template <class ContainerT>
  vw::Matrix<double> operator()(std::vector<ContainerT> const& p1,
                                std::vector<ContainerT> const& p2,
                                vw::Matrix<double> const& /*seed_input*/
                                = vw::Matrix<double>()) const {

    // check consistency
    VW_ASSERT(p1.size() == p2.size(),
               vw::ArgumentErr() << "Cannot compute best fit plane. "
               << "p1 and p2 are not the same size.");
    VW_ASSERT(!p1.empty() && p1.size() >= min_elements_needed_for_fit(p1[0]),
               vw::ArgumentErr() << "Cannot compute best fit plane. "
               << "Insufficient data.\n");

    std::pair<Eigen::Vector3d, Eigen::Vector3d> plane = best_plane_from_points(p1);

    Eigen::Vector3d & centroid = plane.first;
    Eigen::Vector3d & normal = plane.second;

    vw::Matrix<double> result(1, 4);
    for (int col = 0; col < 3; col++)
      result(0, col) = normal[col];

    result(0, 3) = -normal.dot(centroid);

    // Make the z coefficient positive, which will make the normal
    // point "up" in the projected coordinate system.
    if (result(0, 2) < 0) {
      for (int col = 0; col < 4; col++)
        result(0, col) *= -1.0;
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
  double operator()(RelationT const& plane, ContainerT const& p1, ContainerT const& p2) const {
    return dist_to_plane(plane, p1);
  }
};

// Calculate a few properties of the plane fitted to the given points and print them out
void calcPlaneProperties(std::vector<Eigen::Vector3d> const& proj_vec,
                         std::vector<size_t> const& inlier_indices,
                         vw::cartography::GeoReference & dem_georef,
                         vw::Matrix<double> const& plane) {

  double max_error = - 1.0, max_inlier_error = -1.0;
  for (size_t it = 0; it < proj_vec.size(); it++)
    max_error = std::max(max_error, dist_to_plane(plane, proj_vec[it]));

  // Do estimates for the mean height and angle of the plane
  vw::Vector3 mean_point(0, 0, 0);
  double mean_height = 0.0;
  int num = 0;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    Eigen::Vector3d p = proj_vec[inlier_indices[it]];
    vw::Vector3 point(p[0], p[1], p[2]);
    max_inlier_error = std::max(max_inlier_error, dist_to_plane(plane, point));

    // the point is in the stereographic projection
    mean_height += point[2];

    num++;
  }

  mean_height /= num;

  vw::vw_out() << "Max distance to the plane (meters): " << max_error << "\n";
  vw::vw_out() << "Max inlier distance to the plane (meters): " << max_inlier_error << "\n";
  vw::vw_out() << "Mean water-surface height above datum (meters): " << mean_height << "\n";
}

// Save the bathy plane and the projection parameters if needed
void saveBathyPlane(double proj_lat, double proj_lon,
                    vw::Matrix<double> const& plane, std::string const& plane_file) {

  vw::vw_out() << "Writing: " << plane_file << "\n";
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
  bp << "# Latitude and longitude of the local stereographic projection with the WGS_1984 datum:\n";
  bp << proj_lat << " " << proj_lon << "\n";
  bp.close();
}

// Given a DEM surface and a planar surface (with potentially some
// inclination) subtract from each DEM height the height at that
// plane. The height of the plane is obtained by considering the
// current DEM grid point, and another point at same lon-lat but with
// a height 100 meters less, converting these to the given projection, 
// tracing a ray through them, seeing where it intersects the plane, 
// converting that point to geodetic, and taking the height difference.
class DemMinusPlaneView: public vw::ImageViewBase<DemMinusPlaneView>{
  vw::ImageViewRef<float> m_dem;
  vw::cartography::GeoReference m_dem_georef;
  vw::Matrix<double> m_plane;
  double m_dem_nodata_val;
  vw::cartography::GeoReference m_stereographic_georef;

  typedef float PixelT;

public:
  DemMinusPlaneView(vw::ImageViewRef<float> const& dem,
                    vw::cartography::GeoReference const& dem_georef,
                    vw::Matrix<double> const& plane,
                    double dem_nodata_val,
                    vw::cartography::GeoReference const& stereographic_georef):
    m_dem(dem), m_dem_georef(dem_georef), m_plane(plane),
    m_dem_nodata_val(dem_nodata_val),
    m_stereographic_georef(stereographic_georef) {}

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<DemMinusPlaneView> pixel_accessor;

  inline vw::int32 cols() const { return m_dem.cols(); }
  inline vw::int32 rows() const { return m_dem.rows(); }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline pixel_type operator()(double/*i*/, double/*j*/, vw::int32/*p*/ = 0) const {
    vw::vw_throw(vw::NoImplErr() << "DemMinusPlaneView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef vw::CropView< vw::ImageView<pixel_type> > prerasterize_type;

  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {

    // Bring this portion in memory
    vw::ImageView<result_type> cropped_dem = vw::crop(m_dem, bbox);

    vw::ImageView<result_type> tile(bbox.width(), bbox.height());

    for (int col = 0; col < bbox.width(); col++) {
      for (int row = 0; row < bbox.height(); row++) {

        // Handle the case when the DEM is not valid
        if (cropped_dem(col, row) == m_dem_nodata_val) {
          tile(col, row) = m_dem_nodata_val;
          continue;
        }

        vw::Vector2 pix(col + bbox.min().x(), row + bbox.min().y());
        vw::Vector2 lon_lat = m_dem_georef.pixel_to_lonlat(pix);

        vw::Vector3 llh;
        vw::math::subvector(llh, 0, 2) = lon_lat;
        llh[2] = cropped_dem(col, row);

        // Find the DEM point in local projection or ECEF.
        // Find another point on the same ray.
        vw::Vector3 point1, point2;

        point1 = m_stereographic_georef.geodetic_to_point(llh);
        llh[2] -= 100.0;
        point2 = m_stereographic_georef.geodetic_to_point(llh);

        // The ray is P = point1 + t * (point2 - point1), where t is real.
        // The plane is a*x + b*y + c*z + d = 0, where plane = (a, b, c, d).
        // Then, dot(P, plane_normal) + plane_intercept = 0.
        // Use that to find t.
        vw::Vector3 plane_normal(m_plane(0, 0), m_plane(0, 1), m_plane(0, 2));
        double plane_intercept = m_plane(0, 3);

        double t = -(vw::math::dot_prod(point1, plane_normal) + plane_intercept) /
          vw::math::dot_prod(point2 - point1, plane_normal);

        vw::Vector3 P = point1 + t * (point2 - point1);

        // Go back to llh
        llh = m_stereographic_georef.point_to_geodetic(P);

        tile(col, row) = cropped_dem(col, row) - llh[2];
      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

vw::ImageViewRef<float> 
demMinusPlane(vw::ImageViewRef<float> const& dem,
              vw::cartography::GeoReference const& dem_georef,
              vw::Matrix<double> plane,
              double dem_nodata_val,
              vw::cartography::GeoReference const& stereographic_georef) {
  return DemMinusPlaneView(dem, dem_georef, plane, dem_nodata_val,
                           stereographic_georef);
}

// Use RANSAC to find the best plane
void calcBathyPlane(int num_ransac_iterations,
                    double inlier_threshold,
                    std::vector<Eigen::Vector3d> const& proj_vec,
                    vw::Matrix<double> & plane,
                    std::vector<size_t> & inlier_indices) {

  std::vector<Eigen::Vector3d> dummy_vec(proj_vec.size()); // Required by the interface
  int min_num_output_inliers = std::max(proj_vec.size()/2, size_t(3));
  bool reduce_min_num_output_inliers_if_no_fit = true;
  vw::vw_out() << "Starting RANSAC.\n";
  try {
    // Must first create the functor and metric, then pass these to ransac. If
    // created as inline arguments to ransac, these may go go out
    // of scope prematurely, which will result in incorrect behavior.
    BestFitPlaneFunctor func;
    BestFitPlaneErrorMetric error_metric;
    vw::math::RandomSampleConsensus<BestFitPlaneFunctor, BestFitPlaneErrorMetric>
      ransac(func, error_metric, num_ransac_iterations, inlier_threshold,
             min_num_output_inliers, reduce_min_num_output_inliers_if_no_fit);
    plane = ransac(proj_vec, dummy_vec);
    inlier_indices = ransac.inlier_indices(plane, proj_vec, dummy_vec);
  } catch (const vw::math::RANSACErr& e) {
    vw::vw_out() << "RANSAC failed: " << e.what() << "\n";
  }
  vw::vw_out() << "Found " << inlier_indices.size() << " / " << proj_vec.size()
               << " inliers.\n";
}

// Evaluate the height (above the datum) of a water-surface plane at a given
// lon-lat. The plane a*x + b*y + c*z + d = 0 lives in the local stereographic
// frame stereo_georef. Intersect the vertical geodetic line at (lon, lat) with
// the plane and return the geodetic height of the intersection. Mirrors the
// math in DemMinusPlaneView.
static double evalPlaneHeightAtLonLat(vw::Vector2 const& lon_lat,
                                      vw::Matrix<double> const& plane,
                                      vw::cartography::GeoReference const& stereo_georef) {
  vw::Vector3 llh;
  vw::math::subvector(llh, 0, 2) = lon_lat;
  llh[2] = 0.0;
  vw::Vector3 point1 = stereo_georef.geodetic_to_point(llh);
  llh[2] = -100.0;
  vw::Vector3 point2 = stereo_georef.geodetic_to_point(llh);

  vw::Vector3 plane_normal(plane(0, 0), plane(0, 1), plane(0, 2));
  double plane_intercept = plane(0, 3);
  double t = -(vw::math::dot_prod(point1, plane_normal) + plane_intercept) /
    vw::math::dot_prod(point2 - point1, plane_normal);
  vw::Vector3 P = point1 + t * (point2 - point1);
  return stereo_georef.point_to_geodetic(P)[2];
}

// Fit a separate water-surface plane to each connected water body (lake) in a
// georeferenced land/water mask, and write a per-pixel water-surface-elevation
// (WSE) raster on the DEM grid. Water pixels are labeled into connected bodies
// (8-connectivity). For each body with at least min_water_body_pixels pixels, the
// shoreline points (land pixels at the water interface, with heights from the
// DEM) are fit to a plane in that body's own local stereographic frame via
// RANSAC, exactly as the single-plane --ortho-mask path does, but per lake.
// Every water pixel of a fitted lake gets that plane's height; all other pixels
// are nodata. The output WSE raster can be passed to parallel_stereo
// --bathy-plane for multi-water-body bathymetry correction.
void calcMultiWaterBodyPlanes(std::string const& mask_file,
                              vw::cartography::GeoReference const& mask_georef,
                              vw::cartography::GeoReference const& dem_georef,
                              vw::ImageViewRef<vw::PixelMask<float>> interp_dem,
                              double dem_nodata_val,
                              int min_water_body_pixels,
                              int num_ransac_iterations,
                              double outlier_threshold,
                              std::string const& output_wse_raster,
                              std::string const& output_inlier_shapefile,
                              bool save_shapefiles_as_polygons,
                              vw::GdalWriteOptions const& write_opt) {

  // Read the mask and classify every pixel into three classes: WATER, LAND, or
  // NODATA. Unlike the single-plane path, nodata is NOT treated as water here:
  // the (typically large) nodata background would otherwise connect separate
  // lakes into one body. A pixel is WATER if it is valid (not nodata, not NaN)
  // and non-positive, LAND if valid and positive, and NODATA otherwise. This
  // matches the ternary land=1 / water=0 / nodata convention of the ortho masks
  // that this mode consumes.
  vw::vw_out() << "Reading the ortho mask: " << mask_file << "\n";
  float mask_thresh = 0.0f;
  float file_nodata = std::numeric_limits<float>::quiet_NaN();
  bool has_mask_nodata = vw::read_nodata_val(mask_file, file_nodata);
  if (has_mask_nodata)
    vw::vw_out() << "Read ortho mask nodata value: " << file_nodata << ".\n";
  vw::ImageView<float> raw_mask = vw::DiskImageView<float>(mask_file);
  vw::vw_out() << "Non-positive (and non-nodata) pixels are classified as water.\n";

  int cols = raw_mask.cols(), rows = raw_mask.rows();

  // cls: 0 = nodata/neither, 1 = water, 2 = land.
  vw::ImageView<vw::uint8> cls(cols, rows);
  long long num_water_px = 0;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float v = raw_mask(c, r);
      bool is_nodata = (std::isnan(v) || (has_mask_nodata && v == file_nodata));
      if (is_nodata)      cls(c, r) = 0;
      else if (v <= mask_thresh) { cls(c, r) = 1; num_water_px++; }
      else                cls(c, r) = 2;
    }
  }
  vw::vw_out() << "Water pixels: " << num_water_px << ".\n";

  // The labels, shoreline sampling, and output WSE raster all live on the mask
  // grid. The DEM only supplies shoreline heights, read by bilinear interpolation
  // at each point's lon-lat, so it can be on any grid or resolution.

  // Label connected water bodies with a flood fill (8-connectivity). labels is 0
  // for land/nodata, and 1..num_lakes for water bodies.
  vw::vw_out() << "Labeling connected water bodies.\n";
  vw::ImageView<vw::int32> labels(cols, rows);
  vw::fill(labels, 0);
  std::vector<int> lake_size; // lake_size[k-1] = pixel count of label k
  int num_lakes = 0;
  const int dc8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dr8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      if (cls(c, r) != 1 || labels(c, r) != 0)
        continue; // not water, or already labeled
      // New water body: flood fill it.
      num_lakes++;
      int count = 0;
      std::stack<std::pair<int,int>> stk;
      stk.push(std::make_pair(c, r));
      labels(c, r) = num_lakes;
      while (!stk.empty()) {
        std::pair<int,int> p = stk.top(); stk.pop();
        count++;
        for (int k = 0; k < 8; k++) {
          int nc = p.first + dc8[k], nr = p.second + dr8[k];
          if (nc < 0 || nr < 0 || nc >= cols || nr >= rows)
            continue;
          if (cls(nc, nr) == 1 && labels(nc, nr) == 0) {
            labels(nc, nr) = num_lakes;
            stk.push(std::make_pair(nc, nr));
          }
        }
      }
      lake_size.push_back(count);
    }
  }
  vw::vw_out() << "Found " << num_lakes << " connected water bodies.\n";

  // Select the water bodies that are large enough to fit a plane to.
  std::vector<int> label_to_lake(num_lakes + 1, -1); // map full label -> dense lake index
  int num_kept = 0;
  for (int k = 1; k <= num_lakes; k++) {
    if (lake_size[k-1] >= min_water_body_pixels) {
      label_to_lake[k] = num_kept;
      num_kept++;
    }
  }
  vw::vw_out() << "Water bodies with at least " << min_water_body_pixels
               << " pixels: " << num_kept << ".\n";
  if (num_kept == 0)
    vw::vw_throw(vw::ArgumentErr() << "No water body has at least "
                 << min_water_body_pixels << " pixels. Lower --min-water-body-pixels.\n");

  // Collect shoreline points per lake in a single pass over land pixels at the
  // water interface. A land pixel bordering a kept lake contributes a subpixel
  // boundary point (height looked up in the DEM) to that lake.
  std::vector<std::vector<Eigen::Vector3d>> ecef_per_lake(num_kept);
  std::vector<std::vector<vw::Vector3>>     llh_per_lake(num_kept);
  std::vector<std::vector<vw::Vector2>>     xy_per_lake(num_kept);
  const int dc4[4] = {-1, 0, 0, 1};
  const int dr4[4] = {0, -1, 1, 0};

  // A plain water/land float image (water = 0, land or nodata = 1) so the
  // subpixel boundary estimator finds only true land->water interfaces (and not
  // false interfaces at the nodata background).
  vw::ImageView<float> wl(cols, rows);
  for (int r = 0; r < rows; r++)
    for (int c = 0; c < cols; c++)
      wl(c, r) = (cls(c, r) == 1) ? 0.0f : 1.0f;

  vw::vw_out() << "Sampling shoreline points at water-body boundaries.\n";
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      if (cls(c, r) != 2)
        continue; // only land pixels can be shoreline points
      // Which kept lakes does this land pixel border?
      std::set<int> adj_lakes;
      for (int k = 0; k < 4; k++) {
        int nc = c + dc4[k], nr = r + dr4[k];
        if (nc < 0 || nr < 0 || nc >= cols || nr >= rows)
          continue;
        int lab = labels(nc, nr);
        if (lab > 0 && label_to_lake[lab] >= 0)
          adj_lakes.insert(label_to_lake[lab]);
      }
      if (adj_lakes.empty())
        continue;
      // Subpixel boundary location and the DEM-height point for it.
      vw::Vector2 pix;
      if (!calcSubpixBdPoint(wl, 0.0f, c, r, pix))
        continue;
      vw::Vector2 shape_xy = mask_georef.pixel_to_point(pix);
      vw::Vector2 lonlat   = mask_georef.point_to_lonlat(shape_xy);
      for (int lake: adj_lakes)
        addPoint(dem_georef, interp_dem, lonlat, shape_xy,
                 ecef_per_lake[lake], llh_per_lake[lake], xy_per_lake[lake]);
    }
  }

  // Fit a plane to each lake in its own local stereographic frame.
  std::vector<vw::Matrix<double>> lake_plane(num_kept);
  std::vector<vw::cartography::GeoReference> lake_stereo(num_kept);
  std::vector<bool> lake_valid(num_kept, false);
  std::vector<double> lake_wse(num_kept, 0.0);
  vw::geometry::dPoly inlierPoly; // accumulate inliers from all lakes

  // Reverse map dense lake index -> a representative full label, for reporting.
  std::vector<int> lake_to_label(num_kept, 0);
  for (int k = 1; k <= num_lakes; k++)
    if (label_to_lake[k] >= 0) lake_to_label[label_to_lake[k]] = k;

  int min_shore_pts = 10;
  for (int lake = 0; lake < num_kept; lake++) {
    int npts = ecef_per_lake[lake].size();
    if (npts < min_shore_pts) {
      vw::vw_out() << "Lake " << lake_to_label[lake] << ": only " << npts
                   << " shoreline points, skipping (need " << min_shore_pts << ").\n";
      continue;
    }

    double proj_lat = -1.0, proj_lon = -1.0;
    std::vector<Eigen::Vector3d> proj_vec;
    vw::cartography::GeoReference stereo_georef;
    find_projection(dem_georef, llh_per_lake[lake],
                    proj_lat, proj_lon, stereo_georef, proj_vec);

    std::vector<size_t> inliers;
    vw::Matrix<double> plane;
    calcBathyPlane(num_ransac_iterations, outlier_threshold, proj_vec, plane, inliers);
    if (inliers.size() < size_t(min_shore_pts)) {
      vw::vw_out() << "Lake " << lake_to_label[lake] << ": RANSAC found only "
                   << inliers.size() << " inliers, skipping.\n";
      continue;
    }

    lake_plane[lake] = plane;
    lake_stereo[lake] = stereo_georef;
    lake_valid[lake] = true;

    // Water-surface height at the lake center (for reporting), from the plane.
    lake_wse[lake] = evalPlaneHeightAtLonLat(vw::Vector2(proj_lon, proj_lat),
                                             plane, stereo_georef);
    vw::vw_out() << "Lake " << lake_to_label[lake] << ": " << lake_size[lake_to_label[lake]-1]
                 << " water px, " << inliers.size() << "/" << npts << " shoreline inliers, "
                 << "WSE = " << lake_wse[lake] << " m.\n";

    // Accumulate inlier shoreline points for the optional shapefile.
    if (!output_inlier_shapefile.empty())
      for (size_t it = 0; it < inliers.size(); it++)
        addPointToPoly(inlierPoly, xy_per_lake[lake][inliers[it]]);
  }

  // Build the WSE raster on the mask grid: each water pixel of a fitted body gets
  // its plane's height, all other pixels are nodata.
  vw::vw_out() << "Writing the water-surface-elevation raster: " << output_wse_raster << "\n";
  vw::ImageView<float> wse(cols, rows);
  vw::fill(wse, dem_nodata_val);
  long long num_filled = 0;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      int lab = labels(c, r);
      if (lab <= 0) continue;
      int lake = label_to_lake[lab];
      if (lake < 0 || !lake_valid[lake]) continue;
      vw::Vector2 lon_lat = mask_georef.pixel_to_lonlat(vw::Vector2(c, r));
      wse(c, r) = evalPlaneHeightAtLonLat(lon_lat, lake_plane[lake], lake_stereo[lake]);
      num_filled++;
    }
  }
  vw::vw_out() << "Filled " << num_filled << " water pixels.\n";

  bool has_georef = true, has_nodata = true;
  vw::TerminalProgressCallback tpc("asp", ": ");
  vw::cartography::block_write_gdal_image(output_wse_raster, wse, has_georef, mask_georef,
                                          has_nodata, dem_nodata_val, write_opt, tpc);

  // Optionally save the combined inlier shapefile.
  if (!output_inlier_shapefile.empty()) {
    if (save_shapefiles_as_polygons) {
      vw::geometry::dPoly localPoly;
      formSinglePoly(inlierPoly, localPoly);
      inlierPoly = localPoly;
    }
    std::vector<vw::geometry::dPoly> inlierPolyVec;
    inlierPolyVec.push_back(inlierPoly);
    vw::vw_out() << "Writing inlier shapefile: " << output_inlier_shapefile << "\n";
    bool has_shape_georef = true;
    vw::geometry::write_shapefile(output_inlier_shapefile, has_shape_georef,
                                  mask_georef, inlierPolyVec);
  }
}

} // end namespace asp

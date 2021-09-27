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

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/shapeFile.h>

#include <vw/Math/RANSAC.h>

#include <Eigen/Dense>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Compute the 3D locations at the shape corners based on interpolating
// into the DEM and converting either to ECEF or to local projected
// stereographic coordinates.
// If using a curved water surface, compute the stereographic georeference
// with the projection center being the mean lon and lat, and make the 3D locations
// in reference to this projection
void find_points_at_shape_corners(bool use_proj_water_surface,
                                  std::vector<vw::geometry::dPoly> const& polyVec,
                                  vw::cartography::GeoReference const& shape_georef,
                                  vw::cartography::GeoReference const& dem_georef,
                                  ImageViewRef< PixelMask<float> > interp_dem,
                                  std::vector<Eigen::Vector3d> & point_vec,
                                  std::vector<vw::Vector2> & used_shape_vertices,
                                  double & proj_lat, double & proj_lon,
                                  vw::cartography::GeoReference & stereographic_georef) {
  
  // Ensure that the outputs are initialized
  point_vec.clear();
  used_shape_vertices.clear();
  proj_lat = -1.0;
  proj_lon = -1.0;

  int total_num_pts = 0;
  std::vector<vw::Vector3> llh_vec;
  
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

        Vector3 point = dem_georef.datum().geodetic_to_cartesian(llh);
        Eigen::Vector3d eigen_point;
        for (size_t coord = 0; coord < 3; coord++) 
          eigen_point[coord] = point[coord];

        point_vec.push_back(eigen_point);
        used_shape_vertices.push_back(proj_pt);
        llh_vec.push_back(llh);
      }
    }
  }

  std::cout << "Read " << total_num_pts << " vertices, with " << llh_vec.size()
            << " of them having a valid DEM height value."  << std::endl;
  
  // See if to convert to local stereographic projection
  if (use_proj_water_surface) {

    // Find the mean water height
    Vector3 mean_llh;
    for (size_t it = 0; it < llh_vec.size(); it++) 
      mean_llh += llh_vec[it];

    mean_llh /= llh_vec.size();

    // ASP is having a hard time with saving and reading a georef as a wkt string
    // So be conservative and use a WGS_1984 datum only with given lat and lon.
    if (dem_georef.datum().name() != "WGS_1984")
      vw_throw( ArgumentErr() << "Only an input DEM with the "
                << "WGS_1984 datum is supported.\n"
                << "Got: " << dem_georef.datum().name() << ".\n");

    vw::cartography::Datum datum("WGS_1984");
    stereographic_georef.set_datum(datum);
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
      
      point_vec[it] = eigen_point;
    }
  }
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


struct Options : vw::cartography::GdalWriteOptions {
  std::string shapefile, dem, bathy_plane, output_inlier_shapefile, dem_minus_plane;
  double outlier_threshold;
  int num_ransac_iterations;
  bool use_ecef_water_surface;
  Options(): outlier_threshold(0.2), num_ransac_iterations(1000) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("shapefile",   po::value(&opt.shapefile),
     "The shapefile with vertices whose coordinates will be looked up in the DEM.")
    ("dem",   po::value(&opt.dem),
     "The DEM to use.")
    ("bathy-plane",   po::value(&opt.bathy_plane),
     "The output file storing the computed plane as four coefficients a, b, c, d, "
     "with the plane being a*x + b*y + c*z + d = 0.")
    ("outlier-threshold",
     po::value(&opt.outlier_threshold)->default_value(0.2),
     "A value, in meters, to determine the distance from a sampled point on the DEM to the "
     "best-fit plane to determine if it will be marked as outlier and not "
     "included in the calculation of that plane.")
    ("num-ransac-iterations", 
     po::value(&opt.num_ransac_iterations)->default_value(1000),
     "Number of RANSAC iterations to use to find the best-fitting plane.")
    ("output-inlier-shapefile", po::value(&opt.output_inlier_shapefile)->default_value(""),
     "If specified, save at this location the shape file with the inlier vertices.")
    ("dem-minus-plane",
     po::value(&opt.dem_minus_plane),
     "If specified, subtract from the input DEM the best-fit plane and save the "
     "obtained DEM to this GeoTiff file.")
    ("use-ecef-water-surface",
     po::bool_switch(&opt.use_ecef_water_surface)->default_value(false),
     "Compute the best fit plane in ECEF coordinates rather than in a local stereographic "
     "projection. Hence don't model the Earth curvature. Not recommended.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

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

  if (opt.shapefile == "")
    vw_throw( ArgumentErr() << "Missing the input shapefile.\n" << usage << general_options );
  if (opt.dem == "")
    vw_throw( ArgumentErr() << "Missing the input dem.\n" << usage << general_options );
  if (opt.bathy_plane == "")
    vw_throw( ArgumentErr() << "Missing the output bathy plane file.\n"
              << usage << general_options );
}

// This was not updated for a curved bathy plane.

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

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    bool use_proj_water_surface = !opt.use_ecef_water_surface;
    
    // Read the shapefile
    std::cout << "Reading the shapefile: " << opt.shapefile << std::endl;
    bool has_shape_georef;
    std::vector<vw::geometry::dPoly> polyVec;
    std::string poly_color;
    vw::cartography::GeoReference shape_georef;
    read_shapefile(opt.shapefile, poly_color, has_shape_georef, shape_georef, polyVec);
    
    if (!has_shape_georef) 
      vw_throw( ArgumentErr() << "The input shapefile has no georeference.\n" );

    // Read the DEM and its associated data
    // TODO(oalexan1): Think more about the interpolation method
    std::cout << "Reading the DEM: " << opt.dem << std::endl;
    vw::cartography::GeoReference dem_georef;
    if (!read_georeference(dem_georef, opt.dem))
      vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );
    // Note we use a float nodata
    double dem_nodata_val = -std::numeric_limits<float>::max();
    if (!vw::read_nodata_val(opt.dem, dem_nodata_val))
      std::cout << "Warning: Could not read the DEM nodata value. "
                << "Using: " << dem_nodata_val << ".\n";
    else
      std::cout << "Read DEM nodata value: " << dem_nodata_val << ".\n";
    DiskImageView<float> dem(opt.dem);
    
    ImageViewRef< PixelMask<float> > interp_dem
      = interpolate(create_mask(dem, dem_nodata_val),
                    BilinearInterpolation(), ConstantEdgeExtension());

    // Find the ECEF coordinates of the shape corners
    std::vector<Eigen::Vector3d> point_vec;
    std::vector<vw::Vector2> used_shape_vertices;
    double proj_lat, proj_lon; // only for curved water surface
    vw::cartography::GeoReference stereographic_georef;
    find_points_at_shape_corners(use_proj_water_surface,
                                 polyVec, shape_georef, dem_georef, interp_dem, point_vec,
                                 used_shape_vertices, proj_lat, proj_lon,
                                 stereographic_georef);

    // Compute the water surface using RANSAC
    std::vector<Eigen::Vector3d> dummy_vec(point_vec.size()); // Required by the interface
    std::vector<size_t> inlier_indices;
    double inlier_threshold = opt.outlier_threshold;
    int    min_num_output_inliers = std::max(point_vec.size()/2, size_t(3));
    bool   reduce_min_num_output_inliers_if_no_fit = true;
    vw::Matrix<double> plane;
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
      std::cout << "RANSAC failed: " << e.what() << "\n";
    }
    std::cout << "Found " << inlier_indices.size() << " / " << point_vec.size() << " inliers.\n";
    
    double max_error = - 1.0, max_inlier_error = -1.0;
    for (size_t it = 0; it < point_vec.size(); it++) {
      max_error = std::max(max_error, dist_to_plane(plane, point_vec[it]));
    }
    
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

    std::cout << "Max distance to the plane (meters): " << max_error << std::endl;
    std::cout << "Max inlier distance to the plane (meters): " << max_inlier_error << std::endl;
    std::cout << "Mean plane height above datum (meters): " << mean_height << std::endl;
    if (!use_proj_water_surface) {
      // This does not make sense for a curved surface
      Vector3 plane_normal(plane(0, 0), plane(0, 1), plane(0, 2));
      Vector3 surface_normal = mean_point / norm_2(mean_point); // ignore the datum flattening
      double plane_angle = (180.0 / M_PI) * acos(dot_prod(plane_normal, surface_normal));
      std::cout << "Plane inclination (degrees): " << plane_angle << std::endl;
    }
    
    std::cout << "Writing: " << opt.bathy_plane << std::endl;
    std::ofstream bp(opt.bathy_plane.c_str());
    bp.precision(17);
    for (int col = 0; col < plane.cols(); col++) {
      bp << plane(0, col);
      if (col < plane.cols() - 1)
        bp << " ";
      else
        bp << "\n";
    }
    if (use_proj_water_surface) {
      bp << "# Latitude and longitude of the local stereographic projection with the WGS_1984 datum"
         << "and these latitude and longitude values:\n";
      bp << proj_lat << " " << proj_lon << "\n";
    }
    bp.close();

    // Save the shape having the inliers
    if (opt.output_inlier_shapefile != "") {
      std::vector<double> inlier_x, inlier_y;
      for (size_t inlier_it = 0; inlier_it < inlier_indices.size(); inlier_it++) {
        Vector2 p = used_shape_vertices[inlier_indices[inlier_it]];
        inlier_x.push_back(p.x());
        inlier_y.push_back(p.y());
      }
      bool isPolyClosed = true;
      std::string layer = "";
      vw::geometry::dPoly inlierPoly;
      inlierPoly.setPolygon(inlier_x.size(),
                            vw::geometry::vecPtr(inlier_x),
                            vw::geometry::vecPtr(inlier_y),
                            isPolyClosed, poly_color, layer);
      std::vector<vw::geometry::dPoly> inlierPolyVec;
      inlierPolyVec.push_back(inlierPoly);
      std::cout << "Writing inlier shapefile: " << opt.output_inlier_shapefile << "\n";
      write_shapefile(opt.output_inlier_shapefile, has_shape_georef, shape_georef,
                      inlierPolyVec);
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

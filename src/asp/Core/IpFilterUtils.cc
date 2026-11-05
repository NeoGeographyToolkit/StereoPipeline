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

#include <asp/Core/IpFilterUtils.h>
#include <asp/Core/StereoSettings.h>

#include <vw/Cartography/GeoTransform.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/RANSAC.h>
#include <vw/Math/GaussianClustering.h>
#include <vw/Stereo/StereoModel.h>
#include <vw/InterestPoint/Matcher.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/Image/Interpolation.h>
#include <vw/Core/Exception.h>

#include <unordered_map>
#include <sstream>
#include <set>
#include <cmath>

using namespace vw;

namespace asp {

namespace {

// A uniform spatial bin grid of 2D points into square cells of side 'radius'.
// Lets us answer "does this point have any neighbor within radius?" in O(1)
// average, so the left-right filtering below is O(N) rather than O(N^2).
class PointGrid {
public:
  PointGrid(double radius, std::vector<vw::Vector2> const& pts)
    : m_r(radius), m_pts(pts) {
    for (size_t i = 0; i < pts.size(); i++)
      m_cells[key(cellx(pts[i]), celly(pts[i]))].push_back((int)i);
  }
  bool has_neighbor(vw::Vector2 const& p) const {
    int cx = cellx(p), cy = celly(p);
    double r2 = m_r * m_r;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        std::unordered_map<long long, std::vector<int>>::const_iterator it
          = m_cells.find(key(cx + dx, cy + dy));
        if (it == m_cells.end())
          continue;
        for (size_t k = 0; k < it->second.size(); k++) {
          vw::Vector2 d = m_pts[it->second[k]] - p;
          if (d.x()*d.x() + d.y()*d.y() <= r2)
            return true;
        }
      }
    }
    return false;
  }
private:
  int cellx(vw::Vector2 const& p) const { return (int)std::floor(p.x() / m_r); }
  int celly(vw::Vector2 const& p) const { return (int)std::floor(p.y() / m_r); }
  long long key(int cx, int cy) const {
    return (long long)cx * 4294967311LL + (long long)cy;
  }
  double m_r;
  std::vector<vw::Vector2> const& m_pts;
  std::unordered_map<long long, std::vector<int>> m_cells;
};

} // end anonymous namespace

// For mapprojected images, drop interest points with no counterpart in the
// other image within 'radius' pixels.
void filter_ip_by_ground_loc(vw::InterestPointList & ip1,
                             vw::InterestPointList & ip2,
                             vw::GeoReference const& georef1,
                             vw::GeoReference const& georef2,
                             double radius) {

  if (radius <= 0 || ip1.empty() || ip2.empty())
    return;

  vw::GeoTransform left_to_right(georef1, georef2);

  std::vector<vw::InterestPoint> v1(ip1.begin(), ip1.end());
  std::vector<vw::InterestPoint> v2(ip2.begin(), ip2.end());
  std::vector<vw::Vector2> p1(v1.size()), p2(v2.size());
  double const FAR = 1e20;
  for (size_t i = 0; i < v1.size(); i++) {
    try {
      p1[i] = left_to_right.forward(vw::Vector2(v1[i].x, v1[i].y));
    } catch (...) {
      p1[i] = vw::Vector2(FAR, FAR);
    }
  }
  for (size_t j = 0; j < v2.size(); j++)
    p2[j] = vw::Vector2(v2[j].x, v2[j].y);

  PointGrid grid2(radius, p2), grid1(radius, p1);

  vw::InterestPointList keep1, keep2;
  for (size_t i = 0; i < v1.size(); i++)
    if (grid2.has_neighbor(p1[i]))
      keep1.push_back(v1[i]);
  for (size_t j = 0; j < v2.size(); j++)
    if (grid1.has_neighbor(p2[j]))
      keep2.push_back(v2[j]);

  vw::vw_out() << "\t    ip-match-radius " << radius << " px: kept "
               << keep1.size() << "/" << v1.size() << " left, "
               << keep2.size() << "/" << v2.size() << " right interest points.\n";

  ip1 = keep1;
  ip2 = keep2;
}

size_t remove_ip_bbox(vw::BBox2i const& roi, vw::InterestPointList & ip_list,
                      bool remove_outside) {
  size_t prior_size = ip_list.size();
  auto keep_end = std::remove_if(ip_list.begin(), ip_list.end(),
    [&](vw::InterestPoint const& ip) {
      return roi.contains(vw::Vector2i(ip.ix,ip.iy)) xor remove_outside;
    });
  ip_list.erase(keep_end, ip_list.end());
  return prior_size - ip_list.size();
}

void side_ip_filtering(vw::InterestPointList& ip1,
                       vw::InterestPointList& ip2,
                       vw::BBox2i const& bbox1, vw::BBox2i const& bbox2) {

  if (stereo_settings().ip_edge_buffer_percent <= 0)
    return;

  double percent  = static_cast<double>(stereo_settings().ip_edge_buffer_percent)/100.0;
  int   width_left  = floor(static_cast<double>(bbox1.width()) * percent);
  int   width_right = floor(static_cast<double>(bbox2.width()) * percent);
  BBox2 bbox_left (width_left, 0, bbox1.width()-width_left,  bbox1.height());
  BBox2 bbox_right(0,          0, bbox2.width()-width_right, bbox2.height());
  bool  remove_outside = true;

  size_t num_removed_left  = remove_ip_bbox(bbox_left,  ip1, remove_outside);
  size_t num_removed_right = remove_ip_bbox(bbox_right, ip2, remove_outside);
  vw_out() << "Removed: " << num_removed_left << " points from the left side of the left image and "
           << num_removed_right << " points from the right side of the right image.\n";
}

bool tri_ip_filtering(std::vector<vw::InterestPoint> const& matched_ip1,
                      std::vector<vw::InterestPoint> const& matched_ip2,
                      vw::CameraModel* cam1,
                      vw::CameraModel* cam2,
                      std::list<size_t>& valid_indices) {
  typedef std::vector<double> ArrayT;
  ArrayT error_samples(valid_indices.size());

  double angle_tol = vw::StereoModel::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);

  vw::StereoModel model(cam1, cam2, angle_tol);
  size_t count = 0;
  const double HIGH_ERROR = 9999999;
  for (auto it = valid_indices.begin(); it != valid_indices.end(); it++) {
    size_t i = *it;
    model(Vector2(matched_ip1[i].x, matched_ip1[i].y),
          Vector2(matched_ip2[i].x, matched_ip2[i].y), error_samples[count]);
    if (error_samples[count] == 0)
      error_samples[count] = HIGH_ERROR;
    count++;
  }
  VW_ASSERT(count == valid_indices.size(),
            vw::MathErr() << "Error: tri_ip_filtering: count indices not aligned.");

  typedef std::vector<std::pair<Vector<double>, Vector<double> > > ClusterT;
  const size_t NUM_CLUSTERS = 2;
  ClusterT error_clusters = gaussian_clustering<ArrayT>(error_samples.begin(),
      error_samples.end(), NUM_CLUSTERS);

  if ((error_clusters.front().second[0] > error_clusters.back().second[0]) &&
      (error_clusters.back().second[0] != std::numeric_limits<double>::epsilon()))
    std::swap(error_clusters[0], error_clusters[1]);

  if (error_clusters.front().second[0] > 1e6) {
    vw_out() << "\t    Unable to find inlier cluster, keeping best 70% of points.\n";

    const double CUTOFF_PERCENTILE = 0.7;
    std::vector<double> sorted_error(error_samples);
    std::sort(sorted_error.begin(), sorted_error.end());
    size_t last_good_index = static_cast<double>(sorted_error.size()) * CUTOFF_PERCENTILE;
    double cutoff_value = 0;
    if (last_good_index < sorted_error.size())
      cutoff_value = sorted_error[last_good_index];
    if ((asp::stereo_settings().ip_triangulation_max_error > 0) &&
        (cutoff_value > asp::stereo_settings().ip_triangulation_max_error))
      cutoff_value = asp::stereo_settings().ip_triangulation_max_error;

    std::list<size_t> filtered_indices;
    size_t c = 0;
    for (auto i = valid_indices.begin(); i != valid_indices.end(); i++) {
      if (error_samples[c] < cutoff_value)
        filtered_indices.push_back(*i);
      c++;
    }
    valid_indices = filtered_indices;
    return (!valid_indices.empty());
  }

  vw_out() << "\t    Inlier cluster:\n"
           << "\t      Triangulation error: " << error_clusters.front().first[0]
           << " +- " << sqrt(error_clusters.front().second[0]) << " meters\n";

  const double escalar1 = 1.0 / sqrt(2.0 * M_PI * error_clusters.front().second[0]);
  const double escalar2 = 1.0 / sqrt(2.0 * M_PI * error_clusters.back().second[0]);
  const double escalar3 = 1.0 / (2 * error_clusters.front().second[0]);
  const double escalar4 = 1.0 / (2 * error_clusters.back().second[0]);
  size_t error_idx        = 0;
  size_t prior_valid_size = valid_indices.size();
  size_t outlier_count    = 0;
  for (std::list<size_t>::iterator i = valid_indices.begin(); i != valid_indices.end(); i++) {
    double err_diff_front = error_samples[error_idx]-error_clusters.front().first[0];
    double err_diff_back  = error_samples[error_idx]-error_clusters.back ().first[0];

    if (!((escalar1 * exp((-err_diff_front * err_diff_front) * escalar3)) >
           (escalar2 * exp((-err_diff_back  * err_diff_back) * escalar4)) ||
           error_samples[error_idx] < error_clusters.front().first[0]) ||
         ((asp::stereo_settings().ip_triangulation_max_error > 0) &&
           (error_samples[error_idx] > asp::stereo_settings().ip_triangulation_max_error))) {
      i = valid_indices.erase(i);
      i--;
      ++outlier_count;
    }
    error_idx++;
  }
  VW_ASSERT(prior_valid_size == error_idx,
            vw::MathErr() << "tri_ip_filtering: Programmer error. Indices don't seem to be aligned.");

  vw_out() << "\t      Removed " << outlier_count << " points in triangulation filtering.\n";
  return (!valid_indices.empty());
}

bool stddev_ip_filtering(std::vector<vw::InterestPoint> const& ip1,
                         std::vector<vw::InterestPoint> const& ip2,
                         std::list<size_t>& valid_indices) {
  const int NUM_STD_FILTER = 4;
  bool deleted_something = false;
  size_t num_deleted = 0;
  do {
    deleted_something = false;
    Matrix<float> locations1(valid_indices.size(), 2);
    size_t count = 0;
    std::vector<size_t > reverse_lookup  (valid_indices.size());
    std::vector<Vector2> disparity_vector(valid_indices.size());
    for (auto it = valid_indices.begin(); it != valid_indices.end(); it++) {
      size_t index = *it;
      locations1(count, 0) = ip1[index].x;
      locations1(count, 1) = ip1[index].y;
      reverse_lookup  [count] = index;
      disparity_vector[count] = Vector2(ip2[index].x,ip2[index].y) -
        Vector2(ip1[index].x,ip1[index].y);
      count++;
    }
    vw::FLANNTree<float> tree1(asp::stereo_settings().flann_method);
    tree1.load_match_data(locations1, vw::FLANN_DistType_L2);

    std::pair<double,size_t> worse_index;
    worse_index.first = 0;
    for (size_t i = 0; i < valid_indices.size(); i++) {
      Vector<int> indices;
      Vector<double> distance;
      const int NUM_INDICES_TO_GET = 11;
      tree1.knn_search(select_row(locations1, i),
                       indices, distance, NUM_INDICES_TO_GET);

      std::vector<int> good_indices;
      for (size_t j = 0; j < indices.size(); j++) {
        if ((indices[j] < 0) || (indices[j] >= (int)disparity_vector.size()))
          continue;
        good_indices.push_back(indices[j]);
      }

      Vector2 sum;
      for (size_t j = 1; j < good_indices.size(); j++) {
        sum += disparity_vector[good_indices[j]];
      }
      sum = normalize(sum);

      Vector<double> projections(good_indices.size());
      for (size_t j = 0; j < good_indices.size(); j++) {
        projections[j] = dot_prod(disparity_vector[good_indices[j]], sum);
      }
      double mean   = 0;
      double stddev = 0;
      for (size_t j = 1; j < good_indices.size(); j++) {
        mean += projections[j];
        stddev += projections[j]*projections[j];
      }
      mean /= good_indices.size() - 1;
      stddev = sqrt(stddev / (good_indices.size() - 1) - mean*mean);

      double std_distance = std::abs(projections[0]-mean)/stddev;
      if (std_distance > worse_index.first) {
        worse_index.first = std_distance;
        worse_index.second = i;
      }
    }
    if (worse_index.first > NUM_STD_FILTER) {
      std::list<size_t>::iterator it = valid_indices.begin();
      std::advance(it, worse_index.second);
      valid_indices.erase(it);
      deleted_something = true;
      ++num_deleted;
    }
    if (valid_indices.empty())
      return 0;
  } while (deleted_something);

  vw_out() << "\t      Removed " << num_deleted << " points in stddev filtering.\n";
  return valid_indices.size();
}

size_t filter_ip_by_lonlat_and_elevation(vw::TransformPtr         tx_left,
                                         vw::TransformPtr         tx_right,
                                         vw::CameraModel* left_camera_model,
                                         vw::CameraModel* right_camera_model,
                                         vw::Datum        const& datum,
                                         std::vector<vw::InterestPoint> const& ip1_in,
                                         std::vector<vw::InterestPoint> const& ip2_in,
                                         vw::Vector2 const & elevation_limit,
                                         vw::BBox2   const & lon_lat_limit,
                                         std::vector<vw::InterestPoint> & ip1_out,
                                         std::vector<vw::InterestPoint> & ip2_out) {

  const size_t num_ip = ip1_in.size();
  if (elevation_limit[1] <= elevation_limit[0] && lon_lat_limit.empty()) {
    ip1_out = ip1_in;
    ip2_out = ip2_in;
    return num_ip;
  }

  if (elevation_limit[0] < elevation_limit[1])
    vw_out() << "\t    * Applying elevation restriction. Height range: " << elevation_limit[0]
             << " to " << elevation_limit[1] << ".\n";

  if (!lon_lat_limit.empty())
    vw_out() << "\t    * Applying lon-lat restriction: " << lon_lat_limit.min()
             << " to " << lon_lat_limit.max() << ".\n";

  ip1_out.clear();
  ip2_out.clear();
  ip1_out.reserve(num_ip);
  ip2_out.reserve(num_ip);

  double angle_tolerance = vw::StereoModel::robust_1_minus_cos
    (stereo_settings().min_triangulation_angle*M_PI/180);
  vw::StereoModel model(left_camera_model, right_camera_model,
                        angle_tolerance);

  bool aligned_ip = (tx_left.get() != NULL && tx_right != NULL);

  for (size_t i = 0; i < num_ip; i++) {
    Vector2 p1 = Vector2(ip1_in[i].x, ip1_in[i].y);
    Vector2 p2 = Vector2(ip2_in[i].x, ip2_in[i].y);

    if (aligned_ip) {
      p1 = tx_left->reverse (p1);
      p2 = tx_right->reverse(p2);
    }

    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(p1, p2, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3())
      continue;

    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    if ((elevation_limit[0] < elevation_limit[1]) &&
         ((llh[2] < elevation_limit[0]) || (llh[2] > elevation_limit[1])))
      continue;

    Vector2 lon_lat = subvector(llh, 0, 2);
    if ((!lon_lat_limit.empty()) && (!lon_lat_limit.contains(lon_lat)))
      continue;

    ip1_out.push_back(ip1_in[i]);
    ip2_out.push_back(ip2_in[i]);
  }
  vw_out() << "\t    * Removed " << ip1_in.size() - ip1_out.size()
           << " ip using elevation/lonlat filtering.\n";

  return ip1_out.size();
}

void filter_ip_using_cameras(std::vector<vw::InterestPoint> & ip1,
                             std::vector<vw::InterestPoint> & ip2,
                             vw::CameraModel const * cam1,
                             vw::CameraModel const * cam2,
                             vw::Datum  const & datum,
                             double pct, double factor) {

  std::vector<double> tri_errors(ip1.size()), heights(ip1.size());

  double angle_tol = vw::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  vw::StereoModel model(cam1, cam2, angle_tol);
  double HIGH_ERROR = std::numeric_limits<double>::max();
  for (size_t i = 0; i < ip1.size(); i++) {
    Vector3 xyz;
    try {
      xyz = model(Vector2(ip1[i].x, ip1[i].y), Vector2(ip2[i].x, ip2[i].y),
                  tri_errors[i]);
    } catch(...) {
      xyz = Vector3();
    }

    if (tri_errors[i] == 0 || xyz == Vector3()) {
      tri_errors[i]  = HIGH_ERROR;
      heights[i] = HIGH_ERROR;
      continue;
    }

    Vector3 llh = datum.cartesian_to_geodetic(xyz);
    heights[i] = llh[2];
  }

  std::vector<double> vals;
  vals.reserve(ip1.size());
  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    vals.push_back(heights[i]);
  }

  double pct_fraction = 1.0 - pct/100.0;
  double b = -1.0, e = -1.0;
  vw::find_outlier_brackets(vals, pct_fraction, factor, b, e);

  int count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR) continue;
    if (heights[i] < b || heights[i] > e) {
      heights[i] = HIGH_ERROR;
      count++;
    }
  }
  vw_out() << "Number (and fraction) of removed outliers by the height check: "
           << count << " (" << double(count)/ip1.size() << ").\n";

  vals.clear();
  for (size_t i = 0; i < ip1.size(); i++) {
    if (tri_errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR)
      continue;
    vals.push_back(tri_errors[i]);
  }

  double pct2 = (75.0/95.0) * pct;
  double pct_fraction2 = 1.0 - pct2/100.0;
  double factor2 = 2.0 * factor;
  b = -1.0; e = -1.0;
  vw::find_outlier_brackets(vals, pct_fraction2, factor2, b, e);

  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (heights[i] >= HIGH_ERROR || tri_errors[i] >= HIGH_ERROR) continue;
    if (tri_errors[i] > e) {
      tri_errors[i] = HIGH_ERROR;
      count++;
    }
  }

  vw_out() << "Number (and fraction) of removed outliers by the triangulation error check: "
           << count << " (" << double(count)/ip1.size() << ").\n";

  count = 0;
  for (size_t i = 0; i < ip1.size(); i++) {
    if (tri_errors[i] >= HIGH_ERROR || heights[i] >= HIGH_ERROR)
      continue;

    ip1[count] = ip1[i];
    ip2[count] = ip2[i];
    count++;
  }

  ip1.resize(count);
  ip2.resize(count);
}

void ip_filter_using_dem(std::string              const & ip_filter_using_dem,
                         vw::TransformPtr                 tx_left,
                         vw::TransformPtr                 tx_right,
                         boost::shared_ptr<vw::CameraModel> left_camera_model,
                         boost::shared_ptr<vw::CameraModel> right_camera_model,
                         std::vector<vw::InterestPoint> & left_aligned_ip,
                         std::vector<vw::InterestPoint> & right_aligned_ip) {

  vw_out() << "Filtering interest point matches with --ip-filter-using-dem.\n";

  std::string dem_file;
  double max_height_diff = -1.0;
  std::istringstream is(ip_filter_using_dem);
  if (!(is >> dem_file >> max_height_diff))
    vw_throw(ArgumentErr() << "Could not parse correctly option --ip-filter-using-dem.\n");

  if (max_height_diff <= 0.0)
    vw_throw(ArgumentErr() << "Positive height diff value expected in --ip-filter-using-dem.\n");

  vw::GeoReference dem_georef;
  DiskImageView<float> dem_disk_image(dem_file);
  ImageViewRef<PixelMask<float>> dem;
  boost::shared_ptr<DiskImageResource> dem_rsrc(DiskImageResourcePtr(dem_file));
  if (dem_rsrc->has_nodata_read())
    dem = create_mask(dem_disk_image, dem_rsrc->nodata_read());
  else
    dem = pixel_cast<PixelMask<float>>(dem_disk_image);

  bool has_georef = vw::read_georeference(dem_georef, dem_file);
  if (!has_georef)
    vw_throw(ArgumentErr() << "There is no georeference information in: "
             << dem_file << ".\n");

  PixelMask<float> nodata_pix(0); nodata_pix.invalidate();
  ValueEdgeExtension<PixelMask<float>> nodata_ext(nodata_pix);

  ImageViewRef<PixelMask<float>> interp_dem
    = interpolate(dem, BilinearInterpolation(), nodata_ext);

  double angle_tol = vw::StereoModel
    ::robust_1_minus_cos(stereo_settings().min_triangulation_angle*M_PI/180);
  vw::StereoModel model(left_camera_model.get(), right_camera_model.get(),
                        angle_tol);

  std::set<int> invalid_indices;
  for (size_t it = 0; it < left_aligned_ip.size(); it++) {

    Vector2 left_pix  = tx_left->reverse (Vector2(left_aligned_ip[it].x,  left_aligned_ip[it].y));
    Vector2 right_pix = tx_right->reverse(Vector2(right_aligned_ip[it].x, right_aligned_ip[it].y));

    double err = -1.0;
    Vector3 xyz(0.0, 0.0, 0.0);
    try {
      xyz = model(left_pix, right_pix, err);
    } catch(...) {
      xyz = Vector3();
    }

    if (err <= 0.0 || xyz == Vector3()) {
      invalid_indices.insert(it);
      continue;
    }

    Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
    Vector2 pix = dem_georef.lonlat_to_pixel(Vector2(llh[0], llh[1]));
    PixelMask<float> dem_val = interp_dem(pix.x(), pix.y());

    if (!is_valid(dem_val) || std::abs(dem_val.child() - llh[2]) > max_height_diff)
      invalid_indices.insert(it);
  }

  int num_total   = left_aligned_ip.size();
  int num_invalid = invalid_indices.size();
  int num_valid   = num_total - num_invalid;
  vw_out() << "Removed " << num_invalid << " interest points out of " << num_total << " ("
           << 100.0 * double(num_invalid) / num_total << " pct).\n";

  int good_it = 0;
  for (int it = 0; it < num_total; it++) {
    if (invalid_indices.find(it) != invalid_indices.end())
      continue;

    left_aligned_ip[good_it]  = left_aligned_ip[it];
    right_aligned_ip[good_it] = right_aligned_ip[it];
    good_it++;
  }

  if (good_it != num_valid)
    vw_throw(ArgumentErr() << "Book-keeping failure in ip filtering using DEM.\n");

  left_aligned_ip.resize(num_valid);
  right_aligned_ip.resize(num_valid);

  return;
}

vw::BBox2 search_range_using_spread(double max_disp_spread,
                                    std::vector<vw::InterestPoint> const& left_ip,
                                    std::vector<vw::InterestPoint> const& right_ip) {
  std::vector<double> dx, dy;

  for (size_t i = 0; i < left_ip.size(); i++) {
    double diffX = right_ip[i].x - left_ip[i].x;
    double diffY = right_ip[i].y - left_ip[i].y;
    dx.push_back(diffX);
    dy.push_back(diffY);
  }
  if (dx.empty())
    vw_throw(vw::ArgumentErr() << "No interest points left.");

  std::sort(dx.begin(), dx.end());
  std::sort(dy.begin(), dy.end());
  double mid_x = dx[dx.size()/2];
  double mid_y = dy[dy.size()/2];

  double half = max_disp_spread / 2.0;
  vw::BBox2 spread_box(mid_x - half, mid_y - half, max_disp_spread, max_disp_spread);

  return spread_box;
}

} // namespace asp

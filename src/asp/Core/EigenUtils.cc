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


/// \file EigenUtils.cc
///

#include <asp/Core/EigenUtils.h>

using namespace vw;
using namespace vw::cartography;

namespace asp {

vw::Vector3 cartesian_to_geodetic_adj(vw::cartography::GeoReference const&
                                      geo, vw::Vector3 xyz){

  // cartesian_to_geodetic() returns longitude between -180 and 180.
  // Sometimes this is 360 degrees less than what desired,
  // so here we do an adjustment.
  // To do: This may not be perfectly fool-proof.
  vw::Vector3 G  = geo.datum().cartesian_to_geodetic(xyz);
  vw::Vector2 ll = geo.pixel_to_lonlat(vw::Vector2(0, 0));
  G(0) += 360*round((ll[0] - G(0))/360.0);
  return G;
}

void null_check(const char* token, std::string const& line){
  if (token == NULL)
    vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
}

// Out of the elements 0, 1,..., n - 1, pick m unique
// random elements and sort them in increasing order.
// TODO(oalexan1): See the newly added pick_random_subset() function in VW,
// but note that that one does not sort the elements.  
void pick_at_most_m_unique_elems_from_n_elems(std::int64_t m, std::int64_t n,
                                              std::vector<std::int64_t>& elems){

  elems.clear();

  if (m < 1 || n < 1) return;
  if (m > n) m = n;

  std::vector<std::int64_t> all(n);
  for (std::int64_t i = 0; i < n; i++) all[i] = i;

  // Swap a randomly selected element from index 0 to j with the one
  // at index j. Then decrement j. Done after m elements are
  // processed.
  for (std::int64_t j = n-1; j >= n-m; j--){
    std::int64_t r = rand()%(j+1); // 0 <= r <= j
    std::swap(all[r], all[j]);
  }

  elems.resize(m);
  for (std::int64_t i = 0; i < m; i++) elems[i] = all[n-m+i];
  std::sort(elems.begin(), elems.end());

}

// Return at most m random points out of the input point cloud.
void random_pc_subsample(std::int64_t m, DoubleMatrix& points){

  std::int64_t n = points.cols();
  std::vector<std::int64_t> elems;
  pick_at_most_m_unique_elems_from_n_elems(m, n, elems);
  m = elems.size();

  for (std::int64_t col = 0; col < m; col++){
    for (std::int64_t row = 0; row < DIM; row++)
      points(row, col) = points(row, elems[col]);
  }
  points.conservativeResize(Eigen::NoChange, m);
}

std::int64_t load_csv_aux(std::string const& file_name, std::int64_t num_points_to_load,
                          vw::BBox2 const& lonlat_box,
                          bool calc_shift, vw::Vector3 & shift,
                          vw::cartography::GeoReference const& geo, CsvConv const& csv_conv,
                          bool & is_lola_rdr_format, double & median_longitude,
                          bool verbose, DoubleMatrix & data) {

  // Note: The input CsvConv object is responsible for parsing out the
  //       type of information contained in the CSV file.

  is_lola_rdr_format = false;

  std::int64_t num_total_points = csv_file_size(file_name);

  std::string sep_str = csv_separator();
  const char* sep = sep_str.c_str();

  const std::int64_t bufSize = 1024;
  char temp[bufSize];
  std::ifstream file( file_name.c_str() );
  if (!file)
    vw_throw(vw::IOErr() << "Unable to open file: " << file_name << "\n");

  // We will randomly pick or not a point with probability load_ratio
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  data.conservativeResize(DIM+1, std::min(num_points_to_load, num_total_points));

  // Peek at the first valid line and see how many elements it has
  std::string line;
  while ( getline(file, line, '\n') ) {
    if (is_valid_csv_line(line))
      break;
  }

  file.clear(); file.seekg(0, std::ios_base::beg); // go back to start of file
  strncpy(temp, line.c_str(), bufSize);
  const char* token = strtok (temp, sep);
  std::int64_t numTokens = 0;
  while (token != NULL){
    numTokens++;
    token = strtok (NULL, sep);
  }
  if (numTokens < 3)
    vw::vw_throw(vw::IOErr() << "Expecting at least three fields on each "
                          << "line of file: " << file_name << "\n");

  // This is a fatal error
  if (!csv_conv.is_configured())
    vw::vw_throw(vw::ArgumentErr() << "The CSV format string is not set.\n");

  // TODO(oalexan1): We parse these guessed file types manually but we should
  // use a CsvConv object to do it!

  if (is_lola_rdr_format && geo.datum().semi_major_axis() != geo.datum().semi_minor_axis() ){
    vw_throw( vw::ArgumentErr() << "The CSV file was detected to be in the"
              << " LOLA RDR format, yet the datum semi-axes are not equal "
              << "as expected for the Moon.\n" );
  }

  bool shift_was_calc = false;
  bool is_first_line  = true;
  std::int64_t points_count = 0;
  std::vector<double> longitudes;
  line = "";
  while (getline(file, line, '\n')) {

    if (!is_first_line && !line.empty() && line[0] == '#' && verbose) {
      vw::vw_out() << "Ignoring line starting with comment: " << line << std::endl;
      continue;
    }
    
    if (points_count >= num_points_to_load)
      break;

    if (!is_valid_csv_line(line))
      continue;

    // Randomly skip a percentage of points
    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > load_ratio)
      continue;

    // We went with C-style file reading instead of C++ in this instance
    // because we found it to be significantly faster on large files.

    vw::Vector3 xyz;
    double lon = 0.0, lat = 0.0;

    if (csv_conv.is_configured()){

      // Parse custom CSV file with given format string
      bool success;
      CsvConv::CsvRecord vals = csv_conv.parse_csv_line(is_first_line, success, line);
      if (!success)
        continue;

      xyz = csv_conv.csv_to_cartesian(vals, geo);

      // Decide if the point is in the box. Also save for the future
      // the longitude of the point, we'll use it to compute the mean longitude.
      vw::Vector2 lonlat = csv_conv.csv_to_lonlat(vals, geo);
      lon = lonlat[0]; // Needed for mean calculation below
      lat = lonlat[1];

      // TODO: We really need a lonlat bbox function that handles wraparound!!!!!!
      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(lonlat)
                              && !lonlat_box.contains(lonlat+vw::Vector2(360,0))
                              && !lonlat_box.contains(lonlat-vw::Vector2(360,0))) {
        continue;
      }

    } else if (!is_lola_rdr_format) {

      // lat,lon,height format
      double height;

      strncpy(temp, line.c_str(), bufSize);
      const char* token = strtok(temp, sep); null_check(token, line);
      std::int64_t ret = sscanf(token, "%lg", &lat);

      token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &lon);

      token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &height);

      // Be prepared for the fact that the first line may be the header.
      if (ret != 3){
        if (!is_first_line){
          vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
        }else{
          is_first_line = false;
          continue;
        }
      }
      is_first_line = false;

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(vw::Vector2(lon, lat)))
        continue;

      vw::Vector3 llh( lon, lat, height );
      xyz = geo.datum().geodetic_to_cartesian( llh );
      if ( xyz == vw::Vector3() || !(xyz == xyz) ) continue; // invalid and NaN check

    }else{

      // Load a RDR_*PointPerRow_csv_table.csv file used for LOLA. Code
      // copied from Ara Nefian's lidar2dem tool.
      // We will ignore lines which do not start with year (or a value that
      // cannot be converted into an integer greater than zero, specifically).

      std::int64_t year, month, day, hour, min;
      double lat, rad, sec, is_invalid;

      strncpy(temp, line.c_str(), bufSize);
      const char* token = strtok(temp, sep); null_check(token, line);

      std::int64_t ret = sscanf(token, "%lld-%lld-%lldT%lld:%lld:%lg", &year, &month, &day, &hour,
                       &min, &sec);
      if( year <= 0 )
        continue;

      token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &lon);

      token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &lat);
      token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &rad);
      rad *= 1000; // km to m

      // Scan 7 more fields, until we get to the is_invalid flag.
      for (std::int64_t i = 0; i < 7; i++)
        token = strtok(NULL, sep); null_check(token, line);
      ret += sscanf(token, "%lg", &is_invalid);

      // Be prepared for the fact that the first line may be the header.
      if (ret != 10){
        if (!is_first_line){
          vw_throw( vw::IOErr() << "Failed to read line: " << line << "\n" );
        }else{
          is_first_line = false;
          continue;
        }
      }
      is_first_line = false;

      if (is_invalid)
        continue;

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(vw::Vector2(lon, lat)))
        continue;

      vw::Vector3 lonlatrad( lon, lat, 0 );

      xyz = geo.datum().geodetic_to_cartesian( lonlatrad );
      if ( xyz == vw::Vector3() || !(xyz == xyz) )
        continue; // invalid and NaN check

      // Adjust the point so that it is at the right distance from
      // planet center.
      xyz = rad*(xyz/norm_2(xyz));
    }

    if (calc_shift && !shift_was_calc){
      shift = xyz;
      shift_was_calc = true;
    }

    for (std::int64_t row = 0; row < DIM; row++)
      data(row, points_count) = xyz[row] - shift[row];
    data(DIM, points_count) = 1;

    points_count++;
    longitudes.push_back(lon);

    // Throw an error if the lon and lat are not within bounds.
    // Note that we allow some slack for lon, perhaps the point
    // cloud is say from 350 to 370 degrees.
    if (std::abs(lat) > 90.0)
      vw_throw(vw::ArgumentErr() << "Invalid latitude value: "
               << lat << " in " << file_name << "\n");
    if (lon < -360.0 || lon > 2*360.0)
      vw_throw(vw::ArgumentErr() << "Invalid longitude value: "
               << lon << " in " << file_name << "\n");
  }
  data.conservativeResize(Eigen::NoChange, points_count);

  median_longitude = 0.0;
  std::sort(longitudes.begin(), longitudes.end());
  if (longitudes.size() > 0) 
    median_longitude = longitudes[longitudes.size()/2];

  return num_total_points;
}

// Load a csv file
void load_csv(std::string const& file_name,
                 std::int64_t num_points_to_load,
                 vw::BBox2 const& lonlat_box,
                 bool calc_shift,
                 vw::Vector3 & shift,
                 vw::cartography::GeoReference const& geo,
                 CsvConv const& csv_conv,
                 bool & is_lola_rdr_format,
                 double & median_longitude,
                 bool verbose,
                 DoubleMatrix & data){

  std::int64_t num_total_points = load_csv_aux(file_name, num_points_to_load,
                                      lonlat_box,
                                      calc_shift, shift,
                                      geo, csv_conv, is_lola_rdr_format,
                                      median_longitude, verbose, data);
  
  std::int64_t num_loaded_points = data.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){
    // We loaded too few points. Just load them all, as CSV files are
    // not too large, we will drop extraneous points later.
    load_csv_aux(file_name, num_total_points, lonlat_box,
                 calc_shift, shift,
                 geo, csv_conv, is_lola_rdr_format,
                 median_longitude,
                 false, // Skip repeating same messages
                 data);
  }
  
  return;
}

// Load a DEM
template<typename DemPixelType>
void load_dem_pixel_type(std::string const& file_name,
                         std::int64_t num_points_to_load, vw::BBox2 const& lonlat_box,
                         bool calc_shift, vw::Vector3 & shift,
                         bool verbose, DoubleMatrix & data){
  
  data.conservativeResize(DIM+1, num_points_to_load);

  vw::cartography::GeoReference dem_geo;
  bool has_georef = vw::cartography::read_georeference( dem_geo, file_name );
  if (!has_georef)
    vw_throw(vw::ArgumentErr() << "DEM: " << file_name
                               << " does not have a georeference.\n");

  vw::DiskImageView<DemPixelType> dem(file_name);
  DemPixelType nodata = std::numeric_limits<DemPixelType>::quiet_NaN();
  {
    boost::shared_ptr<vw::DiskImageResource> dem_rsrc( new vw::DiskImageResourceGDAL(file_name) );
    if (dem_rsrc->has_nodata_read())
      nodata = dem_rsrc->nodata_read();
  }
  
  // Load only points within lonlat_box
  vw::BBox2i pix_box;
  if (!lonlat_box.empty()){
    // Need a lot of catch statements, as lonlat_to_pixel() can throw things
    try { pix_box.grow(dem_geo.lonlat_to_pixel(lonlat_box.min())); } catch(...){}
    try { pix_box.grow(dem_geo.lonlat_to_pixel(lonlat_box.max())); } catch(...){}
    try {
      pix_box.grow(dem_geo.lonlat_to_pixel(vw::Vector2(lonlat_box.min().x(),
                                                       lonlat_box.max().y())));
    }catch(...){}
    try{
      pix_box.grow(dem_geo.lonlat_to_pixel(vw::Vector2(lonlat_box.max().x(),
                                                       lonlat_box.min().y())));
    }catch(...){}
    pix_box.expand(1); // to counteract casting to int
    pix_box.crop(bounding_box(dem));
  }
  if (pix_box.empty())
    pix_box = bounding_box(dem);

  // We will randomly pick or not a point with probability load_ratio.
  // Use int64_t to avoid integer overflow
  std::int64_t num_points = std::int64_t(pix_box.width()) * std::int64_t(pix_box.height());
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  bool shift_was_calc = false;
  std::int64_t points_count = 0;

  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(pix_box.width() );
  if (verbose)
    tpc.report_progress(0);

  for (std::int64_t i = pix_box.min().x(); i < pix_box.max().x(); i++ ) {

    for (std::int64_t j = pix_box.min().y(); j < pix_box.max().y(); j++ ) {
      if (points_count >= num_points_to_load)
        break;

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio)
        continue;

      DemPixelType h = dem(i, j);
      if ( h == nodata || std::isnan(h) || std::isinf(h) )
        continue;
      
      vw::Vector2 lonlat = dem_geo.pixel_to_lonlat( vw::Vector2(i,j) );

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(lonlat))
        continue;

      vw::Vector3 llh( lonlat.x(), lonlat.y(), h );
      vw::Vector3 xyz = dem_geo.datum().geodetic_to_cartesian( llh );
      if ( xyz == vw::Vector3() || !(xyz == xyz) )
        continue; // invalid and NaN check

      if (calc_shift && !shift_was_calc){
        shift = xyz;
        shift_was_calc = true;
      }

      for (std::int64_t row = 0; row < DIM; row++)
        data(row, points_count) = xyz[row] - shift[row];
      data(DIM, points_count) = 1; // Extend to be a homogenous coordinate

      points_count++;
    } // end y loop

    if (verbose)
      tpc.report_incremental_progress( inc_amount );
  } // end x loop
  if (verbose)
    tpc.report_finished();

  data.conservativeResize(Eigen::NoChange, points_count);

}

// Load a DEM
void load_dem(std::string const& file_name,
              std::int64_t num_points_to_load, vw::BBox2 const& lonlat_box,
              bool calc_shift, vw::Vector3 & shift,
              bool verbose, DoubleMatrix & data){

  boost::shared_ptr<vw::DiskImageResource> dem_rsrc( new vw::DiskImageResourceGDAL(file_name) );
  vw::ImageFormat image_fmt = dem_rsrc->format();
  if (image_fmt.channel_type == vw::VW_CHANNEL_FLOAT64) {
    // We could have loaded this DEM as float, the difference in the
    // final transform is very minor if we do so, but it is better to
    // use the full input accuracy.
    load_dem_pixel_type<double>(file_name,  
                                num_points_to_load, lonlat_box,  
                                calc_shift, shift,  
                                verbose, data);
  }else{
    load_dem_pixel_type<float>(file_name,  
                               num_points_to_load, lonlat_box,  
                               calc_shift, shift,  
                               verbose, data);
  }
}

vw::int64 load_pc_aux(std::string const& file_name,
                      std::int64_t num_points_to_load,
                      vw::BBox2 const& lonlat_box,
                      bool calc_shift,
                      vw::Vector3 & shift,
                      vw::cartography::GeoReference const& geo,
                      bool verbose, DoubleMatrix & data){

  data.conservativeResize(DIM+1, num_points_to_load);

  // To do: Is it faster to to do for_each?
  vw::ImageViewRef<vw::Vector3> point_cloud = read_asp_point_cloud<DIM>(file_name);

  // We will randomly pick or not a point with probability load_ratio
  // Use std::int64_t to avoid integer overflow.
  vw::int64 num_total_points = std::int64_t(point_cloud.cols()) * std::int64_t(point_cloud.rows());
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  bool shift_was_calc = false;
  vw::int64 points_count = 0;

  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(point_cloud.rows() );
  if (verbose) tpc.report_progress(0);

  for (std::int64_t j = 0; j < point_cloud.rows(); j++ ) {

    for ( std::int64_t i = 0; i < point_cloud.cols(); i++ ) {

      if (points_count >= num_points_to_load)
        break;

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio)
        continue;

      vw::Vector3 xyz = point_cloud(i, j);
      if ( xyz == vw::Vector3() || !(xyz == xyz) )
        continue; // invalid and NaN check

      if (calc_shift && !shift_was_calc){
        shift = xyz;
        shift_was_calc = true;
      }

      // Skip points outside the given box
      if (!lonlat_box.empty()){
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(xyz);
        if ( !lonlat_box.contains(subvector(llh, 0, 2)))
          continue;
      }

      for (std::int64_t row = 0; row < DIM; row++)
        data(row, points_count) = xyz[row] - shift[row];
      data(DIM, points_count) = 1;

      points_count++;
    }
    if (verbose) tpc.report_incremental_progress( inc_amount );
  }
  if (verbose) tpc.report_finished();

  data.conservativeResize(Eigen::NoChange, points_count);

  return num_total_points;
}

void load_pc(std::string const& file_name,
             std::int64_t num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             bool verbose, DoubleMatrix & data){

  vw::int64 num_total_points = load_pc_aux(file_name, num_points_to_load,
                                          lonlat_box, calc_shift, shift,
                                          geo, verbose, data);

  std::int64_t num_loaded_points = data.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, std::int64_t(10000000));
    if (verbose)
      vw::vw_out() << "Too few points were loaded. Trying again." << std::endl;
    load_pc_aux(file_name, num_points_to_load, lonlat_box,
                calc_shift, shift, geo, verbose, data);
  }

}

// Find the best-fitting plane to a set of points. It will throw an
// error if called with less than 3 points.
void bestFitPlane(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& centroid,
                  Eigen::Vector3d& plane_normal) {
  
  std::int64_t num_points = points.size();
  if (num_points < std::int64_t(3)) 
    vw_throw( ArgumentErr() << "Need 3 points to fit a plane.\n");

  // Copy coordinates to  matrix in Eigen format
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_points);

  for (std::int64_t i = 0; i < num_points; i++) coord.col(i) = points[i];

  // calculate centroid
  centroid = Eigen::Vector3d(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

  // subtract centroid
  for (std::int64_t it = 0; it < 3; it++)
    coord.row(it).array() -= centroid(it);

  // We only need the left-singular matrix here
  // https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

  plane_normal = svd.matrixU().rightCols<1>();
}
  
// Compute a rigid transform between n point correspondences.
// There exists another version of this using vw matrices
// in VisionWorkbench called find_3D_transform().  
void computeRigidTransform(const std::vector<Eigen::Vector3d>& src,
                           const std::vector<Eigen::Vector3d>& dst,
                           Eigen::Matrix3d & rot, Eigen::Vector3d & trans){

  // Initialize the outputs
  rot   = Eigen::Matrix3d::Zero(3, 3);
  trans = Eigen::Vector3d::Zero(3, 1);
    
  assert(src.size() == dst.size());
  std::int64_t pairSize = src.size();
  Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);
  for (std::int64_t i=0; i<pairSize; ++i){
    center_src += src[i];
    center_dst += dst[i];
  }
  center_src /= (double)pairSize;
  center_dst /= (double)pairSize;
  
  Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);
  for (std::int64_t i=0; i<pairSize; ++i){
    for (std::int64_t j=0; j<3; ++j)
      S(i, j) = src[i][j] - center_src[j];
    for (std::int64_t j=0; j<3; ++j)
      D(i, j) = dst[i][j] - center_dst[j];
  }
  Eigen::MatrixXd Dt = D.transpose();
  Eigen::Matrix3d H = Dt*S;
  Eigen::Matrix3d W, U, V;
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  Eigen::MatrixXd H_(3, 3);
  for (std::int64_t i=0; i<3; ++i) for (std::int64_t j=0; j<3; ++j) H_(i, j) = H(i, j);
  svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
  if (!svd.computeU() || !svd.computeV()) {
    // Nothing to do, return the identity transform
    return;
  }

  Eigen::Matrix3d Vt = svd.matrixV().transpose();

  rot   = svd.matrixU()*Vt;
  trans = center_dst - rot*center_src;	
}

/// Read a 4x4 rotation + translation + scale transform from disk.
void read_transform(Eigen::MatrixXd & T, std::string const& transFile){

  T = Eigen::MatrixXd::Zero(4, 4);
    
  vw::vw_out() << "Reading: " << transFile << std::endl;
  std::ifstream is(transFile.c_str());
  for (std::int64_t row = 0; row < T.rows(); row++){
    for (std::int64_t col = 0; col < T.cols(); col++){
      double a;
      if (! (is >> a) )
        vw_throw( vw::IOErr() << "Failed to read initial transform from: "
                  << transFile << "\n" );
      T(row, col) = a;
    }
  }
  
  if (T(3, 3) != 1) {
    vw_throw( vw::ArgumentErr()
              << "The transform must have a 1 in the lower-right corner.\n");
  }
  
}

/// Write a 4x4 rotation + translation + scale transform to disk.
void write_transform(Eigen::MatrixXd const& T, std::string const& transFile){
  if (T(3, 3) != 1) {
    vw_throw( vw::ArgumentErr()
              << "The initial transform must have a 1 in the lower-right corner.\n");
  }

  std::ofstream tf(transFile.c_str());
  tf.precision(17);
  tf << T << "\n";
  tf.close();
}
  
// Apply a rotation + translation transform to a vector3
vw::Vector3 apply_transform_to_vec(Eigen::MatrixXd const& transform,
                                   vw::Vector3 const& p){
  Eigen::Vector4d P;
  for (size_t it = 0; it < 3; it++) P[it] = p[it];
  P[3] = 1;
  P = transform * P;
  vw::Vector3 q;
  for (size_t it = 0; it < 3; it++) q[it] = P[it];
  return q;
}

}

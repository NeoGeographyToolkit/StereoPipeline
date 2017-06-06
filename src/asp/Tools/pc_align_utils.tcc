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

// This tool uses libpointmatcher for alignment,
// https://github.com/ethz-asl/libpointmatcher
// Copyright (c) 2010--2012,
// Francois Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
// You can contact the authors at <f dot pomerleau at gmail dot com> and
// <stephane at magnenat dot net>

// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



/// Analyze a file name to determine the file type
std::string get_file_type(std::string const& file_name){

  if (asp::is_csv(file_name))
    return "CSV";
  if (asp::is_las(file_name))
    return "LAS";

  // Note that any tif, ntf, and cub file with one channel with georeference be
  // interpreted as a DEM.
  int nc = vw::get_num_channels(file_name);

  vw::cartography::GeoReference geo;
  bool has_georef = vw::cartography::read_georeference(geo, file_name);

  if (nc == 1 && has_georef)
    return "DEM";
  if (nc >= 3)
    return "PC";
  vw_throw(vw::ArgumentErr() << "File: " << file_name
                         << " is neither a point cloud nor a DEM.\n");
}



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

template<typename T>
typename PointMatcher<T>::DataPoints::Labels form_labels(int dim){

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;

  Labels labels;
  for (int i=0; i < dim; i++){
    std::string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  return labels;
}

void pick_at_most_m_unique_elems_from_n_elems(int m, int n, std::vector<int>& elems){

  // Out of the elements 0, 1,..., n - 1, pick m unique
  // random elements and sort them in increasing order.

  elems.clear();

  if (m < 1 || n < 1) return;
  if (m > n) m = n;

  std::vector<int> all(n);
  for (int i = 0; i < n; i++) all[i] = i;

  // Swap a randomly selected element from index 0 to j with the one
  // at index j. Then decrement j. Done after m elements are
  // processed.
  for (int j = n-1; j >= n-m; j--){
    int r = rand()%(j+1); // 0 <= r <= j
    std::swap(all[r], all[j]);
  }

  elems.resize(m);
  for (int i = 0; i < m; i++) elems[i] = all[n-m+i];
  std::sort(elems.begin(), elems.end());

}

template<typename T>
void random_pc_subsample(int m, typename PointMatcher<T>::DataPoints& points){

  // Return at most m random points out of the input point cloud.

  int n = points.features.cols();
  std::vector<int> elems;
  pick_at_most_m_unique_elems_from_n_elems(m, n, elems);
  m = elems.size();

  for (int col = 0; col < m; col++){
    for (int row = 0; row < DIM; row++)
      points.features(row, col) = points.features(row, elems[col]);
  }
  points.features.conservativeResize(Eigen::NoChange, m);
}

template<typename T>
int load_csv_aux(std::string const& file_name, int num_points_to_load,
                 vw::BBox2 const& lonlat_box, bool verbose,
                 bool calc_shift, vw::Vector3 & shift,
                 vw::cartography::GeoReference const& geo, asp::CsvConv const& csv_conv,
                 bool & is_lola_rdr_format, double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data){

  // Note: The input CsvConv object is responsible for parsing out the
  //       type of information contained in the CSV file.

  PointMatcherSupport::validateFile(file_name);

  is_lola_rdr_format = false;

  int num_total_points = asp::csv_file_size(file_name);

  std::string sep_str = asp::csv_separator();
  const char* sep = sep_str.c_str();

  const int bufSize = 1024;
  char temp[bufSize];
  std::ifstream file( file_name.c_str() );
  if( !file ) {
    vw_throw( vw::IOErr() << "Unable to open file \"" << file_name << "\"" );
  }

  // We will randomly pick or not a point with probability load_ratio
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  data.features.conservativeResize(DIM+1, std::min(num_points_to_load, num_total_points));
  data.featureLabels = form_labels<T>(DIM);

  // Peek at the first valid line and see how many elements it has
  std::string line;
  while ( getline(file, line, '\n') ) {
    if (asp::is_valid_csv_line(line))
      break;
  }

  file.clear(); file.seekg(0, std::ios_base::beg); // go back to start of file
  strncpy(temp, line.c_str(), bufSize);
  const char* token = strtok (temp, sep);
  int numTokens = 0;
  while (token != NULL){
    numTokens++;
    token = strtok (NULL, sep);
  }
  if (numTokens < 3){
    vw_throw( vw::IOErr() << "Expecting at least three fields on each "
                          << "line of file: " << file_name << "\n" );
  }

  if (!csv_conv.is_configured()){
    if (numTokens > 20){
      is_lola_rdr_format = true;
      if (verbose)
        vw::vw_out() << "Guessing file " << file_name << " to be in LOLA RDR PointPerRow format.\n";
    }else{
      is_lola_rdr_format = false;
      if (verbose)
        vw::vw_out() << "Guessing file " << file_name
                     << " to be in latitude,longitude,height above datum (meters) format.\n";
    }
  }
  // TODO: We parse these guessed file types manually but we should use a CsvConv object to do it!!!!!

  if (is_lola_rdr_format && geo.datum().semi_major_axis() != geo.datum().semi_minor_axis() ){
    vw_throw( vw::ArgumentErr() << "The CSV file was detected to be in the"
              << " LOLA RDR format, yet the datum semi-axes are not equal "
              << "as expected for the Moon.\n" );
  }

  bool shift_was_calc = false;
  bool is_first_line  = true;
  int points_count = 0;
  mean_longitude = 0.0;
  line = "";
  while ( getline(file, line, '\n') ){

    if (!is_first_line && !line.empty() && line[0] == '#') {
      vw::vw_out() << "Ignoring line starting with comment: " << line << std::endl;
      continue;
    }
    
    if (points_count >= num_points_to_load)
      break;

    if (!asp::is_valid_csv_line(line))
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
      asp::CsvConv::CsvRecord vals = csv_conv.parse_csv_line(is_first_line, success, line);
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

    }else if (!is_lola_rdr_format){

      // lat,lon,height format
      double height;

      strncpy(temp, line.c_str(), bufSize);
      const char* token = strtok(temp, sep); null_check(token, line);
      int ret = sscanf(token, "%lg", &lat);

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

      int year, month, day, hour, min;
      double lat, rad, sec, is_invalid;

      strncpy(temp, line.c_str(), bufSize);
      const char* token = strtok(temp, sep); null_check(token, line);

      int ret = sscanf(token, "%d-%d-%dT%d:%d:%lg", &year, &month, &day, &hour,
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
      for (int i = 0; i < 7; i++)
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

    for (int row = 0; row < DIM; row++)
      data.features(row, points_count) = xyz[row] - shift[row];
    data.features(DIM, points_count) = 1;

    points_count++;
    mean_longitude += lon;

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
  data.features.conservativeResize(Eigen::NoChange, points_count);

  mean_longitude /= points_count;

  return num_total_points;
}

// Load a csv file
template<typename T>
void load_csv(std::string const& file_name,
                 int num_points_to_load,
                 vw::BBox2 const& lonlat_box,
                 bool verbose,
                 bool calc_shift,
                 vw::Vector3 & shift,
                 vw::cartography::GeoReference const& geo,
                 asp::CsvConv const& csv_conv,
                 bool & is_lola_rdr_format,
                 double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data){

  int num_total_points = load_csv_aux<T>(file_name, num_points_to_load,
                                         lonlat_box, verbose,
                                         calc_shift, shift,
                                         geo, csv_conv, is_lola_rdr_format,
                                         mean_longitude, data
                                         );

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){
    // We loaded too few points. Just load them all, as CSV files are
    // not too large, we will drop extraneous points later.
    load_csv_aux<T>(file_name, num_total_points, lonlat_box,
                    false, // Skip repeating same messages
                    calc_shift, shift,
                    geo, csv_conv, is_lola_rdr_format,
                    mean_longitude, data
                    );
  }

  return;
}

// Load a DEM
template<typename T, typename DemPixelType>
void load_dem_pixel_type(bool verbose, std::string const& file_name,
                         int num_points_to_load, vw::BBox2 const& lonlat_box,
                         bool calc_shift, vw::Vector3 & shift,
                         typename PointMatcher<T>::DataPoints & data){
  
  PointMatcherSupport::validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

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

  // We will randomly pick or not a point with probability load_ratio
  int    num_points = pix_box.width()*pix_box.height();
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_points);

  bool shift_was_calc = false;
  int  points_count   = 0;

  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(pix_box.width() );
  if (verbose)
    tpc.report_progress(0);

  for (int i = pix_box.min().x(); i < pix_box.max().x(); i++ ) {

    for (int j = pix_box.min().y(); j < pix_box.max().y(); j++ ) {
      if (points_count >= num_points_to_load)
        break;

      double r = (double)std::rand()/(double)RAND_MAX;
      if (r > load_ratio)
        continue;

      if ( dem(i, j) == nodata || std::isnan(dem(i, j)) || std::isinf(dem(i, j)) )
        continue;
      
      vw::Vector2 lonlat = dem_geo.pixel_to_lonlat( vw::Vector2(i,j) );

      // Skip points outside the given box
      if (!lonlat_box.empty() && !lonlat_box.contains(lonlat))
        continue;

      vw::Vector3 llh( lonlat.x(), lonlat.y(), dem(i,j) );
      vw::Vector3 xyz = dem_geo.datum().geodetic_to_cartesian( llh );
      if ( xyz == vw::Vector3() || !(xyz == xyz) )
        continue; // invalid and NaN check

      if (calc_shift && !shift_was_calc){
        shift = xyz;
        shift_was_calc = true;
      }

      for (int row = 0; row < DIM; row++)
        data.features(row, points_count) = xyz[row] - shift[row];
      data.features(DIM, points_count) = 1; // Extend to be a homogenous coordinate

      points_count++;
    } // end y loop

    if (verbose)
      tpc.report_incremental_progress( inc_amount );
  } // end x loop
  if (verbose)
    tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

}


// Load a DEM
template<typename T>
void load_dem(bool verbose, std::string const& file_name,
              int num_points_to_load, vw::BBox2 const& lonlat_box,
              bool calc_shift, vw::Vector3 & shift,
              typename PointMatcher<T>::DataPoints & data){

  PointMatcherSupport::validateFile(file_name);

  boost::shared_ptr<vw::DiskImageResource> dem_rsrc( new vw::DiskImageResourceGDAL(file_name) );
  vw::ImageFormat image_fmt = dem_rsrc->format();
  if (image_fmt.channel_type == vw::VW_CHANNEL_FLOAT64) {
    // We could have loaded this DEM as float, the difference in the
    // final transform is very minor if we do so, but it is better to
    // use the full input accuracy.
    load_dem_pixel_type<T, double>(verbose, file_name,  
                                   num_points_to_load, lonlat_box,  
                                   calc_shift, shift,  
                                   data);
  }else{
    load_dem_pixel_type<T, float>(verbose, file_name,  
                                  num_points_to_load, lonlat_box,  
                                  calc_shift, shift,  
                                  data);
  }
}

template<typename T>
vw::int64 load_pc_aux(bool verbose,
                  std::string const& file_name,
                  int num_points_to_load,
                  vw::BBox2 const& lonlat_box,
                  bool calc_shift,
                  vw::Vector3 & shift,
                  vw::cartography::GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data){

  PointMatcherSupport::validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

  // To do: Is it faster to to do for_each?
  vw::ImageViewRef<vw::Vector3> point_cloud = asp::read_asp_point_cloud<DIM>(file_name);

  // We will randomly pick or not a point with probability load_ratio
  vw::int64 num_total_points = point_cloud.cols()*point_cloud.rows();
  double load_ratio = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  bool shift_was_calc = false;
  vw::int64 points_count = 0;

  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  double inc_amount = 1.0 / double(point_cloud.rows() );
  if (verbose) tpc.report_progress(0);

  for (int j = 0; j < point_cloud.rows(); j++ ) {

    for ( int i = 0; i < point_cloud.cols(); i++ ) {

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

      for (int row = 0; row < DIM; row++)
        data.features(row, points_count) = xyz[row] - shift[row];
      data.features(DIM, points_count) = 1;

      points_count++;
    }
    if (verbose) tpc.report_incremental_progress( inc_amount );
  }
  if (verbose) tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

  return num_total_points;
}

template<typename T>
vw::int64 load_las_aux(bool verbose,
                  std::string const& file_name,
                  int num_points_to_load,
                  vw::BBox2 const& lonlat_box,
                  bool calc_shift,
                  vw::Vector3 & shift,
                  vw::cartography::GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data){

  PointMatcherSupport::validateFile(file_name);

  data.features.conservativeResize(DIM+1, num_points_to_load);
  data.featureLabels = form_labels<T>(DIM);

  vw::cartography::GeoReference las_georef;
  bool has_georef = asp::georef_from_las(file_name, las_georef);
  if (!has_georef)
    vw_throw(vw::ArgumentErr() << "LAS: " << file_name
                               << " does not have a georeference.\n");

  std::ifstream ifs;
  ifs.open(file_name.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  // We will randomly pick or not a point with probability load_ratio
  vw::int64 num_total_points = asp::las_file_size(file_name);
  double load_ratio
    = (double)num_points_to_load/std::max(1.0, (double)num_total_points);

  bool shift_was_calc = false;
  vw::int64 points_count = 0;

  vw::TerminalProgressCallback tpc("asp", "\t--> ");
  int hundred = 100;
  int spacing = num_total_points/hundred;
  double inc_amount = 1.0 / hundred;
  if (verbose) tpc.report_progress(0);

  while (reader.ReadNextPoint()){

    if (points_count >= num_points_to_load)
      break;

    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > load_ratio)
      continue;

    liblas::Point const& p = reader.GetPoint();
    vw::Vector3 xyz(p.GetX(), p.GetY(), p.GetZ());
    if (has_georef){
      vw::Vector2 ll = las_georef.point_to_lonlat(subvector(xyz, 0, 2));
      xyz = las_georef.datum().geodetic_to_cartesian(vw::Vector3(ll[0], ll[1], xyz[2]));
    }

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

    for (int row = 0; row < DIM; row++)
      data.features(row, points_count) = xyz[row] - shift[row];
    data.features(DIM, points_count) = 1;

    if (verbose && points_count%spacing == 0) tpc.report_incremental_progress( inc_amount );

    points_count++;
  }

  if (verbose) tpc.report_finished();

  data.features.conservativeResize(Eigen::NoChange, points_count);

  return num_total_points;
}

template<typename T>
void load_pc(bool verbose,
             std::string const& file_name,
             int num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             ){

  vw::int64 num_total_points = load_pc_aux<T>(verbose,
                                          file_name, num_points_to_load,
                                          lonlat_box, calc_shift, shift,
                                          geo, data);

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, 10000000);
    if (verbose)
      vw::vw_out() << "Too few points were loaded. Trying again." << std::endl;
    load_pc_aux<T>(verbose,
                   file_name, num_points_to_load, lonlat_box,
                   calc_shift, shift, geo, data);
  }

}

template<typename T>
void load_las(bool verbose,
             std::string const& file_name,
             int num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             ){

  vw::int64 num_total_points = load_las_aux<T>(verbose,
                                          file_name, num_points_to_load,
                                          lonlat_box, calc_shift, shift,
                                          geo, data);

  int num_loaded_points = data.features.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, 10000000);
    if (verbose)
      vw::vw_out() << "Too few points were loaded. Trying again." << std::endl;
    load_las_aux<T>(verbose,
                   file_name, num_points_to_load, lonlat_box,
                   calc_shift, shift, geo, data);
  }

}

// Load file from disk and convert to libpointmatcher's format
template<typename T>
void load_file(std::string const& file_name,
               int num_points_to_load,
               vw::BBox2 const& lonlat_box,
               bool calc_shift,
               vw::Vector3 & shift,
               vw::cartography::GeoReference const& geo,
               asp::CsvConv const& csv_conv,
               bool   & is_lola_rdr_format,
               double & mean_longitude,
               bool verbose,
               typename PointMatcher<T>::DataPoints & data){

  if (verbose)
    vw::vw_out() << "Reading: " << file_name << std::endl;

  // We will over-write this below for CSV and DEM files where
  // longitude is available.
  mean_longitude = 0.0;

  std::string file_type = get_file_type(file_name);
  if (file_type == "DEM")
    load_dem<T>(verbose,
                file_name, num_points_to_load, lonlat_box,
                calc_shift, shift, data);
  else if (file_type == "PC")
    load_pc<T>(verbose,
               file_name, num_points_to_load, lonlat_box, calc_shift, shift,
               geo, data);
  else if (file_type == "LAS")
    load_las<T>(verbose,
                file_name, num_points_to_load, lonlat_box, calc_shift, shift,
                geo, data);
  else if (file_type == "CSV"){
    bool verbose = true;
    load_csv<T>(file_name, num_points_to_load, lonlat_box, verbose,
                calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                mean_longitude, data
                );
  }else
    vw_throw( vw::ArgumentErr() << "Unknown file type: " << file_name << "\n" );

  if (verbose)
    vw::vw_out() << "Loaded points: " << data.features.cols() << std::endl;

}

// Calculate the lon-lat bounding box of the points and bias it based
// on max displacement (which is in meters). This is used to throw
// away points in the other cloud which are not within this box.
vw::BBox2 calc_extended_lonlat_bbox(vw::cartography::GeoReference const& geo,
                                int num_sample_pts,
                                asp::CsvConv const& csv_conv,
                                std::string const& file_name,
                                double max_disp){

  // If the user does not want to use the max-displacement parameter,
  // or if there is no datum to use to convert to/from lon/lat,
  // there is not much we can do.
  if (max_disp < 0.0 || geo.datum().name() == UNSPECIFIED_DATUM)
    return vw::BBox2();

  PointMatcherSupport::validateFile(file_name);
  PointMatcher<RealT>::DataPoints points;

  double mean_longitude = 0.0; // to convert back from xyz to lonlat
  bool verbose = false;
  bool calc_shift = false; // won't shift the points
  vw::Vector3 shift = vw::Vector3(0, 0, 0);
  vw::BBox2 dummy_box;
  bool is_lola_rdr_format;
  // Load a sample of points, hopefully enough to estimate the box
  // reliably.
  load_file<RealT>(file_name, num_sample_pts, dummy_box,
                   calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
                   mean_longitude, verbose, points);

  // Bias the xyz points in several directions by max_disp, then
  // convert to lon-lat and grow the box. This is a rough
  // overestimate, but should be good enough.
  vw::BBox2 box;
  for (int col = 0; col < points.features.cols(); col++){
    vw::Vector3 p;
    for (int row = 0; row < DIM; row++) p[row] = points.features(row, col);

    for (int x = -1; x <= 1; x += 2){
      for (int y = -1; y <= 1; y += 2){
        for (int z = -1; z <= 1; z += 2){
          vw::Vector3 q = p + vw::Vector3(x, y, z)*max_disp;
          vw::Vector3 llh = geo.datum().cartesian_to_geodetic(q);
          llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjust
          box.grow(subvector(llh, 0, 2));
        }
      }
    }
  }

  return box;
}

// Sometime the box we computed with cartesian_to_geodetic is offset
// from the box computed with pixel_to_lonlat by 360 degrees.
// Fix that.
void adjust_lonlat_bbox(std::string const& file_name,
                        vw::BBox2 & box){

  using namespace vw;

  // Can only adjust DEM boxes
  if (get_file_type(file_name) != "DEM")
    return;

  cartography::GeoReference georef;
  bool has_georef = cartography::read_georeference(georef, file_name);
  if (!has_georef)
    vw_throw(ArgumentErr() << "DEM: " << file_name << " does not have a georeference.\n");

  DiskImageView<float> dem(file_name);
  BBox2 box2 = georef.pixel_to_lonlat_bbox(bounding_box(dem));

  double mean_lon  = (box.min().x() + box.max().x())/2.0;
  double mean_lon2 = (box2.min().x() + box2.max().x())/2.0;

  double lon_offset = mean_lon2 - mean_lon;
  lon_offset = 360.0*round(lon_offset/360.0);

  box += Vector2(lon_offset, 0);
}

double calc_mean(std::vector<double> const& errs, int len){
  double mean = 0.0;
  for (int i = 0; i < len; i++){
    mean += errs[i];
  }
  if (len == 0) return 0;
  return mean/len;
}

double calc_stddev(std::vector<double> const& errs, double mean){
  double stddev = 0.0;
  int len = errs.size();
  for (int i = 0; i < len; i++){
    stddev += (errs[i] - mean)*(errs[i] - mean);
  }
  if (len == 0) return 0;
  return sqrt(stddev/len);
}

PointMatcher<RealT>::Matrix apply_shift(PointMatcher<RealT>::Matrix const& T,
                                        vw::Vector3 const& shift){

  // Consider a 4x4 matrix T which implements a rotation + translation
  // y = A*x + b. Consider a point s in space close to the points
  // x. We want to make that the new origin, so the points x get
  // closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
  // transform becomes y2 + s = A*(x2 + s) + b, or
  // y2 = A*x2 + b + A*s - s. Encode the obtained transform into another
  // 4x4 matrix T2.

  VW_ASSERT(T.cols() == 4 && T.rows() == 4,
            vw::ArgumentErr() << "Expected square matrix of size 4.");

  Eigen::MatrixXd A = T.block(0, 0, 3, 3);
  Eigen::MatrixXd b = T.block(0, 3, 3, 1);

  Eigen::MatrixXd s = b;
  for (int i = 0; i < 3; i++) s(i, 0) = shift[i];

  Eigen::MatrixXd b2 = b + A*s - s;
  PointMatcher<RealT>::Matrix T2 = T;
  T2.block(0, 3, 3, 1) = b2;

  return T2;
}



void calc_translation_vec(DP const& source, DP const& trans_source,
                          vw::Vector3 & shift, // from planet center to current origin
                          vw::cartography::Datum const& datum,
                          vw::Vector3 & source_ctr_vec,
                          vw::Vector3 & source_ctr_llh,
                          vw::Vector3 & trans_xyz,
                          vw::Vector3 & trans_ned,
                          vw::Vector3 & trans_llh){

  Eigen::VectorXd source_ctr
    = source.features.rowwise().sum() / source.features.cols();
  Eigen::VectorXd trans_source_ctr
    = trans_source.features.rowwise().sum() / trans_source.features.cols();

  vw::Vector3 trans_source_ctr_vec;
  for (int row = 0; row < DIM; row++){
    source_ctr_vec[row]       = source_ctr(row, 0);
    trans_source_ctr_vec[row] = trans_source_ctr(row, 0);
  }

  // Make these vectors in reference to the center of the planet
  source_ctr_vec       += shift;
  trans_source_ctr_vec += shift;

  trans_xyz = trans_source_ctr_vec - source_ctr_vec;

  source_ctr_llh = datum.cartesian_to_geodetic(source_ctr_vec);
  vw::Vector3 trans_source_ctr_llh = datum.cartesian_to_geodetic(trans_source_ctr_vec);
  trans_llh = trans_source_ctr_llh - source_ctr_llh;

  vw::Matrix3x3 M = datum.lonlat_to_ned_matrix(subvector(source_ctr_llh, 0, 2));
  trans_ned = M*trans_xyz;
}

void calc_max_displacment(DP const& source, DP const& trans_source){

  double max_obtained_disp = 0.0;
  int numPts = source.features.cols();
  for(int col = 0; col < numPts; col++){
    vw::Vector3 s, t;
    for (int row = 0; row < DIM; row++){
      s[row] = source.features(row, col);
      t[row] = trans_source.features(row, col);
    }
    max_obtained_disp = std::max(max_obtained_disp, norm_2(s - t));
  }

  vw::vw_out() << "Maximum displacement of source points: "
           << max_obtained_disp << " m" << std::endl;
}


namespace asp{
  /// Apply a transformation matrix to a vw::Vector3 in homogenous coordinates
  vw::Vector3 apply_transform(PointMatcher<RealT>::Matrix const& T, vw::Vector3 const& P){

    Eigen::VectorXd V(4); // Copy our 3D Vector into a homogenous Eigen Vector
    V[0] = P[0];
    V[1] = P[1];
    V[2] = P[2];
    V[3] = 1;
    V = T*V; // Apply the transform to the new vector
    vw::Vector3 Q; // Copy the transformed Eigen vector back to our 3D vector class
    Q[0] = V[0];
    Q[1] = V[1];
    Q[2] = V[2];
    return Q;
  }
}

/// Apply a given transform to the point cloud in input file,
/// and save it.
/// - Note: We transform the entire point cloud, not just the resampled
///         version used in alignment.
void save_trans_point_cloud(vw::cartography::GdalWriteOptions const& opt,
                            std::string input_file,
                            std::string out_prefix,
                            vw::cartography::GeoReference const& geo,
                            asp::CsvConv const& csv_conv,
                            PointMatcher<RealT>::Matrix const& T){

  std::string file_type = get_file_type(input_file);

  std::string output_file;
  if (file_type == "CSV")
    output_file = out_prefix + ".csv";
  else if (file_type == "LAS")
    output_file = out_prefix + boost::filesystem::path(input_file).extension().string();
  else
    output_file = out_prefix + ".tif";
  vw::vw_out() << "Writing: " << output_file << std::endl;

  if (file_type == "DEM"){

    vw::cartography::GeoReference dem_geo;
    bool has_georef = vw::cartography::read_georeference( dem_geo, input_file );
    if (!has_georef) vw_throw(vw::ArgumentErr() << "DEM: " << input_file
                           << " does not have a georeference.\n");

    vw::DiskImageView<float> dem(input_file);
    double nodata = std::numeric_limits<double>::quiet_NaN();
    {
      boost::shared_ptr<vw::DiskImageResource> dem_rsrc
        ( new vw::DiskImageResourceGDAL(input_file) );
      if (dem_rsrc->has_nodata_read()) nodata = dem_rsrc->nodata_read();
    }
    vw::ImageViewRef<vw::Vector3> point_cloud =
      geodetic_to_cartesian( dem_to_geodetic( create_mask(dem, nodata),
                                              dem_geo ),
                             dem_geo.datum() );

    // Save the georeference with the cloud, to help point2dem later
    bool has_nodata2 = false; // the cloud should not use DEM nodata
    vw::cartography::block_write_gdal_image(output_file,
                                per_pixel_filter(point_cloud, TransformPC(T)),
                                has_georef, dem_geo,
                                has_nodata2, nodata,
                                opt, vw::TerminalProgressCallback("asp", "\t--> "));

  }else if (file_type == "PC"){

    // Need this logic because we cannot open an image
    // with n channels without knowing n beforehand.
    int nc = vw::get_num_channels(input_file);
    switch(nc){
    case 3:  save_trans_point_cloud_n<3>(opt, geo, input_file, output_file, T);  break;
    case 4:  save_trans_point_cloud_n<4>(opt, geo, input_file, output_file, T);  break;
    case 6:  save_trans_point_cloud_n<6>(opt, geo, input_file, output_file, T);  break;
    default:
      vw_throw( vw::ArgumentErr() << "The point cloud from " << input_file
                << " has " << nc << " channels, which is not supported.\n" );
    }

  }else if (file_type == "LAS"){

    vw::int64 num_total_points = asp::las_file_size(input_file);
    vw::cartography::GeoReference las_georef;
    bool has_georef = asp::georef_from_las(input_file, las_georef);

    std::ifstream ifs;
    ifs.open(input_file.c_str(), std::ios::in | std::ios::binary);
    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    std::ofstream ofs;
    ofs.open(output_file.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    int hundred = 100;
    int spacing = num_total_points/hundred;
    double inc_amount = 1.0 / hundred;
    vw::int64 count = 0;
    while (reader.ReadNextPoint()){

      liblas::Point const& in_las_pt = reader.GetPoint();
      vw::Vector3 P(in_las_pt.GetX(), in_las_pt.GetY(), in_las_pt.GetZ());
      if (has_georef){
        // Go from projected space to xyz
        vw::Vector2 ll = las_georef.point_to_lonlat(subvector(P, 0, 2));
        P = las_georef.datum().geodetic_to_cartesian(vw::Vector3(ll[0], ll[1], P[2]));
      }
      P = asp::apply_transform(T, P);
      if (has_georef){
        // Go from xyz to projected space
        vw::Vector3 llh = las_georef.datum().cartesian_to_geodetic(P);
        subvector(P, 0, 2) = las_georef.lonlat_to_point(subvector(llh, 0, 2));
        P[2] = llh[2];
      }

      liblas::Point out_las_pt(&header);
      out_las_pt.SetCoordinates(P[0], P[1], P[2]);
      writer.WritePoint(out_las_pt);

      if (count%spacing == 0) tpc.report_incremental_progress( inc_amount );

      count++;
    }
    tpc.report_finished();

  }else if (file_type == "CSV"){

    // Write a CSV file in format consistent with the input CSV file.

    vw::BBox2   empty_box;
    bool        verbose = false;
    bool        calc_shift = true;
    vw::Vector3 shift;
    bool        is_lola_rdr_format;
    double      mean_longitude;
    DP          point_cloud;
    load_csv<RealT>(input_file, std::numeric_limits<int>::max(),
                    empty_box, verbose, calc_shift, shift,
                    geo, csv_conv, is_lola_rdr_format,
                    mean_longitude, point_cloud);

    std::ofstream outfile( output_file.c_str() );
    outfile.precision(16);

    // Write the header line
    if (csv_conv.is_configured()){
      outfile << "# " << csv_conv.write_header_string(",");
      outfile << std::endl;
    }else{
      if (is_lola_rdr_format)
        outfile << "# longitude,latitude,radius (km)" << std::endl;
      else
        outfile << "# latitude,longitude,height above datum (meters)" << std::endl;
    }

    // Save the datum, may be useful to know what it was
    if (geo.datum().name() != UNSPECIFIED_DATUM) {
      outfile << "# " << geo.datum() << std::endl;
      outfile << "# Projection: " << geo.overall_proj4_str() << std::endl;
    }

    int numPts = point_cloud.features.cols();
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    int hundred = 100;
    int spacing = numPts/hundred;
    double inc_amount = 1.0 / hundred;
    for (int col = 0; col < numPts; col++){

      Eigen::VectorXd V(DIM + 1);
      for (int row = 0; row < DIM; row++)
        V[row] = point_cloud.features(row, col) + shift[row];
      V[DIM] = 1;

      // Apply the transform
      V = T*V;

      vw::Vector3 P;
      for (int row = 0; row < DIM; row++) P[row] = V[row];

      if (csv_conv.is_configured()){

        vw::Vector3 csv = csv_conv.cartesian_to_csv(P, geo, mean_longitude);
        outfile << csv[0] << ',' << csv[1] << ',' << csv[2] << std::endl;

      }else{
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
        llh[0] += 360.0*round((mean_longitude - llh[0])/360.0); // 360 deg adjustment

        if (is_lola_rdr_format)
          outfile << llh[0] << ',' << llh[1] << ',' << norm_2(P)/1000.0 << std::endl;
        else
          outfile << llh[1] << ',' << llh[0] << ',' << llh[2] << std::endl;
      }

      if (col%spacing == 0) tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();
    outfile.close();

  }else{
    vw_throw( vw::ArgumentErr() << "Unknown file type: " << input_file << "\n" );
  }
} // end save_trans_point_cloud



InterpolationReadyDem load_interpolation_ready_dem(std::string                  const& dem_path,
                                                   vw::cartography::GeoReference     & georef) {
  // Load the georeference from the DEM
  bool has_georef = vw::cartography::read_georeference( georef, dem_path );
  if (!has_georef)
    vw::vw_throw(vw::ArgumentErr() << "DEM: " << dem_path << " does not have a georeference.\n");

  // Set up file handle to the DEM and read the nodata value
  vw::DiskImageView<float> dem(dem_path);
  double nodata = std::numeric_limits<double>::quiet_NaN();
  {
    boost::shared_ptr<vw::DiskImageResource> dem_rsrc( new vw::DiskImageResourceGDAL(dem_path) );
    if (dem_rsrc->has_nodata_read())
      nodata = dem_rsrc->nodata_read();
  }
  
  // Set up interpolation + mask view of the DEM
  vw::ImageViewRef< vw::PixelMask<float> > masked_dem = create_mask(dem, nodata);
  return InterpolationReadyDem(interpolate(masked_dem));
}


bool interp_dem_height(vw::ImageViewRef< vw::PixelMask<float> > const& dem,
                       vw::cartography::GeoReference const & georef,
                       vw::Vector3                   const & lonlat,
                       double                              & dem_height) {
  // Convert the lon/lat location into a pixel in the DEM.
  vw::Vector2 pix;
  try {
    pix = georef.lonlat_to_pixel(subvector(lonlat, 0, 2));
  }catch(...){
    return false;
  }
  
  double c = pix[0], r = pix[1];

  // Quit if the pixel falls outside the DEM.
  if (c < 0 || c >= dem.cols()-1 || // TODO: This ought to be an image class function
      r < 0 || r >= dem.rows()-1 )
    return false;

  // Interpolate the DEM height at the pixel location
  vw::PixelMask<float> v = dem(c, r);
  if (!is_valid(v))
    return false;

  dem_height = v.child();
  return true;
}

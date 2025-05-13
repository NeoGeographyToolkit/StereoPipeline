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

// This tool uses libpointmatcher for alignment. BSD license.
// https://github.com/ethz-asl/libpointmatcher
// Copyright (c) 2010--2012,
// Francois Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
// You can contact the authors at <f dot pomerleau at gmail dot com> and
// <stephane at magnenat dot net>

#include <asp/PcAlign/pc_align_utils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PdalUtils.h>
#include <asp/Core/PointCloudAlignment.h>

#include <pointmatcher/PointMatcher.h>

namespace asp {

using namespace vw;

void load_las_multi_attempt(std::string const& file_name,
                            std::int64_t num_points_to_load,
                            vw::BBox2 const& lonlat_box,
                            bool calc_shift,
                            vw::Vector3 & shift,
                            vw::cartography::GeoReference const& geo,
                            bool verbose, Eigen::MatrixXd & data){

  std::int64_t num_total_points 
    = load_las(file_name, num_points_to_load, lonlat_box, geo, verbose, calc_shift,
               shift, data); // outputs

  int num_loaded_points = data.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points){

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, std::int64_t(10000000));
    if (verbose)
      vw::vw_out() << "Too few points were loaded. Trying again." << std::endl;
    load_las(file_name, num_points_to_load, lonlat_box, geo, verbose, calc_shift,
             shift, data); // outputs
  }

}

// Load xyz points from disk into a matrix with 4 columns. Last column is just ones.
void load_cloud_as_mat(std::string const& file_name,
                       std::int64_t num_points_to_load,
                       vw::BBox2 const& lonlat_box,
                       bool calc_shift,
                       vw::Vector3 & shift,
                       vw::cartography::GeoReference const& geo,
                       CsvConv const& csv_conv,
                       bool   & is_lola_rdr_format,
                       double & median_longitude,
                       bool verbose,
                       Eigen::MatrixXd & data) {
 
  if (verbose)
    vw::vw_out() << "Reading: " << file_name << std::endl;

  // We will over-write this below for CSV and DEM files where
  // longitude is available.
  median_longitude = 0.0;

  std::string file_type = get_cloud_type(file_name);
  if (file_type == "DEM")
    load_dem(file_name, num_points_to_load, lonlat_box,
	     calc_shift, shift, verbose, data);
  else if (file_type == "PC")
    load_pc(file_name, num_points_to_load, lonlat_box, calc_shift, shift,
	    geo, verbose, data);
  else if (file_type == "LAS")
    load_las_multi_attempt(file_name, num_points_to_load, lonlat_box, calc_shift, shift,
                           geo, verbose, data);
  else if (file_type == "CSV") {
    bool verbose = true;
    load_csv(file_name, num_points_to_load, lonlat_box, 
             calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
             median_longitude, verbose, data);
  } else {
    vw::vw_throw(vw::ArgumentErr() << "Unknown file type: " << file_name << "\n");
  }

  if (data.cols() == 0) 
    vw::vw_throw(vw::ArgumentErr() << "File: " << file_name << " has 0 valid points.\n");
  
  if (verbose)
    vw::vw_out() << "Loaded points: " << data.cols() << std::endl;

  return;
}

// Load xyz points from disk in libpointmatcher's format.
void load_cloud(std::string const& file_name,
               std::int64_t num_points_to_load,
               vw::BBox2 const& lonlat_box,
               bool calc_shift,
               vw::Vector3 & shift,
               vw::cartography::GeoReference const& geo,
               CsvConv const& csv_conv,
               bool   & is_lola_rdr_format,
               double & median_longitude,
               bool verbose,
               typename PointMatcher<double>::DataPoints & data){
  
  data.featureLabels = form_labels<double>(DIM);
  PointMatcherSupport::validateFile(file_name);

  load_cloud_as_mat(file_name, num_points_to_load,  lonlat_box,  calc_shift,
                    shift,  geo,  csv_conv,  is_lola_rdr_format,  median_longitude,
                    verbose,  data.features);
  
}

// Calculate the lon-lat bounding box of the points and bias it based
// on max displacement (which is in meters). This is used to throw
// away points in the other cloud which are not within this box.
// Handle the situation when there is an initial transform applied
// to the source points.
void calc_extended_lonlat_bbox(vw::cartography::GeoReference const& geo,
                               int num_sample_pts,
                               CsvConv const& csv_conv,
                               std::string const& file_name,
                               double max_disp,
                               Eigen::MatrixXd const transform,
                               vw::BBox2 & out_box, 
                               vw::BBox2 & trans_out_box) {

  // Initialize
  out_box       = vw::BBox2();
  trans_out_box = vw::BBox2();
    
  // If the user does not want to use the max-displacement parameter,
  // or if there is no datum to use to convert to/from lon/lat,
  // there is not much we can do.
  if (max_disp < 0.0 || geo.datum().name() == UNSPECIFIED_DATUM)
    return;

  PointMatcherSupport::validateFile(file_name);
  PointMatcher<double>::DataPoints points;

  double      median_longitude = 0.0; // to convert back from xyz to lonlat
  bool        verbose        = false;
  bool        calc_shift     = false; // won't shift the points
  vw::Vector3 shift          = vw::Vector3(0, 0, 0);
  vw::BBox2   dummy_box;
  bool        is_lola_rdr_format;
  // Load a sample of points, hopefully enough to estimate the box reliably.
  load_cloud(file_name, num_sample_pts, dummy_box,
             calc_shift, shift, geo, csv_conv, is_lola_rdr_format,
             median_longitude, verbose, points);

  bool has_transform = (transform != Eigen::MatrixXd::Identity(DIM + 1, DIM + 1));

  // For the first point, figure out how much shift in lonlat a small
  // shift in XYZ produces.  We will use this to expand out from the
  // test points when computing the bounding box.
  vw::Vector3 p1;
  vw::BBox2   box1, box1_trans;
  for (int row = 0; row < DIM; row++)
    p1[row] = points.features(row, 0);

  for (int x = -1; x <= 1; x += 2){
    for (int y = -1; y <= 1; y += 2){
      for (int z = -1; z <= 1; z += 2){
        vw::Vector3 q   = p1 + vw::Vector3(x, y, z)*max_disp;
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(q);
        llh[0] += 360.0*round((median_longitude - llh[0])/360.0); // 360 deg adjust
        box1.grow(subvector(llh, 0, 2));

        // Do the same thing in transformed coordinates
        if (has_transform) {
          vw::Vector3 qT   = apply_transform_to_vec(transform, q);
          vw::Vector3 llhT = geo.datum().cartesian_to_geodetic(qT);
          llhT[0] += 360.0*round((median_longitude - llhT[0])/360.0); // 360 deg adjust
          box1_trans.grow(subvector(llhT, 0, 2));
        }
      }
    }
  }

  const double EXPANSION_MARGIN = 1.05; // Pad the size a little bit just to be safe.
  const double rad_lon  = EXPANSION_MARGIN * box1.width       () / 2.0;
  const double rad_lat  = EXPANSION_MARGIN * box1.height      () / 2.0;
  const double rad_lonT = EXPANSION_MARGIN * box1_trans.width () / 2.0;
  const double rad_latT = EXPANSION_MARGIN * box1_trans.height() / 2.0;

  // Make a box around each point the size of the box we computed earlier and 
  //  keep growing the output bounding box.
  for (int col = 0; col < points.features.cols(); col++){
    vw::Vector3 p;
    for (int row = 0; row < DIM; row++)
      p[row] = points.features(row, col);

    vw::Vector3 q   = p;
    vw::Vector3 llh = geo.datum().cartesian_to_geodetic(q);
    llh[0] += 360.0*round((median_longitude - llh[0])/360.0); // 360 deg adjust
    vw::BBox2 b(llh[0]-rad_lon, llh[1]-rad_lat, rad_lon*2, 2*rad_lat);
    out_box.grow(b);

    // Do the same thing in transformed coordinates
    if (has_transform) {
      vw::Vector3 qT   = apply_transform_to_vec(transform, q);
      vw::Vector3 llhT = geo.datum().cartesian_to_geodetic(qT);
      llhT[0] += 360.0*round((median_longitude - llhT[0])/360.0); // 360 deg adjust
      vw::BBox2 bT(llhT[0]-rad_lonT, llhT[1]-rad_latT, 2*rad_lonT, 2*rad_latT);
      trans_out_box.grow(bT);
    }
  }

  if (!has_transform)
    trans_out_box = out_box;

  return;
}

// Sometime the box we computed with cartesian_to_geodetic is offset
// from the box computed with pixel_to_lonlat by 360 degrees.
// Fix that.
void adjust_lonlat_bbox(std::string const& file_name, vw::BBox2 & box){

  using namespace vw;

  // Can only adjust DEM boxes
  if (get_cloud_type(file_name) != "DEM")
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

// Compute the translation vector from the source points (before any initial alignment
// applied to them), and the source points after alignment.   
void calc_translation_vec(Eigen::MatrixXd const& initT,
                          DP const& source, DP const& trans_source,
                          vw::Vector3 & shift, // from planet center to current origin
                          vw::cartography::Datum const& datum,
                          vw::Vector3 & source_ctr_vec,
                          vw::Vector3 & source_ctr_llh,
                          vw::Vector3 & trans_xyz,
                          vw::Vector3 & trans_ned,
                          vw::Vector3 & trans_llh,
                          vw::Matrix3x3 & NedToEcef){

  // The center of gravity of the source points (after the initial transform is applied to them)
  Eigen::VectorXd source_ctr
    = source.features.rowwise().sum() / source.features.cols();

  // Undo the initial transform, if any 
  Eigen::MatrixXd invInitT = initT.inverse();
  source_ctr = invInitT*source_ctr;

  // The center of gravity of the source points after aligning to the reference cloud
  Eigen::VectorXd trans_source_ctr
    = trans_source.features.rowwise().sum() / trans_source.features.cols();

  // Copy to VW's vectors
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

  // The matrix to go from the NED coordinate system to the ECEF coordinate system
  NedToEcef = datum.lonlat_to_ned_matrix(source_ctr_llh);
  
  trans_ned = inverse(NedToEcef)*trans_xyz;
}

// Calculate the maximum displacement from the source points (after
// any initial transform is applied to them) to the source points
// after alignment with the reference.
double calc_max_displacement(DP const& source, DP const& trans_source){

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

  return max_obtained_disp;
}

/// Save a transformed point cloud with N bands
template<int n> // Number of bands
void save_trans_point_cloud_n(vw::GdalWriteOptions const& opt,
                              vw::cartography::GeoReference const& geo,
                              std::string input_file,
                              std::string output_file,
                              Eigen::MatrixXd const& T){

  // We will try to save the transformed cloud with a georef. Try to get it from
  // the input cloud, or otherwise from the "global" georef.
  vw::cartography::GeoReference curr_geo;
  bool has_georef = vw::cartography::read_georeference(curr_geo, input_file);
  if (!has_georef && geo.datum().name() != UNSPECIFIED_DATUM){
    has_georef = true;
    curr_geo = geo;
  }

  // There is no nodata
  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float

  vw::ImageViewRef<vw::Vector<double, n>> point_cloud = read_asp_point_cloud<n>(input_file);
  vw::cartography::block_write_gdal_image(output_file,
                              per_pixel_filter(point_cloud, TransformPC(T)),
                              has_georef, curr_geo,
                              has_nodata, nodata,
                              opt, vw::TerminalProgressCallback("asp", "\t--> "));
}

/// Apply a given transform to the point cloud in input file,
/// and save it.
/// - Note: We transform the entire point cloud, not just the resampled
///         version used in alignment.
void save_trans_point_cloud(vw::GdalWriteOptions const& opt,
                            std::string input_file,
                            std::string out_prefix,
                            vw::cartography::GeoReference const& geo,
                            CsvConv const& csv_conv,
                            Eigen::MatrixXd const& T){

  std::string file_type = get_cloud_type(input_file);

  std::string output_file;
  if (file_type == "CSV")
    output_file = out_prefix + ".csv";
  else if (file_type == "LAS")
    output_file = out_prefix + boost::filesystem::path(input_file).extension().string();
  else
    output_file = out_prefix + ".tif";
  vw::vw_out() << "Writing: " << output_file << std::endl;

  if (file_type == "DEM") {
    // TODO(oalexan1): This must be a function.
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
      geodetic_to_cartesian(dem_to_geodetic(create_mask(dem, nodata), dem_geo),
                             dem_geo.datum());

    // Save the georeference with the cloud, to help point2dem later
    bool has_nodata2 = false; // the cloud should not use DEM nodata
    vw::cartography::block_write_gdal_image(output_file,
                                per_pixel_filter(point_cloud, TransformPC(T)),
                                has_georef, dem_geo,
                                has_nodata2, nodata,
                                opt, vw::TerminalProgressCallback("asp", "\t--> "));

  }else if (file_type == "PC") {

    // Need this logic because we cannot open an image
    // with n channels without knowing n beforehand.
    // TODO(oalexan1): This must be a function.
    int nc = vw::get_num_channels(input_file);
    switch(nc){
    case 3:  save_trans_point_cloud_n<3>(opt, geo, input_file, output_file, T);  break;
    case 4:  save_trans_point_cloud_n<4>(opt, geo, input_file, output_file, T);  break;
    case 6:  save_trans_point_cloud_n<6>(opt, geo, input_file, output_file, T);  break;
    default:
      vw_throw( vw::ArgumentErr() << "The point cloud from " << input_file
                << " has " << nc << " channels, which is not supported.\n" );
    }

  }else if (file_type == "LAS") {

    asp::apply_transform_to_las(input_file, output_file, T);
    
  }else if (file_type == "CSV") {

    // Write a CSV file in format consistent with the input CSV file.
    // TODO(oalexan1): This must be a function.

    vw::BBox2   empty_box;
    bool        verbose = false;
    bool        calc_shift = true;
    vw::Vector3 shift;
    bool        is_lola_rdr_format;
    double      median_longitude;
    DP          point_cloud;
    load_cloud(input_file, std::numeric_limits<int>::max(),
	       empty_box, calc_shift, shift,
	       geo, csv_conv, is_lola_rdr_format,
	       median_longitude, verbose, point_cloud);

    std::ofstream outfile(output_file.c_str());
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
      outfile << "# Projection: " << geo.get_wkt() << std::endl;
    }

    int numPts = point_cloud.features.cols();
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    int hundred = 100;
    int spacing = std::max(numPts/hundred, 1);
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

        vw::Vector3 csv = csv_conv.cartesian_to_csv(P, geo, median_longitude);
        outfile << csv[0] << ',' << csv[1] << ',' << csv[2] << std::endl;

      }else{
        vw::Vector3 llh = geo.datum().cartesian_to_geodetic(P); // lon-lat-height
        llh[0] += 360.0*round((median_longitude - llh[0])/360.0); // 360 deg adjustment

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

InterpolationReadyDem load_interpolation_ready_dem(std::string const& dem_path,
                                                   vw::cartography::GeoReference & georef) {
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


/// Try to read the georef/datum info, need it to read CSV files.
void read_georef(std::vector<std::string> const& clouds,
                 std::string const& datum_str,
                 std::string const& csv_srs, 
                 double semi_major_axis,
                 double semi_minor_axis,
                 std::string & csv_format_str,
                 asp::CsvConv& csv_conv, vw::cartography::GeoReference& geo) {

  // Use an initialized datum for the georef, so later we can check
  // if we manage to populate it.
  {
    vw::cartography::Datum datum(UNSPECIFIED_DATUM, "User Specified Spheroid",
              "Reference Meridian", 1, 1, 0);
    geo.set_datum(datum);
  }
  
  // First, get the datum from the DEM if available.
  bool is_good = false;
  std::string dem_file = "";
  for (size_t it = 0; it < clouds.size(); it++) {
    if (asp::get_cloud_type(clouds[it]) == "DEM") {
      dem_file = clouds[it];
      break;
    }
  }
  
  if (dem_file != "") {
    vw::cartography::GeoReference local_geo;
    bool have_georef = vw::cartography::read_georeference(local_geo, dem_file);
    if (!have_georef)
      vw::vw_throw(vw::ArgumentErr() << "DEM: " << dem_file << " does not have a georeference.\n");
    geo = local_geo;
    is_good = true;
  }

  // Then, try to set it from the pc file if available.
  // Either one, or both or neither of the pc files may have a georef.
  std::string pc_file = "";
  for (size_t it = 0; it < clouds.size(); it++) {
    if (is_good)
      break;
    if (asp::get_cloud_type(clouds[it]) == "PC") {
      vw::cartography::GeoReference local_geo;
      if (vw::cartography::read_georeference(local_geo, clouds[it])){
        pc_file = clouds[it];
        geo = local_geo;
        vw::vw_out() << "Detected datum from " << pc_file << ":\n" << geo.datum() << std::endl;
        is_good = true;
        break;
      }
    }
  }
  
  // Then, try to set it from the las file if available.
  // Either one, or both or neither of the las files may have a georef.
  std::string las_file = "";
  for (size_t it = 0; it < clouds.size(); it++) {
    if (is_good)
      break;
    if (asp::get_cloud_type(clouds[it]) == "LAS") {
      vw::cartography::GeoReference local_geo;
      if (asp::georef_from_las(clouds[it], local_geo)) {
        las_file = clouds[it];
        geo = local_geo;
        vw::vw_out() << "Detected datum from " << las_file << ":\n" << geo.datum() << std::endl;
        is_good = true;
      }
    }
  }
  
  // We should have read in the datum from an input file, but check to see if
  //  we should override it with input parameters.
  if (datum_str != "") {
    // If the user set the datum, use it.
    vw::cartography::Datum datum;
    datum.set_well_known_datum(datum_str);
    geo.set_datum(datum);
    is_good = true;
  } else if (semi_major_axis > 0 && semi_minor_axis > 0) {
    // Otherwise, if the user set the semi-axes, use that.
    vw::cartography::Datum datum("User Specified Datum", "User Specified Spheroid",
                                 "Reference Meridian",
                                 semi_major_axis, semi_minor_axis, 0.0);
    geo.set_datum(datum);
    is_good = true;
  }

  // This must be the last as it has priority. Add to the georef based on --csv-srs.
  if (csv_conv.parse_georef(geo))
    is_good = true;

  if (is_good)
    vw::vw_out() << "Will use datum (for CSV files): " << geo.datum() << std::endl;

  // A lot of care is needed below.
  if (!is_good  && (csv_format_str == "" || csv_conv.get_format() != asp::CsvConv::XYZ)) {
    // There is no DEM/LAS to read the datum from, and the user either
    // did not specify the CSV format (then we set it to lat, lon,
    // height), or it is specified as containing lat, lon, rather than xyz.
    bool has_csv = false;
    for (size_t it = 0; it < clouds.size(); it++) 
      has_csv = has_csv || ( asp::get_cloud_type(clouds[it]) == "CSV" );
    if (has_csv) {
      // We are in trouble, will not be able to convert input lat, lon, to xyz.
      vw::vw_throw( vw::ArgumentErr() << "Cannot detect the datum. "
                    << "Please specify it via --csv-srs or --datum or "
                    << "--semi-major-axis and --semi-minor-axis.\n" );
    } else {
      // The inputs have no georef. Will have to write xyz.
      vw::vw_out() << "No datum specified. Will write output CSV files "
                   << "in the x,y,z format." << std::endl;
      csv_format_str = "1:x 2:y 3:z";
      csv_conv.parse_csv_format(csv_format_str, csv_srs);
      is_good = true;
    }
  }

  if (!is_good)
    vw::vw_throw(vw::InputErr() << "The datum is required and could not be set.\n");

  return;
}

void extract_rotation_translation(const double * transform, vw::Quat & rotation, 
                                  vw::Vector3 & translation) {
  
  vw::Vector3 axis_angle;
  for (int i = 0; i < 3; i++){
    translation[i] = transform[i];
    axis_angle[i]  = transform[i+3];
  }
  rotation = vw::math::axis_angle_to_quaternion(axis_angle);
}

/// Extracts the full GCC coordinate of a single point from a LibPointMatcher point cloud.
/// - The shift converts from the normalized coordinate to the actual GCC coordinate.
/// - No bounds checking is performed on the point index.
Vector3 get_cloud_gcc_coord(DP const& point_cloud, vw::Vector3 const& shift, int index) {
  Vector3 gcc_coord;
  for (int row = 0; row < DIM; ++row)
     gcc_coord[row] = point_cloud.features(row, index) + shift[row];
  return gcc_coord;
}

bool interp_dem_height(vw::ImageViewRef<vw::PixelMask<float> > const& dem,
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

// Consider a 4x4 matrix T which implements a rotation + translation
// y = A*x + b. Consider a point s in space close to the points
// x. We want to make that the new origin, so the points x get
// closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
// transform becomes y2 + s = A*(x2 + s) + b, or
// y2 = A*x2 + b + A*s - s. Encode the obtained transform into another
// 4x4 matrix T2.
Eigen::MatrixXd apply_shift(Eigen::MatrixXd const& T,
                                        vw::Vector3 const& shift){

  VW_ASSERT(T.cols() == 4 && T.rows() == 4,
            vw::ArgumentErr() << "Expected square matrix of size 4.");

  Eigen::MatrixXd A = T.block(0, 0, 3, 3);
  Eigen::MatrixXd b = T.block(0, 3, 3, 1);

  Eigen::MatrixXd s = b;
  for (int i = 0; i < 3; i++) s(i, 0) = shift[i];

  Eigen::MatrixXd b2 = b + A*s - s;
  Eigen::MatrixXd T2 = T;
  T2.block(0, 3, 3, 1) = b2;

  return T2;
}


} // end namespace asp


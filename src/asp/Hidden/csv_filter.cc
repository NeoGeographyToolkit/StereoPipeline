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

// This tool does one of two things:
// 1. Read a DEM and a CSV file, with the csv format for example, being:
// --csv-format '1:file 2:lon 3:lat 4:radius_km'
// and throw away all CSV points further than a given distance from
// the DEM. Save to disk the obtained points and the x,y grid.
// Matlab can then be used to do natural neighbor interpolation.
// Invoke with --max-height-diff <val>, --projected-point-file,
// and --projected-grid.
// 2. After Matlab did the interpolation, read the results back
// in, as text, and create the interpolated DEM.
// Use with: --interpolated-csv and --interpolated-dem.
// Here is the Matlab script that does natural neighbor:

// P = load('values.txt');

// V = P(:, 3);
// P = P(:, 1:2);
// p='natural';
// %p='linear';

// F = scatteredInterpolant(P,V, p);
// G = load('grid.txt');
// interpV = F(G);
// G = [G interpV]; 
// file = [p '.txt'];
// disp(sprintf('saving file %s', file));
// save(file, 'G', '-ascii', '-double');

// Optionally, grouping the points by track can be done, by parsing
// the LOLA RDR file, but this was not helpful.

// This is work in progess.

#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>

#include <limits>
#include <cstring>
#include <ctime>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

struct Options : public vw::cartography::GdalWriteOptions {
  std::string csv_format_str, datum, csv_proj4_str;
  std::string csv_file, reference_dem;
  double max_height_diff;
  std::string projected_point_file, projected_grid, interpolated_csv, interpolated_dem;
  
  Options():max_height_diff(-1){}

};


void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("csv-format",   po::value(&opt.csv_format_str)->default_value(""), asp::csv_opt_caption().c_str())
    ("datum",        po::value(&opt.datum)->default_value(""),
     "Use this datum for CSV files instead of auto-detecting it. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("csv-proj4",  po::value(&opt.csv_proj4_str)->default_value(""), "The PROJ.4 string to use to interpret the entries in input CSV files.")
    ("max-height-diff",  po::value(&opt.max_height_diff)->default_value(-1), "The maximum absolute height difference between a CSV file entry and the DEM value at that location in order to keep that CSV entry.")
    ("projected-point-file",  po::value(&opt.projected_point_file)->default_value(""), "After reading the CSV file entries and filtering them, write them in projected coordinates (projected x, projected y, height above datum).")
    ("projected-grid",  po::value(&opt.projected_grid)->default_value(""), "After reading the CSV file entries and filtering them, write the x, y grid at which we want these values interpolated using natural neighbor.")
    ("interpolated-csv",  po::value(&opt.interpolated_csv)->default_value(""), "Read from disk the CSV values interpolated on the grid.")
    ("interpolated-dem",  po::value(&opt.interpolated_dem)->default_value(""), "Write the interpolated values as a DEM.");
    
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("csv-file", po::value(&opt.csv_file), "The CSV file to create a DEM from using natural neighbor interpolation.")
    ("reference-dem",  po::value(&opt.reference_dem),  "The reference DEM to use for removing outliers from the CSV file.");

  po::positional_options_description positional_desc;
  positional_desc.add("csv-file",      1);
  positional_desc.add("reference-dem", 1);

  string usage("[options] <csv file> <reference dem>"); 
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if (opt.csv_format_str == "")
    vw_throw(ArgumentErr() << "The CSV format string was not set.\n" << usage << general_options << "\n");

  if (opt.csv_file == "" )
    vw_throw(ArgumentErr() << "The CSV file was not specified.\n" << usage << general_options << "\n");

  if (opt.reference_dem == "" )
    vw_throw(ArgumentErr() << "The reference DEM file was not specified.\n" << usage << general_options << "\n");

  if (opt.max_height_diff <= 0)
    vw_throw(ArgumentErr() << "The value --max-height-diff must be specified and must be positive.\n"
	     << usage << general_options << "\n");

  if ( (opt.projected_point_file == "" || opt.projected_grid == "" )  &&
       (opt.interpolated_csv == "" || opt.interpolated_dem == "" ) )
    vw_throw(ArgumentErr() << "Either both of --projected-point-file and --projected-grid must be specified, when we write these files, or --interpolated-csv and --interpolated-dem must be specified when we read the former and write the latter.\n"
	     << usage << general_options << "\n");
 
}

// Convert 2009-09-21T23:32:28.76345700 to seconds
double parse_time(std::string& time_str){
  int year, month, day, hour, min;
  double sec;

  // Replace all '-' and 'T' and ':' with spaces to scan things easier
  char * str = const_cast<char*>(time_str.c_str());
  for (size_t i = 0; i < time_str.size(); i++){
    if (str[i] == 'T' || str[i] == ':' || str[i] == '-')
      str[i] = ' ';
  }
  
  if (sscanf(time_str.c_str(), "%d %d %d %d %d %lf", &year, &month, &day, &hour, &min, &sec) != 6 )
    vw_throw(ArgumentErr() << "Could not parse time: " << time_str << ".\n");

  struct tm t = {0};  // Initalize to all 0's
  t.tm_year = year - 1900;  // This is year-1900, so 112 = 2012
  t.tm_mon = month - 1; // month must start from 0
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = sec;
  time_t timeSinceEpoch = mktime(&t); // for some reason, the answer I get is off by an hour. Dunno.

  return double(timeSinceEpoch) + (sec - int(sec)); // add back the fractional part
}

  
int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    // Read the DEM georef
    GeoReference csv_georef, dem_georef;
    bool has_georef = read_georeference(dem_georef, opt.reference_dem);
    if (!has_georef) 
      vw_throw(ArgumentErr() << "Canot load a georeference from: " << opt.reference_dem << ".\n");
    
    // Configure a CSV converter object according to the input parameters
    asp::CsvConv csv_conv;
    if (opt.csv_proj4_str == ""){
      // TODO: Need to test this.
      opt.csv_proj4_str = dem_georef.overall_proj4_str();
    }
    csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str); // Modifies csv_conv
    if (!csv_conv.is_configured()) 
      vw_throw(ArgumentErr() << "Could not configure the csv parser.\n");

    // Set the georef for CSV files, if user's csv_proj4_str if 
    // specified
    csv_georef = dem_georef;
    csv_conv.parse_georef(csv_georef);
    if (opt.datum != "")
      csv_georef.set_datum(opt.datum);

    // Read the CSV file
    std::list<asp::CsvConv::CsvRecord> csv_records;
    typedef std::list<asp::CsvConv::CsvRecord>::const_iterator RecordIter;
    csv_conv.read_csv_file(opt.csv_file, csv_records);
    
    std::vector<Vector3> csv_llh;
    std::vector<double> timestamp;
    std::vector<std::string> timestamp_str;
    for (RecordIter iter = csv_records.begin(); iter != csv_records.end(); iter++) {
      Vector3 xyz = csv_conv.csv_to_cartesian(*iter, csv_georef);
      if (xyz == Vector3() || xyz != xyz)
	continue; // invalid point
      Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz);
      csv_llh.push_back(llh);

#if 0
      // Turn off parsing the time column
      std::string time_str = iter->file;
      double val = parse_time(time_str);
      timestamp.push_back(val);
      timestamp_str.push_back(time_str);
#endif
    }
    
    // Read the DEM
    DiskImageView<double> dem(opt.reference_dem);
    
    // Read the no-data
    double dem_nodata = -std::numeric_limits<double>::max();
    {
      // Use a scope to free up fast this handle
      DiskImageResourceGDAL dem_rsrc(opt.reference_dem);
      if (dem_rsrc.has_nodata_read()) {
	dem_nodata = dem_rsrc.nodata_read();
	vw_out() << "\tFound input nodata value for DEM: " << dem_nodata << endl;
      }
    }

    // We will interpolate into the DEM to find the difference
    ImageViewRef< PixelMask<double> > interp_dem
      = interpolate(create_mask(dem, dem_nodata),
		    BicubicInterpolation(), ConstantEdgeExtension());

    // Object used to do geodetic_to_point
    GeodeticToPoint G2P(dem_georef);

    // If two time stamps differ by more than this, we declare them in
    // different groups.
    //double spacing = 60;
    int group_id = -1;
    //int prev_it = -1;
    double group_size = 0;
    double group_sum = 0;
    std::vector<double> means;
    std::vector<std::string> timestamp_str_sub;
    
    std::vector<Vector3> llh_vec;
    std::vector<int> group_ids;
    
    if (opt.projected_point_file != "" && opt.projected_grid != ""){

      // Filter CSV points and save them. Also save the grid for interpolation.
      
      std::ofstream phandle(opt.projected_point_file.c_str());
      phandle.precision(17);

      //std::ofstream bf("tmp_before.txt");
      //bf.precision(18);
      
      vw_out() << "Saving the values in the projected coordiante system to: "
	       << opt.projected_point_file << std::endl;
      for (int it = 0; it < int(csv_llh.size()); it++) {
      
	Vector3 llh = csv_llh[it];
	Vector2 ll  = subvector(llh, 0, 2);
	Vector2 pix = dem_georef.lonlat_to_pixel(ll);
      
	// Check for out of range
	bool out_of_range = (pix[0] < 0 || pix[0] > dem.cols() - 1 ||
			     pix[1] < 0 || pix[1] > dem.rows() - 1);

	if (out_of_range) continue;
	
	PixelMask<double> dem_ht = interp_dem(pix[0], pix[1]);
	if (!is_valid(dem_ht))
	  continue;
	
	double diff = llh[2] - dem_ht.child();
	
	if (std::abs(diff) > opt.max_height_diff)
	  continue;

	Vector3 point = G2P(llh); // geodetic to point
	phandle << point[0] << ' ' << point[1] << ' ' << point[2] << std::endl;

#if 0
	if (prev_it < 0 || std::abs(timestamp[prev_it] - timestamp[it] ) > spacing ) {

	  group_id++;

	  // Save up the data from the current group
	  if (group_size > 0) {
            double mean = group_sum/group_size;
	    means.push_back(mean);
	  }

	  // Prepare for the next group
	  group_size = 1;
	  group_sum = diff;
	  
	}else{
	  group_size++;
	  group_sum += diff;
	}
	prev_it = it;

	llh_vec.push_back(llh);
	group_ids.push_back(group_id);
        timestamp_str_sub.push_back(timestamp_str[it]);
#endif
        
      }
      phandle.close();
      
      // Save up the data from the last group
      if (group_size > 0) {
	means.push_back(group_sum/group_size);
      }
      
      if (group_id + 1 != int(means.size())){
	vw_throw(ArgumentErr() << "Book-keeping error.\n");
      }

#if 0
      // Subtract the mean from each group
      std::ofstream af("tmp_after.txt");
      af.precision(18);
      for (size_t it = 0; it < llh_vec.size(); it++){
	Vector3 llh = llh_vec[it];
	int group_id = group_ids[it];
	if (group_id < 0 || group_id >= int(means.size())){
	  vw_throw(ArgumentErr() << "Book-keeping error.\n");
	}
	double mean = means[group_id];
	//llh[2] -= mean; // temporary!!!
	af << timestamp_str_sub[it]
           << " " << llh[0] << ' ' << llh[1] << ' ' << llh[2]
           << ' ' << group_id << " " << mean << std::endl;
      }
      af.close();
#endif
      
      std::ofstream ghandle(opt.projected_grid.c_str());
      ghandle.precision(17);
      vw_out() << "Saving the grid to interpolate at to: " << opt.projected_grid << std::endl;
      for (int c = 0; c < dem.cols(); c++){
	for (int r = 0; r < dem.rows(); r++){
	  Vector2 lonlat = dem_georef.pixel_to_lonlat(Vector2(c, r));
	  Vector2 pt = dem_georef.lonlat_to_point(lonlat);
	  ghandle << pt[0] << ' ' << pt[1] << std::endl;
	}
      }
      ghandle.close();
      
    }else if (opt.interpolated_csv != "" && opt.interpolated_dem != "" ){
      
      // Read from disk the CSV values interpolated in Matlab. Put the results
      // into a DEM.
      
      std::vector<Vector3> grid_vals;
      std::ifstream ihandle(opt.interpolated_csv.c_str());
      Vector3 P;
      while(ihandle >> P[0] >> P[1] >> P[2])
	grid_vals.push_back(P);
      ihandle.close();

      // Fill these values into the DEM. Note that we create an
      // in-memory DEM.
      // TODO: This DEM may not fit entirely in memory.
      ImageView<double> out_dem = copy(dem);
      
      for (int c = 0; c < dem.cols(); c++){
	for (int r = 0; r < dem.rows(); r++){
	  out_dem(c, r) = dem_nodata;
	}
      }

      for (size_t p = 0; p < grid_vals.size(); p++){
	P = grid_vals[p];
	Vector2 lonlat = dem_georef.point_to_lonlat(subvector(P, 0, 2));
	Vector2 pix = dem_georef.lonlat_to_pixel(lonlat);

	// We are meant to be perfectly on the integer grid.
	Vector2 ipix = round(pix); 
	if (norm_2(pix - ipix) > 1e-1)
	  vw_throw(ArgumentErr() << "Problem: Off the grid: " << pix << ' ' << ipix << "\n");

	// Check for out of range. Must not happen.
	bool out_of_range = (ipix[0] < 0 || ipix[0] > dem.cols() - 1 ||
			     ipix[1] < 0 || ipix[1] > dem.rows() - 1);

	if (out_of_range)
	  vw_throw(ArgumentErr() << "Problem: Out of range: " << ipix << ' '
                   << dem.cols() << ' ' << dem.rows() << "\n");

	out_dem(ipix[0], ipix[1]) = P[2]; // copy the height
      }

      bool has_georef = true, has_nodata = true;
      vw_out() << "Writing: " << opt.interpolated_dem << std::endl;
      TerminalProgressCallback tpc("asp", ": ");
      block_write_gdal_image(opt.interpolated_dem, out_dem, has_georef, dem_georef,
                             has_nodata, dem_nodata, opt, tpc);
    }else
      vw_throw(ArgumentErr() << "Could not do anything useful.\n");

  } ASP_STANDARD_CATCHES;

  return 0;
}

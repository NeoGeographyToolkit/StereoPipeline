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


#include <asp/Core/PointUtils.h>
#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>

using std::endl;
using std::string;

using namespace vw;
using namespace vw::cartography;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Function to convert a masked geodetic vector to a masked altitude vector
class MGeodeticToMAltitude : public ReturnFixedType<PixelMask<double> > {
public:
  PixelMask<double> operator()(PixelMask<Vector3> const& v) const {
    if (!is_valid(v)) {
      return PixelMask<double>();
    }
    return PixelMask<double>(v.child()[2]);
  }
};

struct Options : vw::cartography::GdalWriteOptions {
  string dem1_file, dem2_file, output_prefix, csv_format_str, csv_proj4_str;
  double nodata_value;

  bool use_float, use_absolute;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("nodata-value",    po::value(&opt.nodata_value)->default_value(-32768),      
                        "The no-data value to use, unless present in the DEM geoheaders.")
    ("output-prefix,o", po::value(&opt.output_prefix),                            
                        "Specify the output prefix.")
    ("float",           po::bool_switch(&opt.use_float)->default_value(false),    
                        "Output using float (32 bit) instead of using doubles (64 bit).")
    ("absolute",        po::bool_switch(&opt.use_absolute)->default_value(false), 
     "Output the absolute difference as opposed to just the difference.")
    ("csv-format",     po::value(&opt.csv_format_str)->default_value(""),
     asp::csv_opt_caption().c_str())
    ("csv-proj4",      po::value(&opt.csv_proj4_str)->default_value(""), "The PROJ.4 string to use to interpret the entries in input CSV file. If not specified, it will be borrowed from the DEM.");
  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("dem1", po::value(&opt.dem1_file), "Explicitly specify the first dem")
    ("dem2", po::value(&opt.dem2_file), "Explicitly specify the second dem");

  po::positional_options_description positional_desc;
  positional_desc.add("dem1", 1);
  positional_desc.add("dem2", 1);

  std::string usage("[options] <dem1> <dem2>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (opt.dem1_file.empty() || opt.dem2_file.empty())
    vw_throw(ArgumentErr() << "Requires <dem1> and <dem2> in order to proceed.\n\n"
             << usage << general_options);

  if (opt.output_prefix.empty()) {
    opt.output_prefix = fs::basename(opt.dem1_file) + "__" + fs::basename(opt.dem2_file);
  }

  vw::create_out_dir(opt.output_prefix);
}

void georef_sanity_checks(GeoReference const& georef1, GeoReference const& georef2){

  // We don't support datum changes!
  if (std::abs(georef1.datum().semi_major_axis()
               - georef2.datum().semi_major_axis()) > 0.1 ||
      std::abs(georef1.datum().semi_minor_axis()
               - georef2.datum().semi_minor_axis()) > 0.1 ||
      georef1.datum().meridian_offset() != georef2.datum().meridian_offset()) {
    vw_throw(NoImplErr() << "geodiff can't difference DEMs which have differing "
             << "datum radii or meridian offsets.\n");
  }
  if (georef1.datum().semi_major_axis() == georef2.datum().semi_major_axis() &&
      georef1.datum().semi_minor_axis() == georef2.datum().semi_minor_axis() &&
      georef1.datum().meridian_offset() == georef2.datum().meridian_offset() &&
      georef1.datum().proj4_str()       != georef2.datum().proj4_str()) {
    vw_out(WarningMessage) << "Found DEMs with same datum radii and meridian offsets but "
                           << "different projection strings. Use some caution.\n";
  }
  
}

void dem2dem_diff(Options& opt){
  
  DiskImageResourceGDAL dem1_rsrc(opt.dem1_file), dem2_rsrc(opt.dem2_file);
  double dem1_nodata = opt.nodata_value, dem2_nodata = opt.nodata_value;
  if (dem1_rsrc.has_nodata_read()) {
    dem1_nodata = dem1_rsrc.nodata_read();
    opt.nodata_value = dem1_nodata;
    vw_out() << "\tFound input nodata value for DEM 1: " << dem1_nodata << endl;
    vw_out() << "Using this nodata value on output.\n";
  }
  if (dem2_rsrc.has_nodata_read()) {
    dem2_nodata = dem2_rsrc.nodata_read();
    vw_out() << "\tFound input nodata value for DEM 2: " << dem2_nodata << endl;
  }

  DiskImageView<double> dem1_disk_image_view(dem1_rsrc), dem2_disk_image_view(dem2_rsrc);

  GeoReference dem1_georef, dem2_georef;
  bool has_georef1 = read_georeference(dem1_georef, dem1_rsrc);
  bool has_georef2 = read_georeference(dem2_georef, dem2_rsrc);
  if (!has_georef1 || !has_georef2) 
    vw_throw(ArgumentErr() << "geodiff cannot difference files without a georeference.\n");
  
  georef_sanity_checks(dem1_georef, dem2_georef);

  // Generate a bounding box that is the minimum of the two BBox areas
  BBox2 crop_box = bounding_box(dem1_disk_image_view);

  // Transform the second DEM's bounding box to first DEM's pixels
  GeoTransform gt(dem2_georef, dem1_georef);
  BBox2 box21 = gt.forward_bbox(bounding_box(dem2_disk_image_view));
  crop_box.crop(box21);

  if (crop_box.empty()) 
    vw_throw(ArgumentErr() << "The two DEMs do not have a common area.\n");
    
  ImageViewRef<PixelMask<double> > dem2_trans =
    crop(geo_transform
	 (per_pixel_filter(dem_to_geodetic
			   (create_mask(dem2_disk_image_view, dem2_nodata),
			    dem2_georef),
			   MGeodeticToMAltitude()),
	  dem2_georef, dem1_georef,
      ValueEdgeExtension<PixelMask<double> >(PixelMask<double>())),
     crop_box);
    
  ImageViewRef<double> difference;
  if (opt.use_absolute) {
    difference =
      apply_mask(abs(crop(create_mask(dem1_disk_image_view, dem1_nodata), crop_box) - dem2_trans),
                 opt.nodata_value);
  } else {
    difference =
      apply_mask(crop(create_mask(dem1_disk_image_view, dem1_nodata), crop_box) - dem2_trans,
                 opt.nodata_value);
  }
    
  GeoReference crop_georef = crop(dem1_georef, crop_box);
    
  std::string output_file = opt.output_prefix + "-diff.tif";
  vw_out() << "Writing difference file: " << output_file << "\n";
    
  if (opt.use_float) {
    ImageViewRef<float> difference_float = channel_cast<float>(difference);
    boost::scoped_ptr<DiskImageResourceGDAL>
      rsrc(vw::cartography::build_gdal_rsrc(output_file,
                                            difference_float, opt));
    rsrc->set_nodata_write(opt.nodata_value);
    write_georeference(*rsrc, crop_georef);
    block_write_image(*rsrc, difference_float,
                      TerminalProgressCallback("asp", "\t--> Differencing: "));
  } else {
    boost::scoped_ptr<DiskImageResourceGDAL>
      rsrc(vw::cartography::build_gdal_rsrc(output_file,
                                            difference, opt));
    rsrc->set_nodata_write(opt.nodata_value);
    write_georeference(*rsrc, crop_georef);
    block_write_image(*rsrc, difference,
                      TerminalProgressCallback("asp", "\t--> Differencing: "));
  }
}

// From a DEM, subtract a csv file. Reverse the sign is 'reverse' is true.
void dem2csv_diff(Options & opt, std::string const& dem_file,
                  std::string const & csv_file, bool reverse){
  
  if (opt.csv_format_str == "")
    vw_throw(ArgumentErr() << "CSV files were passed in, but the "
             << "CSV format string was not set.\n");

  // Read the DEM
  DiskImageView<double> dem(dem_file);

  // Read the no-data
  double dem_nodata = opt.nodata_value;
  {
    // Use a scope to free up fast this handle
    DiskImageResourceGDAL dem_rsrc(dem_file);
    if (dem_rsrc.has_nodata_read()) {
      dem_nodata = dem_rsrc.nodata_read();
      opt.nodata_value = dem_nodata;
      vw_out() << "\tFound input nodata value for DEM: " << dem_nodata << endl;
    }
  }
  
  // Read the DEM georef
  GeoReference dem_georef;
  bool has_georef = read_georeference(dem_georef, dem_file);
  if (!has_georef) 
    vw_throw(ArgumentErr() << "geodiff cannot load a georeference from: " << dem_file << ".\n");

  if (opt.csv_proj4_str == "") {
    // Copy from the DEM
    opt.csv_proj4_str = dem_georef.overall_proj4_str();
  }
  
  // Configure a CSV converter object according to the input parameters
  asp::CsvConv csv_conv;
  csv_conv.parse_csv_format(opt.csv_format_str, opt.csv_proj4_str); // Modifies csv_conv
  if (!csv_conv.is_configured()) 
    vw_throw(ArgumentErr() << "Could not configure the csv parser.\n");

  // Set the georef for CSV files
  GeoReference csv_georef = dem_georef;
  csv_conv.parse_georef(csv_georef);

  std::list<asp::CsvConv::CsvRecord> csv_records;
  typedef std::list<asp::CsvConv::CsvRecord>::const_iterator RecordIter;
  csv_conv.read_csv_file(csv_file, csv_records);
  
  std::vector<Vector3> csv_llh;
  for (RecordIter iter = csv_records.begin(); iter != csv_records.end(); iter++) {
    Vector3 xyz = csv_conv.csv_to_cartesian(*iter, csv_georef);
    if (xyz == Vector3() || xyz != xyz)
      continue; // invalid point
    Vector3 llh = dem_georef.datum().cartesian_to_geodetic(xyz); // use the dem's datum
    csv_llh.push_back(llh);
  }

  // We will interpolate into the DEM to find the difference
  ImageViewRef< PixelMask<double> > interp_dem
    = interpolate(create_mask(dem, dem_nodata),
		  BilinearInterpolation(), ConstantEdgeExtension());


  // Save the diffs
  int    count     = 0;
  double diff_min  = std::numeric_limits<double>::max();
  double diff_max  = -diff_min;
  double diff_mean = 0.0;
  double diff_std  = 0.0;
  
  std::vector<Vector3> csv_diff;
  for (size_t it = 0; it < csv_llh.size(); it++) {

    Vector3 llh = csv_llh[it];
    Vector2 ll  = subvector(llh, 0, 2);
    Vector2 pix = dem_georef.lonlat_to_pixel(ll);
    
    // Check for out of range
    if (pix[0] < 0 || pix[0] > dem.cols() - 1) continue;
    if (pix[1] < 0 || pix[1] > dem.rows() - 1) continue;
    PixelMask<double> dem_ht = interp_dem(pix[0], pix[1]);
    if (!is_valid(dem_ht))
      continue;

    double diff = dem_ht.child() - llh[2];
    if (reverse) 
      diff *= -1;
    if (opt.use_absolute)
      diff = std::abs(diff);

    if (diff > diff_max) diff_max = diff;
    if (diff < diff_min) diff_min = diff;

    diff_mean += diff;
    diff_std  += diff*diff;
    count     += 1;
    csv_diff.push_back(Vector3(ll[0], ll[1], diff));
  }

  if (count > 0) {
    diff_mean /= count;
    diff_std = diff_std/count - diff_mean*diff_mean;
    if (diff_std < 0)
      diff_std = 0; // just in case, for numerical noise
    diff_std = std::sqrt(diff_std);
  }

  vw_out() << "Max difference:       " << diff_max  << std::endl;
  vw_out() << "Min difference:       " << diff_min  << std::endl;
  vw_out() << "Mean difference:      " << diff_mean << std::endl;
  vw_out() << "StdDev of difference: " << diff_std  << std::endl;

  std::string output_file = opt.output_prefix + "-diff.csv";
  vw_out() << "Writing difference file: " << output_file << "\n";
  std::ofstream outfile( output_file.c_str() );
  outfile.precision(16);
  outfile << "# longitude,latitude, height diff (m)" << std::endl;
  outfile << "# " << dem_georef.datum() << std::endl; // dem's datum
  outfile << "# Max difference:       " << diff_max  << std::endl;
  outfile << "# Min difference:       " << diff_min  << std::endl;
  outfile << "# Mean difference:      " << diff_mean << std::endl;
  outfile << "# StdDev of difference: " << diff_std  << std::endl;
  for (size_t it = 0; it < csv_diff.size(); it++) {
    Vector3 diff = csv_diff[it];
    outfile << diff[0] << "," << diff[1] << "," << diff[2] << std::endl;
  }
}

// Subtract from the first dem the second. One of them can be a CSV file.
int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    bool is_dem1_csv = asp::is_csv(opt.dem1_file);
    bool is_dem2_csv = asp::is_csv(opt.dem2_file);

    if (is_dem1_csv && is_dem2_csv) 
      vw_throw(ArgumentErr()
               << "Cannot do the diff of two csv files. One of them "
               << "can be converted to a DEM using point2dem fist.\n");
    
    bool reverse = false; // true if first DEM is a csv
    if (is_dem1_csv) {
      reverse = true;
      dem2csv_diff(opt, opt.dem2_file, opt.dem1_file, reverse);
    }else if (is_dem2_csv){
      reverse = false;
      dem2csv_diff(opt, opt.dem1_file, opt.dem2_file, reverse);
    }else{
      // Both are regular DEMs
      dem2dem_diff(opt);
    }

  } ASP_STANDARD_CATCHES;
  
  return 0;
}

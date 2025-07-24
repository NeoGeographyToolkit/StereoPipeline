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

// Convert an ASP point cloud to LAS 1.2 format using PDAL.

/// \file point2las.cc
///

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PdalUtils.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Statistics.h>
#include <vw/FileIO/FileUtils.h>
#include <vw/Cartography/PointImageManipulation.h>

#include <fstream>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>

using namespace vw;
namespace po = boost::program_options;

// A class to collect some positive errors, and return the error at
// given percentile multiplied by given factor.
class PercentileErrorAccum : public ReturnFixedType<void> {
  typedef double accum_type;
  std::vector<accum_type> m_vals;
public:
  typedef accum_type value_type;
  
  PercentileErrorAccum() { m_vals.clear(); }
  
  void operator()( accum_type const& value ) {
    // Don't add zero errors, those most likely came from invalid points
    if (value > 0)
      m_vals.push_back(value);
  }
  
  int size() {
    return m_vals.size();
  }
  
  value_type value(Vector2 const& outlier_removal_params, bool use_tukey_outlier_removal){

    // Care here with empty sets
    if (m_vals.empty()) {
      vw_out() << "Found no positive triangulation errors in the sample.\n";
      return 0.0;
    }
    
    std::sort(m_vals.begin(), m_vals.end());
    int len = m_vals.size();
    vw_out() << "Collected a sample of " << len << " positive triangulation errors.\n";

    double mean = vw::math::mean(m_vals);
    vw_out() << "For this sample: "
             << "min = "     << m_vals.front()
             << ", mean = "  << mean
             << ", stdev = " << vw::math::standard_deviation(m_vals, mean)
             << ", max = " << m_vals.back() << "." << std::endl;

    int i25 = round((len - 1) * 0.25);
    int i50 = round((len - 1) * 0.50);
    int i75 = round((len - 1) * 0.75);

    double Q1 = m_vals[i25];
    double Q2 = m_vals[i50];
    double Q3 = m_vals[i75];
    vw_out() << "Error percentiles: " 
             << "Q1 (25%): " << Q1 << ", "
             << "Q2 (50%): " << Q2 << ", "
             << "Q3 (75%): " << Q3 << "."
             << std::endl;

    if (use_tukey_outlier_removal) {
      vw_out() << "Using as outlier cutoff the Tukey formula Q3 + 1.5*(Q3 - Q1)." << std::endl;
      return Q3 + 1.5*(Q3 - Q1);
    }
    
    double pct    = outlier_removal_params[0]/100.0; // e.g., 0.75
    double factor = outlier_removal_params[1];
    int k         = (int)round((len - 1) * pct);
    
    vw_out() << "Using as outlier cutoff the " << outlier_removal_params[0] << " percentile times "
             << factor << "." << std::endl;
    
    return m_vals[k] * factor;
  }

};

struct Options: vw::GdalWriteOptions {
  // Input
  std::string reference_spheroid, datum;
  std::string pointcloud_file, intensity_file;
  std::string target_srs_string;
  bool        compressed, use_tukey_outlier_removal, ecef, no_input_georef;
  Vector2     outlier_removal_params;
  double      max_valid_triangulation_error;
  int         num_samples;
  bool save_triangulation_error, save_stddev, dem;
  
  // Output
  std::string out_prefix;
  Options() : compressed(false), max_valid_triangulation_error(0.0), num_samples(0) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("compressed,c", 
     po::bool_switch(&opt.compressed)->default_value(false)->implicit_value(true),
     "Compress using laszip.")
    ("output-prefix,o", 
     po::value(&opt.out_prefix), "Specify the output prefix.")
    ("datum", po::value(&opt.datum)->default_value(""),
     "Create a geo-referenced LAS file in respect to this datum. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("reference-spheroid,r", 
     po::value(&opt.reference_spheroid)->default_value(""),
     "This is identical to the datum option.")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""),
     "Specify the output projection as a GDAL projection string (WKT, GeoJSON, or PROJ). If not provided, will be read from the point cloud, if available.")
    ("remove-outliers-params", 
     po::value(&opt.outlier_removal_params)->default_value(Vector2(75.0, 3.0), "pct factor"),
     "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("use-tukey-outlier-removal", 
     po::bool_switch(&opt.use_tukey_outlier_removal)->default_value(false)->implicit_value(true),
     "Remove outliers above Q3 + 1.5*(Q3 - Q1). Takes precedence over "
     "--remove-outliers-params.")
    ("max-valid-triangulation-error", 
     po::value(&opt.max_valid_triangulation_error)->default_value(0.0),
     "Outlier removal based on threshold. Points with triangulation error larger than this, if positive (measured in meters) will be removed from the cloud. Takes precedence over the above methods.")
    ("save-intensity-from-image", po::value(&opt.intensity_file)->default_value(""),
     "Save the intensity of each triangulated point, as borrowed from the aligned left "
     "image L.tif specified via this option, in the W field of the LAS file, in double "
     "precision. This bumps the LAS file version from 1.2 to 1.4.")
    ("save-triangulation-error", 
     po::bool_switch(&opt.save_triangulation_error)->default_value(false),
     "Save the triangulation error from the input point cloud as the TextureU field "
     "in the LAS file, in double precision. Take into account the outlier filtering."
     "This bumps the LAS file version from 1.2 to 1.4")
    ("save-stddev", 
      po::bool_switch(&opt.save_stddev)->default_value(false),
      "Save the standard deviations of the horizontal and vertical components of uncertainty "
      "from the ASP point cloud file to the TextureV and TextureW fields in the LAS file, "
      "in double precision. This bumps the LAS file version from 1.2 to 1.4.")
    ("num-samples-for-outlier-estimation", 
     po::value(&opt.num_samples)->default_value(1000000),
     "Approximate number of samples to pick from the input cloud to find the outlier cutoff based on triangulation error.")
    ("ecef", 
     po::bool_switch(&opt.ecef)->default_value(false)->implicit_value(true),
     "Save the point cloud in ECEF, rather than with a projection relative to a datum.")
    ("dem", 
      po::bool_switch(&opt.dem)->default_value(false)->implicit_value(true),
      "Assume the input file is a DEM.")
    ("no-input-georef",
      po::bool_switch(&opt.no_input_georef)->default_value(false)->implicit_value(true),
      "Do not attempt to read the georeference from the input point cloud.")
    ("help,h", "Display this help message");
  
  general_options.add( vw::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_file), "Input point cloud");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("[options] <point-cloud>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.pointcloud_file.empty())
    vw_throw(ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options);

  if (opt.out_prefix.empty())
    opt.out_prefix =
      vw::prefix_from_filename(opt.pointcloud_file);

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr() 
             << "The value of --t_srs is empty. Then it must not be set at all.\n");

  // reference_spheroid and datum are aliases
  boost::to_lower(opt.reference_spheroid);
  boost::to_lower(opt.datum);
  if (opt.datum != "" && opt.reference_spheroid != "")
    vw_throw( ArgumentErr() << "Both --datum and --reference-spheroid were specified.\n");
  if (opt.datum == "")
    opt.datum = opt.reference_spheroid;

  double pct = opt.outlier_removal_params[0], factor = opt.outlier_removal_params[1];
  if (pct <= 0.0 || pct > 100.0 || factor <= 0.0)
    vw_throw( ArgumentErr() 
             << "Invalid values were provided for outlier removal parameters.\n");

  if (opt.max_valid_triangulation_error < 0.0) 
    vw_throw( ArgumentErr() 
             << "The maximum valid triangulation error must be non-negative.\n");

  if (opt.num_samples <= 0) 
    vw_throw( ArgumentErr() << "Must pick a positive number of samples.\n");

  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
}

void find_error_image_and_do_stats(Options& opt, ImageViewRef<double> & error_image) {
      
  std::vector<std::string> pointcloud_files;
  pointcloud_files.push_back(opt.pointcloud_file);
  error_image = asp::point_cloud_error_image(pointcloud_files);
  
  if (error_image.rows() == 0 || error_image.cols() == 0) {
    vw_out() << "The point cloud files must have an equal number of channels which "
             << "must be 4 or 6 to be able to remove outliers.\n";
    opt.max_valid_triangulation_error = 0.0;
    return;
  }

  if (opt.max_valid_triangulation_error > 0.0) {
    vw_out() << "Using the set maximum valid triangulation error as outlier cutoff: "
             << opt.max_valid_triangulation_error << "." << std::endl;
    return;
  }
    
  vw_out() << "Estimating the maximum valid triangulation error (outlier cutoff).\n";
    
  int num_err_cols = error_image.cols();
  int num_err_rows = error_image.rows();
    
  double area = std::max(num_err_cols * num_err_rows, 1);
  int sample_rate = round(sqrt(double(area) / double(opt.num_samples)));
  if (sample_rate < 1) 
    sample_rate = 1;
    
  Stopwatch sw;
  sw.start();
  PixelAccumulator<PercentileErrorAccum> error_accum;
  for_each_pixel(subsample(error_image, sample_rate),
                 error_accum,
                 TerminalProgressCallback
                 ("asp", "Error estim: "));

  opt.max_valid_triangulation_error = error_accum.value(opt.outlier_removal_params,
                                                        opt.use_tukey_outlier_removal);
  
  sw.stop();
  vw_out(DebugMessage, "asp") << "Elapsed time: " << sw.elapsed_seconds() << std::endl;
  vw_out() << "Found the maximum valid triangulation error (outlier cutoff): "
           << opt.max_valid_triangulation_error << "." << std::endl;
}

// Read a DEM and convert it to having projected points and elevation
void read_dem(std::string const& pointcloud_file, 
              bool ecef, bool no_input_georef,
              bool & has_georef,
              cartography::GeoReference & georef,
              ImageViewRef<Vector3> & point_image) {

  has_georef = vw::cartography::read_georeference(georef, pointcloud_file);
  
  // Sanity checks
  if (!has_georef)
    vw::vw_throw(vw::ArgumentErr() 
              << "The input DEM file does not have a georeference.\n");
  if (ecef)
    vw::vw_throw(vw::ArgumentErr() 
              << "Options --ecef and --dem are not compatible.\n");
    if (no_input_georef) 
    vw::vw_throw(vw::ArgumentErr() 
              << "Options --no-input-georef and --dem are not compatible.\n");
    
  double nodata_val = -std::numeric_limits<double>::max();
  vw::read_nodata_val(pointcloud_file, nodata_val);
  
  // Form the point image as projected coordinates and elevation
  vw::DiskImageView<double> dem(pointcloud_file);
  point_image = vw::cartography::dem_to_proj(vw::create_mask(dem, nodata_val), georef);
}

int main(int argc, char *argv[]) {
  
  // TODO(oalexan1): need to understand what is the optimal strategy
  // for traversing the input point cloud file to minimize the reading
  // time.

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // // See if to the points relative to a georeference
    // cartography::Datum datum;
    // cartography::GeoReference georef;
    // bool have_user_datum = false;
    // bool has_georef = false; // Will not have a georef when writing XYZ in ECEF

    cartography::Datum datum;
    cartography::GeoReference georef;
    bool have_user_datum = false, have_input_georef = false;
    bool have_out_georef = false;
    ImageViewRef<Vector3> point_image;
    
    if (!opt.dem) {
      if (!opt.ecef) {
        have_user_datum = asp::read_user_datum(0, 0, opt.datum, datum);
        if (!opt.no_input_georef)
          have_input_georef = vw::cartography::read_georeference(georef, opt.pointcloud_file);
        if (have_input_georef && opt.target_srs_string.empty())
          opt.target_srs_string = georef.get_wkt();
      
        if (have_user_datum || !opt.target_srs_string.empty()) {
          // Set the srs string into georef
          asp::set_srs_string(opt.target_srs_string,
                              have_user_datum, datum, georef);
          have_out_georef = true;
          datum = georef.datum();
        }
      }

      // Save the las file with given georeference, if present
      point_image = asp::read_asp_point_cloud<3>(opt.pointcloud_file);
      
      if (have_out_georef) {
        // See if to use [-180, 180] or [0, 360]
        vw::BBox2 lonlat_box = asp::estim_lonlat_box(point_image, georef.datum());
        georef.set_image_ll_box(lonlat_box);
        // Convert cartesian to projected coordinates via geodetic
        point_image = cartesian_to_geodetic(point_image, datum);
        point_image = geodetic_to_point(point_image, georef);
      }
    } else {
      // The input is a DEM. Resulting point_image will be in projected_coordinates.
      read_dem(opt.pointcloud_file, opt.ecef, opt.no_input_georef,
              have_out_georef, georef, point_image);
      datum = georef.datum();
      have_out_georef = true; 
    }
    
    // The error image, if provided
    ImageViewRef<double> error_image;
    if (opt.outlier_removal_params[0] < 100.0 || opt.max_valid_triangulation_error > 0.0)
      find_error_image_and_do_stats(opt, error_image);

    // The intensity image, if provided
    vw::ImageViewRef<float> intensity;
    if (opt.intensity_file != "")
      intensity = DiskImageView<float>(opt.intensity_file);

    // For saving the stddev
    vw::ImageViewRef<vw::Vector6> full_point_image; 
    vw::ImageViewRef<double> horizontal_stddev, vertical_stddev;
    if (opt.save_stddev) {
      std::vector<std::string> pointcloud_files;
      pointcloud_files.push_back(opt.pointcloud_file);

      bool has_sd = asp::has_stddev(pointcloud_files);
      if (!has_sd)
        vw_throw(ArgumentErr() << "The input point cloud file does not have "
                 << "standard deviations.\n");

      full_point_image 
        = asp::form_point_cloud_composite<vw::Vector6>(pointcloud_files, 
                                                       ASP_MAX_SUBBLOCK_SIZE);
      // Channel 3 is the error image, 4 and 5 are the stddevs
      horizontal_stddev = vw::select_channel(full_point_image, 4);
      vertical_stddev   = vw::select_channel(full_point_image, 5);
    }
    
    BBox3 cloud_bbox = asp::pointcloud_bbox(point_image, have_out_georef);

    // The las format stores the values as 32 bit integers. So, for a
    // given point, we store round((point-offset)/scale), as well as
    // the offset and scale values. Here we decide the values for
    // offset and scale to lose minimum amount of precision. We make
    // the scale almost as large as it can be without causing integer overflow.
    // TODO(oalexan1): This can have its own issues if later
    // this cloud is shifted a lot. Better pick more sensible
    // scale. 1 mm should be enough. Note that sometimes
    // the coordinates are in degrees, not in meters.
    Vector3 offset = (cloud_bbox.min() + cloud_bbox.max())/2.0;
    double  maxInt = std::numeric_limits<int32>::max();
    maxInt *= 0.95; // Just in case stay a bit away
    Vector3 scale  = cloud_bbox.size()/(2.0*maxInt);
    for (size_t i = 0; i < scale.size(); i++)
      if (scale[i] <= 0.0) scale[i] = 1.0e-16; // avoid degeneracy

    asp::write_las(have_out_georef, georef, 
                   point_image, error_image, intensity,
                   horizontal_stddev, vertical_stddev,
                   offset, scale, opt.compressed, 
                   opt.save_triangulation_error,
                   opt.max_valid_triangulation_error,
                   opt.out_prefix);
    return 0;
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

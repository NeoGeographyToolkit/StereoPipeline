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


/// \file point2las.cc
///

#include <fstream>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <liblas/liblas.hpp>

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>

#include <vw/Cartography/PointImageManipulation.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Math/Statistics.h>

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
  
  int size(){
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

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::string reference_spheroid, datum;
  std::string pointcloud_file;
  std::string target_srs_string;
  bool        compressed, use_tukey_outlier_removal;
  Vector2     outlier_removal_params;
  double      max_valid_triangulation_error;
  double      triangulation_error_factor;
  int         num_samples;
  
  // Output
  std::string out_prefix;
  Options() : compressed(false), max_valid_triangulation_error(0.0), num_samples(0) {}
};

void handle_arguments( int argc, char *argv[], Options& opt ) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("compressed,c", po::bool_switch(&opt.compressed)->default_value(false)->implicit_value(true),
     "Compress using laszip.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("datum", po::value(&opt.datum),
          "Create a geo-referenced LAS file in respect to this datum. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("reference-spheroid,r", po::value(&opt.reference_spheroid),
     "This is identical to the datum option.")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""),
     "Specify a custom projection (PROJ.4 string).")
    ("remove-outliers-params", po::value(&opt.outlier_removal_params)->default_value(Vector2(75.0, 3.0), "pct factor"),
     "Outlier removal based on percentage. Points with triangulation error larger than pct-th percentile times factor will be removed as outliers. [default: pct=75.0, factor=3.0]")
    ("use-tukey-outlier-removal", po::bool_switch(&opt.use_tukey_outlier_removal)->default_value(false)->implicit_value(true),
     "Remove outliers above Q3 + 1.5*(Q3 - Q1). Takes precedence over the above approach.")

    ("max-valid-triangulation-error", po::value(&opt.max_valid_triangulation_error)->default_value(0.0),
     "Outlier removal based on threshold. Points with triangulation error larger than this, if positive (measured in meters) will be removed from the cloud. Takes precedence over the above methods.")
    ("triangulation-error-factor", po::value(&opt.triangulation_error_factor)->default_value(0.0),
     "If this factor is positive, save the point cloud triangulation error to the 2-byte LAS intensity field by storing min(round(factor*error), 65535). Resulting values that equal 65535 should be treated with caution.")
    ("num-samples-for-outlier-estimation", po::value(&opt.num_samples)->default_value(1000000),
     "Approximate number of samples to pick from the input cloud to find the outlier cutoff based on triangulation error.");
  
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_file), "Input Point Cloud");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("[options] <point-cloud>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.pointcloud_file.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options );

  if ( opt.out_prefix.empty() )
    opt.out_prefix =
      vw::prefix_from_filename( opt.pointcloud_file );

  // reference_spheroid and datum are aliases.
  boost::to_lower(opt.reference_spheroid);
  boost::to_lower(opt.datum);
  if (opt.datum != "" && opt.reference_spheroid != "")
    vw_throw( ArgumentErr() << "Both --datum and --reference-spheroid were specified.\n");
  if (opt.datum == "")
    opt.datum = opt.reference_spheroid;

  double pct = opt.outlier_removal_params[0], factor = opt.outlier_removal_params[1];
  if (pct <= 0.0 || pct > 100.0 || factor <= 0.0){
    vw_throw( ArgumentErr() << "Invalid values were provided for outlier removal parameters.\n");
  }

  if (opt.max_valid_triangulation_error < 0.0) 
    vw_throw( ArgumentErr() << "The maximum valid triangulation error must be non-negative.\n");

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
                 ("asp","Error estim : ") );

  opt.max_valid_triangulation_error = error_accum.value(opt.outlier_removal_params,
                                                        opt.use_tukey_outlier_removal);
  
  sw.stop();
  vw_out(DebugMessage, "asp") << "Elapsed time: " << sw.elapsed_seconds() << std::endl;
  vw_out() << "Found the maximum valid triangulation error (outlier cutoff): "
           << opt.max_valid_triangulation_error << "." << std::endl;
}

int main( int argc, char *argv[] ) {
  
  // TODO(oalexan1): need to understand what is the optimal strategy
  // for traversing the input point cloud file to minimize the reading
  // time.

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    ImageViewRef<double> error_image;
    if (opt.outlier_removal_params[0] < 100.0 || opt.max_valid_triangulation_error > 0.0)
      find_error_image_and_do_stats(opt, error_image);

    // Save the las file in respect to a reference spheroid if provided
    // by the user.
    liblas::Header header;
    cartography::Datum datum;
    bool have_user_datum = asp::read_user_datum(0, 0, opt.datum, datum);

    cartography::GeoReference georef;
    bool have_input_georef = vw::cartography::read_georeference(georef, opt.pointcloud_file);
    if (have_input_georef && opt.target_srs_string.empty()) {
      opt.target_srs_string = georef.overall_proj4_str();
    }

    bool is_geodetic = false;
    if (have_user_datum || !opt.target_srs_string.empty()){

      // Set the srs string into georef.
      asp::set_srs_string(opt.target_srs_string,
                          have_user_datum, datum,
                          have_input_georef, georef);
      liblas::SpatialReference ref;
      std::string target_srs = georef.overall_proj4_str();

      ref.SetFromUserInput(target_srs);
      vw_out() << "Using projection string: '" << target_srs << "'"<< std::endl;
      header.SetSRS(ref);

      is_geodetic = true;
      datum = georef.datum();
    }

    // Save the las file with given georeference, if present
    ImageViewRef<Vector3> point_image = asp::read_asp_point_cloud<3>(opt.pointcloud_file);
    if (is_geodetic) {
      point_image = cartesian_to_geodetic(point_image, datum);
      double avg_lon = asp::find_avg_lon(point_image); // see if to use [-180, 180] or [0, 360]
      point_image = geodetic_to_point(asp::recenter_longitude(point_image, avg_lon), georef);
    }

    BBox3 cloud_bbox = asp::pointcloud_bbox(point_image, is_geodetic);

    // The las format stores the values as 32 bit integers. So, for a
    // given point, we store round((point-offset)/scale), as well as
    // the offset and scale values. Here we decide the values for
    // offset and scale to lose minimum amount of precision. We make
    // the scale almost as large as it can be without causing integer overflow.
    Vector3 offset = (cloud_bbox.min() + cloud_bbox.max())/2.0;
    double  maxInt = std::numeric_limits<int32>::max();
            maxInt *= 0.95; // Just in case stay a bit away
    Vector3 scale  = cloud_bbox.size()/(2.0*maxInt);
    for (size_t i = 0; i < scale.size(); i++){
      if (scale[i] <= 0.0) scale[i] = 1.0e-16; // avoid degeneracy
    }

    // The line below causes trouble with compression in libLAS-1.7.0.
    //header.SetDataFormatId(liblas::ePointFormat1);
    header.SetScale (scale [0], scale [1], scale [2]);
    header.SetOffset(offset[0], offset[1], offset[2]);

    // Populate the min and max fields of the LAS header
    header.SetMax(cloud_bbox.max().x(),cloud_bbox.max().y(),cloud_bbox.max().z());
    header.SetMin(cloud_bbox.min().x(),cloud_bbox.min().y(),cloud_bbox.min().z());

    std::string lasFile;
    header.SetCompressed(opt.compressed);
    if (opt.compressed)
      lasFile = opt.out_prefix + ".laz";
    else
      lasFile = opt.out_prefix + ".las";

    vw_out() << "Writing LAS file: " << lasFile + "\n";
    std::ofstream ofs;
    ofs.open(lasFile.c_str(), std::ios::out | std::ios::binary);
    liblas::Writer writer(ofs, header);

    TerminalProgressCallback tpc("asp", "\t--> ");
    long long int num_total_points = 0;
    long long int num_kept_points = 0;
    
    for (int row = 0; row < point_image.rows(); row++){
      tpc.report_fractional_progress(row, point_image.rows());
      for (int col = 0; col < point_image.cols(); col++){

        Vector3 point = point_image(col, row);

        // Skip no-data points
        bool is_good = ( (!is_geodetic && point != vw::Vector3()) ||
                         (is_geodetic  && !boost::math::isnan(point.z())) );
        if (!is_good) continue;

        num_total_points++;
        
        if (opt.max_valid_triangulation_error > 0.0 &&
            error_image(col, row) > opt.max_valid_triangulation_error) 
          continue;

        num_kept_points++;
#if 0
        // For comparison later with las2txt.
        std::cout.precision(16);
        std::cout << "\npoint " << point[0] << ' ' << point[1] << ' '
                  << point[2] << std::endl;
#endif

        liblas::Point las_point(&header);
        las_point.SetCoordinates(point[0], point[1], point[2]);

        if (opt.triangulation_error_factor > 0.0) {
          // Scale the triangulation error, clamp it, and save it as
          // uint16.  The LAS 1.2 format has no fields (apart from the
          // taken already x, y, and z) with 32-bit values, so uint16
          // is all one can do.
          double scaled_error = opt.triangulation_error_factor * error_image(col, row);
          scaled_error = round(scaled_error);
          scaled_error = std::max(scaled_error, 0.0); // should not be necessary
          scaled_error = std::min(scaled_error, double(std::numeric_limits<std::uint16_t>::max()));
          las_point.SetIntensity(std::uint16_t(scaled_error));
        }
          
        writer.WritePoint(las_point);
      }
    }
    tpc.report_finished();

    vw_out () << "Saved: " << num_kept_points << " points." << std::endl;
    
    if (opt.max_valid_triangulation_error > 0.0) {
      long long int num_excluded = num_total_points - num_kept_points;
      double percent = 100.0 * double(num_excluded)/num_total_points;
      percent = round(percent * 100.0)/100.0; // don't keep too many digits
      vw_out() << "Excluded based on triangulation error " << num_excluded << " points ("
               << percent << "%)." << std::endl;
    }
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

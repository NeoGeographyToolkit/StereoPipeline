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

// C++ implementation of the Nuth and Kaab alignment method from:
// https://github.com/dshean/demcoreg/tree/master

// TODO(oalexan1): Implement tilt correction.
// TODO(oalexan1): For now the algo is reimplemented precisely. Later we can
// deal with improvements.
// TODO(oalexan1): It is assumed both input DEMs are equally dense. The work
// will happen in the source DEM domain. Later we can deal with the case when
// we want it done in the reference DEM domain. This is temporary.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/DemUtils.h>

#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

struct Options: vw::GdalWriteOptions {
  std::string ref, src, out_prefix, res;
  int poly_order, max_iter;
  double tol, max_offset, max_dz;
  bool tiltcorr;
  vw::Vector2 slope_lim;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("ref,r", po::value(&opt.ref)->default_value(""),
     "Reference DEM.")
    ("src,s", po::value(&opt.src)->default_value(""),
     "Source DEM to align to the reference.")
    ("output-prefix,o", po::value(&opt.out_prefix)->default_value(""), 
     "Output prefix for writing all produced files.")
    ("poly-order", po::value(&opt.poly_order)->default_value(1), 
     "Specify the order of the polynomial fit.")
    ("tol", po::value(&opt.tol)->default_value(0.1),
      "Stop when iterative translation magnitude is below this tolerance (meters).")
    ("max-offset", po::value(&opt.max_offset)->default_value(100.0),
     "Maximum expected horizontal translation magnitude (meters).")
    ("max-dz", po::value(&opt.max_dz)->default_value(100.0),
     "Maximum expected vertical offset in meters, used to filter outliers.")
    ("tiltcorr", po::bool_switch(&opt.tiltcorr)->default_value(false),
     "After a preliminary translation, fit a polynomial to residual elevation offsets "
     "and remove.")
    ("res", po::value(&opt.res)->default_value("mean"),
     "Regrid the input DEMs to this resolution given the resolutions of input datasets. "
     "Options: min, max, mean, common_scale_factor.")
    ("slope-lim", 
     po::value(&opt.slope_lim)->default_value(vw::Vector2(0.1, 40.0), "0.1, 40.0"),
     "Minimum and maximum surface slope limits to consider (degrees).")
    ("max-iter", po::value(&opt.max_iter)->default_value(30),
     "Maximum number of iterations, if tolerance is not reached.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("-r ref_dem.tif -s src_dem.tif -o output_prefix [options]\n");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // On failure print the usage as well, for when the command is called with no
  // arguments.
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "The output prefix was not set.\n"
             << usage << general_options);

  // The ref and src DEMs must be provided.
  if (opt.ref == "" || opt.src == "")
    vw::vw_throw(vw::ArgumentErr() << "The reference and source DEMs must be provided.\n");

  // The poly order must be positive.
  if (opt.poly_order < 1)
    vw::vw_throw(vw::ArgumentErr() << "The polynomial order must be at least 1.\n");
  
  // The tol must be positive.
  if (opt.tol <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The tolerance must be positive.\n");
  
  // Max offset must be positive. Same with max_dz.
  if (opt.max_offset <= 0.0 || opt.max_dz <= 0.0)
    vw::vw_throw(vw::ArgumentErr() 
                 << "The maximum horizontal and vertical offsets must be positive.\n");

  // Check that res is one of the allowed values.
  if (opt.res != "min" && opt.res != "max" && opt.res != "mean" &&
      opt.res != "common_scale_factor")
    vw::vw_throw(vw::ArgumentErr() << "Unknown value for --res: " << opt.res << ".\n");
  
  // Check that slope limits are positive and in the right order.
  if (opt.slope_lim[0] <= 0.0 || opt.slope_lim[1] <= 0.0 || 
      opt.slope_lim[0] >= opt.slope_lim[1])
    vw::vw_throw(vw::ArgumentErr() 
                 << "The slope limits must be positive and in increasing order.\n");
    
  // Create the output directory
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);
}

void run_nuth(Options const& opt) {

  vw::vw_out() << "Reference DEM: " << opt.ref << "\n";
  vw::vw_out() << "Source DEM:    " << opt.src << "\n";
  
  // The ref DEM must have a georeference
  vw::cartography::GeoReference ref_georef, src_georef;
  bool has_ref_georef = vw::cartography::read_georeference(ref_georef, opt.ref);
  bool has_src_georef = vw::cartography::read_georeference(src_georef, opt.src);
  if (!has_ref_georef || !has_src_georef)
    vw::vw_throw(vw::ArgumentErr() << "The input DEMs must have georeferences.\n");
  
  // The georeferences must be in projected coordinates
  if (!ref_georef.is_projected() || !src_georef.is_projected())
    vw::vw_throw(vw::ArgumentErr() 
      << "The input DEMs must be in a projected coordinate system, in units of meter. "
      << "Use gdalwarp with the option -r cubic to reproject them. "
      << "Consider using an equidistant or UTM projection.\n");

  // We don't support datum changes
  if (std::abs(ref_georef.datum().semi_major_axis()
               - src_georef.datum().semi_major_axis()) > 0.1 ||
      std::abs(ref_georef.datum().semi_minor_axis()
               - src_georef.datum().semi_minor_axis()) > 0.1 ||
      ref_georef.datum().meridian_offset() != src_georef.datum().meridian_offset())
    vw::vw_throw(vw::NoImplErr() 
                 << "The input DEMs must have the same datum radii and meridian "
                 << "Use gdalwarp -r cubic to reproject to a shared datum.");

  // Reference and source DEM resolutions
  double ref_tr = ref_georef.transform()(0, 0);
  double src_tr = src_georef.transform()(0, 0);
  vw::vw_out() << "Reference DEM grid size: " << ref_tr << " meters.\n";
  vw::vw_out() << "Source DEM grid size: " << src_tr << " meters.\n";

  // Prefer the reference resolution to be smaller
  if (ref_tr > src_tr)
    vw::vw_out(vw::WarningMessage) 
      << "The reference DEM resolution is larger than the source DEM resolution. "
      << "This may lead to suboptimal results. It is strongly suggest to swap the "
      << "reference and source DEMs.\n";
  
  // The (1, 1) value in both transforms must be negative, as otherwise this a
  // non-standard transform that we do not handle.
  if (ref_georef.transform()(1, 1) >= 0 || src_georef.transform()(1, 1) >= 0)
    vw::vw_throw(vw::ArgumentErr() 
      << "The input DEMs must have y direction pointing down, so the second coordinate "
      << " of the pixel size in the geoheader be negative. The provided input DEMs "
      << " are not standard and are not supported.\n");

  // Read the DEMs and their no-data values as double
  vw::DiskImageResourceGDAL ref_rsrc(opt.ref), src_rsrc(opt.src);
  double ref_nodata = -std::numeric_limits<double>::max();
  double src_nodata = ref_nodata;
  if (ref_rsrc.has_nodata_read())
    ref_nodata = ref_rsrc.nodata_read();
  if (src_rsrc.has_nodata_read())
    src_nodata = src_rsrc.nodata_read(); 
  vw:: DiskImageView<double> ref_dem(ref_rsrc), src_dem(src_rsrc);
  
  // Copying verbatim from dem_align.py: Use original source dataset coordinate
  // system. Potentially issues with distortion and xyz/tiltcorr offsets for
  // DEMs with large extent.
  // TODO(oalexan1): Here is where dem_align.py starts assuming that 
  // one works in the source DEM domain. 
  vw::cartography::GeoReference local_georef = src_georef;
  std::string local_srs = local_georef.get_wkt();
  std::cout << "--> Using local SRS: " << local_srs << std::endl; 

  // Transform the second DEM's bounding box to first DEM's pixels
  vw::BBox2i src_crop_box = bounding_box(src_dem);
  vw::cartography::GeoTransform gt(ref_georef, src_georef);
  vw::BBox2i ref2src_box = gt.forward_bbox(bounding_box(ref_dem));
  src_crop_box.crop(ref2src_box);

  if (src_crop_box.empty()) 
    vw::vw_throw(vw::ArgumentErr() << "The two DEMs do not have a common area.\n");

  // Crop the src DEM to the shared box, then transform and crop the ref DEM.
  // to src DEM domain. Use bicubic interpolation for the transform.
  // TODO(oalexan1): This will need changing when the grids are not the same.
  vw::ImageViewRef<vw::PixelMask<double>> src_crop 
    = vw::crop(create_mask(src_dem, src_nodata), src_crop_box);
  vw::ImageViewRef<vw::PixelMask<double>> ref_trans 
    = asp::warpCrop(ref_dem, ref_nodata, src_georef, ref_georef, src_crop_box, 
                    "bicubic");
  
  // Recall that we work in the src domain
  vw::cartography::GeoReference crop_georef 
    = vw::cartography::crop(local_georef, src_crop_box);
  
  // Write the cropped and warped ref
  vw::TerminalProgressCallback ref_tpc("asp", ": ");
  bool has_ref_no_data = true;
  std::string ref_crop_file = opt.out_prefix + "-ref-crop.tif"; 
  vw::vw_out() << "Writing: " << ref_crop_file << "\n";
  vw::cartography::block_write_gdal_image(ref_crop_file,
                                          vw::apply_mask(ref_trans, ref_nodata),
                                          has_ref_georef, crop_georef,
                                          has_ref_no_data, ref_nodata,
                                          opt, ref_tpc);
  // Write the cropped src
  vw::TerminalProgressCallback src_tpc("asp", ": ");
  bool has_src_no_data = true;
  std::string src_crop_file = opt.out_prefix + "-src-crop.tif";
  vw::vw_out() << "Writing: " << src_crop_file << "\n";
  vw::cartography::block_write_gdal_image(src_crop_file,
                                          vw::apply_mask(src_crop, src_nodata),
                                          has_src_georef, crop_georef,
                                          has_src_no_data, src_nodata,
                                          opt, src_tpc);
  
}

int main(int argc, char *argv[]) {

  Options opt;
  try {

    // TODO(oalexan1): Need to convert from boost conventions to dem_align.py
    // conventions. Example: instead of --poly-order use -polyorder, and instead
    // of --max-offset use -max_offset, etc. 
    handle_arguments(argc, argv, opt);
    run_nuth(opt);

  } ASP_STANDARD_CATCHES;

  return 0;
}

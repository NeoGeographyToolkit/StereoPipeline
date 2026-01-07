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

/// \file bathy_plane_calc.cc

#include <asp/Core/PointUtils.h>
#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/BathyPlaneCalc.h>

#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Cartography/shapeFile.h>
#include <vw/Camera/CameraModel.h>
#include <vw/FileIO/FileUtils.h>

#include <Eigen/Dense>

#include <random>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <vector>

namespace po = boost::program_options;

struct Options: vw::GdalWriteOptions {
  std::string shapefile, dem, mask, ortho_mask, camera, stereo_session, bathy_plane,
    water_height_measurements, lon_lat_measurements, csv_format_str,
    output_inlier_shapefile, bundle_adjust_prefix, output_outlier_shapefile,
    mask_boundary_shapefile, dem_minus_plane;

  double outlier_threshold;
  int num_ransac_iterations, num_samples;
  bool save_shapefiles_as_polygons, use_ecef_water_surface;
  Options(): outlier_threshold(0.5), num_ransac_iterations(1000) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("shapefile",   po::value(&opt.shapefile),
     "The shapefile with vertices whose coordinates will be looked up in the DEM "
     "with bilinear interpolation.")
    ("dem",   po::value(&opt.dem),
     "The DEM to use.")
    ("mask",   po::value(&opt.mask),
     "An input mask, created from a raw camera image and hence having the same dimensions, "
     "with values of 1 on land and 0 on water, or positive values on land and nodata "
     "values on water. The larger of the nodata value and zero is used as the water "
     "value. The heights will be looked up in the DEM with bilinear interpolation.")
    ("camera",   po::value(&opt.camera),
     "The camera file to use with the mask.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment at this output prefix, if the cameras changed "
     "based on bundle adjustment or alignment.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select "
     "this automatically by the file extension, except for xml cameras. See the doc for options.")
    ("bathy-plane",   po::value(&opt.bathy_plane),
     "The output file storing the computed plane as four coefficients a, b, c, d, "
     "with the plane being a*x + b*y + c*z + d = 0.")
    ("outlier-threshold",
     po::value(&opt.outlier_threshold)->default_value(0.5),
     "A value, in meters, to determine the distance from a sampled point on the DEM to the "
     "best-fit plane to determine if it will be marked as outlier and not "
     "included in the calculation of that plane. Its value should be roughly the expected "
     "vertical uncertainty of the DEM.")
    ("num-ransac-iterations",
     po::value(&opt.num_ransac_iterations)->default_value(1000),
     "Number of RANSAC iterations to use to find the best-fitting plane.")
    ("output-inlier-shapefile", po::value(&opt.output_inlier_shapefile)->default_value(""),
     "If specified, save at this location the shape file with the inlier vertices.")
    ("output-outlier-shapefile", po::value(&opt.output_outlier_shapefile)->default_value(""),
     "If specified, save at this location the shape file with the outlier vertices.")
    ("mask-boundary-shapefile", po::value(&opt.mask_boundary_shapefile)->default_value(""),
     "If specified together with a mask, camera, and DEM, save a random "
     "sample of points (their number given by ``--num-samples``) at the "
     "mask boundary (water-land interface) to this shapefile and exit. "
     "The heights will be looked up in the DEM with bilinear interpolation.")
    ("num-samples",
     po::value(&opt.num_samples)->default_value(10000),
     "Number of samples to pick at the water-land interface if using a mask.")
    ("save-shapefiles-as-polygons",
     po::bool_switch(&opt.save_shapefiles_as_polygons)->default_value(false),
     "Save the inlier and outlier shapefiles as polygons, rather than "
     "discrete vertices. May be more convenient for processing in a GIS tool.")
    ("ortho-mask",   po::value(&opt.ortho_mask),
     "An input mask, that is georeferenced and aligned with the DEM, with positive values "
     "on land and 0 or nodata values on water. The larger of the nodata value and zero is "
     "used as the water value.")
    ("lon-lat-measurements",
     po::value(&opt.lon_lat_measurements)->default_value(""),
     "Use this CSV file having longitude and latitude measurements for the water surface. "
     "The heights will be looked up in the DEM with bilinear interpolation. The option "
     "--csv-format must be used.")
    ("water-height-measurements",
     po::value(&opt.water_height_measurements)->default_value(""),
     "Use this CSV file having longitude, latitude, and height measurements for the water "
     "surface, in degrees and meters, respectively, relative to the WGS84 datum. The "
     "option --csv-format must be used.")
    ("csv-format", po::value(&opt.csv_format_str)->default_value(""),
     "Specify the format of the CSV file having water height measurements or lon and lat "
     "values. The format should have a list of entries with syntax "
     "column_index:column_type (indices start from 1). Example: '2:lon 3:lat "
     "4:height_above_datum'.")
    ("dem-minus-plane",
     po::value(&opt.dem_minus_plane),
     "If specified, subtract from the input DEM the best-fit plane and save the "
     "obtained DEM to this GeoTiff file.")
    ("use-ecef-water-surface",
     po::bool_switch(&opt.use_ecef_water_surface)->default_value(false),
     "Compute the best fit plane in ECEF coordinates rather than in a local stereographic "
     "projection. Hence don't model the Earth curvature. Not recommended.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

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

  // Need this to be able to load adjusted camera models. This must be set
  // before loading the cameras. 
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  bool use_shapefile  = !opt.shapefile.empty();
  bool use_mask       = !opt.mask.empty();
  bool use_ortho_mask = !opt.ortho_mask.empty();
  bool use_meas       = !opt.water_height_measurements.empty();
  bool use_lon_lat    = !opt.lon_lat_measurements.empty();

  if (use_mask && opt.camera.empty())
    vw::vw_throw(vw::ArgumentErr() << "If using a mask, must specify a camera.\n"
             << usage << general_options);

  if (use_shapefile + use_mask + use_ortho_mask + use_meas + use_lon_lat != 1)
    vw::vw_throw(vw::ArgumentErr() 
              << "Must use either a mask and camera, an ortho-mask, a shapefile, "
              << "water height measurements, or lon-lat measurements, "
              << "and just one of these.\n");

  if (!use_meas && opt.dem == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing the input dem.\n" << usage << general_options);

  if (opt.bathy_plane.empty() && opt.mask_boundary_shapefile.empty())
    vw::vw_throw(vw::ArgumentErr() << "Must set either --bathy-plane or --mask-boundary-shapefile.\n"
             << usage << general_options);

  if ((use_meas || use_lon_lat) && opt.csv_format_str == "")
    vw::vw_throw(vw::ArgumentErr() << "Must set the option --csv-format.\n"
             << usage << general_options);

  if (opt.use_ecef_water_surface && (use_meas || use_lon_lat))
    vw::vw_throw(vw::ArgumentErr() << "Cannot use --use-ecef-water-surface with "
             << "--water-height-measurements or --lon-lat-measurements.\n"
             << usage << general_options);

  if (!opt.mask_boundary_shapefile.empty() &&
      (opt.mask.empty() && opt.ortho_mask.empty()))
    vw::vw_throw(vw::ArgumentErr() << "If using --mask-boundary-shapefile, then "
             << "must specify a mask (with a camera) or an ortho-mask.\n"
             << usage << general_options);

  if ((use_mask || use_ortho_mask) && opt.num_samples <= 0)
    vw::vw_throw(vw::ArgumentErr() << "A positive number of samples must be specified.\n"
             << usage << general_options);

  if (!opt.dem_minus_plane.empty() && opt.dem.empty())
    vw::vw_throw(vw::ArgumentErr() << "The option --dem must be set if using --dem-minus-plane.\n"
             << usage << general_options);

  // Create the output prefix  
  std::string out_prefix = opt.bathy_plane;
  if (opt.bathy_plane.empty())
    out_prefix = opt.mask_boundary_shapefile;
  // Remove the extension, which is the text after the last dot
  size_t pos = out_prefix.rfind(".");
  if (pos != std::string::npos)
    out_prefix = out_prefix.substr(0, pos);

  // Create the output directory and turn on logging to file
  vw::create_out_dir(out_prefix);
  asp::log_to_file(argc, argv, "", out_prefix);
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Load the camera if we use the mask and the camera
    bool use_shapefile  = !opt.shapefile.empty();
    bool use_mask       = !opt.mask.empty();
    bool use_ortho_mask = !opt.ortho_mask.empty();
    bool use_meas       = !opt.water_height_measurements.empty();
    bool use_lon_lat    = !opt.lon_lat_measurements.empty();

    boost::shared_ptr<vw::camera::CameraModel> camera_model;
    if (use_mask) {
      std::string out_prefix;
      asp::SessionPtr
        session(asp::StereoSessionFactory::create(opt.stereo_session, // may change
                                                  opt,
                                                  opt.mask, opt.mask,
                                                  opt.camera, opt.camera,
                                                  out_prefix));
      camera_model = session->camera_model(opt.mask, opt.camera);
    }

    // Only WGS84 is supported. Note that dem_georef and shape_georef
    // may or may not be overwritten below depending on the case.
    vw::cartography::GeoReference dem_georef;
    vw::cartography::Datum datum("WGS_1984");
    dem_georef.set_datum(datum);
    dem_georef.set_geographic();
    bool has_shape_georef = true;
    vw::cartography::GeoReference shape_georef = dem_georef;

    float dem_nodata_val = -std::numeric_limits<float>::max();
    vw::ImageViewRef<float> dem;
    vw::ImageViewRef<vw::PixelMask<float>> masked_dem;
    vw::ImageViewRef<vw::PixelMask<float>> interp_dem;

    if (use_shapefile || use_mask || use_ortho_mask || use_lon_lat || 
        !opt.dem_minus_plane.empty()) {
      // Read the DEM and its associated data
      // TODO(oalexan1): Think more about the interpolation method
      vw::vw_out() << "Reading the DEM: " << opt.dem << "\n";
      if (!vw::cartography::read_georeference(dem_georef, opt.dem))
        vw::vw_throw(vw::ArgumentErr() << "The input DEM has no georeference.\n");

      // We assume the WGS_1984 datum
      if (dem_georef.datum().name() != "WGS_1984")
        vw::vw_throw(vw::ArgumentErr() << "Only an input DEM with the "
                  << "WGS_1984 datum is supported.\n"
                  << "Got: " << dem_georef.datum().name() << ".\n");

      // Note we use a float nodata
      if (!vw::read_nodata_val(opt.dem, dem_nodata_val))
        vw::vw_out() << "Warning: Could not read the DEM nodata value. "
                 << "Using: " << dem_nodata_val << ".\n";
      else
        vw::vw_out() << "Read DEM nodata value: " << dem_nodata_val << ".\n";

      // Read the DEM
      dem = vw::DiskImageView<float>(opt.dem);
      masked_dem = vw::create_mask(dem, dem_nodata_val);
      vw::PixelMask<float> nodata_pix(0);
      nodata_pix.invalidate();
      vw::ValueEdgeExtension<vw::PixelMask<float>> ext_nodata(nodata_pix);
      interp_dem = vw::interpolate(masked_dem, vw::BilinearInterpolation(), ext_nodata);
      
      shape_georef = dem_georef; // may get overwritten below
    }

    bool use_proj_water_surface = !opt.use_ecef_water_surface;
    std::vector<Eigen::Vector3d> shape_point_vec;
    std::vector<vw::Vector3> llh_vec;
    double proj_lat = -1.0, proj_lon = -1.0; // only for curved water surface
    std::vector<vw::Vector2> used_vertices;
    vw::cartography::GeoReference stereographic_georef;
    std::string poly_color = "green";

    if (use_mask) {
      // Read the mask. The nodata value is the largest of what
      // is read from the mask file and the value 0, as pixels
      // over land are supposed to be positive and be valid data.
      vw::vw_out() << "Reading the mask: " << opt.mask << "\n";
      float mask_nodata_val = -std::numeric_limits<float>::max();
      if (vw::read_nodata_val(opt.mask, mask_nodata_val))
        vw::vw_out() << "Read mask nodata value: " << mask_nodata_val << ".\n";
      mask_nodata_val = std::max(0.0f, mask_nodata_val);
      if (std::isnan(mask_nodata_val))
        mask_nodata_val = 0.0f;
      vw::vw_out() << "Pixels with values no more than " << mask_nodata_val
               << " are classified as water.\n";
      vw::DiskImageView<float> mask(opt.mask);
      shape_georef = dem_georef;
      asp::sampleMaskBd(mask, mask_nodata_val,
                        camera_model, shape_georef,
                        dem_georef, masked_dem,
                        opt.num_samples,
                        shape_point_vec, llh_vec,
                        used_vertices);

      if (!opt.mask_boundary_shapefile.empty()) {
        asp::saveShape(shape_point_vec, opt.mask_boundary_shapefile);
        return 0;
      }

    } else if (use_ortho_mask) {
      // Read the ortho mask. Its georef will become the shape georef for this case.
      vw::vw_out() << "Reading the ortho mask: " << opt.ortho_mask << "\n";
      if (!vw::cartography::read_georeference(shape_georef, opt.ortho_mask))
        vw::vw_throw(vw::ArgumentErr() << "The input ortho-mask has no georeference.\n");
      asp::sampleOrthoMaskBd(opt.ortho_mask, shape_georef, dem_georef, interp_dem,
                             opt.num_samples, shape_point_vec, llh_vec,
                             used_vertices);

      if (!opt.mask_boundary_shapefile.empty()) {
        asp::saveShape(shape_point_vec, opt.mask_boundary_shapefile);
        return 0;
      }
      
    } else if (use_shapefile) {

      // Read the shapefile, overwriting shape_georef
      vw::vw_out() << "Reading the shapefile: " << opt.shapefile << "\n";
      std::vector<vw::geometry::dPoly> polyVec;
      vw::geometry::read_shapefile(opt.shapefile, poly_color, has_shape_georef, 
                                   shape_georef, polyVec);
      if (!has_shape_georef)
        vw::vw_throw(vw::ArgumentErr() << "The input shapefile has no georeference.\n");

      // Find the ECEF coordinates of the shape corners
      asp::find_points_at_shape_corners(polyVec, shape_georef, dem_georef, interp_dem, 
                                        shape_point_vec, llh_vec, used_vertices);
    } else if (use_meas) {
      asp::find_points_from_meas_csv(opt.water_height_measurements, opt.csv_format_str,
                                     shape_georef,
                                     // Outputs
                                     llh_vec, used_vertices);
    } else if (use_lon_lat) {
      shape_georef = dem_georef;
      has_shape_georef = true;
      asp::find_points_from_lon_lat_csv(opt.lon_lat_measurements, opt.csv_format_str,
                                        shape_georef, dem_georef, interp_dem,
                                        // Outputs
                                        shape_point_vec, llh_vec, used_vertices);
    }

    // See if to convert to local stereographic projection
    std::vector<Eigen::Vector3d> local_proj_point_vec;
    if (use_proj_water_surface)
      asp::find_projection(// Inputs
                           dem_georef, llh_vec,
                           // Outputs
                           proj_lat, proj_lon,
                           stereographic_georef,
                           local_proj_point_vec);

    // Compute the water surface using RANSAC
    std::vector<size_t> inlier_indices;
    double inlier_threshold = opt.outlier_threshold;
    vw::Matrix<double> plane;
    asp::calcBathyPlane(use_proj_water_surface, opt.num_ransac_iterations, inlier_threshold,
                        local_proj_point_vec, plane, inlier_indices);
    asp::calcPlaneProperties(use_proj_water_surface, local_proj_point_vec, inlier_indices,
                             dem_georef, plane);
    asp::saveBathyPlane(use_proj_water_surface, proj_lat, proj_lon,
                        plane, opt.bathy_plane);

    // Save the shape having the inliers.
    if (opt.output_inlier_shapefile != "") {
      vw::geometry::dPoly inlierPoly;
      for (size_t inlier_it = 0; inlier_it < inlier_indices.size(); inlier_it++)
        asp::addPointToPoly(inlierPoly, used_vertices[inlier_indices[inlier_it]]);

      if (opt.save_shapefiles_as_polygons) {
        vw::geometry::dPoly localPoly;
        asp::formSinglePoly(inlierPoly, localPoly);
        inlierPoly = localPoly;
      }

      std::vector<vw::geometry::dPoly> inlierPolyVec;
      inlierPolyVec.push_back(inlierPoly);
      vw::vw_out() << "Writing inlier shapefile: " << opt.output_inlier_shapefile << "\n";
      vw::geometry::write_shapefile(opt.output_inlier_shapefile, has_shape_georef, 
                                     shape_georef, inlierPolyVec);
    }

    // Save the shape having the outliers.
    if (opt.output_outlier_shapefile != "") {

      // First put the inliers in a set so we can exclude them
      std::set<int> inlier_set;
      for (size_t inlier_it = 0; inlier_it < inlier_indices.size(); inlier_it++)
        inlier_set.insert(inlier_indices[inlier_it]);

      vw::geometry::dPoly outlierPoly;
      for (size_t it = 0; it < used_vertices.size(); it++) {

        if (inlier_set.find(it) != inlier_set.end())
          continue; // an inlier, skip it

        asp::addPointToPoly(outlierPoly, used_vertices[it]);
      }

      if (opt.save_shapefiles_as_polygons) {
        vw::geometry::dPoly localPoly;
        asp::formSinglePoly(outlierPoly, localPoly);
        outlierPoly = localPoly;
      }

      std::vector<vw::geometry::dPoly> outlierPolyVec;
      outlierPolyVec.push_back(outlierPoly);
      vw::vw_out() << "Writing outlier shapefile: " << opt.output_outlier_shapefile << "\n";
      vw::geometry::write_shapefile(opt.output_outlier_shapefile, has_shape_georef, 
                                     shape_georef, outlierPolyVec);
    }

    if (opt.dem_minus_plane != "") {
      bool has_nodata = true, has_georef = true;
      vw::vw_out() << "Writing: " << opt.dem_minus_plane << "\n";
      vw::TerminalProgressCallback tpc("asp", ": ");
      auto dem_minus_plane 
        = asp::demMinusPlane(dem, dem_georef, plane, dem_nodata_val, use_proj_water_surface,
                             stereographic_georef);
      vw::cartography::block_write_gdal_image(opt.dem_minus_plane, dem_minus_plane, 
                                              has_georef, dem_georef, 
                                              has_nodata, dem_nodata_val, opt, tpc);
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}

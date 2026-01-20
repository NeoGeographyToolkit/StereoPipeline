// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

/// Compute the footprint of a camera on a DEM/datum, print it, and optionally
///  write a KML file.

#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/FileIO/KML.h>
#include <vw/Cartography/shapeFile.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace vw::cartography;

struct Options : public vw::GdalWriteOptions {
  std::string image_file, camera_file, stereo_session, bundle_adjust_prefix,
         datum_str, dem_file, target_srs_string, output_shp, output_kml;
  bool quick;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("datum", po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the heights. Options: WGS_1984, D_MOON (1,737,400 "
     "meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters). Also accepted: Earth "
     "(=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("t_srs", po::value(&opt.target_srs_string)->default_value(""), 
     "Specify the output projection (PROJ or WKT string).")
    ("quick", po::bool_switch(&opt.quick)->default_value(false),
	   "Use a faster but less accurate computation.")
    ("output-shp", po::value(&opt.output_shp),
     "Save the convex hull of the points sampled on the camera footprint as a shapefile "
     "with this name.")
    ("output-kml", po::value(&opt.output_kml),
     "Create an output KML file at this path.")
    ("session-type,t", po::value(&opt.stereo_session)->default_value(""),
     "Select the stereo session type to use for processing. Usually the program can select "
     "this automatically by the file extension, except for xml cameras. See the doc for "
     "options.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this "
     "output prefix.")
    ("dem-file", po::value(&opt.dem_file)->default_value(""),
     "Instead of using a longitude-latitude-height box, sample the surface of this DEM.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);

  std::string usage("[options] <camera-image> <camera-model>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);

  if (opt.image_file.empty())
    vw_throw(ArgumentErr() << "Missing input image.\n" << usage << general_options);

  if (boost::iends_with(opt.image_file, ".cub") && opt.stereo_session == "")
    opt.stereo_session = "isis";

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  // Must specify the DEM or the datum
  if (opt.dem_file.empty() && opt.datum_str.empty() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr() << "Need to provide a DEM, a datum, or a t_srs string.\n" 
             << usage << general_options);

  // This is a bug fix. The user by mistake passed in an empty projection string.
  if (!vm["t_srs"].defaulted() && opt.target_srs_string.empty())
    vw_throw(ArgumentErr()
             << "The value of --t_srs is empty. Then it must not be set at all.\n");
}

int main(int argc, char *argv[]) {

  Options opt;
  try {

    handle_arguments(argc, argv, opt);

        asp::SessionPtr session(asp::StereoSessionFactory::create
		       (opt.stereo_session, // may change inside
                        opt,
                        opt.image_file,  opt.image_file,
                        opt.camera_file, opt.camera_file,
                        "",
                        "",
                        false)); // Do not allow promotion from normal to map projected session

    if (opt.camera_file.empty())
      vw_throw(ArgumentErr() << "Missing input camera.\n");

    boost::shared_ptr<CameraModel> cam = session->camera_model(opt.image_file, opt.camera_file);

    // The input nodata value
    float input_nodata_value = -std::numeric_limits<float>::max();
    vw::read_nodata_val(opt.image_file, input_nodata_value);

    // Just get the image size
    vw::Vector2i image_size = vw::file_image_size(opt.image_file);

    // Perform the computation
    GeoReference target_georef;

    BBox2 footprint_bbox;
    float mean_gsd = 0.0;
    std::vector<Vector3> llh_coords;
    if (opt.dem_file.empty()) { 
                               
      // No DEM available, intersect with the datum.

      // Initialize the georef/datum
      bool have_user_datum = (opt.datum_str != "");
      cartography::Datum datum(opt.datum_str);
      target_georef = GeoReference(datum);
      vw::cartography::set_srs_string(opt.target_srs_string, have_user_datum, datum, target_georef);
      vw_out() << "Using georef: " << target_georef << "\n";

      std::vector<Vector2> coords2;
      footprint_bbox = camera_bbox(target_georef, cam, image_size[0], image_size[1],
                                   mean_gsd, &coords2);
      for (std::size_t i=0; i<coords2.size(); ++i) {
        Vector3 proj_coord(coords2[i][0], coords2[i][1], 0.0);
        llh_coords.push_back(target_georef.point_to_geodetic(proj_coord));
      }

    } else { 
            
      // DEM provided, intersect with it

      // Load the DEM
      float dem_nodata_val = -std::numeric_limits<float>::max();
      vw::read_nodata_val(opt.dem_file, dem_nodata_val);
      ImageViewRef<PixelMask<float>> dem
        = create_mask(DiskImageView<float>(opt.dem_file), dem_nodata_val);

      GeoReference dem_georef;
      if (!read_georeference(dem_georef, opt.dem_file))
        vw_throw(ArgumentErr() << "Missing georef.\n");

      target_georef = dem_georef; // return box in this projection
      vw_out() << "Using georef: " << target_georef << "\n";
      int num_samples = 100; // should be enough
      footprint_bbox = camera_bbox(dem, dem_georef,
                                   target_georef, cam,
                                   image_size[0], image_size[1],
                                   mean_gsd, opt.quick, &llh_coords, num_samples);
      for (std::size_t i = 0;  i < llh_coords.size(); i++) 
        llh_coords[i] = target_georef.datum().cartesian_to_geodetic(llh_coords[i]);
    }

    // Print out the results
    vw_out() << "Computed footprint bounding box:\n" << footprint_bbox << "\n";
    vw_out() << "Computed mean gsd: " << mean_gsd << "\n";
  
    if (opt.output_shp != "") {
      // Must convert to projected coordinates
      std::vector<vw::Vector3> proj_coords(llh_coords.size());
      for (std::size_t i = 0; i < llh_coords.size(); i++)
        proj_coords[i] = target_georef.geodetic_to_point(llh_coords[i]);
      
      vw::geometry::dPoly poly;
      vw::geometry::convexHull(proj_coords, poly);
      bool has_geo = true;
      vw::vw_out() << "Writing: " << opt.output_shp << "\n";
      vw::geometry::write_shapefile(opt.output_shp, has_geo, target_georef, poly);
    }
    
    if (opt.output_kml != "") {
      // Create the KML file if specified by the user
      KMLFile kml(opt.output_kml, "footprint");
      const bool HIDE_LABELS = true;
      std::string base = "http://maps.google.com/mapfiles/kml/shapes/"; 
      std::string dot_cir = base + "placemark_circle.png"; 
      std::string dot_hlt = base + "placemark_circle_highlight.png"; 
      kml.append_style("dot", "", 1.2, dot_cir,  HIDE_LABELS);
      kml.append_style("dot_highlight", "", 1.4, dot_hlt);
      kml.append_stylemap("placemark", "dot", "dot_highlight");
      kml.append_line(llh_coords, "intersections", "placemark");
      vw_out() << "Writing: " << opt.output_kml << "\n";
      kml.close_kml();
    }
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

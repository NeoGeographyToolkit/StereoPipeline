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

// Compute the footprint of a camera on a DEM

#include <asp/Sessions/StereoSessionFactory.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/FileIO/KML.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace vw::camera;
using namespace std;
using namespace vw::cartography;


struct Options : public vw::cartography::GdalWriteOptions {
  string image_file, camera_file, stereo_session, bundle_adjust_prefix,
         datum_str, dem_file, target_srs_string, output_kml;
  bool quick;
  //BBox2i image_crop_box;
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("datum",            po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the heights. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("t_srs",         po::value(&opt.target_srs_string)->default_value(""), "Specify the output projection (PROJ.4 string). Can also be an URL or in WKT format, as in GDAL.")
    ("quick",            po::bool_switch(&opt.quick)->default_value(false),
	     "Use a faster but less accurate computation.")
    ("output-kml", po::value(&opt.output_kml),
     "Create an output KML file at this path.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the input camera model type. Normally this is auto-detected, but may need to be specified if the input camera model is in XML format. Options: pinhole isis rpc dg spot5 aster.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    // TODO: Support this feature!
    //("image-crop-box", po::value(&opt.image_crop_box)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
    // "The output image and RPC model should not exceed this box, specified in input image pixels as minx miny widx widy.")
    ("dem-file",   po::value(&opt.dem_file)->default_value(""),
     "Instead of using a longitude-latitude-height box, sample the surface of this DEM.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);

  string usage("[options] <camera-image> <camera-model>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);

  
  if ( opt.image_file.empty() )
    vw_throw( ArgumentErr() << "Missing input image.\n" << usage << general_options );

  if (boost::iends_with(opt.image_file, ".cub") && opt.stereo_session == "" )
    opt.stereo_session = "isis";

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

 
  // Must specify the DEM or the datum somehow
  if (opt.dem_file.empty() && opt.datum_str.empty() && opt.target_srs_string.empty())
    vw_throw( ArgumentErr() << "Need to provide a DEM, a datum, or a t_srs string.\n" << usage << general_options );


  //// Convert from width and height to min and max
  //if (!opt.image_crop_box.empty()) {
  //  BBox2 b = opt.image_crop_box; // make a copy
  //  opt.image_crop_box = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
  //}
}

int main( int argc, char *argv[] ) {

  Options opt;
  //try {

    handle_arguments(argc, argv, opt);

    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSessionFactory::create
		       (opt.stereo_session, // may change inside
                        opt,
                        opt.image_file,  opt.image_file,
                        opt.camera_file, opt.camera_file,
                        "",
                        "",
                        false) ); // Do not allow promotion from normal to map projected session
    
    // If the session was above auto-guessed as isis, adjust for the fact
    // that the isis .cub file also has camera info.
    if ((session->name() == "isis" || session->name() == "isismapisis") ){
      // The user did not provide an output file. Then the camera
      // information is contained within the image file and what is in
      // the camera file is actually the output file.
      opt.camera_file = opt.image_file;
    }

    if ( opt.camera_file.empty() )
      vw_throw( ArgumentErr() << "Missing input camera.\n" );

   
    boost::shared_ptr<CameraModel> cam = session->camera_model(opt.image_file, opt.camera_file);

    // The input nodata value
    float input_nodata_value = -std::numeric_limits<float>::max(); 
    vw::read_nodata_val(opt.image_file, input_nodata_value);

    // Just get the image size
    vw::Vector2i image_size = vw::file_image_size(opt.image_file);

//    // The bounding box -> Add this feature in the future!
//    BBox2 image_box = bounding_box(input_img);
//    if (!opt.image_crop_box.empty()) 
//      image_box.crop(opt.image_crop_box);
    
    // Perform the computation
    
    GeoReference target_georef;
    
    BBox2 footprint_bbox;
    float mean_gsd=0;
    std::vector<Vector3> coords;
    if (opt.dem_file.empty()) { // No DEM available, intersect with the datum.

      // Initialize the georef/datum
      bool have_user_datum = (opt.datum_str != "");
      cartography::Datum datum(opt.datum_str);
      target_georef = GeoReference(datum);
      
      asp::set_srs_string(opt.target_srs_string, have_user_datum, datum, target_georef);
      vw_out() << "Using georef: " << target_georef << std::endl;

      std::vector<Vector2> coords2;
      footprint_bbox = camera_bbox(target_georef, cam, image_size[0], image_size[1], mean_gsd, &coords2);
      for (size_t i=0; i<coords2.size(); ++i) {
        Vector3 proj_coord(coords2[i][0], coords2[i][1], 0.0);
        coords.push_back(target_georef.point_to_geodetic(proj_coord));
      }
      
    } else { // DEM provided, intersect with it.

      // Load the DEM
      float dem_nodata_val = -std::numeric_limits<float>::max(); 
      vw::read_nodata_val(opt.dem_file, dem_nodata_val);
      ImageViewRef< PixelMask<double> > dem = create_mask
        (channel_cast<double>(DiskImageView<float>(opt.dem_file)), dem_nodata_val);
      
      GeoReference dem_georef;
      if (!read_georeference(dem_georef, opt.dem_file))
        vw_throw( ArgumentErr() << "Missing georef.\n");

      target_georef = dem_georef; // return box in this projection
      vw_out() << "Using georef: " << target_georef << std::endl;
      
      footprint_bbox = camera_bbox(dem, dem_georef, target_georef, cam,
                                   image_size[0], image_size[1], mean_gsd, opt.quick, &coords);
      for (size_t i=0; i<coords.size(); ++i)
        coords[i] = target_georef.datum().cartesian_to_geodetic(coords[i]);
    }
    
    // Print out the results    
    vw_out() << "Computed footprint bounding box:\n" << footprint_bbox << std::endl;
    vw_out() << "Computed mean gsd: " << mean_gsd << std::endl;
 
    if (opt.output_kml == "")
      return 0;

    // Create the KML file if specified by the user.
    // - TODO: Create an accurate footprint instead of a bounding box!
    KMLFile kml(opt.output_kml, "footprint");

    // Style listing

    // Placemark Style
    const bool HIDE_LABELS = true;
    kml.append_style( "dot", "", 1.2,
                      "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png", 
                      HIDE_LABELS);
    kml.append_style( "dot_highlight", "", 1.4,
                      "http://maps.google.com/mapfiles/kml/shapes/placemark_circle_highlight.png");
    kml.append_stylemap( "placemark", "dot",
                         "dot_highlight" );


    /*
    std::vector<Vector2> lonlat[4];
    std::vector<Vector3> coordinates[4];
    lonlat[0] = target_georef.point_to_lonlat(footprint_bbox.min());
    lonlat[1] = target_georef.point_to_lonlat(Vector2(footprint_bbox.max()[0], footprint_bbox.min()[1]);
    lonlat[2] = target_georef.point_to_lonlat(footprint_bbox.max());
    lonlat[3] = target_georef.point_to_lonlat(Vector2(footprint_bbox.min()[0], footprint_bbox.max()[1]);
    for (int i=0; i<4; ++i)
      coordinates[i] = Vector3(lonlat[i][0], lonlat[i][1], 0);
    kml.append_line(coordinates);
    */  
    
    kml.append_line(coords, "intersections", "placemark");
    vw_out() << "Writing: " << opt.output_kml << std::endl; 
    kml.close_kml();
    

    
  //} ASP_STANDARD_CATCHES;

  return 0;
}

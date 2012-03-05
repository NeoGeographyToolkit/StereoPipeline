// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

/// \file point2dem.cc
///
#include <asp/Tools/point2dem.h>

using namespace vw;
using namespace vw::cartography;

#include <asp/Core/OrthoRasterizer.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
namespace po = boost::program_options;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
  template<> struct PixelFormatID<Vector4>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_4_CHANNEL; };
}

enum ProjectionType {
  SINUSOIDAL,
  MERCATOR,
  TRANSVERSEMERCATOR,
  ORTHOGRAPHIC,
  STEREOGRAPHIC,
  LAMBERTAZIMUTHAL,
  UTM,
  PLATECARREE
};

struct Options : asp::BaseOptions {
  // Input
  std::string pointcloud_filename, texture_filename;

  // Settings
  float dem_spacing, nodata_value;
  double semi_major, semi_minor;
  std::string reference_spheroid;
  double phi_rot, omega_rot, kappa_rot;
  std::string rot_order;
  double proj_lat, proj_lon, proj_scale;
  double x_offset, y_offset, z_offset;
  unsigned utm_zone;
  ProjectionType projection;
  bool has_nodata_value, has_alpha, do_normalize, do_error;

  // Output
  std::string  out_prefix, output_file_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description manipulation_options("Manipulation options");
  manipulation_options.add_options()
    ("x-offset", po::value(&opt.x_offset)->default_value(0), "Add a horizontal offset to the DEM")
    ("y-offset", po::value(&opt.y_offset)->default_value(0), "Add a horizontal offset to the DEM")
    ("z-offset", po::value(&opt.z_offset)->default_value(0), "Add a vertical offset to the DEM")
    ("rotation-order", po::value(&opt.rot_order)->default_value("xyz"),"Set the order of an euler angle rotation applied to the 3D points prior to DEM rasterization")
    ("phi-rotation", po::value(&opt.phi_rot)->default_value(0),"Set a rotation angle phi")
    ("omega-rotation", po::value(&opt.omega_rot)->default_value(0),"Set a rotation angle omega")
    ("kappa-rotation", po::value(&opt.kappa_rot)->default_value(0),"Set a rotation angle kappa");

  po::options_description projection_options("Projection options");
  projection_options.add_options()
    ("reference-spheroid,r", po::value(&opt.reference_spheroid),"Set a reference surface to a hard coded value (one of [ earth, moon, mars].  This will override manually set datum information.")
    ("semi-major-axis", po::value(&opt.semi_major),"Set the dimensions of the datum.")
    ("semi-minor-axis", po::value(&opt.semi_minor),"Set the dimensions of the datum.")
    ("sinusoidal", "Save using a sinusoidal projection")
    ("mercator", "Save using a Mercator projection")
    ("transverse-mercator", "Save using a transverse Mercator projection")
    ("orthographic", "Save using an orthographic projection")
    ("stereographic", "Save using a stereographic projection")
    ("lambert-azimuthal", "Save using a Lambert azimuthal projection")
    ("utm", po::value(&opt.utm_zone), "Save using a UTM projection with the given zone")
    ("proj-lat", po::value(&opt.proj_lat), "The center of projection latitude (if applicable)")
    ("proj-lon", po::value(&opt.proj_lon), "The center of projection longitude (if applicable)")
    ("proj-scale", po::value(&opt.proj_scale), "The projection scale (if applicable)")
    ("dem-spacing,s", po::value(&opt.dem_spacing)->default_value(0.0), "Set the DEM post size (if this value is 0, the post spacing size is computed for you)");

  po::options_description general_options("General Options");
  general_options.add_options()
    ("nodata-value", po::value(&opt.nodata_value), "Nodata value to use on output. This is the same as default-value.")
    ("use-alpha", po::bool_switch(&opt.has_alpha)->default_value(false),
     "Create images that have an alpha channel")
    ("normalized,n", po::bool_switch(&opt.do_normalize)->default_value(false),
     "Also write a normalized version of the DEM (for debugging)")
    ("orthoimage", po::value(&opt.texture_filename), "Write an orthoimage based on the texture file given as an argument to this command line option")
    ("errorimage", po::bool_switch(&opt.do_error)->default_value(false), "Write a triangule error image.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file");
  general_options.add( manipulation_options );
  general_options.add( projection_options );
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.pointcloud_filename), "Input Point Cloud");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("<point-cloud> ...");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options,
                             positional, positional_desc, usage );

  if ( opt.pointcloud_filename.empty() )
    vw_throw( ArgumentErr() << "Missing point cloud.\n"
              << usage << general_options );
  if ( opt.out_prefix.empty() )
    opt.out_prefix =
      prefix_from_pointcloud_filename( opt.pointcloud_filename );

  boost::to_lower( opt.reference_spheroid );
  if ( vm.count("sinusoidal") )          opt.projection = SINUSOIDAL;
  else if ( vm.count("mercator") )       opt.projection = MERCATOR;
  else if ( vm.count("transverse-mercator") ) opt.projection = TRANSVERSEMERCATOR;
  else if ( vm.count("orthographic") )   opt.projection = ORTHOGRAPHIC;
  else if ( vm.count("stereographic") )  opt.projection = STEREOGRAPHIC;
  else if ( vm.count("lambert-azimuthal") ) opt.projection = LAMBERTAZIMUTHAL;
  else if ( vm.count("utm") )            opt.projection = UTM;
  else                                   opt.projection = PLATECARREE;
  opt.has_nodata_value = vm.count("nodata-value");
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    DiskImageView<Vector4> point_disk_image(opt.pointcloud_filename);
    ImageViewRef<Vector3> point_image = select_points(point_disk_image);

    // Apply an (optional) rotation to the 3D points before building the mesh.
    if (opt.phi_rot != 0 || opt.omega_rot != 0 || opt.kappa_rot != 0) {
      vw_out() << "\t--> Applying rotation sequence: " << opt.rot_order
               << "      Angles: " << opt.phi_rot << "   "
               << opt.omega_rot << "  " << opt.kappa_rot << "\n";
      point_image =
        point_transform( point_image,
                         math::euler_to_rotation_matrix(opt.phi_rot, opt.omega_rot,
                                                        opt.kappa_rot, opt.rot_order) );
    }

    // Select a cartographic datum. There are several hard coded datums
    // that can be used here, or the user can specify their own.
    cartography::Datum datum;
    if ( opt.reference_spheroid != "" ) {
      if (opt.reference_spheroid == "mars") {
        datum.set_well_known_datum("D_MARS");
        vw_out() << "\t--> Re-referencing altitude values using standard MOLA\n";
      } else if (opt.reference_spheroid == "moon") {
        datum.set_well_known_datum("D_MOON");
        vw_out() << "\t--> Re-referencing altitude values using standard lunar\n";
      } else if (opt.reference_spheroid == "earth") {
        vw_out() << "\t--> Re-referencing altitude values using WGS84\n";
      } else {
        vw_throw( ArgumentErr() << "\t--> Unknown reference spheriod: "
                  << opt.reference_spheroid
                  << ". Current options are [ earth, moon, mars ]\nExiting." );
      }
      vw_out() << "\t    Axes [" << datum.semi_major_axis() << " " << datum.semi_minor_axis() << "] meters\n";
    } else if (opt.semi_major != 0 && opt.semi_minor != 0) {
      vw_out() << "\t--> Re-referencing altitude values to user supplied datum.\n"
               << "\t    Semi-major: " << opt.semi_major << "  Semi-minor: " << opt.semi_minor << "\n";
      datum = cartography::Datum("User Specified Datum",
                                 "User Specified Spheroid",
                                 "Reference Meridian",
                                 opt.semi_major, opt.semi_minor, 0.0);
    }

    // Set up the georeferencing information.  We specify everything
    // here except for the affine transform, which is defined later once
    // we know the bounds of the orthorasterizer view.  However, we can
    // still reproject the points in the point image without the affine
    // transform because this projection never requires us to convert to
    // or from pixel space.
    GeoReference georef(datum);

    // If the data was left in cartesian coordinates, we need to give
    // the DEM a projection that uses some physical units (meters),
    // rather than lon, lat.  This is actually mainly for compatibility
    // with Viz, and it's sort of a hack, but it's left in for the time
    // being.
    //
    // Otherwise, we honor the user's requested projection and convert
    // the points if necessary.
    switch( opt.projection ) {
    case SINUSOIDAL:
      georef.set_sinusoidal(opt.proj_lon); break;
    case MERCATOR:
      georef.set_mercator(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
    case TRANSVERSEMERCATOR:
      georef.set_transverse_mercator(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
    case ORTHOGRAPHIC:
      georef.set_orthographic(opt.proj_lat,opt.proj_lon); break;
    case STEREOGRAPHIC:
      georef.set_stereographic(opt.proj_lat,opt.proj_lon,opt.proj_scale); break;
    case LAMBERTAZIMUTHAL:
      georef.set_lambert_azimuthal(opt.proj_lat,opt.proj_lon); break;
    case UTM:
      georef.set_UTM( opt.utm_zone ); break;
    default: // Handles plate carree
      break;
    }

    // Determine if we should using a longitude range between
    // [-180, 180] or [0,360]. We determine this by looking at the
    // average location of the points. If the average location has a
    // negative x value (think in ECEF coordinates) then we should
    // be using [0,360].
    int32 subsample_amt = int32(norm_2(Vector2(point_image.cols(),point_image.rows()))/1024.0);
    if (subsample_amt < 1 ) subsample_amt = 1;
    PixelAccumulator<MeanAccumulator<Vector3> > mean_accum;
    for_each_pixel( subsample(point_image, subsample_amt),
                    mean_accum,
                    TerminalProgressCallback("asp","Statistics: ") );
    Vector3 avg_location = mean_accum.value();

    // We trade off readability here to avoid ImageViewRef dereferences
    if (opt.x_offset != 0 || opt.y_offset != 0 || opt.z_offset != 0) {
      vw_out() << "\t--> Applying offset: " << opt.x_offset
               << " " << opt.y_offset << " " << opt.z_offset << "\n";
      point_image =
        geodetic_to_point(point_image_offset(recenter_longitude(cartesian_to_geodetic(point_image,georef),
                                                                avg_location.x() >= 0 ? 0 : 180),
                                             Vector3(opt.x_offset,
                                                     opt.y_offset,
                                                     opt.z_offset)),georef);
    } else {
      point_image =
        geodetic_to_point(recenter_longitude(cartesian_to_geodetic(point_image,georef), avg_location.x() >= 0 ? 0 : 180),georef);
    }

    // Rasterize the results to a temporary file on disk so as to speed
    // up processing in the orthorasterizer, which accesses each pixel
    // multiple times.
    typedef BlockRasterizeView<ImageViewRef<Vector3> > PointCacheT;
    BlockRasterizeView<ImageViewRef<Vector3> > point_image_cache =
      block_cache(point_image,Vector2i(vw_settings().default_tile_size(),
                                       vw_settings().default_tile_size()),0);

    // write out the DEM, texture, and extrapolation mask as
    // georeferenced files.
    OrthoRasterizerView<PixelGray<float>, PointCacheT>
      rasterizer(point_image_cache, select_channel(point_image_cache,2),
                 opt.dem_spacing, TerminalProgressCallback("asp","QuadTree: ") );
    if (!opt.has_nodata_value) {
      rasterizer.set_use_minz_as_default(true);
    } else {
      rasterizer.set_use_minz_as_default(false);
      rasterizer.set_default_value(opt.nodata_value);
    }

    vw_out() << "\t--> DEM spacing: " << rasterizer.spacing() << " pt/px\n";
    vw_out() << "\t             or: " << 1.0/rasterizer.spacing() << " px/pt\n";

    if (opt.has_alpha)
      rasterizer.set_use_alpha(true);

    // Now we are ready to specify the affine transform.
    georef.set_transform(rasterizer.geo_transform());
    vw_out() << "\nOutput Georeference: \n\t" << georef << std::endl;

    { // Write out the DEM.
      if ( opt.output_file_type == "tif" && opt.has_nodata_value ) {
        boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc( opt.out_prefix + "-DEM.tif", rasterizer, opt) );
        rsrc->set_nodata_write( opt.nodata_value );
        write_georeference( *rsrc, georef );
        block_write_image( *rsrc, rasterizer,
                           TerminalProgressCallback("asp","DEM: ") );
      } else {
        asp::block_write_gdal_image(
          opt.out_prefix + "-DEM." + opt.output_file_type,
          rasterizer, georef, opt,
          TerminalProgressCallback("asp","DEM: ") );
      }
    }

    // Write triangulation error image if requested
    if ( opt.do_error ) {
      rasterizer.set_texture( select_channel(point_disk_image,3) );
      if ( opt.output_file_type == "tif" && opt.has_nodata_value ) {
        boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc( opt.out_prefix + "-DEMError.tif", rasterizer, opt) );
        rsrc->set_nodata_write( opt.nodata_value );
        write_georeference( *rsrc, georef );
        block_write_image( *rsrc, rasterizer,
                           TerminalProgressCallback("asp","DEMError: ") );
      } else {
        asp::write_gdal_georeferenced_image(opt.out_prefix + "-DEMError."+opt.output_file_type,
                                            rasterizer, georef, opt, TerminalProgressCallback("asp","DEMError:") );
      }
    }

    // Write DRG if the user requested and provided a texture file
    if (!opt.texture_filename.empty()) {
      rasterizer.set_use_minz_as_default(false);
      DiskImageView<PixelGray<float> > texture(opt.texture_filename);
      rasterizer.set_texture(texture);
      asp::write_gdal_georeferenced_image(opt.out_prefix + "-DRG.tif",
             rasterizer, georef, opt, TerminalProgressCallback("asp","DRG:") );
    }

    // Write out a normalized version of the DEM, if requested (for debugging)
    if (opt.do_normalize) {
      DiskImageView<PixelGray<float> >
        dem_image(opt.out_prefix + "-DEM." + opt.output_file_type);

      if ( opt.has_nodata_value ) {
        asp::block_write_gdal_image(
          opt.out_prefix + "-DEM-normalized.tif",
          apply_mask(channel_cast_rescale<uint8>(normalize(create_mask(dem_image,opt.nodata_value)))),
          georef, opt, TerminalProgressCallback("asp","Normalized:") );
      } else {
        asp::block_write_gdal_image(
          opt.out_prefix + "-DEM-normalized.tif",
          channel_cast_rescale<uint8>(normalize(dem_image)),
          georef, opt, TerminalProgressCallback("asp","Normalized:") );
      }
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}

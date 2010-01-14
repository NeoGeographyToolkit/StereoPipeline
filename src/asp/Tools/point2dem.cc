// __BEGIN_LICENSE__
// Copyright (C) 2006-2009 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


/// \file point2dem.cc
///

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <stdlib.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Math.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::cartography;

#include <asp/Core/OrthoRasterizer.h>

// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_pointcloud_filename(std::string const& filename) {
  std::string result = filename;

  // First case: filenames that match <prefix>-PC.<suffix>
  int index = result.rfind("-PC.");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // Second case: filenames that match <prefix>.<suffix>
  index = result.rfind(".");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // No match
  return result;
}

// Apply an offset to the points in the PointImage
class PointOffsetFunc : public UnaryReturnSameType {
  Vector3 m_offset;

public:
  PointOffsetFunc(Vector3 const& offset) : m_offset(offset) {}

  template <class T>
  T operator()(T const& p) const {
    if (p == T()) return p;
    return p + m_offset;
  }
};

template <class ImageT>
UnaryPerPixelView<ImageT, PointOffsetFunc>
inline point_image_offset( ImageViewBase<ImageT> const& image, Vector3 const& offset) {
  return UnaryPerPixelView<ImageT,PointOffsetFunc>( image.impl(), PointOffsetFunc(offset) );
}


// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

class PointTransFunc : public ReturnFixedType<Vector3> {
  Matrix3x3 m_trans;
public:
  PointTransFunc(Matrix3x3 trans) : m_trans(trans) {}
  Vector3 operator() (Vector3 const& pt) const { return m_trans*pt; }
};


int main( int argc, char *argv[] ) {
  set_debug_level(VerboseDebugMessage+11);

  std::string pointcloud_filename, out_prefix = "", output_file_type, texture_filename;
  float dem_spacing, default_value=0;
  double semi_major, semi_minor;
  std::string reference_spheroid;
  double phi_rot, omega_rot, kappa_rot;
  std::string rot_order;
  double proj_lat=0, proj_lon=0, proj_scale=1;
  double x_offset, y_offset, z_offset;
  unsigned utm_zone;

  po::options_description desc("Options");
  desc.add_options()
    ("help,h", "Display this help message")
    ("default-value", po::value<float>(&default_value), "Explicitly set the default (missing pixel) value.  By default, the min z value is used.")
    ("use-alpha", "Create images that have an alpha channel")
    ("dem-spacing,s", po::value<float>(&dem_spacing)->default_value(0), "Set the DEM post size (if this value is 0, the post spacing size is computed for you)")
    ("normalized,n", "Also write a normalized version of the DEM (for debugging)")
    ("orthoimage", po::value<std::string>(&texture_filename), "Write an orthoimage based on the texture file given as an argument to this command line option")
    ("grayscale", "Use grayscale image processing for creating the orthoimage")
    ("offset-files", "Also write a pair of ascii offset files (for debugging)")
    ("input-file", po::value<std::string>(&pointcloud_filename), "Explicitly specify the input file")
    ("texture-file", po::value<std::string>(&texture_filename), "Specify texture filename")
    ("output-prefix,o", po::value<std::string>(&out_prefix), "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("tif"), "Specify the output file")
    ("xyz-to-lonlat", "Convert from xyz coordinates to longitude, latitude, altitude coordinates.")
    ("reference-spheroid,r", po::value<std::string>(&reference_spheroid)->default_value(""),"Set a reference surface to a hard coded value (one of [moon , mars].  This will override manually set datum information.")
    ("semi-major-axis", po::value<double>(&semi_major)->default_value(0),"Set the dimensions of the datum.")
    ("semi-minor-axis", po::value<double>(&semi_minor)->default_value(0),"Set the dimensions of the datum.")
    ("x-offset", po::value<double>(&x_offset)->default_value(0), "Add a horizontal offset to the DEM")
    ("y-offset", po::value<double>(&y_offset)->default_value(0), "Add a horizontal offset to the DEM")
    ("z-offset", po::value<double>(&z_offset)->default_value(0), "Add a vertical offset to the DEM")

    ("sinusoidal", "Save using a sinusoidal projection")
    ("mercator", "Save using a Mercator projection")
    ("transverse-mercator", "Save using a transverse Mercator projection")
    ("orthographic", "Save using an orthographic projection")
    ("stereographic", "Save using a stereographic projection")
    ("lambert-azimuthal", "Save using a Lambert azimuthal projection")
    ("utm", po::value<unsigned>(&utm_zone), "Save using a UTM projection with the given zone")
    ("proj-lat", po::value<double>(&proj_lat), "The center of projection latitude (if applicable)")
    ("proj-lon", po::value<double>(&proj_lon), "The center of projection longitude (if applicable)")
    ("proj-scale", po::value<double>(&proj_scale), "The projection scale (if applicable)")
    ("rotation-order", po::value<std::string>(&rot_order)->default_value("xyz"),"Set the order of an euler angle rotation applied to the 3D points prior to DEM rasterization")
    ("phi-rotation", po::value<double>(&phi_rot)->default_value(0),"Set a rotation angle phi")
    ("omega-rotation", po::value<double>(&omega_rot)->default_value(0),"Set a rotation angle omega")
    ("kappa-rotation", po::value<double>(&kappa_rot)->default_value(0),"Set a rotation angle kappa");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <pointcloud> ..." << std::endl;
  usage << std::endl << desc << std::endl;

  if( vm.count("help") ) {
    vw_out() << usage.str();
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    vw_out() << "Error: Must specify exactly one pointcloud file and one texture file!" << std::endl;
    vw_out() << usage.str();
    return 1;
  }

  if( out_prefix == "" ) {
    out_prefix = prefix_from_pointcloud_filename(pointcloud_filename);
  }

  DiskImageView<Vector3> point_disk_image(pointcloud_filename);
  ImageViewRef<Vector3> point_image = point_disk_image;

  // Apply an (optional) rotation to the 3D points before building the mesh.
  if (phi_rot != 0 || omega_rot != 0 || kappa_rot != 0) {
    vw_out() << "\t--> Applying rotation sequence: " << rot_order << "      Angles: " << phi_rot << "   " << omega_rot << "  " << kappa_rot << "\n";
    Matrix3x3 rotation_trans = math::euler_to_rotation_matrix(phi_rot,omega_rot,kappa_rot,rot_order);
    point_image = per_pixel_filter(point_image, PointTransFunc(rotation_trans));
  }

  if (vm.count("xyz-to-lonlat") ) {
    vw_out() << "\t--> Reprojecting points into longitude, latitude, altitude.\n";
    point_image = cartography::xyz_to_lon_lat_radius(point_image);
  }

  // Select a cartographic DATUM.  There are several hard coded datums
  // that can be used here, or the user can specify their own.
  cartography::Datum datum;
  if ( reference_spheroid != "" ) {
    if (reference_spheroid == "mars") {
      const double MOLA_PEDR_EQUATORIAL_RADIUS = 3396000.0;
      vw_out() << "\t--> Re-referencing altitude values using standard MOLA spherical radius: " << MOLA_PEDR_EQUATORIAL_RADIUS << "\n";
      datum = cartography::Datum("D_MARS",
                                 "MARS",
                                 "Reference Meridian",
                                 MOLA_PEDR_EQUATORIAL_RADIUS,
                                 MOLA_PEDR_EQUATORIAL_RADIUS,
                                 0.0);
    } else if (reference_spheroid == "moon") {
      const double LUNAR_RADIUS = 1737400;
      vw_out() << "\t--> Re-referencing altitude values using standard lunar spherical radius: " << LUNAR_RADIUS << "\n";
      datum = cartography::Datum("D_MOON",
                                 "MOON",
                                 "Reference Meridian",
                                 LUNAR_RADIUS,
                                 LUNAR_RADIUS,
                                 0.0);
    } else {
      vw_out() << "\t--> Unknown reference spheroid: " << reference_spheroid << ".  Current options are [ moon , mars ]\nExiting.\n\n";
      exit(0);
    }
  } else if (semi_major != 0 && semi_minor != 0) {
    vw_out() << "\t--> Re-referencing altitude values to user supplied datum.  Semi-major: " << semi_major << "  Semi-minor: " << semi_minor << "\n";
    datum = cartography::Datum("User Specified Datum",
                               "User Specified Spheroid",
                               "Reference Meridian",
                               semi_major, semi_minor, 0.0);
  }

  if (x_offset != 0 || y_offset != 0 || z_offset != 0) {
    vw_out() << "\t--> Applying offset: " << x_offset << " " << y_offset << " " << z_offset << "\n";
    point_image = point_image_offset(point_image, Vector3(x_offset,y_offset,z_offset));
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
  if (!vm.count("xyz-to-lonlat"))
    georef.set_mercator(0,0,1);
  else if( vm.count("sinusoidal") ) georef.set_sinusoidal(proj_lon);
  else if( vm.count("mercator") ) georef.set_mercator(proj_lat,proj_lon,proj_scale);
  else if( vm.count("transverse-mercator") ) georef.set_transverse_mercator(proj_lat,proj_lon,proj_scale);
  else if( vm.count("orthographic") ) georef.set_orthographic(proj_lat,proj_lon);
  else if( vm.count("stereographic") ) georef.set_stereographic(proj_lat,proj_lon,proj_scale);
  else if( vm.count("lambert-azimuthal") ) georef.set_lambert_azimuthal(proj_lat,proj_lon);
  else if( vm.count("utm") ) georef.set_UTM( utm_zone );

  if (vm.count("xyz-to-lonlat"))
    point_image = cartography::project_point_image(point_image, georef);

  // Rasterize the results to a temporary file on disk so as to speed
  // up processing in the orthorasterizer, which accesses each pixel
  // multiple times.
  DiskCacheImageView<Vector3> point_image_cache(point_image, "tif");


  // ----> For debugging: (for Larry) <-----
  //  write_image("test2.tif", channel_cast<float>(select_channel(point_image_cache,2)-1134.2), TerminalProgressCallback());

  // write out the DEM, texture, and extrapolation mask as
  // georeferenced files.
  OrthoRasterizerView<PixelGray<float> > rasterizer(point_image_cache, select_channel(point_image_cache,2), dem_spacing);
  if (!vm.count("default-value") ) {
    rasterizer.set_use_minz_as_default(true);
  } else {
    rasterizer.set_use_minz_as_default(false);
    rasterizer.set_default_value(default_value);
  }

  if (vm.count("use-alpha")) {
    rasterizer.set_use_alpha(true);
  }

  vw::BBox3 dem_bbox = rasterizer.bounding_box();
  vw_out() << "\nDEM Bounding box: " << dem_bbox << "\n";

  // Now we are ready to specify the affine transform.
  Matrix3x3 georef_affine_transform = rasterizer.geo_transform();
  vw_out() << "Georeferencing Transform: " << georef_affine_transform << "\n";
  georef.set_transform(georef_affine_transform);

  // Write out a georeferenced orthoimage of the DTM with alpha.
  if (vm.count("orthoimage")) {
    rasterizer.set_use_minz_as_default(false);
    DiskImageView<PixelGray<float> > texture(texture_filename);
    rasterizer.set_texture(texture);
    ImageViewRef<PixelGray<float> > block_drg_raster = block_cache(rasterizer, Vector2i(rasterizer.cols(), 2048), 0);
    if (vm.count("use-alpha")) {
      write_georeferenced_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(apply_mask(block_drg_raster,PixelGray<float>(-32000))), georef, TerminalProgressCallback() );
    }
    else {
      write_georeferenced_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(block_drg_raster), georef, TerminalProgressCallback() );
    }

  } else {

    { // Write out the DEM.
      vw_out() << "\nWriting DEM.\n";
      ImageViewRef<PixelGray<float> > block_dem_raster =
        block_cache(rasterizer, Vector2i(rasterizer.cols(), 2024), 0);
      write_georeferenced_image( out_prefix + "-DEM." + output_file_type,
                                 block_dem_raster, georef,
                                 TerminalProgressCallback());
    }

    // Write out a normalized version of the DTM (for debugging)
    if (vm.count("normalized")) {
      vw_out() << "\nWriting normalized DEM.\n";

      DiskImageView<PixelGray<float> > dem_image(out_prefix + "-DEM." + output_file_type);

      write_georeferenced_image( out_prefix + "-DEM-normalized.tif",
                                 channel_cast_rescale<uint8>(normalize(dem_image)),
                                 georef, TerminalProgressCallback() );
    }
  }

  // Write out the offset files
  if (vm.count("offset-files")) {
    vw_out() << "Offset: " << dem_bbox.min().x()/rasterizer.spacing() << "   " << dem_bbox.max().y()/rasterizer.spacing() << "\n";
    std::string offset_filename = out_prefix + "-DRG.offset";
    FILE* offset_file = fopen(offset_filename.c_str(), "w");
    fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.spacing()), -int(dem_bbox.max().y()/rasterizer.spacing()));
    fclose(offset_file);
    offset_filename = out_prefix + "-DEM-normalized.offset";
    offset_file = fopen(offset_filename.c_str(), "w");
    fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.spacing()), -int(dem_bbox.max().y()/rasterizer.spacing()));
    fclose(offset_file);
  }
  return 0;
}

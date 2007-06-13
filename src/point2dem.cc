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
#include <vw/Stereo.h>
#include <vw/Cartography.h>
using namespace vw;
using namespace vw::stereo;
using namespace vw::cartography;

#include "stereo.h"
#include "DEM.h"
#include "nff_terrain.h"
#include "OrthoRasterizer.h"


// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_XYZ; };
}

int main( int argc, char *argv[] ) {
  set_debug_level(VerboseDebugMessage+11);

  std::string input_file_name, out_prefix, output_file_type, texture_filename;
  unsigned cache_size, max_triangles;
  float dem_spacing, default_value;
  unsigned simplemesh_h_step, simplemesh_v_step;
  
  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("minz-is-default", "Use the smallest z value as the default (missing pixel) value")
    ("default_value", po::value<float>(&default_value)->default_value(0), "Explicitly set the default (missing pixel) value")
    ("dem-spacing,s", po::value<float>(&dem_spacing)->default_value(0), "Set the DEM post size (if this value is 0, the post spacing size is computed for you)")
    ("normalized,n", "Also write a normalized version of the DEM (for debugging)")    
    ("orthoimage", po::value<std::string>(&texture_filename), "Write an orthoimage based on the texture file given as an argument to this command line option")
    ("grayscale", "Use grayscale image processing for creating the orthoimage")
    ("offset-files", "Also write a pair of ascii offset files (for debugging)")    
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
    ("input-file", po::value<std::string>(&input_file_name), "Explicitly specify the input file")
    ("texture-file", po::value<std::string>(&texture_filename), "Specify texture filename")
    ("output-prefix,o", po::value<std::string>(&out_prefix)->default_value("terrain"), "Specify the output prefix")
    ("output-filetype,t", po::value<std::string>(&output_file_type)->default_value("tif"), "Specify the output file");

  po::positional_options_description p;
  p.add("input-file", 1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  Cache::system_cache().resize( cache_size*1024*1024 ); 

  if( vm.count("help") ) {
    std::cout << desc << std::endl;
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    std::cout << "Error: Must specify exactly one pointcloud file and one texture file!" << std::endl;
    std::cout << desc << std::endl;
    return 1;
  }

  DiskImageView<Vector3> point_image(input_file_name);

  // Write out the DEM, texture, and extrapolation mask
  // as georeferenced files.
  vw::cartography::OrthoRasterizer<Vector3> rasterizer(point_image);
  rasterizer.set_dem_spacing(dem_spacing);
  if (vm.count("minz-is-default") )
    rasterizer.use_minz_as_default = true; 
  else       
    rasterizer.use_minz_as_default = true; 
  rasterizer.set_default_value(default_value);    
  vw::BBox<float,3> dem_bbox = rasterizer.bounding_box();
  std::cout << "DEM Bounding box: " << dem_bbox << "\n";
  
  // Set up the georeferencing information
  GeoReference georef;

  // FIXME: Use Mercator projection for now 
  georef.set_mercator(0,0,1);
  georef.set_transform(rasterizer.geo_transform());
  ImageView<PixelGrayA<float> > ortho_image = rasterizer(vw::select_channel(point_image, 2));    
  write_georeferenced_image(out_prefix + "-DEM." + output_file_type, ortho_image, georef);
  if (vm.count("write-normalized"))
    write_georeferenced_image(out_prefix + "-DEM-normalized.tif", channel_cast_rescale<uint8>(ortho_image), georef);

  
  // Write out a georeferenced orthoimage of the DTM with alpha.
  if (vm.count("orthoimage")) {
    if (vm.count("grayscale")) {
      ImageView<PixelGrayA<float> > ortho_image = rasterizer(vw::select_channel(point_image, 2));    
      DiskImageView<PixelGray<float> > texture(texture_filename);
      ortho_image = rasterizer(select_channel(texture, 0));
      write_georeferenced_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(ortho_image), georef);
    } else {
      ImageView<PixelRGB<float> > ortho_image = rasterizer(vw::select_channel(point_image, 2));    
      DiskImageView<PixelRGB<float> > texture(texture_filename);
      ortho_image = rasterizer(select_channel(texture, 0));
      write_georeferenced_image(out_prefix + "-DRG.tif", channel_cast_rescale<uint8>(ortho_image), georef);
    }
  }

  if (vm.count("offset-files")) {
    // Write out the offset files
    std::cout << "Offset: " << dem_bbox.min().x()/rasterizer.dem_spacing() << "   " << dem_bbox.max().y()/rasterizer.dem_spacing() << "\n";
    std::string offset_filename = out_prefix + "-DRG.offset";
    FILE* offset_file = fopen(offset_filename.c_str(), "w");
    fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.dem_spacing()), -int(dem_bbox.max().y()/rasterizer.dem_spacing()));
    fclose(offset_file);
    offset_filename = out_prefix + "-DEM-normalized.offset";
    offset_file = fopen(offset_filename.c_str(), "w");
    fprintf(offset_file, "%d\n%d\n", int(dem_bbox.min().x()/rasterizer.dem_spacing()), -int(dem_bbox.max().y()/rasterizer.dem_spacing()));
    fclose(offset_file);
  }
  return 0;
}

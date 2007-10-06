// __BEGIN_LICENSE__
// 
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2006 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
// 
// __END_LICENSE__

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "MRO/DiskImageResourceDDD.h"

#include <vw/Core/Cache.h>
#include <vw/Core/ProgressCallback.h>
#include <vw/Math/Matrix.h>
#include <vw/Image/Transform.h>
#include <vw/Image/Palette.h>
#include <vw/FileIO/DiskImageResource.h>
#include <vw/FileIO/DiskImageResourceJPEG.h>
#include <vw/FileIO/DiskImageResourceGDAL.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/GeoTransform.h>
#include <vw/Cartography/FileIO.h>
#include <vw/Mosaic/ImageComposite.h>
#include <vw/Mosaic/KMLQuadTreeGenerator.h>
using namespace vw;
using namespace vw::math;
using namespace vw::cartography;
using namespace vw::mosaic;

//  mask_zero_pixels()
//
struct MaskZeroPixelFunc: public vw::ReturnFixedType<vw::PixelGrayA<vw::uint16> > {
  
  vw::PixelGrayA<uint16> operator() (vw::PixelGray<uint16> const& pix) const {
    if (pix.v() == 0) 
      return vw::PixelGrayA<uint16>();  // Mask pixel
    else
      return vw::PixelGrayA<uint16>(pix.v());
  }
};

template <class ViewT>
vw::UnaryPerPixelView<ViewT, MaskZeroPixelFunc> 
mask_zero_pixels(vw::ImageViewBase<ViewT> const& view) {
  return vw::per_pixel_filter(view.impl(), MaskZeroPixelFunc());
}

/// \cond INTERNAL
template <class PixelT>
class RescalePixelsWithAlphaFunc: public UnaryReturnSameType {
  typedef typename CompoundChannelType<PixelT>::type channel_type;
  channel_type m_old_min, m_new_min,m_old_max, m_new_max;
  double m_old_to_new_ratio;
public:
  RescalePixelsWithAlphaFunc( channel_type old_min, channel_type old_max, 
                              channel_type new_min, channel_type new_max )
    : m_old_min(old_min), m_new_min(new_min), m_old_max(old_max), m_new_max(new_max)
  {
    if( old_max == old_min ) { m_old_to_new_ratio = 0.0; }
    else { m_old_to_new_ratio = (new_max - new_min)/(double)(old_max - old_min); }
  }
  
  PixelT operator()( PixelT value ) const {
    PixelT result;
    for (int i = 0; i < CompoundNumChannels<PixelT>::value-1; ++i) {
      result[i] = ((value[i] - m_old_min) * m_old_to_new_ratio + m_new_min);
      // Limit to the min and max values
      if (result[i] > m_new_max) result[i] = m_new_max;
      if (result[i] < m_new_min) result[i] = m_new_min;
    }
    // Copy the alpha value
    result[CompoundNumChannels<PixelT>::value-1] = value[CompoundNumChannels<PixelT>::value-1];
    return result;
  }
};
/// \endcond

/// Renormalize the values in an image to fall within the range [low,high).
template <class ImageT, class ValT>
UnaryPerPixelView<ImageT, RescalePixelsWithAlphaFunc<typename ImageT::pixel_type> >
inline rescale_pixels_with_alpha( ImageViewBase<ImageT> const& image, ValT old_low, ValT old_high, ValT new_low, ValT new_high ) {
  typedef RescalePixelsWithAlphaFunc<typename ImageT::pixel_type> func_type;
  func_type func(old_low, old_high, new_low, new_high );
  return UnaryPerPixelView<ImageT, func_type >( image.impl(), func );
}


Vector2 malin_pixel_to_lonlat(Vector2 pix, double x_offset, double y_offset, double extent, double fullwidth) {
  double lon, lat;
  int m_pole_flag = 1;
  double x = pix[0];
  double y = pix[1];

  double prime_meridian = 0;
  double p1 = 1./(tan(M_PI/4. - (M_PI/180.)*extent/2.));
  
  double px = 2.*((x + x_offset) - fullwidth/2.)/(p1*fullwidth);
  double py = 2.*(fullwidth/2. - (y + y_offset))/(p1*fullwidth);
  
  if (m_pole_flag == 0) py = -py;
  
  if (py != 0) {
    lon = (180./M_PI)*atan2(px, py);
  } else {
    if (px > 0) lon = 90.0;
    else lon = 270;
  }

    lon += 180. + prime_meridian;
    if (lon > 360.0) lon -= 360.;

    lat = 90. - (180./M_PI)*2.*atan(sqrt(px*px + py*py));
    if (m_pole_flag == 0) lat = -lat;

    lon = 360-lon;
    if (lon > 180) lon -= 360;
    return Vector2(lon, lat);
  }




int main( int argc, char *argv[] ) {
  
  vw::DiskImageResource::register_file_type( ".ddd", vw::DiskImageResourceDDD::type_static(), &vw::DiskImageResourceDDD::construct_open, &vw::DiskImageResourceDDD::construct_create );

  std::vector<std::string> image_files;
  std::string output_file_name;
  std::string output_file_type;
  double north_lat=90.0, south_lat=-90.0;
  double east_lon=180.0, west_lon=-180.0;
  double proj_lat=0, proj_lon=0, proj_scale=1;
  unsigned utm_zone;
  int patch_size, patch_overlap;
  float jpeg_quality;
  unsigned cache_size;
  int max_lod_pixels;
  double nudge_x=0, nudge_y=0;
  std::string palette_file;
  float palette_scale=1.0, palette_offset=0.0;
  int draw_order_offset;

  po::options_description general_options("General Options");
  general_options.add_options()
    ("output-name,o", po::value<std::string>(&output_file_name)->default_value("output"), "Specify the base output filename")
    ("quiet,q", "Quiet output")
    ("verbose,v", "Verbose output")
    ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
    ("help", "Display this help message");

  po::options_description projection_options("Projection Options");
  projection_options.add_options()
    ("north", po::value<double>(&north_lat), "The northernmost latitude in degrees")
    ("south", po::value<double>(&south_lat), "The southernmost latitude in degrees")
    ("east", po::value<double>(&east_lon), "The easternmost latitude in degrees")
    ("west", po::value<double>(&west_lon), "The westernmost latitude in degrees")
    ("sinusoidal", "Assume a sinusoidal projection")
    ("mercator", "Assume a Mercator projection")
    ("transverse-mercator", "Assume a transverse Mercator projection")
    ("orthographic", "Assume an orthographic projection")
    ("stereographic", "Assume a stereographic projection")
    ("lambert-azimuthal", "Assume a Lambert azimuthal projection")
    ("utm", po::value<unsigned>(&utm_zone), "Assume UTM projection with the given zone")
    ("proj-lat", po::value<double>(&proj_lat), "The center of projection latitude (if applicable)")
    ("proj-lon", po::value<double>(&proj_lon), "The center of projection longitude (if applicable)")
    ("proj-scale", po::value<double>(&proj_scale), "The projection scale (if applicable)")
    ("nudge-x", po::value<double>(&nudge_x), "Nudge the image, in projected coordinates")
    ("nudge-y", po::value<double>(&nudge_y), "Nudge the image, in projected coordinates");
    
  po::options_description output_options("Output Options");
  output_options.add_options()
    ("file-type", po::value<std::string>(&output_file_type)->default_value("auto"), "Output file type")
    ("jpeg-quality", po::value<float>(&jpeg_quality)->default_value(0.75), "JPEG quality factor (0.0 to 1.0)")
    ("palette-file", po::value<std::string>(&palette_file), "Apply a palette from the given file")
    ("palette-scale", po::value<float>(&palette_scale), "Apply a scale factor before applying the palette")
    ("palette-offset", po::value<float>(&palette_offset), "Apply an offset before applying the palette")
    ("patch-size", po::value<int>(&patch_size)->default_value(256), "Patch size, in pixels")
    ("patch-overlap", po::value<int>(&patch_overlap)->default_value(0), "Patch overlap, in pixels (must be even)")
    ("patch-crop", "Crop output patches")
    ("max-lod-pixels", po::value<int>(&max_lod_pixels)->default_value(1024), "Max LoD in pixels, or -1 for none")
    ("draw-order-offset", po::value<int>(&draw_order_offset)->default_value(200), "Set an offset for the KML <drawOrder> tag for this overlay")
    ("composite-overlay", "Composite images using direct overlaying (default)")
    ("composite-multiband", "Composite images using multi-band blending");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-file", po::value<std::vector<std::string> >(&image_files));

  po::options_description options("Allowed Options");
  options.add(general_options).add(projection_options).add(output_options).add(hidden_options);

  po::positional_options_description p;
  p.add("input-file", -1);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: image2kml [options] <filename>..." << std::endl << std::endl;
  usage << general_options << std::endl;
  usage << output_options << std::endl;
  usage << projection_options << std::endl;

  if( vm.count("help") ) {
    std::cout << usage.str();
    return 1;
  }

  if( vm.count("input-file") < 1 ) {
    std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }

  if( patch_size <= 0 ) {
    std::cerr << "Error: The patch size must be a positive number!  (You specified " << patch_size << ".)" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }
    
  if( patch_overlap<0 || patch_overlap>=patch_size || patch_overlap%2==1 ) {
    std::cerr << "Error: The patch overlap must be an even nonnegative number" << std::endl;
    std::cerr << "smaller than the patch size!  (You specified " << patch_overlap << ".)" << std::endl << std::endl;
    std::cout << usage.str();
    return 1;
  }
  
  TerminalProgressCallback tpc;
  const ProgressCallback *progress = &tpc;
  if( vm.count("verbose") ) {
    set_debug_level(VerboseDebugMessage);
    progress = &ProgressCallback::dummy_instance();
  }
  else if( vm.count("quiet") ) {
    set_debug_level(WarningMessage);
  }

  DiskImageResourceJPEG::set_default_quality( jpeg_quality );
  Cache::system_cache().resize( cache_size*1024*1024 );

  GeoReference output_georef;
  output_georef.set_well_known_geogcs("WGS84");
  int total_resolution = 1024;

  // Use a MARS spheroid datum
  const double MOLA_PEDR_EQUATORIAL_RADIUS =  3396000.0;
  vw::cartography::Datum mars_datum;
  mars_datum.name() = "IAU2000 Mars Spheroid";
  mars_datum.spheroid_name() = "IAU2000 Mars Spheroid";
  mars_datum.meridian_name() = "Mars Prime Meridian";
  mars_datum.set_semi_major_axis(MOLA_PEDR_EQUATORIAL_RADIUS);
  mars_datum.set_semi_minor_axis(MOLA_PEDR_EQUATORIAL_RADIUS);

  // Read in georeference info and compute total resolution
  std::vector<GeoReference> georeferences;
  for( unsigned i=0; i<image_files.size(); ++i ) {
    std::cout << "Adding file " << image_files[i] << std::endl;
    DiskImageResourceDDD file_resource( image_files[i] );

    // Pull the relevent metadata out of the image header
    double fullwidth = atol(file_resource.query("projection_fullwidth").c_str());
    double extent = atol(file_resource.query("projection_extent").c_str());
    double projection_x_offset = atol(file_resource.query("projection_x_offset").c_str());
    double projection_y_offset = atol(file_resource.query("projection_y_offset").c_str());

    // Compute the resolution of the image in the projected space.
    double p1 = 1./(tan(M_PI/4. - extent/2.));
    double pixels_per_degree = (p1/2.)*M_PI*fullwidth/360.0;
    double meters_per_degree = 2*M_PI*mars_datum.semi_major_axis()/360.0;
    double meters_per_pixel = meters_per_degree / pixels_per_degree;
    std::cout << "Image Scale: " << meters_per_pixel << " meters per pixel.\n";

    // Set up the affine transform matrix
    Matrix3x3 m = identity_matrix<3>();
    m(0,0) = meters_per_pixel;
    m(1,1) = -meters_per_pixel;
    m(0,2) = (projection_x_offset-fullwidth/2)*m(0,0);
    m(1,2) = (projection_y_offset-fullwidth/2)*m(1,1);
    std::cout << "Affine Transform: " << m << "\n";
    GeoReference input_georef(mars_datum, m);
    input_georef.set_stereographic(90,0,1,0,0);

    if( vm.count("nudge-x") || vm.count("nudge-y") ) {
      Matrix3x3 m = input_georef.transform();
      m(0,2) += nudge_x;
      m(1,2) += nudge_y;
      input_georef.set_transform( m );
    }
    
    georeferences.push_back( input_georef );

    GeoTransform geotx( input_georef, output_georef );
    std::cout << "OUTPUT: " << geotx.forward(Vector2(0,0)) << "\n";
    Vector2 center_pixel( file_resource.cols()/2, file_resource.rows()/2 );
    int resolution = GlobalKMLTransform::compute_resolution( geotx, center_pixel );
    if( resolution > total_resolution ) total_resolution = resolution;
  }

  // Configure the composite
  ImageComposite<PixelGrayA<uint8> > composite;
  GlobalKMLTransform kmltx( total_resolution );

  // Add the transformed input files to the composite
  for( unsigned i=0; i<image_files.size(); ++i ) {
    GeoTransform geotx( georeferences[i], output_georef );
    ImageViewRef<PixelGrayA<uint8> > source = channel_cast<uint8>(rescale_pixels_with_alpha(mask_zero_pixels(vw::DiskImageView<uint16>( image_files[i] )), 200, 1100, 0, 255));
    
    std::cout << "Preparing to write image... \n";
    BBox2i bbox = compose(kmltx,geotx).forward_bbox( BBox2i(0,0,source.cols(),source.rows()) );
    std::cout << "Bounding box: " << bbox << "\n";
    composite.insert( crop( transform( source, compose(kmltx,geotx) ), bbox ),
                      bbox.min().x(), bbox.min().y() );
  }

  // Compute a tighter Google Earth coordinate system aligned bounding box
  BBox2i bbox = composite.bbox();
  bbox.crop( BBox2i(0,0,total_resolution,total_resolution) );
  int dim = 2 << (int)(log( (std::max)(bbox.width(),bbox.height()) )/log(2));
  if ( dim > total_resolution ) dim = total_resolution;
  BBox2i total_bbox( (bbox.min().x()/dim)*dim, (bbox.min().y()/dim)*dim, dim, dim );
  if ( ! total_bbox.contains( bbox ) ) {
    if( total_bbox.max().x() == total_resolution ) total_bbox.min().x() -= dim;
    else total_bbox.max().x() += dim;
    if( total_bbox.max().y() == total_resolution ) total_bbox.min().y() -= dim;
    else total_bbox.max().y() += dim;
  }

  // Prepare the composite
  if( vm.count("composite-multiband") ) {
    std::cout << "Preparing composite..." << std::endl;
    composite.prepare( total_bbox, *progress );
  }
  else {
    composite.set_draft_mode( true );
    composite.prepare( total_bbox );
  }
  BBox2i data_bbox = composite.bbox();
  data_bbox.crop( BBox2i(0,0,total_bbox.width(),total_bbox.height()) );

  // Prepare the quadtree
  BBox2 ll_bbox( -180.0 + (360.0*total_bbox.min().x())/total_resolution, 
                 180.0 - (360.0*total_bbox.max().y())/total_resolution,
                 (360.0*total_bbox.width())/total_resolution,
                 (360.0*total_bbox.height())/total_resolution );
  KMLQuadTreeGenerator<PixelGrayA<uint8> > quadtree( output_file_name, composite, ll_bbox );
  quadtree.set_max_lod_pixels(max_lod_pixels);
  quadtree.set_crop_bbox( data_bbox );
  quadtree.set_draw_order_offset( draw_order_offset );
  if( vm.count("crop") ) quadtree.set_crop_images( true );
  quadtree.set_output_image_file_type( output_file_type );

  // Generate the composite
  vw_out(InfoMessage) << "Generating KML Overlay..." << std::endl;
  quadtree.generate( *progress );

  return 0;
}

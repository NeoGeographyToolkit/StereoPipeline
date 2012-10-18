// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

/// \file dem_adjust.cc
///

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <GeographicLib/Geoid.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using std::endl;
using std::string;
using namespace vw;
using namespace vw::cartography;
using namespace GeographicLib;

template <class ImageT>
class AdjustDemView : public ImageViewBase<AdjustDemView<ImageT> >
{
  ImageT m_img;
  GeoReference const& m_georef;
  Geoid const& m_geoid;
  double m_nodata_val;

public:

  typedef double pixel_type;
  typedef double result_type;
  typedef ProceduralPixelAccessor<AdjustDemView> pixel_accessor;

  AdjustDemView(ImageT const& img, GeoReference const& georef, Geoid const& geoid, double nodata_val):
    m_img(img), m_georef(georef), m_geoid(geoid), m_nodata_val(nodata_val){}

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t col, size_t row, size_t p=0 ) const {
    
    if ( m_img(col, row, p) == m_nodata_val ) return m_nodata_val;
    
    Vector2     lonlat                 = m_georef.pixel_to_lonlat(Vector2(col, row));
    result_type geoid_height           = m_geoid(lonlat[1], lonlat[0]);
    result_type height_above_ellipsoid = m_img(col, row, p);
    result_type height_above_geoid     = height_above_ellipsoid - Geoid::GEOIDTOELLIPSOID * geoid_height;
    return height_above_geoid;
  }
  
  /// \cond INTERNAL
  typedef AdjustDemView<typename ImageT::prerasterize_type> prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const { return prerasterize_type( m_img.prerasterize(bbox), m_georef, m_geoid, m_nodata_val ); }
  template <class DestT> inline void rasterize( DestT const& dest, BBox2i const& bbox ) const { vw::rasterize( prerasterize(bbox), dest, bbox ); }
  /// \endcond
};

template <class ImageT>
AdjustDemView<ImageT>
adjust_dem( ImageViewBase<ImageT> const& img, GeoReference const& georef, Geoid const& geoid, double nodata_val) {
  return AdjustDemView<ImageT>( img.impl(), georef, geoid, nodata_val );
}

struct Options : asp::BaseOptions {
  string dem_name, output_prefix;
  double nodata_value;
  bool use_float;
};

void handle_arguments( int argc, char *argv[], Options& opt ){
  
  po::options_description general_options("");
  general_options.add_options()
    ("nodata_value", po::value(&opt.nodata_value)->default_value(-32767),
     "The value of no-data pixels, unless specified in the DEM.")
    ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.")
    ("float", po::bool_switch(&opt.use_float)->default_value(false), "Output using float (32 bit) instead of using doubles (64 bit).");

  general_options.add( asp::BaseOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("dem", po::value(&opt.dem_name), "Explicitly specify the DEM.");

  po::positional_options_description positional_desc;
  positional_desc.add("dem", 1);

  std::string usage("[options] <dem>");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

  if ( opt.dem_name.empty() )
    vw_throw( ArgumentErr() << "Requires <dem> in order to proceed.\n\n" << usage << general_options );

  if ( opt.output_prefix.empty() ) {
    opt.output_prefix = fs::basename(opt.dem_name);
  }
}

int main( int argc, char *argv[] ) {

  // Adjust the DEM values so that they are relative to the geoid
  // rather than to the ellipsoid.
  
  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    DiskImageResourceGDAL dem_rsrc(opt.dem_name);
    double nodata_val = opt.nodata_value;
    if ( dem_rsrc.has_nodata_read() ) {
      nodata_val = dem_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for DEM: " << nodata_val << endl;
    }
    
    DiskImageView<double> dem_img(dem_rsrc);
    
    GeoReference dem_georef;
    read_georeference(dem_georef, dem_rsrc);
    
    std::cout << "pass the the path to the data!!!" << std::endl;
    std::cout << "Specify this library in the list of libraries, as well as the data!!!" << std::endl;

    Geoid egm96("egm96-5");
    ImageViewRef<double> adj_dem = adjust_dem(dem_img, dem_georef, egm96, nodata_val);

    std::string adj_dem_file = opt.output_prefix + "-adj.tif";
    vw_out() << "Writing adjusted DEM: " << adj_dem_file << std::endl;
    
    if ( opt.use_float ) {
      ImageViewRef<float> adj_dem_float = channel_cast<float>( adj_dem );
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc(adj_dem_file,
                                                                          adj_dem_float, opt ) );
      rsrc->set_nodata_write( nodata_val );
      write_georeference( *rsrc, dem_georef );
      block_write_image( *rsrc, adj_dem_float,
                         TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: ") );
    } else {
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc(adj_dem_file,
                                                                          adj_dem, opt ) );
      rsrc->set_nodata_write( nodata_val );
      write_georeference( *rsrc, dem_georef );
      block_write_image( *rsrc, adj_dem,
                         TerminalProgressCallback("asp", "\t--> Applying DEM adjustment: ") );
    }
    
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

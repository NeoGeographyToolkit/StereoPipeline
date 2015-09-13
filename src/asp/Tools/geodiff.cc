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


#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>

using std::endl;
using std::string;

using namespace vw;
using namespace vw::cartography;

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Function to convert a masked geodetic vector to a masked altitude vector
class MGeodeticToMAltitude : public ReturnFixedType<PixelMask<double> > {
public:
  PixelMask<double> operator()( PixelMask<Vector3> const& v ) const {
    if ( !is_valid( v ) ) {
      return PixelMask<double>();
    }
    return PixelMask<double>( v.child()[2] );
  }
};

struct Options : asp::BaseOptions {
  string dem1_name, dem2_name, output_prefix;
  double nodata_value;

  bool use_float, use_absolute;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("nodata-value",    po::value(&opt.nodata_value)->default_value(-32768),      "The no-data value to use, unless present in the DEM geoheaders.")
    ("output-prefix,o", po::value(&opt.output_prefix),                            "Specify the output prefix.")
    ("float",           po::bool_switch(&opt.use_float)->default_value(false),    "Output using float (32 bit) instead of using doubles (64 bit).")
    ("absolute",        po::bool_switch(&opt.use_absolute)->default_value(false), "Output the absolute difference as opposed to just the difference.");
  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("dem1", po::value(&opt.dem1_name), "Explicitly specify the first dem")
    ("dem2", po::value(&opt.dem2_name), "Explicitly specify the second dem");

  po::positional_options_description positional_desc;
  positional_desc.add("dem1", 1);
  positional_desc.add("dem2", 1);

  std::string usage("[options] <dem1> <dem2>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.dem1_name.empty() || opt.dem2_name.empty() )
    vw_throw( ArgumentErr() << "Requires <dem1> and <dem2> in order to proceed.\n\n" << usage << general_options );

  if ( opt.output_prefix.empty() ) {
    opt.output_prefix =
      fs::basename(opt.dem1_name) + "__" + fs::basename(opt.dem2_name);
  }

  vw::create_out_dir(opt.output_prefix);
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    DiskImageResourceGDAL dem1_rsrc(opt.dem1_name), dem2_rsrc(opt.dem2_name);
    double dem1_nodata = opt.nodata_value, dem2_nodata = opt.nodata_value;
    if ( dem1_rsrc.has_nodata_read() ) {
      dem1_nodata = dem1_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for DEM 1: " << dem1_nodata << endl;
    }
    if ( dem2_rsrc.has_nodata_read() ) {
      dem2_nodata = dem2_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value for DEM 2: " << dem2_nodata << endl;
    }

    DiskImageView<double> dem1_dmg(dem1_rsrc), dem2_dmg(dem2_rsrc);

    GeoReference dem1_georef, dem2_georef;
    read_georeference(dem1_georef, dem1_rsrc);
    read_georeference(dem2_georef, dem2_rsrc);

    // Transform DEM 2 into the same perspective as DEM 1. However, we
    // don't support datum changes!
    if ( dem1_georef.datum().proj4_str() !=
         dem2_georef.datum().proj4_str() ) {
      vw_throw( NoImplErr() << "GeoDiff can't difference DEMs which are on different datums.\n" );
    }

    // Generate a bounding box that is the minimum of the two BBox areas
    BBox2 crop_box = bounding_box( dem1_dmg );

    // Transform the second DEM's bounding box to first DEM's pixels
    GeoTransform gt(dem2_georef, dem1_georef);
    BBox2 box21 = gt.forward_bbox(bounding_box(dem2_dmg));
    crop_box.crop(box21);

    ImageViewRef<PixelMask<double> > dem2_trans =
      crop(geo_transform( per_pixel_filter(dem_to_geodetic( create_mask(dem2_dmg, dem2_nodata),
                                                            dem2_georef),
                                           MGeodeticToMAltitude()),
                          dem2_georef, dem1_georef,
                          ValueEdgeExtension<PixelMask<double> >(PixelMask<double>()) ),
           crop_box );

    ImageViewRef<double> difference;
    if ( opt.use_absolute ) {
      difference =
        apply_mask(abs(crop(create_mask(dem1_dmg, dem1_nodata), crop_box) - dem2_trans),
                   opt.nodata_value );
    } else {
      difference =
        apply_mask(crop(create_mask(dem1_dmg, dem1_nodata), crop_box) - dem2_trans,
                   opt.nodata_value );
    }

    GeoReference crop_georef = crop(dem1_georef, crop_box);

    std::string output_file = opt.output_prefix + "-diff.tif";
    vw_out() << "Writing difference: " << output_file << "\n";

    if ( opt.use_float ) {
      ImageViewRef<float> difference_float = channel_cast<float>( difference );
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc( output_file,
                                                                           difference_float, opt ) );
      rsrc->set_nodata_write( opt.nodata_value );
      write_georeference( *rsrc, crop_georef );
      block_write_image( *rsrc, difference_float,
                         TerminalProgressCallback("asp", "\t--> Differencing: ") );
    } else {
      boost::scoped_ptr<DiskImageResourceGDAL> rsrc( asp::build_gdal_rsrc( output_file,
                                                                           difference, opt ) );
      rsrc->set_nodata_write( opt.nodata_value );
      write_georeference( *rsrc, crop_georef );
      block_write_image( *rsrc, difference,
                         TerminalProgressCallback("asp", "\t--> Differencing: ") );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}

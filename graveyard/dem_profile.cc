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


/// \file dem_profile2.cc
///

// Standard
#include <iostream>

// Vision Workbench
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Cartography.h>
using namespace vw;

// Boost
#include <boost/program_options.hpp>
namespace po = boost::program_options;

// Main
int main ( int argc, char *argv[] ) {

  std::string input_file;
  std::string start_ll, end_ll;
  std::string start_px, end_px;
  double idx_column, idx_row;
  double nodata_value;

  po::variables_map vm;

  po::options_description general_options("Options");
  general_options.add_options()
    ("col", po::value<double>(&idx_column), "Process only a single column at this index")
    ("row", po::value<double>(&idx_row), "Process only a single row at this index")
    ("nodata-value", po::value<double>(&nodata_value), "Set the nodata value that is used in the DEM")
    //("ll-start", po::value<std::string>(&start_ll), "Start a profile at this Lon Lat location")
    //("ll-end", po::value<std::string>(&end_ll), "End a profile at this Lon Lat location")
    ("px-start", po::value<std::string>(&start_px)->composing(), "Start a profile at this pixel location")
    ("px-end", po::value<std::string>(&end_px), "End a profile at this pixel location")
    ("help,h", "Display this help message.");

  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-dem", po::value<std::string>(&input_file));

  po::options_description options("Allowed Options");
  options.add(general_options).add(hidden_options);

  po::positional_options_description pos;
  pos.add("input-dem", 1);

  po::store( po::command_line_parser( argc, argv ).options(options).positional(pos).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <dem-file> [options] \n\n";
  usage << general_options << "\n";

  if ( vm.count("help") || !vm.count("input-dem") ) {
    vw_out() << usage.str() << "\n";
    return 1;
  } else if ( vm.count("ll-start") ^ vm.count("ll-end") ) {
    vw_out() << "Must specify both ends if creating a Lat Lon Profile!\n";
    vw_out() << usage.str() << "\n";
    return 1;
  } else if ( vm.count("px-start") ^ vm.count("px-end") ) {
    vw_out() << "Must specify both ends if creating a Pixel Profile!\n";
    vw_out() << usage.str() << "\n";
    return 1;
  } else if ( vm.count("ll-start") && ( start_ll.size() != 2 ||
                                        end_ll.size() != 2 ) ) {
    vw_out() << "Start and End Lon Lat locations require 2 values, longitude and latitude.\n";
  }

  // Finally starting to perform some work
  cartography::GeoReference georef;
  cartography::read_georeference(georef, input_file);
  DiskImageView<float> disk_dem_file( input_file );
  ImageViewRef<PixelMask<float> > dem;
  if ( vm.count("nodata-value") ) {
    vw_out() << "\t--> Masking nodata value: " << nodata_value << "\n";
    dem = interpolate(create_mask( disk_dem_file, nodata_value ),
                      BicubicInterpolation(),
                      ZeroEdgeExtension());
  } else {
    DiskImageResource *disk_dem_rsrc = DiskImageResource::open(input_file);
    if ( disk_dem_rsrc->has_nodata_read() ) {
      nodata_value = disk_dem_rsrc->nodata_read();
      vw_out() << "\t--> Extracted nodata value from file: " << nodata_value << "\n";
      dem = interpolate(create_mask( disk_dem_file, nodata_value ),
                        BicubicInterpolation(),
                        ZeroEdgeExtension());
    } else {
      dem = interpolate(pixel_cast<PixelMask<float> >(disk_dem_file),
                        BicubicInterpolation(),
                        ZeroEdgeExtension());
    }
  }

  Vector2 start, end; // Pixel Locations (everything boils down to it
                      // at the moment :/. Maybe in the future where
                      // flying cars are possible, when a user
                      // provides LL coordinates .. SLERP is used.
  if ( vm.count("col") ) {
    start = Vector2(idx_column, 0);
    end = Vector2(idx_column, dem.rows()-1);
  } else if ( vm.count("row") ) {
    start = Vector2(0,idx_row);
    end = Vector2(dem.cols()-1,idx_row);
  } else if ( vm.count("ll-start") && vm.count("ll-end") ) {
    // Parsing string into a ll coordinate.
    if ( start_ll.find(',') == std::string::npos ||
         end_ll.find(',') == std::string::npos ) {
      vw_out() << "Longitude and Latitude values must be delimited by a comma.\n";
      vw_out() << "\tEx: --ll-start -109.54,33.483\n\n";
      vw_out() << usage.str() << "\n";
      return 1;
    }

    size_t s_break = start_px.find(',');
    size_t e_break = end_px.find(',');

    Vector2 ll_start, ll_end;

  } else if ( vm.count("px-start") && vm.count("px-end") ) {
    // Parsing string into pixel coordinates.
    if ( start_px.find(',') == std::string::npos ||
         end_px.find(',') == std::string::npos ) {
      vw_out() << "Column and Row pixel values must be delimited by a comma.\n";
      vw_out() << "\tEx: --px-start 65,109\n\n";
      vw_out() << usage.str() << "\n";
      return 1;
    }

    size_t s_break = start_px.find(',');
    size_t e_break = end_px.find(',');

    start.x() = atof(start_px.substr(0,s_break).c_str());
    start.y() = atof(start_px.substr(s_break+1, start_px.size()-s_break-1).c_str());
    end.x() = atof(end_px.substr(0,e_break).c_str());
    end.y() = atof(end_px.substr(e_break+1, end_px.size()-e_break-1).c_str());
  } else {
    // Dial down the middle if the user provides only a DEM.
    start = Vector2( dem.cols()/2, 0);
    end = Vector2( dem.cols()/2, dem.rows()-1);
  }

  // Debug
  vw_out() << "Start: " << start << "\n";
  vw_out() << "End:   " << end << "\n";

  // Working out the output file
  BBox2i bound = bounding_box( dem );
  if ( !bound.contains( start ) ||
       !bound.contains( end ) ) {
    vw_out() << "Given coordinates are not with in the image!\nExiting early .....\n";
    return 1;
  }

  std::ofstream output_file( "profile.csv" );
  output_file << std::setprecision(12) << "lon,lat,pix_x,pix_y,altitude,radius\n";

  Vector2i delta = end - start;
  for ( float r = 0; r < 1.0; r +=1/norm_2(delta) ) {
    Vector2 c_idx = Vector2(0.5,0.5) + start + r*delta;

    if ( !dem(int(c_idx.x()),int(c_idx.y())).valid() )
      continue;

    Vector2 lonlat_loc = georef.pixel_to_lonlat( c_idx );
    double altitude = dem( c_idx.x(), c_idx.y() );
    Vector3 xyz_pos = georef.datum().geodetic_to_cartesian( Vector3( lonlat_loc.x(),
                                                                     lonlat_loc.y(),
                                                                     altitude ));
    double radius = norm_2( xyz_pos );

    output_file << lonlat_loc.x() << "," << lonlat_loc.y() << "," << c_idx.x()
                << "," << c_idx.y() << "," << altitude << "," << radius << "\n";
  }
  output_file.close();

  return 0;
}

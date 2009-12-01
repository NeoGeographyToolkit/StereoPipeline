// __BEGIN_LICENSE__
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

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1) 
    result.erase(index, result.size());
  return result;
}

// Main
int main ( int argc, char *argv[] ) {

  std::string input_file;
  std::vector<double> start_ll, end_ll;
  std::vector<double> start_px, end_px;
  double idx_column, idx_row;
  double nodata_value;

  po::variables_map vm;

  po::options_description general_options("Options");
  general_options.add_options()
    ("col", po::value<double>(&idx_column), "Process only a single column at this index")
    ("row", po::value<double>(&idx_row), "Process only a single row at this index")
    ("nodata-value", po::value<double>(&nodata_value), "Set the nodata value that is used in the DEM")
    //    ("ll-start", po::value<std::vector<double> >(&start_ll), "Start a profile at this Lon Lat location")
    //("ll-end", po::value<std::vector<double> >(&end_ll), "End a profile at this Lon Lat location")
    //("px-start", po::value<std::vector<double> >(&start_px)->composing(), "Start a profile at this pixel location")
    //("px-end", po::value<std::vector<double> >(&end_px), "End a profile at this pixel location")
    ("i,input-dem", po::value<std::string>(&input_file), "Input DEM to use/")
    ("help,h", "Display this help message");

  /*
  po::options_description hidden_options("");
  hidden_options.add_options()
    ("input-dem", po::value<std::vector<std::string> >(&input_file));
  */

  po::options_description options("Allowed Options");
  options.add(general_options);

  //po::positional_options_description pos;
  //pos.add("input-dem", -1);

  po::store( po::command_line_parser( argc, argv ).options(options).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <dem-file> [options] \n\n";
  usage << general_options << "\n";

  if ( vm.count("help") ) {
    vw_out(0) << usage.str() << "\n";
    return 1;
  } else if ( vm.count("ll-start") ^ vm.count("ll-end") ) {
    vw_out(0) << "Must specify both ends if creating a Lat Lon Profile!\n";
    vw_out(0) << usage.str() << "\n";
    return 1;
  } else if ( vm.count("px-start") ^ vm.count("px-end") ) {
    vw_out(0) << "Must specify both ends if creating a Pixel Profile!\n";
    vw_out(0) << usage.str() << "\n";
    return 1;
  } else if ( vm.count("ll-start") && ( start_ll.size() != 2 ||
                                        end_ll.size() != 2 ) ) {
    vw_out(0) << "Start and End Lon Lat locations require 2 values, longitude and latitude.\n"; 
  } else if ( vm.count("px-start") && ( start_px.size() != 2 ||
                                        end_px.size() != 2 ) ) {
    vw_out(0) << "Start and End pixel locations require 2 values, column and row.\n";
    vw_out(0) << "start: " << start_px.size() << " end: " << end_px.size() << "\n";
    for ( uint i = 0; i  < start_px.size(); i++ )
      vw_out(0) << "\t" << start_px[i] << "\n";
    vw_out(0) << usage.str() << "\n";
    return 1;
  }

  // Finally starting to perform some work
  cartography::GeoReference georef;
  cartography::read_georeference(georef, input_file);
  DiskImageView<float> disk_dem_file( input_file );
  ImageViewRef<PixelMask<float> > dem;
  if ( vm.count("nodata-value") ) {
    vw_out(0) << "\t--> Masking nodata value: " << nodata_value << "\n";
    dem = create_mask( disk_dem_file, nodata_value );
  } else {
    DiskImageResource *disk_dem_rsrc = DiskImageResource::open(input_file);
    if ( disk_dem_rsrc->has_nodata_value() ) {
      nodata_value = disk_dem_rsrc->nodata_value();
      vw_out(0) << "\t--> Extracted nodata value from file: " << nodata_value << "\n";
      dem = create_mask( disk_dem_file, nodata_value );
    } else {
      dem = pixel_cast<PixelMask<float> >(disk_dem_file);
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
  } else if ( vm.count("px-start") && vm.count("px-end") ) {
    //start = Vector2( start_px[0], start_px[1] );
    //end = Vector2( end_px[0], end_px[1] );
  } else {
    // Dial down the middle if the user provides only a DEM.
    start = Vector2( dem.cols()/2, 0);
    end = Vector2( dem.cols()/2, dem.rows()-1);
  }

  // Debug
  vw_out(0) << "Start: " << start << "\n";
  vw_out(0) << "End:   " << end << "\n";

  // Working out the output file
  BBox2i bound = bounding_box( dem );
  if ( !bound.contains( start ) ||
       !bound.contains( end ) ) {
    vw_out(0) << "Given coordinates are not with in the image!\nExiting early .....\n";
    return 1;
  }

  std::ofstream output_file( "profile.csv" );
  output_file << std::setprecision(12) << "lon,lat,pix_x,pix_y,altitude,radius\n";

  Vector2i delta = end - start;
  for ( float r = 0; r < 1.0; r +=1/norm_2(delta) ) {
    Vector2 c_idx = Vector2(0.5,0.5) + start + r*delta;

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

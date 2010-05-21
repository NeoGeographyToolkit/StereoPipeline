// __BEGIN_LICENSE__
// Copyright (C) 2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__

// In the future this should handle different project types

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateCarreePlateManager.h>

using namespace vw;
using namespace vw::platefile;

#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // Input
  std::string url;
};

void perform_mipmap( Options const& opt ) {

  // Load up Plate File
  boost::shared_ptr<PlateFile> platefile =
    boost::shared_ptr<PlateFile>( new PlateFile(opt.url) );

  PlateCarreePlateManager<PixelGrayA<uint8> > platemanager( platefile );

  int full = pow(2.0,platefile->num_levels()-1);
  int quarter = full/4;
  BBox2i affected_tiles(0,quarter,full-1,quarter*2-1);
  std::cout << "Affected Tiles: " << affected_tiles << "\n";

  platefile->write_request();
  platemanager.mipmap(platefile->num_levels()-1,
                      affected_tiles, -1, false,
                      TerminalProgressCallback("photometrytk",
                                               "Mipmapping:") );
  platefile->write_complete();
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("help,h", "Display this help message");

  po::options_description positional("");
  positional.add_options()
    ("url",  po::value<std::string>(&opt.url),  "Input platefile Url");

  po::positional_options_description positional_desc;
  positional_desc.add("url", 1);

  po::options_description all_options;
  all_options.add(general_options).add(positional);

  po::variables_map vm;
  try {
    po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_desc).run(), vm );
    po::notify( vm );
  } catch (po::error &e) {
    vw_throw( ArgumentErr() << "Error parsing input:\n\t"
              << e.what() << general_options );
  }

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " <platefile url>\n";

  if ( vm.count("help") )
    vw_throw( ArgumentErr() << usage.str() << general_options );
  if ( opt.url.empty() )
    vw_throw( ArgumentErr() << "Missing platefile file url!\n"
              << usage.str() << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );
    perform_mipmap( opt );
  } catch ( const ArgumentErr& e ) {
    vw_out() << e.what() << std::endl;
    return 1;
  } catch ( const Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

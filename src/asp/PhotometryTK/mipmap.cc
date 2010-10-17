// __BEGIN_LICENSE__
// Copyright (C) 2006-2010 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


// In the future this should handle different project types

#include <vw/Image.h>
#include <vw/Plate/PlateFile.h>
#include <vw/Plate/PlateCarreePlateManager.h>
#include <asp/Core/Macros.h>

using namespace vw;
using namespace vw::platefile;

#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
namespace po = boost::program_options;

using namespace std;

struct Options {
  // Input
  std::string url;
  std::string region; // <ul_x>,<ul_y>:<lr_x>,<lr_y>@<level>

  // Region string break down
  BBox2i region_bbox;
  int level, top_level;
};

void perform_mipmap( Options & opt ) {

  // Load up Plate File
  boost::shared_ptr<PlateFile> platefile =
    boost::shared_ptr<PlateFile>( new PlateFile(opt.url) );

  PlateCarreePlateManager<PixelGrayA<uint8> > platemanager( platefile );

  // Parsing region string
  if (!opt.region.empty()) {
    boost::char_separator<char> sep(",:@");
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    tokenizer tokens( opt.region, sep);
    tokenizer::iterator tok_iter = tokens.begin();

    if (tok_iter == tokens.end())
      vw_throw( ArgumentErr() << "Region argument is incomplete\n" );
    opt.region_bbox.min()[0] = boost::lexical_cast<int>(*tok_iter);
    ++tok_iter;

    if (tok_iter == tokens.end())
      vw_throw( ArgumentErr() << "Region argument is incomplete\n" );
    opt.region_bbox.min()[1] = boost::lexical_cast<int>(*tok_iter);
    ++tok_iter;

    if (tok_iter == tokens.end())
      vw_throw( ArgumentErr() << "Region argument is incomplete\n" );
    opt.region_bbox.max()[0] = boost::lexical_cast<int>(*tok_iter);
    ++tok_iter;

    if (tok_iter == tokens.end())
      vw_throw( ArgumentErr() << "Region argument is incomplete\n" );
    opt.region_bbox.max()[1] = boost::lexical_cast<int>(*tok_iter);
    ++tok_iter;

    if (tok_iter == tokens.end())
      vw_throw( ArgumentErr() << "Region argument is incomplete\n" );
    opt.level = boost::lexical_cast<int>(*tok_iter);
    ++tok_iter;

    if (tok_iter != tokens.end())
      vw_throw( ArgumentErr() << "Region argument has too many arguments\n" );
  } else {
    // Process everything!
    opt.level = platefile->num_levels()-1;
    int32 full = 1 << opt.level;
    int32 quarter = full/4;
    opt.region_bbox = BBox2i(0,quarter,full-1,quarter*2-1);
  }
  vw_out() << "Processing: " << opt.region_bbox
           << " at level " << opt.level << "\n";

  platefile->write_request();
  platemanager.mipmap(opt.level, opt.region_bbox, -1, false,
                      TerminalProgressCallback("photometrytk",
                                               "Mipmapping:"),
                      opt.top_level);
  platefile->write_complete();
}

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("region", po::value(&opt.region), "Where arg = <ul_x>,<ul_y>:<lr_x>,<lr_y>@<level> - Limit the snapshot to the region bounded by these upper left (ul) and lower right (lr) coordinates at the level specified.")
    ("top-level", po::value(&opt.top_level)->default_value(0), "Top level to process to. Default is to process to the very top, which is zero.")
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
  } ASP_STANDARD_CATCHES;

  return 0;
}

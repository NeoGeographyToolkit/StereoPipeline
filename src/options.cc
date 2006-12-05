// options.cc
// 
// Functions for parsing command line arguments and options is the
// stereo.default file.
//
// Created 03-MAY-2006 by mbroxton.

#include <iostream>
#include <fstream>
#include <iterator>

#include "stereo.h"

using namespace std;
#include <boost/program_options.hpp>
namespace po = boost::program_options;


// A helper class for streaming a vector to std::cout
template<class T>
ostream& operator<<(ostream& os, const vector<T>& v) {
  copy(v.begin(), v.end(), ostream_iterator<T>(cout, "  ")); 
  return os;
}

void ParseCommandLineArgs(int ac, char* av[]) {
  try {
    
    // Declare a group of options that will be allowed only on the CLI
    po::options_description cmdline_options("Command Line Options");
    generic.add_options()
      ("version,v", "print version string")
      ("help,h", "produce help message")
      ("entry_point,e", po::value<int>(), "Set entry point")
      ("stereo_file,s", po::value<string>(), "Stereo Default Filename")
      ("moc_description_file,s", po::value<string>(), "MOC Image Description File");
            
    po::positional_options_description p;
    p.add("primary_input_file", -1);
    p.add("secondary_input_file", -2);
    p.add("output_prefix", -3);
        
    po::variables_map vm;
    po::store(po::command_line_parser(ac, av).
	      options(cmdline_options).positional(p).run(), vm);
    po::notify(vm);

//     ifstream ifs("multiple_sources.cfg");
//     store(parse_config_file(ifs, config_file_options), vm);
//     notify(vm);
    
    if (vm.count("help")) {
      cout << "\nUsage: stereo [options] <Left_input_file> <Right input file> <output_file_prefix>\n"
	   << "the extensions are automaticaly added to the output file\n"
	   << "the parameters are in stereo.default\n\n";
      cout << cmdline_options << "\n";
      exit(0);
    }

    if (vm.count("version")) {
      cout << "Ames Stereo Pipeline, version " << ASP_VERSION_STRING << ".\n";
      exit(0);
    }

  } catch(exception& e) {
    throw vw::Exception() << "ParseCommandLineArguments:  An exception ocurred.\n"
			  << e.what() << "\n";
  }
}

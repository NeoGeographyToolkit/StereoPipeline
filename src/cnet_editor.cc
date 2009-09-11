// boost
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
namespace po = boost::program_options;
namespace fs = boost::filesystem;

// standard
#include <vector>
#include <iostream>

// VisionWorkbench
#include <vw/Math.h>
#include <vw/Camera/ControlNetwork.h>
#include <vw/InterestPoint/InterestData.h>

using namespace vw;
using namespace vw::ip;
using namespace vw::camera;

// Main Executable
int main( int argc, char *argv[] ) {
  std::string cnet_file;
  std::string image_mean_file;
  fs::path output_cnet_file;
  ControlNetwork cnet("ControlNetwork Editor");
  double cutoff_sigma;

  po::options_description general_options("Options");
  general_options.add_options()
    ("cutoff_sigma,c", po::value<double>(&cutoff_sigma)->default_value(2),"This is the highend cutoff sigma.")
    ("output_cnet_file,o", po::value<fs::path>(&output_cnet_file)->default_value("processed.cnet"), "Name of processed control network file to write out.")
    ("help,h","Brings up this.");

  po::options_description positional_options("Positional Options");
  positional_options.add_options()
    ("cnet", po::value<std::string>(&cnet_file), "Control Network")
    ("image-mean", po::value<std::string>(&image_mean_file), "Image Mean file");

  po::positional_options_description positional_options_desc;
  positional_options_desc.add("cnet", 1);
  positional_options_desc.add("image-mean",1);
  
  po::options_description all_options("Allowed Options");
  all_options.add(general_options).add(positional_options);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(all_options).positional(positional_options_desc).run(), vm );
  po::notify( vm );

  std::ostringstream usage;
  usage << "Usage: " << argv[0] << " [options] <cnet> <image-mean>\n\n";
  usage << general_options << std::endl;

  if ( vm.count("help") ||
       !vm.count("cnet") ||
       !vm.count("image-mean") ) {
    std::cout << usage.str() << std::endl;
    return 1;
  }

  // Loading control network file
  std::vector<std::string> tokens;
  boost::split( tokens, cnet_file, boost::is_any_of(".") );
  if ( tokens[tokens.size()-1] == "net" ) {
    cnet.read_isis_pvl_control_network( cnet_file );
  } else if ( tokens[tokens.size()-1] == "cnet" ) {
    cnet.read_binary_control_network( cnet_file );
  } else {
    vw_throw( IOErr() << "Unknown Control Network file extension, \""
		<< tokens[tokens.size()-1] << "\"." );
  }

  // Alright loading image mean file
  std::ifstream f;
  f.open( image_mean_file.c_str(), std::ios::binary | std::ios::in );
  int read_cnet_size;
  f.read((char*)&(read_cnet_size), sizeof(int));
  if ( read_cnet_size != cnet.size() )
    vw_throw( IOErr() << "Image mean file doesn't seem to match Cnet" );
  int error_size;
  std::list<double> image_errors;
  f.read((char*)&(error_size), sizeof(int));
  
  for ( unsigned i = 0; i < error_size; i++ ) {
    double temp;
    f.read((char*)&(temp), sizeof(double));
    image_errors.push_back(temp);
  }

  // Collecting statistics
  double min_image = *(std::min_element(image_errors.begin(),
					image_errors.end()));
  double max_image = *(std::max_element(image_errors.begin(),
					image_errors.end()));
  double mean_image=0, stddev_image=0;
  for( std::list<double>::iterator it = image_errors.begin();
       it != image_errors.end(); it++ ) {
    mean_image += (*it);
    stddev_image += (*it)*(*it);
  }
  mean_image /= error_size;
  stddev_image /=  error_size;
  stddev_image = sqrt( stddev_image - mean_image*mean_image );
  std::cout << "Image min: " << min_image << " max: " << max_image 
	    << " mean: " << mean_image << " stddev: " << stddev_image 
	    << std::endl;

  // Awesome, now clipping based one std_dev
  int clipping_count = 0;
  int other_count = 0;
  int cp_clip_count = 0;
  std::list<double>::iterator image_error = image_errors.begin();
  for ( unsigned cpi = 0; cpi < cnet.size(); cpi++ ) {
    for ( unsigned cmi = 0; cmi < cnet[cpi].size(); cmi++ ) {
      if ( image_error == image_errors.end() )
	vw_throw( IOErr() << "Internal overflow error" );

      // Do clipping
      if ( *image_error >= mean_image + stddev_image*cutoff_sigma ) {
	clipping_count++;
	
	cnet[cpi].delete_measure( cmi );
	cmi--;
      } else
	other_count++;

      image_error++;
    }

    if ( cnet[cpi].size() < 2 ) {
      cnet.delete_control_point( cpi );
      cpi--;
      cp_clip_count++;
    }
  }
  std::cout << float(clipping_count) * 100.0 / float(clipping_count + other_count) 
	    << "% (" << clipping_count << ") of the control measures removed.\n";
  std::cout << float(cp_clip_count) * 100.0 / float(cp_clip_count+cnet.size()) 
	    << "% (" << cp_clip_count << ") of control points removed.\n";  

  std::cout << "\nWriting out new control network\n";
  std::string outfile_str = fs::path(output_cnet_file.parent_path() / output_cnet_file.stem()).string();
  cnet.write_binary_control_network(outfile_str);
}

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

// Take as input a directory of ASTER images. Apply radiometric
// correction to the VNIR_Band3N and VNIR_Band3B images, writing
// <output-prefix>-Band3N.tif and <output-prefix>-Band3B.tif. Generate
// RPC coefficients from input metadata and write
// <output-prefix>-Band3N.xml and <output-prefix>-Band3B.xml. These
// files can then be used as input for stereo.

#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>

#include <limits>
#include <cstring>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

namespace fs = boost::filesystem;
namespace po = boost::program_options;

using namespace vw;
using namespace std;
using namespace vw::cartography;

struct Options : public asp::BaseOptions {
  // Input
  string input_dir, output_prefix; 

  Options(){}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",   po::value(&opt.output_prefix), "Specify the output prefix.");

  general_options.add( asp::BaseOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input_dir", po::value(&opt.input_dir), "An ASTER data directory.");

  po::positional_options_description positional_desc;
  positional_desc.add("input_dir", 1);

  string usage("<input directory> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.input_dir.empty() )
    vw_throw( ArgumentErr() << "Missing input directory.\n" << usage << general_options );

  if ( opt.output_prefix.empty() )
    vw_throw( ArgumentErr() << "Missing output prefix.\n" << usage << general_options );

  // Create the output directory
  vw::create_out_dir(opt.output_prefix);
}

// See if the input file matches the given pattern. If yes, store the result
// in matched_file. If matched_file is not empty originally, that means
// we have more than one match, which is not good. 
bool match_file(std::string const& input_file, std::string const& pattern,
		std::string & matched_file){

  size_t it = input_file.find(pattern);

  // No match
  if (it == std::string::npos)
    return false;

  // Check if it does not match at the end of the string
  if (it + pattern.size() != input_file.size() ) return false;
  
  // We expect only one match
  if (matched_file != "") 
    vw_throw( ArgumentErr()
	      << "Found two files matching: '" << pattern << "'. Those are: "
	      << matched_file << " and " << input_file << "\n");
  
  matched_file = input_file;
  
  std::cout << "file is " << input_file << std::endl;
  std::cout << "it is " << it << std::endl;

  return true;
}

// Identify the appropriate inputs in the ASTER directory
void locate_inputs(Options const& opt, 
		   std::string& nadir_image,
		   std::string& back_image,
		   std::string& nadir_sat_pos,
		   std::string& back_sat_pos,
		   std::string& nadir_sight_vec,
		   std::string& back_sight_vec,
		   std::string& nadir_corr_table,
		   std::string& back_corr_table,
		   std::string& nadir_longitude,
		   std::string& back_longitude,
		   std::string& nadir_latitude,
		   std::string& back_latitude,
		   std::string& nadir_lattice_point,
		   std::string& back_lattice_point){
  
  // Iterate through all files
  for (fs::directory_iterator itr(opt.input_dir); itr != fs::directory_iterator(); itr++){
    
    std::string file = itr->path().string();
    
    match_file(file, "VNIR_Band3N.ImageData.tif", nadir_image);
    match_file(file, "VNIR_Band3B.ImageData.tif", back_image);
    
    match_file(file, "VNIR_Band3N.SatellitePosition.txt", nadir_sat_pos);
    match_file(file, "VNIR_Band3B.SatellitePosition.txt", back_sat_pos);

    match_file(file, "VNIR_Band3N.SightVector.txt", nadir_sight_vec);
    match_file(file, "VNIR_Band3B.SightVector.txt", back_sight_vec);

    match_file(file, "VNIR_Band3N.RadiometricCorrTable.txt", nadir_corr_table);
    match_file(file, "VNIR_Band3B.RadiometricCorrTable.txt", back_corr_table);

    match_file(file, "VNIR_Band3N.Longitude.txt", nadir_longitude);
    match_file(file, "VNIR_Band3B.Longitude.txt", back_longitude);

    match_file(file, "VNIR_Band3N.Latitude.txt", nadir_latitude);
    match_file(file, "VNIR_Band3B.Latitude.txt", back_latitude);

    match_file(file, "VNIR_Band3N.LatticePoint.txt", nadir_lattice_point);
    match_file(file, "VNIR_Band3B.LatticePoint.txt", back_lattice_point);
  }

  if (nadir_image == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking camera image "
	      << "(VNIR_Band3N.ImageData.tif).\n");
  if (back_image == "")
    vw_throw( ArgumentErr() << "Could not locate the backward-looking camera image "
	      << "(VNIR_Band3B.ImageData.tif).\n");
  
  if (nadir_sat_pos == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking satellite position "
	      << "(VNIR_Band3N.SatellitePosition.txt).\n");
  if (back_sat_pos == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking satellite position "
	      << "(VNIR_Band3B.SatellitePosition.txt).\n");
  
  if (nadir_sight_vec == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking sight vector "
	      << "(VNIR_Band3N.SightVector.txt).\n");
  if (back_sight_vec == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking sight vector "
	      << "(VNIR_Band3B.SightVector.txt).\n");

  if (nadir_corr_table == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking radiometric correction table "
	      << "(VNIR_Band3N.RadiometricCorrTable.txt).\n");
  if (back_corr_table == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking radiometric correction table "
	      << "(VNIR_Band3B.RadiometricCorrTable.txt).\n");
  
  if (nadir_longitude == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking longitude file "
	      << "(VNIR_Band3N.Longitude.txt).\n");
  if (back_longitude == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking longitude file "
	      << "(VNIR_Band3B.Longitude.txt).\n");

  if (nadir_latitude == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking latitude file "
	      << "(VNIR_Band3N.Latitude.txt).\n");
  if (back_latitude == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking latitude file "
	      << "(VNIR_Band3B.Latitude.txt).\n");

  if (nadir_lattice_point == "")
    vw_throw( ArgumentErr() << "Could not locate the nadir-looking lattice point file "
	      << "(VNIR_Band3N.LatticePoint.txt).\n");
  if (back_lattice_point == "")
    vw_throw( ArgumentErr() << "Could not locate the back-looking lattice point file "
	      << "(VNIR_Band3B.LatticePoint.txt).\n");
}

// Apply radiometric corrections
template <class ImageT>
class RadioCorrectView: public ImageViewBase< RadioCorrectView<ImageT> >{
  ImageT m_img;
  std::vector<Vector3> const& m_corr; // alias

public:
  RadioCorrectView( ImageT const& img, std::vector<Vector3> const& corr):
    m_img(img), m_corr(corr){}
    
  typedef typename ImageT::pixel_type input_type;
  typedef float pixel_type;
  typedef float result_type;
  typedef ProceduralPixelAccessor<RadioCorrectView> pixel_accessor;

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "RadioCorrectView::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    ImageView<input_type> input_tile = crop(m_img, bbox); // to speed things up
    ImageView<result_type> tile(bbox.width(), bbox.height());
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){
      Vector3 C = m_corr[col];
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        tile(col - bbox.min().x(), row - bbox.min().y() )
	  = C[1] * input_tile(col - bbox.min().x(), row - bbox.min().y()) / C[2] + C[0];
      }
    }
    
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }
  
  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};
template <class ImageT>
RadioCorrectView<ImageT> radio_correct(ImageT const& img, std::vector<Vector3> const & corr){
  return RadioCorrectView<ImageT>(img, corr);
}

void apply_radiometric_corrections(Options const& opt, 
				   std::string const& input_image,
				   std::string const& corr_table,
				   std::string const& out_image){


  // Extract the corrections
  std::vector<Vector3> corr;
  std::ifstream ifs(corr_table.c_str());
  if (!ifs.good())
    vw_throw( ArgumentErr() << "Could not open file for reading: " << corr_table << "\n" );
  double a, b, c;
  while (ifs >> a >> b >> c){
    Vector3 p(a, b, c);
    corr.push_back(p);
  }
  
  DiskImageView<float> input_img(input_image);

  // Sanity check
  if (input_img.cols() != int(corr.size()) ) 
    vw_throw( ArgumentErr() << "Expecting as many corrections in " << corr_table
	      << " as image columns in " << input_image << "\n" );
  
  bool has_nodata = false;
  double nodata   = -std::numeric_limits<float>::max();
  bool has_georef = false;
  vw::cartography::GeoReference georef;

  // See if there is a no-data value
  {
    boost::shared_ptr<DiskImageResource> img_rsrc( new DiskImageResourceGDAL(input_image) );
    if (img_rsrc->has_nodata_read()){
      has_nodata = true;
      nodata = img_rsrc->nodata_read();
    }
  }
  
  vw_out() << "Writing: " << out_image << std::endl;
  asp::block_write_gdal_image(out_image,
			      radio_correct(input_img, corr),
			      has_georef, georef, 
			      has_nodata, nodata,
			      opt,
			      TerminalProgressCallback("asp", "\t-->: "));
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    std::string nadir_image, back_image, nadir_sat_pos, back_sat_pos;
    std::string nadir_sight_vec, back_sight_vec;
    std::string nadir_corr_table, back_corr_table;
    std::string nadir_longitude, back_longitude;
    std::string nadir_latitude, back_latitude;
    std::string nadir_lattice_point, back_lattice_point;
    
    locate_inputs(opt, nadir_image, back_image, nadir_sat_pos, back_sat_pos,
		  nadir_sight_vec, back_sight_vec, nadir_corr_table, back_corr_table,
		  nadir_longitude, back_longitude, nadir_latitude, back_latitude,
		  nadir_lattice_point, back_lattice_point);

    std::string out_nadir_image = opt.output_prefix + "-Band3N.tif";
    std::string out_back_image  = opt.output_prefix + "-Band3B.tif";
    
    std::string out_nadir_cam = opt.output_prefix + "-Band3N.xml";
    std::string out_back_cam  = opt.output_prefix + "-Band3B.xml";

    std::cout << "nadir_image             " << nadir_image << std::endl;
    std::cout << "back_image              " << back_image << std::endl;
    std::cout << "nadir_sat_pos           " << nadir_sat_pos << std::endl;
    std::cout << "back_sat_pos            " << back_sat_pos << std::endl;
    std::cout << "nadir_sight_vec         " << nadir_sight_vec << std::endl;
    std::cout << "back_sight_vec          " << back_sight_vec << std::endl;
    std::cout << "nadir_corr_table        " << nadir_corr_table << std::endl;
    std::cout << "back_corr_table         " << back_corr_table << std::endl;
    std::cout << "nadir_longitude         " << nadir_longitude << std::endl;
    std::cout << "back_longitude          " << back_longitude << std::endl;
    std::cout << "nadir_latitude          " << nadir_latitude << std::endl;
    std::cout << "back_latitude           " << back_latitude << std::endl;
    std::cout << "nadir_lattice_point     " << nadir_lattice_point << std::endl;
    std::cout << "back_lattice_point      " << back_lattice_point << std::endl;

    std::cout << "output nadir image: " << out_nadir_image << ' ' << out_nadir_cam << std::endl;
    std::cout << "output back image:  " << out_back_image  << ' ' << out_back_cam  << std::endl;

    apply_radiometric_corrections(opt, nadir_image, nadir_corr_table, out_nadir_image);
    apply_radiometric_corrections(opt, back_image,  back_corr_table,  out_back_image);
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

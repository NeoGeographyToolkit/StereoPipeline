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
// files can then be used as input for stereo with -t rpc.

// References:
// -----------
// ASTER User Handbook Version 2
// https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf
//
// IMPROVEMENT OF DEM GENERATION FROM ASTER IMAGES USING SATELLITE
// JITTER ESTIMATION AND OPEN SOURCE IMPLEMENTATION
// Luc Girod, Christopher Nutha, and Andreas Kaab
// http://www.int-arch-photogramm-remote-sens-spatial-inf-sci.net/XL-1-W5/249/2015/isprsarchives-XL-1-W5-249-2015.pdf

#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Core/StringUtils.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/FileUtils.h>
#include <asp/Camera/RPCModelGen.h>

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

struct Options : public vw::cartography::GdalWriteOptions {
  string input_dir, output_prefix; 
  double min_height, max_height;
  int num_samples;
  double penalty_weight;
  Options(): min_height(-1), max_height(-1), num_samples(-1), penalty_weight(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",   po::value(&opt.output_prefix), "Specify the output prefix.")
    ("min-height",     po::value(&opt.min_height)->default_value(0),
     "The minimum height (in meters) above the WGS84 datum of the simulation box in which to compute the RPC approximation.")
    ("max-height",     po::value(&opt.max_height)->default_value(8.0e+3),
     "The maximum height (in meters) above the WGS84 datum of the simulation box in which to compute the RPC approximation.")
    ("num-samples",     po::value(&opt.num_samples)->default_value(100),
     "How many samples to use between the minimum and maximum heights.")
    ("penalty-weight",     po::value(&opt.penalty_weight)->default_value(0.1),
     "Penalty weight to use to keep the higher-order RPC coefficients small. Higher penalty weight results in smaller such coefficients.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input_dir", po::value(&opt.input_dir), "An ASTER data directory.");

  po::positional_options_description positional_desc;
  positional_desc.add("input_dir", 1);

  string usage("<input directory> -o <output prefix>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);
  
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

  // Must match L1A images
  if (input_file.find("L1A") == std::string::npos) 
    return false;

  // We expect only one match
  if (matched_file != "") 
    vw_throw( ArgumentErr()
	      << "Found two files matching: '" << pattern << "'. Those are: "
	      << matched_file << " and " << input_file << "\n");
  
  matched_file = input_file;

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

// Apply the radiometric corrections
template <class ImageT>
class RadioCorrectView: public ImageViewBase< RadioCorrectView<ImageT> >{
  ImageT m_img;
  std::vector<Vector3> const& m_corr; // alias
  bool m_has_nodata;
  double m_nodata;

public:
  RadioCorrectView(ImageT const& img, std::vector<Vector3> const& corr,
		   bool has_nodata, double nodata):
    m_img(img), m_corr(corr), m_has_nodata(has_nodata), m_nodata(nodata){}
    
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
	input_type val = input_tile(col - bbox.min().x(), row - bbox.min().y());
	if (m_has_nodata && val == m_nodata)
	  tile(col - bbox.min().x(), row - bbox.min().y() ) = val;
	else
	  tile(col - bbox.min().x(), row - bbox.min().y() ) = C[1] * val / C[2] + C[0];
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
RadioCorrectView<ImageT> radio_correct(ImageT const& img, std::vector<Vector3> const & corr,
				       bool has_nodata, double nodata){
  return RadioCorrectView<ImageT>(img, corr, has_nodata, nodata);
}

// ASTER L1A images come with radiometric corrections appended, but not applied.
// There is one correction per image column.
void apply_radiometric_corrections(Options const& opt, 
				   std::string const& input_image,
				   std::string const& corr_table,
				   std::string const& out_image){


  // Extract the corrections
  std::vector<Vector3> corr;
  asp::read_3d_points(corr_table, corr);
  
  DiskImageView<float> input_img(input_image);

  // Sanity check
  if (input_img.cols() != int(corr.size()) ) 
    vw_throw( ArgumentErr() << "Expecting as many corrections in " << corr_table
	      << " as image columns in " << input_image << "\n" );
  
  bool has_nodata = false;
  double nodata   = -std::numeric_limits<float>::max();
  // See if there is a no-data value
  {
    boost::shared_ptr<DiskImageResource> img_rsrc(new DiskImageResourceGDAL(input_image));
    if (img_rsrc->has_nodata_read()){
      has_nodata = true;
      nodata = img_rsrc->nodata_read();
    }
  }

  // No georef, this being L1A imagery
  vw::cartography::GeoReference georef;
  bool has_georef = read_georeference(georef, input_image);
  if (has_georef)
    vw_throw( ArgumentErr() << "ASTER L1A images are not supposed to be georeferenced.\n" );

  
  vw_out() << "Writing: " << out_image << std::endl;
  vw::cartography::block_write_gdal_image(out_image,
			      radio_correct(input_img, corr, has_nodata, nodata),
			      has_georef, georef, 
			      has_nodata, nodata,
			      opt,
			      TerminalProgressCallback("asp", "\t-->: "));
}

// Generate lon-lat-height to image pixel correspondences that we will
// use to create the RPC model.
void generate_point_pairs(// Inputs
                          double min_height, double max_height, int num_samples,
                          double penalty_weight,
                          std::string const& sat_pos_file,
                          std::string const& sight_vec_file,
                          std::string const& longitude_file,
                          std::string const& latitude_file,
                          std::string const& lattice_file,
                          // Outputs
			  std::vector< std::vector<vw::Vector3> > & world_sight_mat,
                          Vector3 & llh_scale, Vector3 & llh_offset,
                          Vector2 & pixel_scale, Vector2 & pixel_offset,
                          Vector<double> & normalized_llh,
                          Vector<double> & normalized_pixels){

  // Read the sight vectors 
  std::vector<Vector3> sight_vec;
  asp::read_3d_points(sight_vec_file, sight_vec);

  // Read the satellite positions
  std::vector<Vector3> sat_pos;
  asp::read_3d_points(sat_pos_file, sat_pos);

  int num_rows = sat_pos.size();
  int num_pts = sight_vec.size();
  int num_cols = num_pts / num_rows;

  // Sight vector in world coordinates
  world_sight_mat.clear();
  world_sight_mat.resize(num_rows);
  
  if (num_rows * num_cols != num_pts) 
    vw_throw( ArgumentErr()
	      << "Found " << num_rows << " satellite positions in "
	      << sat_pos_file << " and "
              << num_pts << " sight vectors in " << sight_vec_file
	      << ". The latter must be a multiple of the former.");
  
  // For each satellite position there many sight vectors
  // corresponding to pixel positions on that image line. Clone the
  // satellite positions to make them one per each sight vector. It is
  // easier to work with things that way later.
  std::vector<Vector3> full_sat_pos(num_pts);
  int count = 0;
  for (int row = 0; row < num_rows; row++) {
    for (int col = 0; col < num_cols; col++) {
      full_sat_pos[count] = sat_pos[row];
      count++;
    }
  }
  if (count != num_pts) 
    vw_throw( ArgumentErr() << "Book-keeping failure!\n" );

  std::vector<double> longitude;
  asp::read_1d_points(longitude_file, longitude);
  if (int(longitude.size()) != num_pts)
    vw_throw( ArgumentErr() << "Expecting " << num_pts << " longitude values in "
	      << longitude_file << " but got instead " << longitude.size() << ".\n" );
  
  std::vector<double> latitude;
  asp::read_1d_points(latitude_file, latitude);
  if (int(latitude.size()) != num_pts)
    vw_throw( ArgumentErr() << "Expecting " << num_pts << " latitude values in "
	      << latitude_file << " but got instead " << latitude.size() << ".\n" );

  // Covert geocentric latitude to geodetic latitude. Pages 60 and 79 of
  // https://asterweb.jpl.nasa.gov/content/03_data/04_Documents/aster_user_guide_v2.pdf
  // Geodetic = Arctan [(tan (Latitude)) / 0.99330562]
  double deg2rad = M_PI/180.0;
  for (size_t i = 0; i < latitude.size(); i++) {
    latitude[i] =  atan (tan (deg2rad*latitude[i]) / 0.99330562)/deg2rad;
  }
  
  std::vector<Vector2> pixels;
  asp::read_2d_points(lattice_file, pixels);
  if (int(pixels.size()) != num_pts)
    vw_throw( ArgumentErr() << "Expecting " << num_pts << " pixels in "
	      << lattice_file << " but got instead " << pixels.size() << ".\n" );

  // Convert from geodetic coordinates to xyz
  cartography::Datum datum;
  datum.set_well_known_datum("WGS84");
  std::vector<Vector3> ground_xyz(num_pts);
  for (int i = 0; i < num_pts; i++) {
    ground_xyz[i] = datum.geodetic_to_cartesian(Vector3(longitude[i], latitude[i], 0));
  }

  // Create the sight vectors, from the camera center to the ground, in world
  // coordinates, rather than in spacecraft coordinates, like sight_vec.
  for (int pt = 0; pt < num_pts; pt++) {
    Vector3 G = ground_xyz[pt]; 
    Vector3 C = full_sat_pos[pt];
    int row   = pt/num_cols;
    world_sight_mat[row].push_back( (G-C)/norm_2(G-C) );
  }
  if (world_sight_mat.empty() || (int)world_sight_mat[0].size() != num_cols) {
    vw_throw( ArgumentErr() << "Incorrect number of world sight vectors.\n");
  }
    
  // Form num_samples layers between min_height and max_height.
  // Each point there will have its corresponding pixel value.
  int num_total_pts = num_pts*num_samples;
  std::vector<Vector3> all_llh (num_total_pts);
  std::vector<Vector2> all_pixels(num_total_pts);
  count = 0;
  for (int sample = 0; sample < num_samples; sample++) {
    double height = min_height
      + double(sample)*(max_height - min_height)/(num_samples - 1.0);

    // Find an xyz position at roughly that height on the line
    // connecting the original ground point and the satellite
    // center. We need to solve a quadratic equation for that. We
    // assume the Earth is a sphere.
    for (int pt = 0; pt < num_pts; pt++) {

      Vector3 A = ground_xyz[pt]; 
      Vector3 B = full_sat_pos[pt];
      Vector3 D = B - A;

      // Find t such that norm(A + t*D) = norm(A) + height
      double  d = dot_prod(A, D) * dot_prod(A, D)
        + dot_prod(D, D) * (height*height + 2*norm_2(A)*height);
      double  t = ( -dot_prod(A, D) + sqrt(d) ) / dot_prod(D, D);
      Vector3 P = A + t*D;

      all_llh[count]  = datum.cartesian_to_geodetic(P);
      all_pixels[count] = pixels[pt];
      count++;
    }
  }
  
  // Find the range of lon-lat-heights
  BBox3 llh_box;
  for (size_t i = 0; i < all_llh.size(); i++) 
    llh_box.grow(all_llh[i]);
  
  // Find the range of pixels
  BBox2 pixel_box;
  for (size_t i = 0; i < all_pixels.size(); i++) 
    pixel_box.grow(all_pixels[i]);

  llh_scale  = (llh_box.max() - llh_box.min())/2.0; // half range
  llh_offset = (llh_box.max() + llh_box.min())/2.0; // center point

  pixel_scale  = (pixel_box.max() - pixel_box.min())/2.0; // half range 
  pixel_offset = (pixel_box.max() + pixel_box.min())/2.0; // center point

  normalized_llh.set_size(asp::RPCModel::GEODETIC_COORD_SIZE*num_total_pts);
  normalized_pixels.set_size(asp::RPCModel::IMAGE_COORD_SIZE*num_total_pts
                             + asp::RpcSolveLMA::NUM_PENALTY_TERMS);
  for (size_t i = 0; i < normalized_pixels.size(); i++) {
    // Important: The extra penalty terms are all set to zero here.
    normalized_pixels[i] = 0.0; 
  }

  // Form the arrays of normalized pixels and normalized llh
  for (int pt = 0; pt < num_total_pts; pt++) {

    // Normalize the pixel to -1 <> 1 range
    Vector3 llh_n   = elem_quot(all_llh[pt]    - llh_offset,   llh_scale);
    Vector2 pixel_n = elem_quot(all_pixels[pt] - pixel_offset, pixel_scale);

    subvector(normalized_llh, asp::RPCModel::GEODETIC_COORD_SIZE*pt,
              asp::RPCModel::GEODETIC_COORD_SIZE) = llh_n;
    subvector(normalized_pixels, asp::RPCModel::IMAGE_COORD_SIZE*pt,
              asp::RPCModel::IMAGE_COORD_SIZE   ) = pixel_n;
    
  }

  return;
}

// Save an XML file having all RPC information
void save_xml(int image_cols, int image_rows,
	      std::vector< std::vector<Vector2> > const& lattice_mat,
	      std::vector< std::vector<Vector3> > const& sight_mat,
	      std::vector< std::vector<Vector3> > const& world_sight_mat,
	      std::vector<Vector3>                const& sat_pos,
	      Vector3 const& llh_scale,
              Vector3 const& llh_offset,
              Vector2 const& pixel_scale,
              Vector2 const& pixel_offset,
              asp::RPCModel::CoeffVec const& line_num,
              asp::RPCModel::CoeffVec const& line_den,
              asp::RPCModel::CoeffVec const& samp_num,
              asp::RPCModel::CoeffVec const& samp_den,
              std::string const& out_cam_file){

  std::string lineoffset   = vw::num_to_str(pixel_offset.y());
  std::string sampoffset   = vw::num_to_str(pixel_offset.x());
  std::string latoffset    = vw::num_to_str(llh_offset.y());
  std::string longoffset   = vw::num_to_str(llh_offset.x());
  std::string heightoffset = vw::num_to_str(llh_offset.z());
  
  std::string linescale   = vw::num_to_str(pixel_scale.y());
  std::string sampscale   = vw::num_to_str(pixel_scale.x());
  std::string latscale    = vw::num_to_str(llh_scale.y());
  std::string longscale   = vw::num_to_str(llh_scale.x());
  std::string heightscale = vw::num_to_str(llh_scale.z());

  std::string linenumcoef = vw::vec_to_str(line_num);
  std::string linedencoef = vw::vec_to_str(line_den);
  std::string sampnumcoef = vw::vec_to_str(samp_num);
  std::string sampdencoef = vw::vec_to_str(samp_den);
  
  vw_out() << "Writing: " << out_cam_file << std::endl;
  std::ofstream ofs(out_cam_file.c_str());
  
  ofs << "<isd>\n";

  // Rigorous camera model

  // Lattice points
  ofs << "    <LATTICE_POINT>\n";
  for (size_t row = 0; row < lattice_mat.size(); row++) {
    for (size_t col = 0; col < lattice_mat[row].size(); col++) {
      ofs << vw::vec_to_str(lattice_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </LATTICE_POINT>\n";

  // Sight vector
  ofs << "    <SIGHT_VECTOR>\n";
  for (size_t row = 0; row < sight_mat.size(); row++) {
    for (size_t col = 0; col < sight_mat[row].size(); col++) {
      ofs << vw::vec_to_str(sight_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </SIGHT_VECTOR>\n";

  // Sight vector in world coordinates
  ofs << "    <WORLD_SIGHT_VECTOR>\n";
  for (size_t row = 0; row < world_sight_mat.size(); row++) {
    for (size_t col = 0; col < world_sight_mat[row].size(); col++) {
      ofs << vw::vec_to_str(world_sight_mat[row][col]) << std::endl;
    }
    ofs << std::endl;
  }
  ofs << "    </WORLD_SIGHT_VECTOR>\n";
  
  // Satellite position
  ofs << "    <SAT_POS>\n";
  for (size_t row = 0; row < sat_pos.size(); row++) {
    ofs << vw::vec_to_str(sat_pos[row]) << std::endl;
  }
  ofs << "    </SAT_POS>\n";

  // Image size
  ofs << "    <IMAGE_COLS>" << image_cols << "</IMAGE_COLS>\n";
  ofs << "    <IMAGE_ROWS>" << image_rows << "</IMAGE_ROWS>\n";
  
  // RPC
  ofs << "    <RPB>\n";
  ofs << "        <SATID>ASTER_L1A_VNIR_Band3</SATID>\n";
  ofs << "        <IMAGE>\n";
  ofs << "            <LINEOFFSET>"      << lineoffset   << "</LINEOFFSET>\n";
  ofs << "            <SAMPOFFSET>"      << sampoffset   << "</SAMPOFFSET>\n";
  ofs << "            <LATOFFSET>"       << latoffset    << "</LATOFFSET>\n";
  ofs << "            <LONGOFFSET>"      << longoffset   << "</LONGOFFSET>\n";
  ofs << "            <HEIGHTOFFSET>"    << heightoffset << "</HEIGHTOFFSET>\n";
  ofs << "            <LINESCALE>"       << linescale    << "</LINESCALE>\n";
  ofs << "            <SAMPSCALE>"       << sampscale    << "</SAMPSCALE>\n";
  ofs << "            <LATSCALE>"        << latscale     << "</LATSCALE>\n";
  ofs << "            <LONGSCALE>"       << longscale    << "</LONGSCALE>\n";
  ofs << "            <HEIGHTSCALE>"     << heightscale  << "</HEIGHTSCALE>\n";
  ofs << "            <LINENUMCOEFList>\n";
  ofs << "                <LINENUMCOEF>" << linenumcoef  << "</LINENUMCOEF>\n";
  ofs << "            </LINENUMCOEFList>\n";
  ofs << "            <LINEDENCOEFList>\n";
  ofs << "                <LINEDENCOEF>" << linedencoef  << "</LINEDENCOEF>\n";
  ofs << "            </LINEDENCOEFList>\n";
  ofs << "            <SAMPNUMCOEFList>\n";
  ofs << "                <SAMPNUMCOEF>" << sampnumcoef  << "</SAMPNUMCOEF>\n";
  ofs << "            </SAMPNUMCOEFList>\n";
  ofs << "            <SAMPDENCOEFList>\n";
  ofs << "                <SAMPDENCOEF>" << sampdencoef  << "</SAMPDENCOEF>\n";
  ofs << "            </SAMPDENCOEFList>\n";
  ofs << "        </IMAGE>\n";
  ofs << "    </RPB>\n";
  ofs << "</isd>\n";
  ofs.close();
}

// Create XML files containing rigorous camera info, and compute the RPC coefficients as well.
void gen_xml(double min_height, double max_height, int num_samples,
	     double penalty_weight,
	     std::string const& image_file,
	     std::string const& sat_pos_file,
	     std::string const& sight_vec_file,
	     std::string const& longitude_file,
	     std::string const& latitude_file,
	     std::string const& lattice_file,
	     std::string const& out_cam_file){
  
  std::vector< std::vector<Vector2> > lattice_mat;
  asp::read_matrix_from_file(lattice_file, lattice_mat);
  
  std::vector< std::vector<Vector3> > sight_mat;
  asp::read_matrix_from_file(sight_vec_file, sight_mat);
  
  if (lattice_mat.empty() || sight_mat.empty()     ||
      lattice_mat.size()     != sight_mat.size()   ||
      lattice_mat[0].size() != sight_mat[0].size() ) {
    vw_throw( ArgumentErr() << "Inconsistent lattice point and sight vector information.\n");
  }

  // Read the satellite positions
  std::vector<Vector3> sat_pos;
  asp::read_3d_points(sat_pos_file, sat_pos);
  if (sat_pos.size() != sight_mat.size()) 
    vw_throw( ArgumentErr() << "Inconsistent satellite position and sight vector information.\n");
    
  Vector3 llh_scale, llh_offset;
  Vector2 pixel_scale, pixel_offset;
  Vector<double> normalized_llh;
  Vector<double> normalized_pixels;
  std::vector< std::vector<vw::Vector3> > world_sight_mat; // sight dir in world coords
  generate_point_pairs(// inputs
                       min_height, max_height, num_samples, penalty_weight,  
                       sat_pos_file, sight_vec_file,  
                       longitude_file, latitude_file, lattice_file,  
                       // Outputs
		       world_sight_mat, 
                       llh_scale, llh_offset,  
                       pixel_scale, pixel_offset,  
                       normalized_llh,  
                       normalized_pixels);
  
  // Find the RPC coefficients
  asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
  std::string output_prefix = ""; 
  asp::gen_rpc(// Inputs
               penalty_weight, output_prefix,
               normalized_llh, normalized_pixels,  
               llh_scale, llh_offset, pixel_scale, pixel_offset,
               // Outputs
               line_num, line_den, samp_num, samp_den);
  
#if 0
  // Dump the output to stdout
  asp::print_vec("pixel_scale",  pixel_scale );
  asp::print_vec("pixel_offset", pixel_offset);
  asp::print_vec("llh_scale",    llh_scale   );
  asp::print_vec("llh_offset",   llh_offset  );
  asp::print_vec("line_num",     line_num    );
  asp::print_vec("line_den",     line_den    );
  asp::print_vec("samp_num",     samp_num    );
  asp::print_vec("samp_den",     samp_den    );
#endif

  int image_cols, image_rows;
  DiskImageView<float> input_img(image_file);
  image_cols = input_img.cols();
  image_rows = input_img.rows();
  
  save_xml(image_cols, image_rows, lattice_mat,
	   sight_mat, world_sight_mat, sat_pos,
	   llh_scale, llh_offset, pixel_scale, pixel_offset,  
           line_num, line_den, samp_num, samp_den,  
           out_cam_file);
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

#if 0
    std::cout 	<< "nadir_image             " 	<< nadir_image 		<< std::endl;
    std::cout 	<< "back_image              " 	<< back_image 		<< std::endl;
    std::cout 	<< "nadir_sat_pos           " 	<< nadir_sat_pos 	<< std::endl;
    std::cout 	<< "back_sat_pos            " 	<< back_sat_pos 	<< std::endl;
    std::cout 	<< "nadir_sight_vec         " 	<< nadir_sight_vec 	<< std::endl;
    std::cout 	<< "back_sight_vec          " 	<< back_sight_vec 	<< std::endl;
    std::cout 	<< "nadir_corr_table        " 	<< nadir_corr_table 	<< std::endl;
    std::cout 	<< "back_corr_table         " 	<< back_corr_table 	<< std::endl;
    std::cout 	<< "nadir_longitude         " 	<< nadir_longitude 	<< std::endl;
    std::cout 	<< "back_longitude          " 	<< back_longitude 	<< std::endl;
    std::cout 	<< "nadir_latitude          " 	<< nadir_latitude 	<< std::endl;
    std::cout 	<< "back_latitude           " 	<< back_latitude 	<< std::endl;
    std::cout 	<< "nadir_lattice_point     " 	<< nadir_lattice_point 	<< std::endl;
    std::cout 	<< "back_lattice_point      " 	<< back_lattice_point 	<< std::endl;
    std::cout 	<< "output nadir image:     " 	<< out_nadir_image 	<< ' '
              	<< out_nadir_cam 		<< std::endl;
    std::cout 	<< "output back image:      " 	<< out_back_image  	<< ' '
              	<< out_back_cam  		<< std::endl;
#endif
    
    gen_xml(opt.min_height, opt.max_height, opt.num_samples, opt.penalty_weight,
	    nadir_image, nadir_sat_pos, nadir_sight_vec, nadir_longitude, nadir_latitude,  
	    nadir_lattice_point, out_nadir_cam);
    
    gen_xml(opt.min_height, opt.max_height, opt.num_samples, opt.penalty_weight,
	    back_image, back_sat_pos, back_sight_vec, back_longitude, back_latitude,  
	    back_lattice_point, out_back_cam);
    
    apply_radiometric_corrections(opt, nadir_image, nadir_corr_table, out_nadir_image);
    apply_radiometric_corrections(opt, back_image,  back_corr_table,  out_back_image);
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

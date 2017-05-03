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

// Create an RPC model from point pairs obtained by sampling the xyz
// bounding box of the given DEM. Imitate to some extent the DG WV camera
// model.

// This code needs serious cleanup! Particularly, inputs must be
// passed properly by parsing arguments. Not ready for release in any
// way!  This is an internal debug tool for now.

#include <asp/Sessions/StereoSessionFactory.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Core/StringUtils.h>
#include <vw/Camera/PinholeModel.h>
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
using namespace vw::camera;
using namespace std;
using namespace vw::cartography;

struct Options : public vw::cartography::GdalWriteOptions {
  string input_dir, output_prefix; 
  double gsd; // ground sample distance
  double min_height, max_height;
  int num_samples;
  double penalty_weight;
  Options(): gsd(-1), min_height(-1), max_height(-1), num_samples(-1), penalty_weight(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("output-prefix,o",   po::value(&opt.output_prefix), "Specify the output prefix.")
    ("gsd",     po::value(&opt.gsd)->default_value(-1),
     "Expected resolution on the ground, in meters.")
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

int main( int argc, char *argv[] ) {

  Options opt;
  try {

    std::string img_file = argv[1];
    std::string cam_file = argv[2];
    std::string dem_file = argv[3];
    std::string rpc_file = argv[4];
    std::string session_name  = argv[5];

    std::string crop_img;
    if (argc >= 7) {
      crop_img = argv[6];
    }

    double gsd = -1;
    if (argc >= 8) {
      gsd = atof(argv[7]);
    }

    double rpc_penalty_weight = 0.03;

    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSessionFactory::create
		       (session_name, opt,
			img_file, img_file, cam_file, cam_file, dem_file));
    
    vw_out() << "Loading image and camera: " << img_file << ' ' << cam_file << ".\n";
    boost::shared_ptr<CameraModel> cam = session->camera_model(img_file, cam_file);

    float dem_nodata_val = -std::numeric_limits<float>::max(); 
    vw::read_nodata_val(dem_file, dem_nodata_val);
    ImageView< PixelMask<double> > dem = create_mask
      (channel_cast<double>(DiskImageView<float>(dem_file)), dem_nodata_val);

    double min_ht = std::numeric_limits<double>::max(); 
    double max_ht = -min_ht;
    for (int col = 0; col < dem.cols(); col++) {
      for (int row = 0; row < dem.rows(); row++) {
	if (!is_valid(dem(col, row))) continue;
	min_ht = std::min(min_ht, dem(col, row).child());
	max_ht = std::max(max_ht, dem(col, row).child());
      }
    }

    GeoReference dem_geo;
    if (!read_georeference(dem_geo, dem_file))
      vw_throw( ArgumentErr() << "Missing georef.\n");

    float img_nodata_val = -std::numeric_limits<float>::max(); 
    bool has_img_nodata = vw::read_nodata_val(img_file, img_nodata_val);
    ImageView< PixelMask<float> > input_img
      = create_mask(DiskImageView<float>(img_file), img_nodata_val);
    
    int image_cols = input_img.cols();
    int image_rows = input_img.rows();
    
    BBox2 big_pixel_box;
    big_pixel_box.grow(Vector2(0, 0));
    big_pixel_box.grow(Vector2(image_cols, image_rows));
    
    //vw_out() << "Loading camera model file: " << cam_file << "\n";
    //PinholeModel cam(cam_file);
    //std::cout << "--camera is " << cam << std::endl;

    // TODO: Merge this code with what is in sfs.cc!
    // Generate point pairs
    std::vector<Vector3> all_llh;
    std::vector<Vector2> all_pixels;

    vw_out() << "Projecting pixels into the camera to generate the RPC model.\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(dem.cols());
    tpc.report_progress(0);

    // Find the range of lon-lat-heights
    BBox3 llh_box;
//     asp::RPCModel * cam2 = dynamic_cast<asp::RPCModel*>(cam.get());
//     Vector3 llh_offset2 = cam2->lonlatheight_offset();
//     Vector3 llh_scale2  = cam2->lonlatheight_scale();

//     llh_box.grow(llh_offset2 + llh_scale2);
//     llh_box.grow(llh_offset2 - llh_scale2);
//     std::cout << "---box 2 is " << llh_box << std::endl;
    
    //vw::Vector3 const& lonlatheight_offset() const { return m_lonlatheight_offset; }
    //vw::Vector3 const& lonlatheight_scale () const { return m_lonlatheight_scale;  }
    
    // If the DEM is too big, we need to skip points. About
    // 40,000 points should be good enough to determine 78 RPC
    // coefficients.
    double num = 200.0;
    double delta_col = std::max(1.0, dem.cols()/num);
    double delta_row = std::max(1.0, dem.rows()/num);
    for (double dcol = 0; dcol < dem.cols(); dcol += delta_col) {
      for (double drow = 0; drow < dem.rows(); drow += delta_row) {
	int col = dcol, row = drow; // cast to int

	if (!is_valid(dem(col, row))) continue;
	
	Vector2 pix(col, row);
	Vector2 lonlat = dem_geo.pixel_to_lonlat(pix);
          
	// Lon lat height
	Vector3 llh;
	llh[0] = lonlat[0]; llh[1] = lonlat[1]; llh[2] = dem(col, row).child();
	Vector3 xyz = dem_geo.datum().geodetic_to_cartesian(llh);

	// Go back to llh. This is a bugfix for the 360 deg offset problem.
	llh = dem_geo.datum().cartesian_to_geodetic(xyz);
          
	Vector2 cam_pix = cam->point_to_pixel(xyz);

        if (big_pixel_box.contains(cam_pix)) {
	  all_llh.push_back(llh);
	  all_pixels.push_back(cam_pix);
	  llh_box.grow(llh);
        }
	
      }
      
      tpc.report_incremental_progress( inc_amount );
    }
    tpc.report_finished();
    
    //InterpolationView<EdgeExtensionView< ImageView< PixelMask<double> >, ConstantEdgeExtension >, BilinearInterpolation> interp_dem
    // = interpolate(dem, BilinearInterpolation(), ConstantEdgeExtension());
#if 0
    
    BBox3 llh_box2 = llh_box;
    Vector3 len = llh_box2.max() - llh_box2.min();

//     double b = 0.1;
//     std::cout << "--box before " << llh_box2 << std::endl;
//     llh_box2.max() += b*len;
//     llh_box2.min() -= b*len;
//     std::cout << "--box after " << llh_box2 << std::endl;
      
    llh_box = BBox3();
    all_llh.clear();
    all_pixels.clear();
    num = 30;
    double delta_lon = (llh_box2.max()[0] - llh_box2.min()[0])/num;
    double delta_lat = (llh_box2.max()[1] - llh_box2.min()[1])/num;
    double delta_h   = (llh_box2.max()[2] - llh_box2.min()[2])/num; 
    std::cout << "delta lon, lat, h "
              << delta_lon << ' ' << delta_lat << ' ' << delta_h << std::endl;

    std::cout.precision(20);
    
    vw_out() << "Projecting pixels into the camera to generate the RPC model2.\n";
    vw::TerminalProgressCallback tpc2("asp", "\t--> ");
    tpc2.report_progress(0);

    for (double dlon = llh_box2.min()[0]; dlon <= llh_box2.max()[0]; dlon += delta_lon) {
      for (double dlat = llh_box2.min()[1]; dlat <= llh_box2.max()[1]; dlat += delta_lat) {
        for (double dh = llh_box2.min()[2]; dh <= llh_box2.max()[2]; dh += delta_h) {

#if 0
          Vector2 lonlat(dlon, dlat);
          Vector2 pix = dem_geo.lonlat_to_pixel(lonlat);
          
          //std::cout << "--dlon dlat " << dlon << ' ' << dlat << std::endl;
          
          int col = pix[0], row = pix[1]; // cast to int
          
          //std::cout << "--pixel is " << pix << std::endl;
	
          if (col < 0 || col >= dem.cols()) continue;
          if (row < 0 || row >= dem.rows()) continue;
	  
          if (!is_valid(dem(col, row))) continue;
          
          // Updated pix and lonlat
          pix = Vector2(col, row);
          lonlat = dem_geo.pixel_to_lonlat(pix);
          
          //std::cout << "pix lonlat height " << pix << ' ' << lonlat << ' ' << dem(col, row)
          //		  << std::endl;
          
          
          // Lon lat height
          Vector3 llh;
          llh[0] = lonlat[0]; llh[1] = lonlat[1]; llh[2] = dem(col, row).child();
#else
          
          // Lon lat height
          Vector3 llh(dlon, dlat, dh);

          Vector3 xyz = dem_geo.datum().geodetic_to_cartesian(llh);
          
          // Go back to llh. This is a bugfix for the 360 deg offset problem.
          llh = dem_geo.datum().cartesian_to_geodetic(xyz);
          
          Vector2 cam_pix = cam->point_to_pixel(xyz);
	  if (big_pixel_box.contains(cam_pix) && llh_box2.contains(llh)) {
	    all_llh.push_back(llh);
	    all_pixels.push_back(cam_pix);
	    llh_box.grow(llh);
	  }
#endif
	  
          //llh_box.grow(llh);
	  
        }
      }   
      tpc2.report_incremental_progress( inc_amount );
    }
    tpc2.report_finished();
    
#endif

    BBox2 pixel_box;
    // Find the range of pixels
    for (size_t i = 0; i < all_pixels.size(); i++) {
      if (big_pixel_box.contains(all_pixels[i])) {
        pixel_box.grow(all_pixels[i]);
      }
    }

    if (crop_img == "crop") {
      // Cast to int so that we can crop properly
      pixel_box.min() = floor(pixel_box.min());
      pixel_box.max() = floor(pixel_box.max());
      pixel_box.crop(big_pixel_box);

      std::string out_img = fs::path(rpc_file).replace_extension("tif").string();
      vw_out() << "Writing: " << out_img << std::endl;

      GeoReference img_geo;
      bool has_img_geo = false;
      vw::cartography::block_write_gdal_image(out_img,
                                              apply_mask(crop(input_img, pixel_box),
							 img_nodata_val),
					      has_img_geo, img_geo,
					      has_img_nodata, img_nodata_val,
                                              opt,
                                              TerminalProgressCallback("asp", "\t-->: "));
      for (size_t i = 0; i < all_pixels.size(); i++) {
        all_pixels[i] -= pixel_box.min();
      }
      Vector2 shift = pixel_box.min(); // copy the corner before overwriting it below
      pixel_box -= shift;
    }
    
    Vector3 llh_scale  = (llh_box.max() - llh_box.min())/2.0; // half range
    Vector3 llh_offset = (llh_box.max() + llh_box.min())/2.0; // center point
      
    Vector2 pixel_scale  = (pixel_box.max() - pixel_box.min())/2.0; // half range 
    Vector2 pixel_offset = (pixel_box.max() + pixel_box.min())/2.0; // center point
      
    vw_out() << "Lon-lat-height box for the RPC approx: " << llh_box   << std::endl;
    vw_out() << "Camera pixel box for the RPC approx:   " << pixel_box << std::endl;

    Vector<double> normalized_llh;
    Vector<double> normalized_pixels;
    int num_total_pts = all_llh.size();
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

    // Find the RPC coefficients
    asp::RPCModel::CoeffVec line_num, line_den, samp_num, samp_den;
    std::string output_prefix = "";
    vw_out() << "Generating the RPC approximation using " << num_total_pts << " point pairs.\n";
    asp::gen_rpc(// Inputs
		 rpc_penalty_weight, output_prefix,
		 normalized_llh, normalized_pixels,  
		 llh_scale, llh_offset, pixel_scale, pixel_offset,
		 // Outputs
		 line_num, line_den, samp_num, samp_den);
      
//     m_rpc_model = boost::shared_ptr<asp::RPCModel>
//       (new asp::RPCModel(geo.datum(), line_num, line_den,
// 			 samp_num, samp_den, pixel_offset, pixel_scale,
// 			 llh_offset, llh_scale));
    
#if 1
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


  // TODO: Integrate this with aster2asp existing fun!
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
  
  std::string gsd_str = vw::num_to_str(gsd);
  
  vw_out() << "Writing: " << rpc_file << std::endl;
  std::ofstream ofs(rpc_file.c_str());
  ofs.precision(18);
  
  ofs << "<isd>\n";

  ofs << "   <IMD>\n";
  
//         <VERSION>23.11</VERSION>
//         <GENERATIONTIME>2012-08-20T18:27:06.000000Z</GENERATIONTIME>
//         <PRODUCTORDERID>052783426010_01_P001</PRODUCTORDERID>
//         <PRODUCTCATALOGID>A0300100BF84B600</PRODUCTCATALOGID>
//         <CHILDCATALOGID>21000100BF84F100</CHILDCATALOGID>
//         <IMAGEDESCRIPTOR>Stereo1B</IMAGEDESCRIPTOR>
//         <BANDID>P</BANDID>
//         <PANSHARPENALGORITHM>None</PANSHARPENALGORITHM>
  ofs << "        <NUMROWS>" << image_rows << "</NUMROWS>\n";
  ofs << "        <NUMCOLUMNS>" << image_cols << "</NUMCOLUMNS>\n";
//         <PRODUCTLEVEL>Stereo 1B</PRODUCTLEVEL>
//         <PRODUCTTYPE>Stereo</PRODUCTTYPE>
//         <NUMBEROFLOOKS>2</NUMBEROFLOOKS>
//         <RADIOMETRICLEVEL>Corrected</RADIOMETRICLEVEL>
//         <BITSPERPIXEL>16</BITSPERPIXEL>
//         <COMPRESSIONTYPE>None</COMPRESSIONTYPE>
//         <OUTPUTFORMAT>GeoTIFF</OUTPUTFORMAT>
  ofs << "                <BAND_P>\n";
  ofs << "                    <ULLON>" << llh_box.min()[0] << "</ULLON>\n";
  ofs << "                    <ULLAT>" << llh_box.max()[1] << "</ULLAT>\n";
  ofs << "                    <ULHAE>" << llh_box.max()[2] << "</ULHAE>\n";
  
  ofs << "                    <URLON>" << llh_box.max()[0] << "</URLON>\n";
  ofs << "                    <URLAT>" << llh_box.max()[1] << "</URLAT>\n";
  ofs << "                    <URHAE>" << llh_box.max()[2] << "</URHAE>\n";

  ofs << "                    <LRLON>" << llh_box.max()[0] << "</LRLON>\n";
  ofs << "                    <LRLAT>" << llh_box.min()[1] << "</LRLAT>\n";
  ofs << "                    <LRHAE>" << llh_box.min()[2] << "</LRHAE>\n";

  ofs << "                    <LLLON>" << llh_box.min()[0] << "</LLLON>\n";
  ofs << "                    <LLLAT>" << llh_box.min()[1] << "</LLLAT>\n";
  ofs << "                    <LLHAE>" << llh_box.max()[2] << "</LLHAE>\n";

  //<ABSCALFACTOR>6.288518000000000e-02</ABSCALFACTOR>
  //          <EFFECTIVEBANDWIDTH>3.720000000000000e-01</EFFECTIVEBANDWIDTH>
  //          <TDILEVEL>32</TDILEVEL>
  ofs << "        </BAND_P>\n";
  ofs << "        <IMAGE>\n";
  ofs << "            <SATID>WV01</SATID>\n";
  ofs << "            <MODE>FullSwath</MODE>\n";
  ofs << "            <SCANDIRECTION>Forward</SCANDIRECTION>\n";
  ofs << "            <CATID>0</CATID>\n";
  ofs << "            <TLCTIME>2010-06-21T21:32:55.534775Z</TLCTIME>\n";
  ofs << "            <NUMTLC>2</NUMTLC>\n";   
  ofs << "            <TLCLISTList>\n";                                        
  ofs << "               <TLCLIST>0.0 0.000000000000000e+00</TLCLIST>\n";     
  ofs << "               <TLCLIST>27572.0 1.378600000000000e+00</TLCLIST>\n"; 
  ofs << "            </TLCLISTList>\n";                                       
  ofs << "            <FIRSTLINETIME>2010-06-21T21:32:55.534775Z</FIRSTLINETIME>\n";
  ofs << "            <AVGLINERATE>20000.0</AVGLINERATE>\n";                        
  ofs << "            <EXPOSUREDURATION>0.0016</EXPOSUREDURATION>\n";               
  ofs << "            <MINCOLLECTEDROWGSD>"   << gsd_str << "</MINCOLLECTEDROWGSD>\n";            
  ofs << "            <MAXCOLLECTEDROWGSD>"   << gsd_str << "</MAXCOLLECTEDROWGSD>\n";            
  ofs << "            <MEANCOLLECTEDROWGSD>"  << gsd_str << "</MEANCOLLECTEDROWGSD>\n";          
  ofs << "            <MINCOLLECTEDCOLGSD>"   << gsd_str << "</MINCOLLECTEDCOLGSD>\n";            
  ofs << "            <MAXCOLLECTEDCOLGSD>"   << gsd_str << "</MAXCOLLECTEDCOLGSD>\n";            
  ofs << "            <MEANCOLLECTEDCOLGSD>"  << gsd_str << "</MEANCOLLECTEDCOLGSD>\n";          
  ofs << "            <MEANCOLLECTEDGSD>"     << gsd_str << "</MEANCOLLECTEDGSD>\n";                 
  ofs << "            <MEANPRODUCTGSD>"       << gsd_str << "</MEANPRODUCTGSD>\n";                       
  ofs << "        </IMAGE>\n";
  ofs << "   </IMD>\n";

  // RPC
  ofs << "    <RPB>\n";
  ofs << "        <SATID>WV01</SATID>\n";
  ofs << "         <BANDID>P</BANDID>\n";
  ofs << "         <SPECID>RPC00B</SPECID>\n";
  ofs << "        <IMAGE>\n";
  ofs << "	        <ERRBIAS>1.006000000000000e+01</ERRBIAS>\n"; // why?
  ofs << "	        <ERRRAND>1.100000000000000e-01</ERRRAND>\n"; // why?

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
    
#if 0    
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
#endif
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

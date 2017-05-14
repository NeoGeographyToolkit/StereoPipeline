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

// Create an RPC model from point pairs obtained by sampling a given lon-lat-height box
// or a given DEM. Imitate to some extent the DG WV camera
// model. Optionally save a TIF version of the input image restricted to the same box.
// The saved image and RPC file can be used for stereo with S2P, ASP, and SETSM
// some work needs to be done for these packages to give correct results off-Earth.

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
  double penalty_weight;
  string image_file, camera_file, output_rpc, stereo_session, bundle_adjust_prefix,
    datum_str, dem_file;
  bool no_crop, skip_computing_rpc, save_tif, has_output_nodata;
  BBox2 lon_lat_range;
  BBox2i image_crop_box;
  Vector2 height_range;
  float output_nodata_value;
  double gsd;
  int num_samples;
  Options(): penalty_weight(-1.0), no_crop(false),
             skip_computing_rpc(false), save_tif(false), has_output_nodata(false),
             output_nodata_value(-std::numeric_limits<float>::max()),
             gsd(-1.0), num_samples(-1) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("datum",            po::value(&opt.datum_str)->default_value(""),
     "Use this datum to interpret the heights. Options: WGS_1984, D_MOON (1,737,400 meters), D_MARS (3,396,190 meters), MOLA (3,396,000 meters), NAD83, WGS72, and NAD27. Also accepted: Earth (=WGS_1984), Mars (=D_MARS), Moon (=D_MOON).")
    ("lon-lat-range", po::value(&opt.lon_lat_range)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
     "The longitude-latitude range in which to compute the RPC model. Specify in the format: lon_min lat_min lon_max lat_max.")
    ("height-range", po::value(&opt.height_range)->default_value(Vector2i(0,0),"0 0"),
     "Minimum and maximum heights above the datum in which to compute the RPC model.")
    ("num-samples",     po::value(&opt.num_samples)->default_value(40),
     "How many samples to use in each direction in the longitude-latitude-height range.")
    ("penalty-weight",     po::value(&opt.penalty_weight)->default_value(0.03), // check here!
     "A higher penalty weight will result in smaller higher-order RPC coefficients.")
    ("save-tif-image", po::bool_switch(&opt.save_tif)->default_value(false),
     "Save a TIF version of the input image that approximately corresponds to the input longitude-latitude-height range and which can be used for stereo together with the RPC model.")
    ("output-nodata-value", po::value(&opt.output_nodata_value)->default_value(-std::numeric_limits<float>::max()),
     "Set the image output nodata value.")
    ("session-type,t",   po::value(&opt.stereo_session)->default_value(""),
     "Select the input camera model type. Normally this is auto-detected, but may need to be specified if the input camera model is in XML format. Options: pinhole isis rpc dg spot5 aster.")
    ("bundle-adjust-prefix", po::value(&opt.bundle_adjust_prefix),
     "Use the camera adjustment obtained by previously running bundle_adjust with this output prefix.")
    ("image-crop-box", po::value(&opt.image_crop_box)->default_value(BBox2i(0,0,0,0), "0 0 0 0"),
     "The output image and RPC model should not exceed this box, specified in input image pixels as minx miny widx widy.")
    ("no-crop", po::bool_switch(&opt.no_crop)->default_value(false),
      "Try to create an RPC model over the entire input image, even if the input longitude-latitude-height box covers just a small portion of it. Not recommended.")
    ("skip-computing-rpc", po::bool_switch(&opt.skip_computing_rpc)->default_value(false),
     "Skip computing the RPC model.")
    ("dem-file",   po::value(&opt.dem_file)->default_value(""),
     "Instead of using a longitude-latitude-height box, sample the surface of this DEM.")
    ("gsd",     po::value(&opt.gsd)->default_value(-1),
     "Expected resolution on the ground, in meters. This is needed for SETSM.");

  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("camera-image", po::value(&opt.image_file))
    ("camera-model", po::value(&opt.camera_file))
    ("output-rpc" , po::value(&opt.output_rpc));

  po::positional_options_description positional_desc;
  positional_desc.add("camera-image",1);
  positional_desc.add("camera-model",1);
  positional_desc.add("output-rpc", 1);

  string usage("[options] <camera-image> <camera-model> <output-rpc>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
			    positional, positional_desc, usage,
			    allow_unregistered, unregistered);

  opt.has_output_nodata = vm.count("output-nodata-value");
  
  if ( opt.image_file.empty() )
    vw_throw( ArgumentErr() << "Missing input image.\n" << usage << general_options );

  if (boost::iends_with(opt.image_file, ".cub") && opt.stereo_session == "" )
    opt.stereo_session = "isis";

  // Need this to be able to load adjusted camera models. That will happen
  // in the stereo session.
  asp::stereo_settings().bundle_adjust_prefix = opt.bundle_adjust_prefix;

  // Swap min and max if need be
  if ( opt.lon_lat_range.min().x() > opt.lon_lat_range.max().x() )
    std::swap( opt.lon_lat_range.min().x(), opt.lon_lat_range.max().x() );
  if ( opt.lon_lat_range.min().y() > opt.lon_lat_range.max().y() )
    std::swap( opt.lon_lat_range.min().y(), opt.lon_lat_range.max().y() );
  
  // If we cannot read the data from a DEM, must specify a lot of things.
  if (opt.dem_file.empty()) {
    
    if (opt.datum_str.empty() )
      vw_throw( ArgumentErr() << "Missing input datum.\n" << usage << general_options );

    if (opt.height_range[0] >= opt.height_range[1])
      vw_throw( ArgumentErr() << "Must specify a valid range of heights.\n"
                << usage << general_options );
    
    if (opt.lon_lat_range.empty() )
      vw_throw( ArgumentErr() << "Must specify a valid range of longitude and latitude.\n"
                << usage << general_options );
  }
  
  vw_out() << "Height range is  " << opt.height_range[0] << ' ' << opt.height_range[1] << std::endl;
  vw_out() << "Lon-lat range is " << opt.lon_lat_range.min() << ' ' << opt.lon_lat_range.max()
           << std::endl;

  if (!opt.dem_file.empty()) {
    opt.num_samples *= 5;
    vw_out() << "Since an input DEM was specified, increasing the number of samples "
             << "on its surface to " << opt.num_samples << "^2.\n";
  }

  // Convert from width and height to min and max
  if (!opt.image_crop_box.empty()) {
    BBox2 b = opt.image_crop_box; // make a copy
    opt.image_crop_box = BBox2i(b.min().x(), b.min().y(), b.max().x(), b.max().y());
  }
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {

    handle_arguments(argc, argv, opt);

    typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;
    SessionPtr session(asp::StereoSessionFactory::create
		       (opt.stereo_session, // may change inside
                        opt,
			opt.image_file, opt.image_file,
                        opt.camera_file, opt.camera_file,
                        opt.output_rpc,
                        opt.dem_file,
                        false) ); // Do not allow promotion from normal to map projected session
    
    // If the session was above auto-guessed as isis, adjust for the fact
    // that the isis .cub file also has camera info.
    if ((session->name() == "isis" || session->name() == "isismapisis")
          && opt.output_rpc.empty() ){
      // The user did not provide an output file. Then the camera
      // information is contained within the image file and what is in
      // the camera file is actually the output file.
      opt.output_rpc  = opt.camera_file;
      opt.camera_file = opt.image_file;
    }

    if ( opt.camera_file.empty() )
      vw_throw( ArgumentErr() << "Missing input camera.\n" );

    if ( opt.output_rpc.empty() )
      vw_throw( ArgumentErr() << "Missing output RPC file.\n" );

    // Create the output directory
    vw::create_out_dir(opt.output_rpc);
    
    boost::shared_ptr<CameraModel> cam = session->camera_model(opt.image_file, opt.camera_file);

    // The input nodata value
    float input_nodata_value = -std::numeric_limits<float>::max(); 
    vw::read_nodata_val(opt.image_file, input_nodata_value);

    // If the output nodata value was not specified, use the input one
    if (!opt.has_output_nodata) {
      opt.output_nodata_value = input_nodata_value;
      opt.has_output_nodata = true;
    }

    // Read the image as float and mask it right away
    ImageViewRef< PixelMask<float> > input_img
      = create_mask(DiskImageView<float>(opt.image_file), input_nodata_value);

    // The bounding box
    BBox2 image_box = bounding_box(input_img);
    if (!opt.image_crop_box.empty()) 
      image_box.crop(opt.image_crop_box);
    
    // TODO: Merge this code with what is in sfs.cc!
    // Generate point pairs
    std::vector<Vector3> all_llh;
    std::vector<Vector2> all_pixels;

    vw_out() << "Projecting pixels into the camera to generate the RPC model.\n";
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    double inc_amount = 1.0 / double(opt.num_samples);
    
    if (opt.dem_file.empty()) {

      cartography::Datum datum(opt.datum_str);
      vw_out() << "Using datum: " << datum << std::endl;
      
      BBox2   & ll = opt.lon_lat_range; // shortcut
      Vector2 & H  = opt.height_range;
      double delta_lon = (ll.max()[0] - ll.min()[0])/double(opt.num_samples);
      double delta_lat = (ll.max()[1] - ll.min()[1])/double(opt.num_samples);
      double delta_ht  = (H[1] - H[0])/double(opt.num_samples);
      tpc.report_progress(0);
      for (double lon = ll.min()[0]; lon <= ll.max()[0]; lon += delta_lon) {
        for (double lat = ll.min()[1]; lat <= ll.max()[1]; lat += delta_lat) {
          for (double ht = H[0]; ht <= H[1]; ht += delta_ht) {

            Vector3 llh(lon, lat, ht);
            Vector3 xyz = datum.geodetic_to_cartesian(llh);
            
            // Go back to llh. This is a bugfix for the 360 deg offset problem.
            llh = datum.cartesian_to_geodetic(xyz);
            
            Vector2 cam_pix = cam->point_to_pixel(xyz);
            if (image_box.contains(cam_pix)) {
              all_llh.push_back(llh);
              all_pixels.push_back(cam_pix);
            }
            
          }
        }
        tpc.report_incremental_progress(inc_amount);
      }
      
    }else{
      vw_out() << "Sampling the surface of the DEM: " << opt.dem_file  << std::endl;

      float dem_nodata_val = -std::numeric_limits<float>::max(); 
      vw::read_nodata_val(opt.dem_file, dem_nodata_val);
      ImageView< PixelMask<double> > dem = create_mask
        (channel_cast<double>(DiskImageView<float>(opt.dem_file)), dem_nodata_val);
      
      GeoReference dem_geo;
      if (!read_georeference(dem_geo, opt.dem_file))
        vw_throw( ArgumentErr() << "Missing georef.\n");
      
      
      // If the DEM is too big, we need to skip points. About
      // 40,000 points should be good enough to determine 78 RPC
      // coefficients.
      double delta_col = std::max(1.0, dem.cols()/double(opt.num_samples));
      double delta_row = std::max(1.0, dem.rows()/double(opt.num_samples));
      tpc.report_progress(0);
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
          if (image_box.contains(cam_pix)) {
            all_llh.push_back(llh);
            all_pixels.push_back(cam_pix);
          }
          
        }
        tpc.report_incremental_progress(inc_amount);
      }
    }
    tpc.report_finished();

    // The pixel box
    BBox2 pixel_box;
    for (size_t i = 0; i < all_pixels.size(); i++) 
      pixel_box.grow(all_pixels[i]);

    // Find the range of lon-lat-heights
    BBox3 llh_box;
    for (size_t i = 0; i < all_llh.size(); i++) 
      llh_box.grow(all_llh[i]);

    // If cropping, adjust the pixels
    BBox2 crop_box;
    if (!opt.no_crop) {
      // Cast to int so that we can crop properly
      pixel_box.min() = floor(pixel_box.min());
      pixel_box.max() = ceil(pixel_box.max());
      pixel_box.crop(image_box);

      crop_box = pixel_box; // save it before we modify pixel_box

      // Shift all pixels by the crop corner, including the pixel box itself
      for (size_t i = 0; i < all_pixels.size(); i++) 
        all_pixels[i] -= pixel_box.min();

      // Need to first save the corner before subtracting it, otherwise get wrong result
      Vector2 shift = pixel_box.min(); 
      pixel_box -= shift;
    }

    // We need this line for other tools
    vw_out() << "crop_box "
             << crop_box.min().x() << ' ' << crop_box.min().y() << ' '
             << crop_box.max().x() << ' ' << crop_box.max().y() << std::endl;
    
    if (opt.save_tif) {
      
      ImageView< PixelMask<float> > output_img = input_img;
      if (!opt.no_crop) 
        output_img = crop(input_img, crop_box);
      
      std::string out_img_file = fs::path(opt.output_rpc).replace_extension("tif").string();
      vw_out() << "Writing: " << out_img_file << std::endl;

      GeoReference img_geo;
      bool has_img_geo = false;
      vw::cartography::block_write_gdal_image(out_img_file,
                                              apply_mask(output_img, opt.output_nodata_value),
					      has_img_geo, img_geo,
					      opt.has_output_nodata,
                                              opt.output_nodata_value,
                                              opt,
                                              TerminalProgressCallback("asp", "\t-->: "));
    }

    if (opt.skip_computing_rpc) 
      return 0;
    
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
		 opt.penalty_weight, output_prefix,
		 normalized_llh, normalized_pixels,  
		 llh_scale, llh_offset, pixel_scale, pixel_offset,
		 // Outputs
		 line_num, line_den, samp_num, samp_den);

    // TODO: Integrate this with aster2asp existing functionality!
    // Have a generic function for saving WV RPC files. 
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
  
    std::string gsd_str = vw::num_to_str(opt.gsd);
  
    vw_out() << "Writing: " << opt.output_rpc << std::endl;
    std::ofstream ofs(opt.output_rpc.c_str());
    ofs.precision(18);

    // Header
    ofs << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n";
    ofs << "<isd>\n";

    // Image params
    int image_cols = input_img.cols();
    int image_rows = input_img.rows();
    ofs << "   <IMD>\n";
    ofs << "        <NUMROWS>" << image_rows << "</NUMROWS>\n";
    ofs << "        <NUMCOLUMNS>" << image_cols << "</NUMCOLUMNS>\n";
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
    ofs << "        <BANDID>P</BANDID>\n";
    ofs << "        <SPECID>RPC00B</SPECID>\n";
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

    // Footer
    ofs << "</isd>\n";
    ofs.close();
    
    
  } ASP_STANDARD_CATCHES;

  return 0;
}

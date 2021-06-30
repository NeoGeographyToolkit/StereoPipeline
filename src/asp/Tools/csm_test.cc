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

// Compare the CSM and ISIS models by projecting into both
// from a given DEM, and also doing projections from the camera.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>

#include <vw/Core/Stopwatch.h>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

struct Options : vw::cartography::GdalWriteOptions {
  std::string dem_file, image_file, camera_file;
  int sample_rate; // use one out of these many pixels
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("dem",   po::value(&opt.dem_file),
     "Specify the dem to use to project points into the ISIS and CSM camera models.")
    ("sample-rate",   po::value(&opt.sample_rate)->default_value(1),
     "Use one out of these many pixels when sampling the DEM.");
    
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("image",   po::value(&opt.image_file),   "Satellite image file")
    ("camera",  po::value(&opt.camera_file),  "Camera file");
  
  po::positional_options_description positional_desc;
  positional_desc.add("image",   1);
  positional_desc.add("camera",  1);
  
  std::string usage("--dem dem.tif input.cub input.json [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.dem_file == "" || opt.image_file == "" || opt.camera_file == "")
    vw_throw( ArgumentErr() << "Not all inputs were specified.\n" << usage << general_options );

  if (opt.sample_rate <= 0)
    vw_throw( ArgumentErr() << "The sample rate must be positive.\n" << usage << general_options );
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    std::cout << "Image is " << opt.image_file << std::endl;
    std::cout << "Camera is " << opt.camera_file << std::endl;
    
    // Read the DEM and its associated data
    // TODO(oalexan1): Think more about the interpolation method
    std::cout << "Reading the DEM: " << opt.dem_file << std::endl;
    vw::cartography::GeoReference dem_georef;
    if (!read_georeference(dem_georef, opt.dem_file))
      vw_throw( ArgumentErr() << "The input DEM has no georeference.\n" );
    double dem_nodata_val = -std::numeric_limits<float>::max(); // note we use a float nodata
    if (!vw::read_nodata_val(opt.dem_file, dem_nodata_val))
      vw_throw( ArgumentErr() << "Could not read the DEM nodata value.\n");
    std::cout << "Read DEM nodata value: " << dem_nodata_val << std::endl;
    DiskImageView<float> dem(opt.dem_file);
    std::cout << "The DEM width and height are: " << dem.cols() << ' ' << dem.rows() << std::endl;
    ImageViewRef< PixelMask<float> > masked_dem = create_mask(dem, dem_nodata_val);
    
    // Load the isis camera
    std::string out_prefix;
    std::string isis_session_name; 
    SessionPtr isis_session(asp::StereoSessionFactory::create(isis_session_name, // will change
                                                              opt,
                                                              opt.image_file, opt.image_file,
                                                              opt.image_file, opt.image_file,
                                                              out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> isis_camera_model
      = isis_session->camera_model(opt.image_file, opt.image_file);
    std::cout << "Got isis session: " << isis_session_name << std::endl;
    
    // Load the csm camera
    std::string csm_session_name; 
    SessionPtr csm_session(asp::StereoSessionFactory::create(csm_session_name, // will change
                                                              opt,
                                                              opt.image_file, opt.image_file,
                                                              opt.camera_file, opt.camera_file,
                                                              out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> csm_camera_model
      = csm_session->camera_model(opt.image_file, opt.camera_file);
    std::cout << "Got csm session: " << csm_session_name << std::endl;

    // The input image
    DiskImageView<float> input_image(opt.image_file);
    std::cout << "Image dimensions is "
              << input_image.cols() << ' ' << input_image.rows() << std::endl;

    // Iterate over the DEM
    for (int col = 0; col < dem.cols(); col += opt.sample_rate) {
      for (int row = 0; row < dem.rows(); row += opt.sample_rate) {
        
        if (!is_valid(masked_dem(col, row)))
          continue;

        Vector2 pix(col, row);
        std::cout << "pix is " << pix << std::endl;
        
        double height = masked_dem(col, row).child();

        Vector3 llh;
        subvector(llh, 0, 2) = dem_georef.pixel_to_lonlat(pix); // lon and lat
        llh[2] = height;

        Vector3 xyz = dem_georef.datum().geodetic_to_cartesian(llh);
        Vector2 isis_pix = isis_camera_model->point_to_pixel(xyz);
        Vector2 csm_pix = csm_camera_model->point_to_pixel(xyz);

        std::cout << "llh is " << llh << std::endl;
        std::cout << "xyz is " << xyz << std::endl;
        std::cout << "isis and csm pix and diff " << isis_pix << ' ' << csm_pix << ' '
                  << isis_pix - csm_pix << ' ' << norm_2(isis_pix - csm_pix) << std::endl;


        Vector3 isis_ctr = isis_camera_model->camera_center(csm_pix);
        Vector3 isis_dir = isis_camera_model->pixel_to_vector(csm_pix);
        
        Vector3 csm_ctr = csm_camera_model->camera_center(csm_pix);
        Vector3 csm_dir = csm_camera_model->pixel_to_vector(csm_pix);

        std::cout << "isis and csm dir " << isis_dir << ' ' << csm_dir << ' '
                  << isis_dir - csm_dir << ' ' << norm_2(isis_dir - csm_dir) << std::endl;
        
        std::cout << "isis and csm ctr " << isis_ctr << ' ' << csm_ctr << ' '
                  << isis_ctr - csm_ctr << ' ' << norm_2(isis_ctr - csm_ctr) << std::endl;
      }
    }
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

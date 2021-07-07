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

// Compare the CSM and ISIS models by finding for each of them the
// camera center and ray direction at a set of sampled pixels, then by
// projecting pixels to the ground using the ISIS camera and
// back-projecting the resulting points into the CSM camera, then
// doing this in reverse.

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
  std::string image_file, camera_file;
  int sample_rate; // use one out of these many pixels
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General Options");
  general_options.add_options()
    ("sample-rate",   po::value(&opt.sample_rate)->default_value(100),
     "Use one out of these many pixels when sampling the image.");
    
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );
  
  po::options_description positional("");
  positional.add_options()
    ("image",   po::value(&opt.image_file),   "Satellite image file")
    ("camera",  po::value(&opt.camera_file),  "Camera file");
  
  po::positional_options_description positional_desc;
  positional_desc.add("image",   1);
  positional_desc.add("camera",  1);
  
  std::string usage("input.cub input.json [other options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.image_file == "" || opt.camera_file == "")
    vw_throw( ArgumentErr() << "Not all inputs were specified.\n" << usage << general_options );

  if (opt.sample_rate <= 0)
    vw_throw( ArgumentErr() << "The sample rate must be positive.\n" << usage << general_options );
}

// Sort the diffs and print some stats
void print_diffs(std::string const& tag, std::vector<double> & diffs) {
  std::sort(diffs.begin(), diffs.end());

  vw_out() << "\n";
  
  if (diffs.empty()) 
    vw_out() << "Empty list of diffs for: " << tag << "\n";

  vw_out() << tag << "\n";
  vw_out() << "Min:    " << diffs[0] << "\n";
  vw_out() << "Median: " << diffs[diffs.size()/2] << "\n";
  vw_out() << "Max:    " << diffs.back() << "\n";
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

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

    bool use_sphere_for_datum = false;
    vw::cartography::Datum datum = isis_session->get_datum(isis_camera_model.get(),
                                                           use_sphere_for_datum);
    std::cout << "Datum: " << datum << std::endl;
    
    // Load the csm camera
    std::string csm_session_name; 
    SessionPtr csm_session(asp::StereoSessionFactory::create(csm_session_name, // will change
                                                              opt,
                                                              opt.image_file, opt.image_file,
                                                              opt.camera_file, opt.camera_file,
                                                              out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> csm_camera_model
      = csm_session->camera_model(opt.image_file, opt.camera_file);

    // The input image
    DiskImageView<float> image(opt.image_file);
    std::cout << "Image dimensions: " << image.cols() << ' ' << image.rows() << std::endl;

    // Iterate over the image
    std::vector<double> ctr_diff, dir_diff, isis2csm_diff, csm2isis_diff;
    for (int col = 0; col < image.cols(); col += opt.sample_rate) {
      for (int row = 0; row < image.rows(); row += opt.sample_rate) {

        Vector2 image_pix(col, row);

        Vector3 isis_ctr = isis_camera_model->camera_center(image_pix);
        Vector3 isis_dir = isis_camera_model->pixel_to_vector(image_pix);
        
        Vector3 csm_ctr = csm_camera_model->camera_center(image_pix);
        Vector3 csm_dir = csm_camera_model->pixel_to_vector(image_pix);

        ctr_diff.push_back(norm_2(isis_ctr - csm_ctr));
        dir_diff.push_back(norm_2(isis_dir - csm_dir));
        
        // Shoot a ray from the ISIS camera, intersect it with the datum,
        // and project it back into the CSM camera.
        Vector3 xyz = vw::cartography::datum_intersection(datum, isis_ctr, isis_dir);
        Vector2 csm_pix = csm_camera_model->point_to_pixel(xyz);
        isis2csm_diff.push_back(norm_2(image_pix - csm_pix));
        
        // Shoot a ray from the CSM camera, intersect it with the datum,
        // and project it back into the ISIS camera.
        xyz = vw::cartography::datum_intersection(datum, csm_ctr, csm_dir);
        Vector2 isis_pix = isis_camera_model->point_to_pixel(xyz);
        csm2isis_diff.push_back(norm_2(image_pix - isis_pix));
      }
    }

    vw_out() << "Number of samples used: " << ctr_diff.size() << "\n";

    print_diffs("ISIS vs CSM camera direction diff", dir_diff);
    print_diffs("ISIS vs CSM camera center diff (meters)", ctr_diff);
    print_diffs("ISIS to CSM pixel diff", isis2csm_diff);
    print_diffs("CSM to ISIS pixel diff", csm2isis_diff);
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

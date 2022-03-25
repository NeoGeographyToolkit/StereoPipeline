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

// Tool to compare two camera models for the same image. For example,
// compare ISIS to CSM, linescan to cam2 (for DG or PeruSat), Optical
// bar vs pinhole (with the latter created with
// convert_pinhole_model).

// For each camera model find the camera center and ray direction at a
// set of sampled pixels, then by projecting pixels to the ground
// using the cam1 camera and back-projecting the resulting points into
// the cam2 camera, then doing this in reverse.

#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <asp/Camera/RPCModel.h>
#include <asp/Camera/LinescanPeruSatModel.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Camera/OpticalBarModel.h>
#include <asp/Camera/CsmModel.h>
#include <asp/IsisIO/IsisCameraModel.h>

using namespace vw;
using namespace vw::cartography;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

typedef boost::scoped_ptr<asp::StereoSession> SessionPtr;

struct Options : vw::cartography::GdalWriteOptions {
  std::string image_file, cam1_file, cam2_file, session1, session2;
  int sample_rate; // use one out of these many pixels
  double subpixel_offset;
  bool enable_correct_velocity_aberration, enable_correct_atmospheric_refraction;
  Options() {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {

  po::options_description general_options("General options");
  general_options.add_options()
    ("image", po::value(&opt.image_file),  "Image file.")
    ("cam1",  po::value(&opt.cam1_file),   "Camera 1 file.")
    ("cam2",  po::value(&opt.cam2_file),   "Camera 2 file.")
    ("session1", po::value(&opt.session1),
     "Session to use for camera 1 (if not provided it will be guessed).")
    ("session2", po::value(&opt.session2),
     "Session to use for camera 2 (if not provided it will be guessed).")
    ("sample-rate",   po::value(&opt.sample_rate)->default_value(100),
     "Use one out of these many pixels when sampling the image.")
    ("subpixel-offset",   po::value(&opt.subpixel_offset)->default_value(0.0),
     "Add to each integer pixel this offset (in x and y) when sampling the image.")
    ("enable-correct-velocity-aberration", po::bool_switch(&opt.enable_correct_velocity_aberration)->default_value(false)->implicit_value(true),
     "Turn on velocity aberration correction for Optical Bar and non-ISIS linescan cameras. This option impairs the convergence of bundle adjustment.")
    ("enable-correct-atmospheric-refraction", po::bool_switch(&opt.enable_correct_atmospheric_refraction)->default_value(false)->implicit_value(true),
     "Turn on atmospheric refraction correction for Optical Bar and non-ISIS linescan cameras. This option impairs the convergence of bundle adjustment.");
  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  po::positional_options_description positional_desc;
  
  std::string usage("--image <image file> --cam1 <camera 1 file> --cam2 <camera 2 file> "
                    "[other options]");
  
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  if (opt.image_file == "" || opt.cam1_file == "" || opt.cam2_file == "")
    vw_throw(ArgumentErr() << "Not all inputs were specified.\n" << usage << general_options);

  if (opt.sample_rate <= 0)
    vw_throw(ArgumentErr() << "The sample rate must be positive.\n" << usage << general_options);

  asp::stereo_settings().enable_correct_velocity_aberration
    = opt.enable_correct_velocity_aberration;
  asp::stereo_settings().enable_correct_atmospheric_refraction
    = opt.enable_correct_atmospheric_refraction;
}

// Sort the diffs and print some stats
void print_diffs(std::string const& tag, std::vector<double> & diffs) {
  std::sort(diffs.begin(), diffs.end());

  vw_out() << "\n";
  
  if (diffs.empty()) {
    vw_out() << "Empty list of diffs for: " << tag << "\n";
    return;
  }
  
  vw_out() << tag << "\n";
  vw_out() << "Min:    " << diffs[0] << "\n";
  vw_out() << "Median: " << diffs[diffs.size()/2] << "\n";
  vw_out() << "Max:    " << diffs.back() << "\n";
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    // Load cam1
    std::string out_prefix;
    std::string default_session1 = opt.session1; // save it before it changes
    SessionPtr cam1_session(asp::StereoSessionFactory::create
                               (opt.session1, // may change
                                opt,
                                opt.image_file, opt.image_file,
                                opt.cam1_file, opt.cam1_file,
                                out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> cam1_model
      = cam1_session->camera_model(opt.image_file, opt.cam1_file);

    // Auto-guess the datum
    bool use_sphere_for_datum = false;
    vw::cartography::Datum datum = cam1_session->get_datum(cam1_model.get(),
                                                           use_sphere_for_datum);
    std::cout << "Datum: " << datum << std::endl;
    
    // Load cam2
    std::string default_session2 = opt.session2; // save it before it changes
    SessionPtr cam2_session(asp::StereoSessionFactory::create
                           (opt.session2, // may change
                            opt,
                            opt.image_file, opt.image_file,
                            opt.cam2_file, opt.cam2_file,
                            out_prefix));
    boost::shared_ptr<vw::camera::CameraModel> cam2_model
      = cam2_session->camera_model(opt.image_file, opt.cam2_file);

    if (opt.session1 == opt.session2 && (default_session1 == "" || default_session2 == "")) 
      vw_throw(ArgumentErr() << "The session names for both cameras "
               << "were guessed as: '" << opt.session1 << "'. It is suggested that they be "
               << "explicitly specified using --session1 and --session2.\n");
    
    // Find the input image dimensions
    int image_cols = 0, image_rows = 0;
    try {
      DiskImageView<float> image(opt.image_file);
      image_cols = image.cols();
      image_rows = image.rows();
    } catch(const std::exception& e) {
      // For CSM-to-CSM ground-to-image and image-to-ground comparisons only,
      // the camera has the dimensions if the .cub image is missing.
      asp::CsmModel * csm_model
        = dynamic_cast<asp::CsmModel*>(vw::camera::unadjusted_model(cam1_model.get()));
      if (csm_model != NULL) {
        image_cols = csm_model->get_image_size()[0];
        image_rows = csm_model->get_image_size()[1];
      } else {
        vw::vw_throw(ArgumentErr() << e.what());
      }
    }
    
    std::cout << "Image dimensions: " << image_cols << ' ' << image_rows << std::endl;

    // Iterate over the image
    std::vector<double> ctr_diff, dir_diff, cam1_to_cam2_diff, cam2_to_cam1_diff;
    for (int col = 0; col < image_cols; col += opt.sample_rate) {
      for (int row = 0; row < image_rows; row += opt.sample_rate) {

        Vector2 image_pix(col + opt.subpixel_offset, row + opt.subpixel_offset);
        Vector3 cam1_ctr = cam1_model->camera_center(image_pix);
        Vector3 cam2_ctr = cam2_model->camera_center(image_pix);
        ctr_diff.push_back(norm_2(cam1_ctr - cam2_ctr));
        Vector3 cam1_dir = cam1_model->pixel_to_vector(image_pix);
        Vector3 cam2_dir = cam2_model->pixel_to_vector(image_pix);
        dir_diff.push_back(norm_2(cam1_dir - cam2_dir));
        
        // Shoot a ray from the cam1 camera, intersect it with the datum,
        // and project it back into the cam2 camera.
        Vector3 xyz = vw::cartography::datum_intersection(datum, cam1_ctr, cam1_dir);
        Vector2 cam2_pix = cam2_model->point_to_pixel(xyz);
        cam1_to_cam2_diff.push_back(norm_2(image_pix - cam2_pix));

        // Shoot a ray from the cam2 camera, intersect it with the datum,
        // and project it back into the cam1 camera.
        xyz = vw::cartography::datum_intersection(datum, cam2_ctr, cam2_dir);
        Vector2 cam1_pix = cam1_model->point_to_pixel(xyz);
        cam2_to_cam1_diff.push_back(norm_2(image_pix - cam1_pix));
      }
    }

    vw_out() << "Number of samples used: " << ctr_diff.size() << "\n";
    
    print_diffs("cam1 to cam2 camera direction diff norm", dir_diff);
    print_diffs("cam1 to cam2 camera center diff (meters)", ctr_diff);
    print_diffs("cam1 to cam2 pixel diff", cam1_to_cam2_diff);
    print_diffs("cam2 to cam1 pixel diff", cam2_to_cam1_diff);
    
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

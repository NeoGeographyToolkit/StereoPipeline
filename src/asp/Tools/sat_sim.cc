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

// Tool to create simulated satellite images and/or pinhole cameras for them.
// See the manual for details.

#include <asp/Core/Macros.h>
#include <asp/Core/AspProgramOptions.h>
#include <asp/Core/AspLog.h>
#include <asp/Camera/SyntheticLinescan.h>
#include <asp/Camera/SatSim.h>
#include <asp/Rig/rig_config.h>

#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Camera/PinholeModel.h>
#include <vw/Core/StringUtils.h>
#include <vw/Core/Stopwatch.h>
#include <vw/FileIO/FileUtils.h>

#include <glog/logging.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;
void handle_arguments(int argc, char *argv[], asp::SatSimOptions& opt,
                      rig::RigSet & rig) {

  double NaN = std::numeric_limits<double>::quiet_NaN();
  po::options_description general_options("General options");
  general_options.add_options()
    ("dem", po::value(&opt.dem_file)->default_value(""), "Input DEM file.")
    ("ortho", po::value(&opt.ortho_file)->default_value(""), 
     "Input georeferenced image file.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix. All the "
    "files that are saved will start with this prefix.")
    ("camera-list", po::value(&opt.camera_list)->default_value(""),
     "A file containing the list of pinhole cameras to create synthetic images for. "
     "Then these cameras will be used instead of generating them. Specify one file "
     "per line. The options --first, --last, --num, --focal-length, "
     "and --optical-center will be ignored.")
    ("first", po::value(&opt.first)->default_value(vw::Vector3(), ""),
    "First camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum. See the doc for more info.")
    ("last", po::value(&opt.last)->default_value(vw::Vector3(), ""),
    "Last camera position, specified as DEM pixel column and row, and height above "
    "the DEM datum.")
    ("num", po::value(&opt.num_cameras)->default_value(0),
    "Number of cameras to generate, including the first and last ones. Must be positive. "
    "The cameras are uniformly distributed along the straight edge from first to last (in "
    "projected coordinates).")
    ("first-ground-pos", po::value(&opt.first_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of first camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("last-ground-pos", po::value(&opt.last_ground_pos)->default_value(vw::Vector2(NaN, NaN), ""),
    "Coordinates of last camera ground footprint center (DEM column and row). "
    "If not set, the cameras will look straight down (perpendicular to along "
    "and across track directions).")
    ("focal-length", po::value(&opt.focal_length)->default_value(NaN),
     "Output camera focal length in units of pixel.")
    ("optical-center", po::value(&opt.optical_center)->default_value(vw::Vector2(NaN, NaN),"NaN NaN"),
     "Output camera optical center (image column and row). Units of pixel.")
    ("image-size", po::value(&opt.image_size)->default_value(vw::Vector2(NaN, NaN),
      "NaN NaN"),
      "Output camera image size (width and height).")
    ("roll", po::value(&opt.roll)->default_value(NaN),
    "Camera roll angle, in degrees. See the documentation for more details.")
    ("pitch", po::value(&opt.pitch)->default_value(NaN),
     "Camera pitch angle, in degrees.")
    ("yaw", po::value(&opt.yaw)->default_value(NaN),
     "Camera yaw angle, in degrees.")
    ("velocity", po::value(&opt.velocity)->default_value(NaN),
     "Satellite velocity, in meters per second. Used for modeling jitter. A value of "
     "around 8000 m/s is typical for a satellite like SkySat in Sun-synchronous orbit "
     "(90 minute period) at an altitude of about 450 km. For WorldView, the velocity "
     "is around 7500 m/s, with a higher altitude and longer period.")
    ("jitter-frequency", po::value(&opt.jitter_frequency_str)->default_value(""),
     "Jitter frequency, in Hz. Used for modeling jitter (satellite vibration). "
     "Several frequencies can be specified. Use a quoted list, with spaces "
     "as separators (or separated by commas with no quotes). See also "
     "--jitter-amplitude and --horizontal-uncertainty.")
    ("jitter-phase", po::value(&opt.jitter_phase_str)->default_value(""),
     "Jitter phase, in radians. Measures the jitter phase offset from the start of "
     "the orbit as set by --first. Specify as a quoted list of numbers (or "
     "separated by commas with no quotes). The number "
     "of values must be 3 times the number of frequencies. The order in this list "
     "corresponds to phase for roll, pitch, and yaw for first frequency, then "
     "second frequency, etc. If not specified, will be set to 0.")
    ("horizontal-uncertainty", po::value(&opt.horizontal_uncertainty_str)->default_value(""),
     "Camera horizontal uncertainty on the ground, in meters, at nadir orientation. "
     "Specify as a quoted list of three numbers (or separated by commas with no quotes), "
     "used for roll, pitch, and yaw. The jitter amplitude for each of these angles "
     "is found as "
     "= atan(horizontal_uncertainty / satellite_elevation_above_datum), then converted "
     "to degrees. See also --jitter-amplitude.")
    ("jitter-amplitude", po::value(&opt.jitter_amplitude_str)->default_value(""),
     "Jitter amplitude, in micro radians. Specify as a quoted list having "
     "amplitude in roll, pitch, yaw for first frequency, then for second, and so on. "
     "Separate the values by spaces (or commas with no quotes).")
    ("first-index", po::value(&opt.first_index)->default_value(-1),
     "Index of first camera and/or image to generate, starting from 0. If not set, will "
     "create all images/cameras. This is used for parallelization.")
    ("last-index", po::value(&opt.last_index)->default_value(-1),
     "Index of last image and/or camera to generate, starting from 0. Stop before "
     "this index. If not set, will create all images/cameras. This is used for "
     "parallelization.")
    ("frame-rate", po::value(&opt.frame_rate)->default_value(NaN), 
     "Camera frame rate, per second. Can be in double precision. If set, it will override "
     "--num. The cameras will start from --first (after any position adjustment, if "
     "applicable, per the doc). Set the --velocity value. The last camera will be no further "
     "than the (adjusted) value of --last along the orbit.")
     ("sensor-type", po::value(&opt.sensor_type)->default_value("pinhole"),
      "Sensor type for created cameras and images. Can be one of: pinhole, linescan. "
      "Can use 'frame' instead of 'pinhole'. With a rig, this can be a list of values, "
      "separated by commas, with no spaces, one per sensor, if desired to have different "
      "types for different sensors.")
    ("non-square-pixels", po::bool_switch(&opt.non_square_pixels)->default_value(false)->implicit_value(true),
      "When creating linescan cameras and images, use the provided image height in pixels, "
      "even if that results in non-square pixels. The default is to auto-compute the image "
      "height.")
    ("no-images", po::bool_switch(&opt.no_images)->default_value(false)->implicit_value(true),
     "Create only cameras, and no images. Cannot be used with --camera-list.")
     ("save-ref-cams", po::bool_switch(&opt.save_ref_cams)->default_value(false)->implicit_value(true),
     "For each created camera, save also the 'reference' camera that has no roll, pitch, "
     "yaw, jitter, or 90 degree in-sensor-plane rotation from camera to satellite " 
     "coordinates. Their names have '-ref-' after the output prefix.")
    ("save-as-csm", 
      po::bool_switch(&opt.save_as_csm)->default_value(false)->implicit_value(true),
      "Save Pinhole (frame) cameras in the CSM format, as done for linescan cameras. "
      "Can be used to combine these sensors in bundle adjustment and solving for jitter.")
     ("rig-config", po::value(&opt.rig_config)->default_value(""),
     "Simulate a frame camera rig with this configuration file. Then do not set the image "
     "size, focal length, optical center on the command line, as those are set by the rig. "
     "The transforms on this rig may be adjusted via --rig-sensor-ground-offsets "
     "and --rig-sensor-rotation-angles.")
     ("rig-sensor-ground-offsets",
      po::value(&opt.rig_sensor_ground_offsets)->default_value(""),
     "Modify the input rig so that each sensor center has the given horizontal offsets "
     "from the rig center in the rig plane, and the sensor ground footprint centers have "
     "the given ground plane offsets from the nominal ground footprint center at nadir. "
     "Specify as a quoted list of values, separated by spaces or commas. The order is "
     "sensor1_x sensor1_y ground1_x ground1_y followed by sensor 2, etc. The units are in "
     "meter. These will determine the sensor orientations. If not set, use 0 for all "
     "sensors.")
     ("rig-sensor-rotation-angles",
      po::value(&opt.rig_sensor_rotation_angles)->default_value(""),
      "Modify the input rig by rotating each sensor by the given angle in the sensor "
      "plane. Specify as one number per sensor, in degrees, separated by commas, or "
      "in quotes and separated by spaces.")
     ("sensor-name", po::value(&opt.sensor_name)->default_value("all"),
       "Name of the sensor in the rig to simulate. If not set, will simulate all sensors. "
       "If more than one, list them separated by commas (no spaces).")
     ("model-time", 
      po::bool_switch(&opt.model_time)->default_value(false)->implicit_value(true),
       "Model time at each camera position. See also --start-time.")
     ("reference-time", po::value(&opt.ref_time)->default_value(10000),
       "The measured time, in seconds,  when the satellite is along given orbit, in nadir "
       "orientation, with the center view direction closest to the ground point at "
       "--first-ground-pos. A unique value for each orbit is suggested. A large value "
       "(millions), may result in numerical issues.")
     ("perturb-cameras",
      po::bool_switch(&opt.perturb_cameras)->default_value(false)->implicit_value(true),
      "Apply a jitter perturbation to existing cameras.")
     ("random-pose-perturbation",
       po::bool_switch(&opt.random_pose_perturbation)->default_value(false)->implicit_value(true),
       "Apply a random pose perturbation to existing cameras, with the amplitude " 
       "specified by --horizontal-uncertainty.")
     ("random-position-perturbation", po::value(&opt.random_position_perturbation)->default_value(NaN),
      "Apply a random position perturbation to existing camera centers, with the given "
      "magnitude, in meters.")
     ("dem-height-error-tol", po::value(&opt.dem_height_error_tol)->default_value(0.001),
      "When intersecting a ray with a DEM, use this as the height error tolerance "
      "(measured in meters). It is expected that the default will be always good enough.")
     ("blur-sigma", po::value(&opt.blur_sigma)->default_value(0.0),
      "When creating images, blur them with a Gaussian with this sigma. The sigma is "
      "in input orthoimage pixel units.")
     ("help,h", "Display this help message.")
    ;
  general_options.add(vw::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  po::positional_options_description positional_desc;

  std::string usage("--dem <dem file> --ortho <ortho image file> "
                    "[other options]");

  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                            positional, positional_desc, usage,
                            allow_unregistered, unregistered);

  // When invoked with no arguments or no output prefix, display the help message.
  if (opt.out_prefix == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing output prefix.\n"
                 << usage << general_options);

  // Create the output directory based on the output prefix
  vw::create_out_dir(opt.out_prefix);

  // Turn on logging to file
  asp::log_to_file(argc, argv, "", opt.out_prefix);

  // Some functions use google logging
  google::InitGoogleLogging(argv[0]);

  bool have_perturb = opt.perturb_cameras || opt.random_pose_perturbation ||
                      !std::isnan(opt.random_position_perturbation);
  
  // Only one of the three options above must be set
  if (int(opt.perturb_cameras) + int(opt.random_pose_perturbation) +
      int(!std::isnan(opt.random_position_perturbation)) > 1)
    vw::vw_throw(vw::ArgumentErr() << "Only one of --perturb-cameras, "
                 "--random-pose-perturbation, and --random-position-perturbation "
                 "can be set.\n");
    
  if (opt.dem_file == "")
    vw::vw_throw(vw::ArgumentErr() << "Missing input DEM.\n");
    
  if (opt.ortho_file == "" && !have_perturb)
    vw::vw_throw(vw::ArgumentErr() << "Missing input ortho image.\n");
  
  bool have_rig = !opt.rig_config.empty();
  if (have_rig && opt.camera_list != "")
    vw::vw_throw(vw::ArgumentErr() 
                 << "Cannot specify both --rig-config and --camera-list.\n");
  
  if (have_rig && 
      (!vm["image-size"].defaulted() || !vm["focal-length"].defaulted() ||
       !vm["optical-center"].defaulted()))
      vw::vw_throw(vw::ArgumentErr() << "Cannot specify image size, focal length, or "
        "optical center when using a rig. Those are set in the rig configuration file.\n");

  if (have_rig && !opt.model_time) {
    opt.model_time = true;
    vw::vw_out() << "Will model time as a rig was specified.\n";
  }

  if (!have_rig && (!have_perturb || opt.save_as_csm)) {
    if (std::isnan(opt.image_size[0]) || std::isnan(opt.image_size[1]))
      vw::vw_throw(vw::ArgumentErr() << "The image size must be specified.\n");
    if (opt.image_size[0] <= 1 || opt.image_size[1] <= 1)
      vw::vw_throw(vw::ArgumentErr() << "The image size must be at least 2 x 2.\n");
  }
  
  if (opt.camera_list != "" && opt.no_images && !have_perturb)
    vw::vw_throw(vw::ArgumentErr() << "The --camera-list and --no-images options "
      "cannot be used together.\n");
  
  if (opt.camera_list == "") {
    if (opt.first == vw::Vector3() || opt.last == vw::Vector3())
      vw::vw_throw(vw::ArgumentErr() << "The first and last camera positions must be "
        "specified.\n");

    if (opt.first[2] != opt.last[2])
      vw::vw_out() << "Warning: The first and last camera positions have different "
                   << "heights above the datum. This is supported but is not usual. "
                   << "Check your inputs.\n";
    
    if (std::isnan(opt.frame_rate)) {
      if (opt.num_cameras < 1)
        vw::vw_throw(vw::ArgumentErr() << "The number of cameras must be at least 1.\n");
    } else {
      // Frame rate is set. Then need not set num cameras.
      if (opt.num_cameras > 0)
        vw::vw_throw(vw::ArgumentErr() << "Cannot set both --num and --frame-rate.\n");
      // Must have a positive velocity
      if (std::isnan(opt.velocity) || opt.velocity <= 0.0)
        vw::vw_throw(vw::ArgumentErr() << "The velocity must be positive if using "
         << " --frame-rate.\n");
    }

    // Validate focal length, optical center, and image size
    if (!have_rig) {
      if (std::isnan(opt.focal_length))
        vw::vw_throw(vw::ArgumentErr() << "The focal length must be positive.\n");
      if (std::isnan(opt.optical_center[0]) || std::isnan(opt.optical_center[1]))
        vw::vw_throw(vw::ArgumentErr() << "The optical center must be specified.\n");
    
      // If the optical center is large, the images will show up very oblique.
      // The current logic implicitly assumes that the optical center is close to
      // the image center.
      if (opt.optical_center[0] < 0 || opt.optical_center[1] < 0 ||
          opt.optical_center[0] >= opt.image_size[0] || 
          opt.optical_center[1] >= opt.image_size[1])
        vw::vw_throw(vw::ArgumentErr() << "The optical center must be non-negative and "
                    << "within the image bounds. It is suggested to have it close to "
                    << "the image center.\n");
    }
      
    // Either both first and last ground positions are specified, or none.
    if (std::isnan(norm_2(opt.first_ground_pos)) != 
      std::isnan(norm_2(opt.last_ground_pos)))
      vw::vw_throw(vw::ArgumentErr() << "Either both first and last ground positions "
        "must be specified, or none.\n");

    // Check that either all of roll, pitch, and yaw are specified, or none.
    int ans = int(std::isnan(opt.roll)) +
              int(std::isnan(opt.pitch)) +
              int(std::isnan(opt.yaw));
    if (ans != 0 && ans != 3)
      vw::vw_throw(vw::ArgumentErr() << "Either all of roll, pitch, and yaw must be "
        "specified, or none.\n");
  }

  // Parse jitter frequency
  // Convert from string to vector of doubles
  std::string sep = ", \t\n"; // separators: comma, space, tab, newline
  opt.jitter_frequency = vw::str_to_std_vec(opt.jitter_frequency_str, sep);
  if (opt.jitter_frequency.empty())
    opt.jitter_frequency.push_back(NaN);

  // Horizontal uncertainty must be 3 values. Must specify either this or 
  // jitter amplitude.
  opt.horizontal_uncertainty 
    = vw::str_to_std_vec(opt.horizontal_uncertainty_str, sep);
  if (!opt.horizontal_uncertainty.empty() && opt.horizontal_uncertainty.size() != 3)
    vw::vw_throw(vw::ArgumentErr() << "The horizontal uncertainty must be specified "
      "as three values, separated by commas or spaces.\n");

  // Number of jitter amplitudes must be 3x the number of frequencies. It can 
  // be empty, if horizontal uncertainty is specified.
  opt.jitter_amplitude = vw::str_to_std_vec(opt.jitter_amplitude_str, sep);
  if (opt.jitter_amplitude.empty() && opt.horizontal_uncertainty.empty()) {
    // Default amplitude is 0
    for (size_t i = 0; i < opt.jitter_frequency.size() * 3; i++)
      opt.jitter_amplitude.push_back(0.0);
  }

  // Number of jitter phases must be 3x the number of frequencies.
  opt.jitter_phase = vw::str_to_std_vec(opt.jitter_phase_str, sep);
  if (opt.jitter_phase.empty()) {
    // Default phase is 0
    for (size_t i = 0; i < opt.jitter_frequency.size() * 3; i++)
      opt.jitter_phase.push_back(0.0);
  }

  // Sanity checks
  if (!opt.horizontal_uncertainty_str.empty() && !opt.jitter_amplitude_str.empty()) 
    vw::vw_throw(vw::ArgumentErr() 
      << "Cannot specify both jitter uncertainty and jitter amplitude.\n");
  if (!opt.horizontal_uncertainty.empty() && !opt.jitter_amplitude.empty()) 
    vw::vw_throw(vw::ArgumentErr() 
      << "Cannot specify both jitter uncertainty and jitter amplitude.\n");

  bool have_pose_perturb = opt.random_pose_perturbation || opt.perturb_cameras;
  bool model_jitter = (!std::isnan(opt.jitter_frequency[0]) || have_pose_perturb);
  if (model_jitter) {

    bool have_roll_pitch_yaw = !std::isnan(opt.roll) && !std::isnan(opt.pitch) &&
      !std::isnan(opt.yaw);
    if (!have_roll_pitch_yaw && !have_pose_perturb)
      vw::vw_throw(vw::ArgumentErr() << "Modelling jitter requires specifying --roll, --pitch, and --yaw.\n");
    
    if (opt.camera_list != "" && !have_pose_perturb)
      vw::vw_throw(vw::ArgumentErr() << "The --camera-list option must not be set "
        << "when modeling jitter.\n");

    // See if the user specified either horizontal uncertainty or jitter amplitude
    if (opt.horizontal_uncertainty_str.empty() && opt.jitter_amplitude_str.empty() &&
        std::isnan(opt.random_position_perturbation))
      vw::vw_throw(vw::ArgumentErr() << "Must specify either horizontal uncertainty "
        << "or jitter amplitude.\n");

    if (3 * opt.jitter_frequency.size() != opt.jitter_phase.size())
      vw::vw_throw(vw::ArgumentErr() << "The number of jitter phases must be "
        << "three times the number of jitter frequencies.\n");

    if (opt.horizontal_uncertainty.empty()) {
      // Jitter amplitude was specified
      if (3 * opt.jitter_frequency.size() != opt.jitter_amplitude.size())
        vw::vw_throw(vw::ArgumentErr() << "The number of jitter amplitudes must be "
          << "three times the number of jitter frequencies.\n");
    } else {
      // Horizontal uncertainty was specified.
      if (opt.horizontal_uncertainty.size() != 3)
        vw::vw_throw(vw::ArgumentErr() << "The number of horizontal uncertainty values "
          << "must be 3.\n");
      
      if (opt.horizontal_uncertainty[0] < 0 || opt.horizontal_uncertainty[1] < 0 ||
          opt.horizontal_uncertainty[2] < 0)
        vw::vw_throw(vw::ArgumentErr() 
                     << "The horizontal uncertainty must be non-negative.\n");
    }

    // Check that all jitter frequencies are not NaN and positive
    for (size_t i = 0; i < opt.jitter_frequency.size(); i++) {
      if (std::isnan(opt.jitter_frequency[i]) && !opt.random_pose_perturbation &&
          std::isnan(opt.random_position_perturbation))
        vw::vw_throw(vw::ArgumentErr() << "The jitter frequency must be specified.\n");
      if (opt.jitter_frequency[i] <= 0)
        vw::vw_throw(vw::ArgumentErr() << "The jitter frequency must be positive.\n");
    }
    
  } // end if model jitter

  if (opt.velocity <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The satellite velocity must be positive.\n");

  // Checks for linescan cameras
  bool have_linescan = (opt.sensor_type.find("linescan") != std::string::npos);
  if (have_linescan && std::isnan(opt.velocity))
    vw::vw_throw(vw::ArgumentErr() << "The satellite velocity must be specified "
      << "in order to create linescan cameras.\n");
  if (opt.non_square_pixels && !have_linescan)
    vw::vw_throw(vw::ArgumentErr() << "Cannot specify --non-square-pixels unless creating "
      << "linescan cameras.\n");

  // Sanity check the first and last indices
  int ans = int(opt.first_index < 0) + int(opt.last_index < 0);
  if (ans != 0 && ans != 2)
    vw::vw_throw(vw::ArgumentErr() << "Either both first and last indices must be "
      "specified, or none.\n");
  if (ans == 0 && opt.first_index >= opt.last_index)
    vw::vw_throw(vw::ArgumentErr() << "The first index must be less than "
      "the last index.\n");

  // Check for sensor type. With a rig, it will be checked later, per sensor.
  if (!have_rig) {
    if (opt.sensor_type == "frame")
      opt.sensor_type = "pinhole"; // pinhole is same as frame
    if (opt.sensor_type != "pinhole" && opt.sensor_type != "linescan")
      vw::vw_throw(vw::ArgumentErr() 
                 << "The sensor type must be either pinhole/frame or linescan.\n");
  }

  if (opt.model_time) {
    // Must have the velocity set
    if (std::isnan(opt.velocity))
      vw::vw_throw(vw::ArgumentErr() << "Must set the velocity to model time.\n");
      
     // Must have first ground pos
    if (std::isnan(norm_2(opt.first_ground_pos)))
      vw::vw_throw(vw::ArgumentErr() 
                   << "Must set the first ground position to model time.\n"); 
      
    if (opt.camera_list != "")
      vw::vw_throw(vw::ArgumentErr() << "Cannot model time with --camera-list.\n");
  }
  
  // Check the reference time. We want the time accurate to within 1e-8 seconds,
  // and that is tricky when the time is large.
  if (opt.ref_time <= 0 || opt.ref_time >= 1e+6)
    vw::vw_throw(vw::ArgumentErr() << "The reference time is not positive or is too large. "
                 << "This can cause numerical issues.\n");
    
  if (have_rig) {
    // Read the rig configuration
    bool have_rig_transforms = true; // dictated by the api
    rig::readRigConfig(opt.rig_config, have_rig_transforms, rig);
    // Must have just one rig
    if (rig.cam_set.size() != 1)
      vw::vw_throw(vw::ArgumentErr() << "Only one rig (reference sensor) is supported "
                   << "in sat_sim.\n");  

    for (size_t i = 0; i < rig.cam_params.size(); i++) {
      auto const& params = rig.cam_params[i];
    
      // If the optical center is large, the images will show up very oblique.
      // The current sat_sim logic implicitly assumes that the optical center is close to
      // the image center, as we shoot rays to the ground not through the optical
      // center but through the middle of the image.
      if (params.GetOpticalOffset()[0] < 0 || params.GetOpticalOffset()[1] < 0 ||
          params.GetOpticalOffset()[0] >= params.GetDistortedSize()[0] || 
          params.GetOpticalOffset()[1] >= params.GetDistortedSize()[1])
        vw::vw_throw(vw::ArgumentErr() << "The optical center must be non-negative and "
                    << "within the image bounds. It is suggested to have it close to "
                    << "the image center.\n");
    
      if (params.GetDistortion().size() != 0)
        vw::vw_throw(vw::ArgumentErr() << "Distortion is not supported in sat_sim.\n");
    }
  }

  if (have_perturb) {
    // must have camera list
    if (opt.camera_list.empty())
      vw::vw_throw(vw::ArgumentErr() << "Must have camera list to perturb cameras.\n");
      
    // Sensor type must be pinhole
    if (opt.sensor_type != "pinhole")
      vw::vw_throw(vw::ArgumentErr() 
                   << "Perturbing cameras is only supported for pinhole cameras.\n");
      
    // Must not have a rig
    if (!opt.rig_config.empty())
      vw::vw_throw(vw::ArgumentErr() 
                   << "Perturbing cameras is not supported for rigs.\n");
    
    // Must set the satellite velocity
    if (std::isnan(opt.velocity) && opt.perturb_cameras)
      vw::vw_throw(vw::ArgumentErr() 
                   << "Must set the satellite velocity to perturb cameras.\n");
    
    // The jitter frequency must be set and not NaN
    if ((opt.jitter_frequency.empty() || std::isnan(opt.jitter_frequency[0])) &&
         opt.perturb_cameras)
      vw::vw_throw(vw::ArgumentErr() 
                   << "Must set the jitter frequency to perturb cameras.\n");
    
    if ((std::isnan(opt.velocity) || opt.velocity <= 0.0) && 
        std::isnan(opt.random_position_perturbation))
      vw::vw_throw(vw::ArgumentErr() << "The satellite velocity must be set and positive if "
                   << "perturbing existing cameras.\n");
  
    // No images will be created
    opt.no_images = true;
  }
  
 // Blur sigma must be non-negative
 if (opt.blur_sigma < 0)
   vw::vw_throw(vw::ArgumentErr() << "The blur sigma must be non-negative.\n");
   
  return;
}

int main(int argc, char *argv[]) {

  asp::SatSimOptions opt;
  rig::RigSet rig;
  try {
    handle_arguments(argc, argv, opt, rig);

    // Read the DEM
    vw::ImageViewRef<vw::PixelMask<float>> dem;
    float dem_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference dem_georef;
    vw::cartography::readGeorefImage(opt.dem_file, dem_nodata_val, dem_georef, dem);
    double height_guess = vw::cartography::demHeightGuess(dem); // for image-to-ground

    // Read the ortho image
    vw::ImageViewRef<vw::PixelMask<float>> ortho;
    float ortho_nodata_val = -std::numeric_limits<float>::max(); // will change
    vw::cartography::GeoReference ortho_georef;
    bool have_perturb = opt.perturb_cameras || opt.random_pose_perturbation ||
                        !std::isnan(opt.random_position_perturbation);

    if (!have_perturb)
      vw::cartography::readGeorefImage(opt.ortho_file, ortho_nodata_val, ortho_georef, ortho);

    std::vector<std::string> cam_names;
    std::vector<vw::CamPtr> cams;
    bool external_cameras = false;
    std::string suffix = ""; 
    if (!opt.camera_list.empty()) {
      // Read the cameras
      external_cameras = true;
      if (opt.sensor_type == "pinhole")
        asp::readPinholeCameras(opt, cam_names, cams);
      else
        asp::readLinescanCameras(opt, cam_names, cams);
      
      if (have_perturb)
        asp::perturbCameras(opt, suffix, dem_georef, cam_names, cams);
        
      // Generate images
      if (!opt.no_images)
        asp::genImages(opt, external_cameras, cam_names, cams, suffix, dem_georef, dem, 
          height_guess, ortho_georef, ortho, ortho_nodata_val);
      
    } else if (opt.rig_config.empty()) {
      // Generate the cameras   
      // The matrix cam2world_no_jitter is only needed with linescan cameras,
      // but compute it for consistency in all cases.
      bool have_rig = false;
      int rig_sensor_index = -1;
      asp::genCamerasImages(ortho_nodata_val, have_rig, rig_sensor_index,
                            dem, height_guess, ortho_georef, ortho, opt, 
                            rig, dem_georef, suffix); 

    } else {
      // The rig needs special treatment 
      asp::genRigCamerasImages(opt, rig, dem_georef, dem, height_guess, 
                               ortho_georef, ortho, ortho_nodata_val);
    }

    if (!opt.rig_config.empty())
      asp::writeRelRig(opt.out_prefix, rig);

  } ASP_STANDARD_CATCHES;

  return 0;
}

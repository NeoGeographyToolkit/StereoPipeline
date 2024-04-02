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

/// \file disp2ip.cc

// Produce additional interest points based on disparity maps. This is a
// tool for a very specialized purpose.

#include <asp/Core/Common.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <vw/BundleAdjustment/ControlNetwork.h>
#include <asp/Core/Nvm.h>
#include <asp/Core/Macros.h>
#include <asp/Core/DisparityProcessing.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace asp {
  
struct DispToIpOptions: vw::GdalWriteOptions {
  std::string left_raw_image_list, left_slog_image_list, right_slog_image_list,
  optical_center_list,
  stereo_prefix_list, input_nvm, output_nvm;
};


void handle_arguments(int argc, char *argv[], DispToIpOptions& opt) {
  
  po::options_description general_options("");
  general_options.add_options()
  ("left-raw-image-list", po::value(&opt.left_raw_image_list)->default_value(""),
   "List of raw left images, one per line.")
  ("left-slog-image-list", po::value(&opt.left_slog_image_list)->default_value(""),
   "List of left images after applying the slog filter, one per line.")
  ("right-slog-image-list", po::value(&opt.right_slog_image_list)->default_value(""),
    "List of right images after applying the slog filter, one per line.")
  ("stereo-prefix-list", po::value(&opt.stereo_prefix_list)->default_value(""),
  "List of stereo prefixes, one per line. Each prefix is for a stereo run "
  "with a left slog and right slog image, with affine epipolar alignment. "
  "Stereo could have been run with --correlator-mode, so without cameras.")
  ("optical-center-list", po::value(&opt.optical_center_list)->default_value(""),
    "List of optical centers for all slog images. On each line must have the image name, "
    "optical center column, then row. NVM files have the interest points shifted "
    "relative to the optical center.")
  ("input-nvm", po::value(&opt.input_nvm)->default_value(""),
    "Input NVM file, having interest point matches between the left raw images.")
  ("output-nvm", po::value(&opt.output_nvm)->default_value(""),
    "Output NVM file, having interest point matches between left-left, left-right, "
    "and right-right slog image combinations, produced with the help of disparity maps.");

  general_options.add(vw::GdalWriteOptionsDescription(opt));
  
  po::options_description positional("");
  po::positional_options_description positional_desc;
  
  std::string usage("[options]");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);
  
  // All inputs must be non-empty
  if (opt.left_raw_image_list.empty() || opt.left_slog_image_list.empty() ||
      opt.right_slog_image_list.empty() || opt.stereo_prefix_list.empty() ||
      opt.optical_center_list.empty() ||
      opt.input_nvm.empty() || opt.output_nvm.empty())
    vw::vw_throw(vw::ArgumentErr() << "Must specify all input files.\n");

  // Read the lists 
  std::vector<std::string> left_raw_image_files, left_slog_image_files, right_slog_image_files, stereo_prefixes;
  asp::read_list(opt.left_raw_image_list, left_raw_image_files);
  asp::read_list(opt.left_slog_image_list, left_slog_image_files);
  asp::read_list(opt.right_slog_image_list, right_slog_image_files);
  asp::read_list(opt.stereo_prefix_list, stereo_prefixes);
  
  // All lists must have the same size
  if (left_raw_image_files.size() != left_slog_image_files.size() ||
      left_raw_image_files.size() != right_slog_image_files.size() ||
      left_raw_image_files.size() != stereo_prefixes.size())
    vw::vw_throw(vw::ArgumentErr() << "All input lists must have the same size.\n");    

  // Read the optical center offsets for the slog images
  std::map<std::string, Eigen::Vector2d>  offsets;
  asp::readNvmOffsets(opt.optical_center_list, offsets); 

  // Number of offsets must equal the number of slog images
  if (offsets.size() != left_slog_image_files.size() + right_slog_image_files.size())
    vw::vw_throw(vw::ArgumentErr() << "Number of optical center offsets must equal "
                 << "the number of slog images.\n");

  // Test reading the nvm
  asp::nvmData nvm;
  bool nvm_no_shift = false; // have an offsets file
  asp::ReadNVM(opt.input_nvm, nvm_no_shift, nvm);
  
  // Test writing the nvm to tmp.nvm
  asp::WriteNVM(nvm, "tmp.nvm");
  //exit(0);
  
  // Read the nvm for the raw images
  vw::ba::ControlNetwork cnet("raw");
  //bool nvm_no_shift = false; // have an offsets file
  readNvmAsCnet(opt.input_nvm, nvm_no_shift, cnet);  
  
  // Create the output directory  
  vw::create_out_dir(opt.output_nvm);

  // Read the affine epipolar alignment from stereo, so we can undo it
  std::vector<vw::TransformPtr> left_trans, right_trans;

  // Invalid disparity pixel
  typedef typename DispImageType::pixel_type DispPixelT;
  DispPixelT invalid_disp; invalid_disp.invalidate();
  vw::ValueEdgeExtension<DispPixelT> invalid_ext(invalid_disp);

  // Iterate over images, and read the transforms to undo the alignment
  // in disparities. Also read the disparities and prepare for interpolation. 
  int num_runs = stereo_prefixes.size();
  std::vector<asp::DispImageType> disparities(num_runs);
  for (int i = 0; i < num_runs; i++) {
    std::cout << "now doing index " << i << std::endl;
    stereo_settings().correlator_mode = true; // No cameras assumed
    asp::stereo_settings().alignment_method = "affineepipolar";
    std::string stereo_prefix = stereo_prefixes[i]; 
    std::cout << "--prefix is " << stereo_prefix << std::endl;
    std::string session_name = "pinhole"; 
    asp::SessionPtr 
      session(asp::StereoSessionFactory::create(session_name, // may change
                                                opt, 
                                                left_slog_image_files[i], 
                                                right_slog_image_files[i],
                                                "", "", // No cameras
                                                stereo_prefix));
    left_trans.push_back(session->tx_left());
    right_trans.push_back(session->tx_right());
    
    // Check that the pointers are not null
    if (left_trans.back().get() == NULL || right_trans.back().get() == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Could not read the alignment transforms "
                   << "for stereo prefix: " << stereo_prefix);
      
    std::string dispFile = stereo_prefix + "-F.tif"; 
    std::cout << "--file is " << dispFile << std::endl;
    asp::DispImageType disp = session->pre_pointcloud_hook(dispFile);
    std::cout << "--disp cols and rows are " << disp.cols() << ' ' << disp.rows() << std::endl;
    // Must check that the sizes are positive
    if (disp.cols() == 0 || disp.rows() == 0)
      vw::vw_throw(vw::ArgumentErr() << "Empty disparity image: " << dispFile << "\n");
      
    // Add the disparity with interpolation to the list
    disparities[i] = interpolate(disp, vw::BilinearInterpolation(), invalid_ext);
    // Use bilinear interpolation with no data
  }
  
  // Red the raw image list from the cnet. The order is not the same as in list 
  // of raw images. 
  std::vector<std::string> & cnet_list = cnet.get_image_list(); // alias
  for (size_t i = 0; i < cnet_list.size(); i++) {
    std::cout << "cnet image " << i << " is " << cnet_list[i] << std::endl;
  }

  // The map from each left raw image to its index in that list
  std::map<std::string, int> left_raw_name_to_index;
  for (size_t i = 0; i < left_raw_image_files.size(); i++)
    left_raw_name_to_index[left_raw_image_files[i]] = i;
 
  // The images in the cnet are in a random order usually, as produced by Theia 
  // Need to find correspondences between the cnet indices and raw image indices
  std::map<int, int> cnet_index_to_raw_index;
  for (size_t i = 0; i < cnet_list.size(); i++) {
    std::cout << "cnet image " << i << " is " << cnet_list[i] << std::endl;
    // Look up in the raw image map
    auto it = left_raw_name_to_index.find(cnet_list[i]);
    if (it == left_raw_name_to_index.end())
      vw::vw_throw(vw::ArgumentErr() << "Could not find image " << cnet_list[i] << " in the raw image list.\n");
    int raw_index = it->second;
    std::cout << "--> raw index is " << raw_index << std::endl;
    cnet_index_to_raw_index[i] = raw_index;
  }

  // Replace in the cnet the left images with the left slog images
  for (size_t i = 0; i < cnet_list.size(); i++) {
    std::cout << "cnet image " << i << " is " << cnet_list[i] << std::endl;
    // find corresponding raw image index
    auto it = cnet_index_to_raw_index.find(i);
    if (it == cnet_index_to_raw_index.end())
      vw::vw_throw(vw::ArgumentErr() << "Could not find image " << cnet_list[i]
                    << " in the raw image list.\n");
    int raw_index = it->second;
    cnet_list[i] = left_slog_image_files[raw_index];
  }
  
  // Now add the right slog images to the cnet. This will also update
  // cnet_list, as it is an alias.
  // TODO(oalexan1): This must be a separate function.
  // TODO(oalexan1): This must be tested.
  std::cout << "--curr cnet list size is " << cnet_list.size() << std::endl;
  for (size_t i = 0; i < right_slog_image_files.size(); i++)
    cnet.add_image_name(right_slog_image_files[i]);
  std::cout << "--new cnet list size is " << cnet_list.size() << std::endl;

  // Number of right slog images (same as number of left slog images)
  int num_right_images = right_slog_image_files.size();
  std::cout << "num right images is " << num_right_images << std::endl;
  
  // TODO(oalexan1): Make this into a function.
  int net_size = cnet.size();    
  // Iterate over the cnet. For each feature in the input cnet, we will
  // add correspondences based on the disparity maps.
  for (int i = 0; i < net_size; i++) {
    vw::ba::ControlPoint & cp = cnet[i]; // alias
    
    //std::cout << "Control point " << i << " has " << cp.size() << " measures.\n";
    int num_measures = cp.size();
    for (int j = 0; j < num_measures; j++) {
      vw::ba::ControlMeasure & cm = cp[j];
      //std::cout << "  Measure " << j << " is " << cm << std::endl;
      int cid = cm.image_id();
      //std::cout << "cid is " << cid << std::endl;
      vw::Vector2 pix = cm.position();
      vw::Vector2 sigma = cm.sigma();
      
      // Find the raw image index corresponding to this cnet image index
      auto it = cnet_index_to_raw_index.find(cid);
      if (it == cnet_index_to_raw_index.end())
        vw::vw_throw(vw::ArgumentErr() << "Could not find an index in the raw image list.\n");
      int raw_index = it->second;
      
      //std::cout << "raw index is " << raw_index << std::endl;
      
      // Apply the alignment transform to the pixel
      //std::cout << "input pix is " << pix << std::endl;
      //std::cout << "trans pointer is " << left_trans[raw_index].get() << std::endl;
      pix = left_trans[raw_index]->forward(pix);
      //std::cout << "trans pix is " << pix << std::endl;
      
      // Find the interpolated disparity value
      auto disp = disparities[raw_index](pix[0], pix[1]);
      
      // Skip invalid disparities
      if (!is_valid(disp))
        continue;
      
      // Find the right transformed pixel
      vw::Vector2 right_pix = pix + disp.child();
      //std::cout << "right pix is " << right_pix << std::endl;
      
      // Undo the transform
      right_pix = right_trans[raw_index]->reverse(right_pix);
      //std::cout << "undone right pix is " << right_pix << std::endl;
      
      // Add the measure
      //std::cout << "--add measure " << right_pix[0] << ' ' << right_pix[1] << std::endl;
      cp.add_measure(vw::ba::ControlMeasure(right_pix[0], right_pix[1], 
                                            sigma[0], sigma[1], 
                                            num_right_images + raw_index));
      //std::cout << "done adding measure\n";
    }
  }
  
  // TODO(oalexan1): Must sort the control network by image name.
  // Maybe this can be done in the nvm format.
  // TODO(oalexan1): Must test only with 2 stereo pairs.
  std::vector<Eigen::Affine3d> world_to_cam = nvm.cid_to_cam_t_global;
  // duplicate these to make poses for the right images
  // These will not be accurate, but will be close enough for now.
  for (int i = 0; i < num_right_images; i++)
    world_to_cam.push_back(world_to_cam[i]);

  // Convert the cnet to the nvm format
  asp::cnetToNvm(cnet, offsets, world_to_cam, nvm);
    
  // Write the nvm to disk
  asp::WriteNVM(nvm, opt.output_nvm);
    
  // TODO(oalexan1): Must check if any matches were found in the right
  // image.
}

void disp2ip(DispToIpOptions const& opt) {
}

} // end namespace asp

int main(int argc, char *argv[]) {

  asp::DispToIpOptions opt;
  try {
    asp::handle_arguments(argc, argv, opt);
    asp::disp2ip(opt);
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

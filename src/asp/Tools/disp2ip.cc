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
  
// Given a cnet between a set of left images, and disparities from left
// to right images, add more interest points to the cnet based on the
// disparities. 
void completeCnetFromDisparities(vw::ba::ControlNetwork               & cnet,
                                 std::map<int, int>              const& cnet_to_raw_index,
                                 std::vector<asp::DispImageType> const& interp_disparities,
                                 std::vector<vw::TransformPtr>   const& left_trans,
                                 std::vector<vw::TransformPtr>   const& right_trans) {

  // All input quantities must have the same size
  size_t num_left_images = cnet_to_raw_index.size();  
  if (interp_disparities.size() != num_left_images ||
      left_trans.size()         != num_left_images ||
      right_trans.size()        != num_left_images)
    vw::vw_throw(vw::ArgumentErr() << "Input datasets must have the same size.\n");
    
  int net_size = cnet.size();    
  
  for (int i = 0; i < net_size; i++) {
    vw::ba::ControlPoint & cp = cnet[i]; // alias
    
    int num_measures = cp.size();
    for (int j = 0; j < num_measures; j++) {
      vw::ba::ControlMeasure & cm = cp[j];
      int cid = cm.image_id();
      vw::Vector2 pix = cm.position();
      vw::Vector2 sigma = cm.sigma();
      
      // Find the raw image index corresponding to this cnet image index
      auto it = cnet_to_raw_index.find(cid);
      if (it == cnet_to_raw_index.end())
        vw::vw_throw(vw::ArgumentErr() << "Could not find an index in the raw image list.\n");
      int raw_index = it->second;
      
      // Apply the alignment transform to the left pixel
      pix = left_trans[raw_index]->forward(pix);
      
      // Find the interpolated disparity value
      auto disp = interp_disparities[raw_index](pix[0], pix[1]);
      
      // Skip invalid disparities
      if (!is_valid(disp))
        continue;
      
      // Find the right transformed pixel
      vw::Vector2 right_pix = pix + disp.child();
      
      // Undo the alignment transform in the right pixel
      right_pix = right_trans[raw_index]->reverse(right_pix);
      
      // Add the measure
      cp.add_measure(vw::ba::ControlMeasure(right_pix[0], right_pix[1], 
                                            sigma[0], sigma[1], 
                                            num_left_images + raw_index));
    }
  }
}
  
struct DispToIpOptions: vw::GdalWriteOptions {
  std::string left_raw_image_list, left_filtered_image_list, right_filtered_image_list,
  optical_center_list,
  stereo_prefix_list, input_nvm, output_nvm;
};

void disp2ip(int argc, char *argv[], DispToIpOptions& opt) {
  
  po::options_description general_options("");
  general_options.add_options()
  ("left-raw-image-list", po::value(&opt.left_raw_image_list)->default_value(""),
   "List of raw left images, one per line.")
  ("left-filtered-image-list", po::value(&opt.left_filtered_image_list)->default_value(""),
   "List of left images after applying a filter, one per line.")
  ("right-filtered-image-list", po::value(&opt.right_filtered_image_list)->default_value(""),
    "List of right images after applying a filter, one per line.")
  ("stereo-prefix-list", po::value(&opt.stereo_prefix_list)->default_value(""),
   "List of stereo prefixes, one per line. Each prefix is for a stereo run "
   "with a left filtered and right filtered image, with affine epipolar alignment. "
   "Stereo could have been run with --correlator-mode, so without cameras.")
  ("optical-center-list", po::value(&opt.optical_center_list)->default_value(""),
    "List of optical centers for all filtered images. On each line must have the image name, "
    "optical center column, then row. The order of images is not important.")
  ("input-nvm", po::value(&opt.input_nvm)->default_value(""),
    "Input NVM file, having interest point matches between the left raw images.")
  ("output-nvm", po::value(&opt.output_nvm)->default_value(""),
    "Output NVM file, having interest point matches between all filtered images, "
    "produced with the help of disparity maps.");

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
  if (opt.left_raw_image_list.empty() || opt.left_filtered_image_list.empty() ||
      opt.right_filtered_image_list.empty() || opt.stereo_prefix_list.empty() ||
      opt.optical_center_list.empty() ||
      opt.input_nvm.empty() || opt.output_nvm.empty())
    vw::vw_throw(vw::ArgumentErr() << "Must specify all input files.\n");

  // Create the output directory  
  vw::create_out_dir(opt.output_nvm);

  // Read the lists 
  std::vector<std::string> left_raw_image_files, left_filtered_image_files, right_filtered_image_files, stereo_prefixes;
  asp::read_list(opt.left_raw_image_list, left_raw_image_files);
  asp::read_list(opt.left_filtered_image_list, left_filtered_image_files);
  asp::read_list(opt.right_filtered_image_list, right_filtered_image_files);
  asp::read_list(opt.stereo_prefix_list, stereo_prefixes);
  
  // All lists must have the same size
  if (left_raw_image_files.size() != left_filtered_image_files.size() ||
      left_raw_image_files.size() != right_filtered_image_files.size() ||
      left_raw_image_files.size() != stereo_prefixes.size())
    vw::vw_throw(vw::ArgumentErr() << "All input lists must have the same size.\n");    

  // Read the optical center offsets for the filtered images
  std::map<std::string, Eigen::Vector2d>  offsets;
  asp::readNvmOffsets(opt.optical_center_list, offsets); 

  // Number of offsets must equal the number of filtered images
  if (offsets.size() != left_filtered_image_files.size() + right_filtered_image_files.size())
    vw::vw_throw(vw::ArgumentErr() << "Number of optical center offsets must equal "
                 << "the number of filtered images.\n");

  // Read the nvm and create the cnet
  bool nvm_no_shift = false; // have an offsets file
  rig::nvmData nvm;
  asp::readNvm(opt.input_nvm, nvm_no_shift, nvm);
  vw::ba::ControlNetwork cnet("raw");
  std::vector<Eigen::Affine3d> world_to_cam;
  std::map<std::string, Eigen::Vector2d> raw_offsets;
  asp::nvmToCnet(nvm, cnet, raw_offsets, world_to_cam);
  
  // Invalid disparity pixel
  typedef typename DispImageType::pixel_type DispPixelT;
  DispPixelT invalid_disp; invalid_disp.invalidate();
  vw::ValueEdgeExtension<DispPixelT> invalid_ext(invalid_disp);

  // Iterate over images, and read the transforms to undo the alignment
  // in disparities. Also read the disparities and prepare for interpolation. 
  int num_runs = stereo_prefixes.size();
  std::vector<asp::DispImageType> interp_disparities(num_runs);
  std::vector<vw::TransformPtr> left_trans(num_runs), right_trans(num_runs);
  for (int i = 0; i < num_runs; i++) {
    stereo_settings().correlator_mode = true; // No cameras assumed
    asp::stereo_settings().alignment_method = "affineepipolar";
    std::string stereo_prefix = stereo_prefixes[i]; 
    std::string session_name = "pinhole"; 
    std::string input_dem = ""; // No DEM
    bool allow_map_promote = false, quiet = true;
    asp::SessionPtr 
      session(asp::StereoSessionFactory::create(session_name, // may change
                                                opt, 
                                                left_filtered_image_files[i], 
                                                right_filtered_image_files[i],
                                                "", "", // No cameras
                                                stereo_prefix, input_dem,
                                                allow_map_promote, quiet));
    left_trans[i] = session->tx_left();
    right_trans[i] = session->tx_right();
    
    // Check that the pointers are not null
    if (left_trans[i].get() == NULL || right_trans[i].get() == NULL)
      vw::vw_throw(vw::ArgumentErr() << "Could not read the alignment transforms "
                   << "for stereo prefix: " << stereo_prefix);

    // Read the disparity      
    std::string dispFile = stereo_prefix + "-F.tif"; 
    asp::DispImageType disp = session->pre_pointcloud_hook(dispFile);
    if (disp.cols() == 0 || disp.rows() == 0)
      vw::vw_throw(vw::ArgumentErr() << "Empty disparity image: " << dispFile << "\n");
      
    // Add the disparity with bilinear interpolation to the list. Care with no-data.
    interp_disparities[i] = interpolate(disp, vw::BilinearInterpolation(), invalid_ext);
  }
  
  // Red the raw image list from the cnet. The order is not the same as in list 
  // of raw images. 
  std::vector<std::string> & cnet_list = cnet.get_image_list(); // alias

  // The map from each left raw image to its index in that list
  std::map<std::string, int> left_raw_name_to_index;
  for (size_t i = 0; i < left_raw_image_files.size(); i++)
    left_raw_name_to_index[left_raw_image_files[i]] = i;
 
  // The images in the cnet are in a random order usually, as produced by Theia 
  // Need to find correspondences between the cnet indices and raw image indices
  std::map<int, int> cnet_to_raw_index;
  for (size_t i = 0; i < cnet_list.size(); i++) {
    // Look up in the raw image map
    auto it = left_raw_name_to_index.find(cnet_list[i]);
    if (it == left_raw_name_to_index.end())
      vw::vw_throw(vw::ArgumentErr() << "Could not find image " << cnet_list[i] 
                   << " in the raw image list.\n");
    int raw_index = it->second;
    cnet_to_raw_index[i] = raw_index;
  }

  // Replace in the cnet the left images with the left filtered images
  for (size_t i = 0; i < cnet_list.size(); i++) {
    // find corresponding raw image index
    auto it = cnet_to_raw_index.find(i);
    if (it == cnet_to_raw_index.end())
      vw::vw_throw(vw::ArgumentErr() << "Could not find image " << cnet_list[i]
                    << " in the raw image list.\n");
    int raw_index = it->second;
    cnet_list[i] = left_filtered_image_files[raw_index];
  }
  
  // Number of right filtered images (same as number of left filtered images)
  int num_right_images = right_filtered_image_files.size();

  // Add the right filtered images to the cnet. This will also update
  // cnet_list, as it is an alias.
  for (int i = 0; i < num_right_images; i++)
    cnet.add_image_name(right_filtered_image_files[i]);

  // Make fake camera poses for the right filtered images. Those won't be accurate,
  // but have to create something. They will not be used.
  for (int i = 0; i < num_right_images; i++)
    world_to_cam.push_back(world_to_cam[i]);

  // The core work  
  asp::completeCnetFromDisparities(cnet, cnet_to_raw_index, interp_disparities,
                                   left_trans, right_trans);
  
  // Convert the cnet to the nvm format
  asp::cnetToNvm(cnet, offsets, world_to_cam, nvm);
    
  // Write the nvm to disk
  asp::writeNvm(nvm, opt.output_nvm);
}

} // end namespace asp

int main(int argc, char *argv[]) {

  asp::DispToIpOptions opt;
  try {
    asp::disp2ip(argc, argv, opt);
  } ASP_STANDARD_CATCHES;
  
  return 0;
}

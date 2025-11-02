// __BEGIN_LICENSE__
//  Copyright (c) 2006-2024, United States Government as represented by the
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

/// \file GuiArgs.cc
///

#include <asp/GUI/GuiArgs.h>
#include <asp/GUI/GuiUtilities.h>

#include <vw/FileIO/FileTypes.h>
#include <vw/FileIO/DiskImageView.h>

#include <cstring>

namespace asp {

// TODO(oalexan1): If the same file is repeated, will the book-keeping fail?
// See the .h for more details.
void preprocessArgs(int &argc, char** argv,
                    std::vector<std::map<std::string, std::string>> & properties) {

  // Defaults
  std::string curr_style = "default";
  std::string curr_color = "default";
  std::string curr_colormap = "binary-red-blue";
  std::string colorbar = "0";
  
  // One set of properties for each argument. That to make sure that a filename
  // can show up twice with different properties
  int out_it = 1;
  properties.resize(argc);
  for (int it = 1; it < argc; it++) { // skip program name, so start from 1

    // TODO(oalexan1): Add support for --no-colorize, and make this and --colorize
    // to be able to apply to all subsequent images.
    
    if (std::string(argv[it]) == "--style") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_style = argv[it]; // copy the style value, and move past it
      continue;
    }

    if (std::string(argv[it]) == "--color") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_color = argv[it]; // copy the color value, and move past it
      continue;
    }

    if (std::string(argv[it]) == "--colormap-style") {
      if (it == argc - 1)
        continue; // There is nothing else

      it++;
      curr_colormap = argv[it]; // copy the color value, and move past it
      continue;
    }

    // This is an option with no value
    if (std::string(argv[it]) == "--colorbar") {
      colorbar = "1";
      continue;
    }

    // This is an option with no value
    if (std::string(argv[it]) == "--no-colorbar") {
      colorbar = "0";
      continue;
    }

    // If this argument does not start with a dash, so is not an
    // option, assign to it the properties so far
    if (argv[it][0] != '-') {
      properties[it]["name"] = argv[it];
      properties[it]["style"] = curr_style;  
      properties[it]["color"] = curr_color;
      properties[it]["colormap"] = curr_colormap;
      properties[it]["colorbar"] = colorbar;
    }
    
    // Shift arguments left, which will wipe what we processed above
    argv[out_it] = argv[it]; // this copies pointer addresses
    out_it++; 
  }

  // Update the number of remaining arguments
  argc = out_it;

  return;
}

void lookupPropertyIndices(std::vector<std::map<std::string, std::string>> const&
                           properties,
                           std::vector<std::string> const& images,
                           std::vector<int> & propertyIndices) {

  propertyIndices.clear();

  size_t start_p = 0;
  for (size_t i = 0; i < images.size(); i++) {

    for (size_t p_it = start_p; p_it < properties.size(); p_it++) {
      auto key_ptr = properties[p_it].find("name");
      if (key_ptr == properties[p_it].end())
        continue;
      if (key_ptr->second == images[i]) {
        start_p = p_it; // Found the right index for the property for the current file
        break; 
      }
    }

    if (start_p >= properties.size()) {
      if (properties.empty()) 
        vw::vw_throw(vw::ArgumentErr() << "No image properties were found.\n");

      // For the nvm case, to not go out of bounds
      start_p = properties.size() - 1; 
    }
    propertyIndices.push_back(start_p);

    start_p++; // next time start the search after the entry just identified
  }
  
}

// Keep only image files, shape files, and csv files
void filterImages(std::vector<std::string> & image_files) {

  // Form a local copy
  std::vector<std::string> local_files;
 
  local_files.clear();
  for (size_t i = 0; i < image_files.size(); i++) {
    bool is_image = true;
    try {
      vw::DiskImageView<double> img(image_files[i]);
    } catch(...) {
      is_image = false;
    }
    
    // Accept shape files and csv files alongside images
    if (!is_image &&
        !vw::has_shp_extension(image_files[i]) &&
        !asp::hasCsv(image_files[i]))
      continue;

    local_files.push_back(image_files[i]);
  }
  
  // Overwrite the input files
  image_files = local_files;
}

// Remove option and vals
void rmOptionVals(int argc, char ** argv, std::string const& opt, int num_vals) {
  // Wipe say --left-image-crop-win 0 0 100 100, that is, an option
  // with 4 values.
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) != opt)
      continue;
    
    for (int j = i; j < i + num_vals + 1; j++) {
      if (j >= argc) break;
      // To avoid problems with empty strings, set the string to space instead
      if (strlen(argv[j]) > 0) {
        argv[j][0] = ' ';
        argv[j][1] = '\0';
      }
    }
  }
}

} // namespace asp
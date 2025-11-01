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

/// \file GuiArgs.h
///

#ifndef __ASP_GUI_GUI_ARGS_H__
#define __ASP_GUI_GUI_ARGS_H__

#include <string>
#include <vector>
#include <map>

#include <vw/FileIO/GdalIO.h>
#include <asp/GUI/GuiUtilities.h>

namespace asp {

// Given an input string as:
// stereo_gui --style line --color red --colormap-style inferno file1.txt \
//   --color green file2.txt
// extract each style, color, and colormap. Any of these apply for any following
// files, till modified by another such option. The default style and color
// are "default". Later, these will be either read from the files
// themselves, or otherwise set to "line" and "green". Then modify the
// args to remove these options, as the boost parser cannot parse
// repeated options.
void preprocessArgs(int &argc, char** argv,
                    std::vector<std::map<std::string, std::string>> & properties);

// Given a vector of properties, with each property having a
// potentially non-unique image name as an attribute and other
// attributes as well, and a list of images, find the index of the
// property for that image. Such logic is necessary because the same
// image may show up twice, but with different properties each
// time. So, the first occurrence of an image is matched to the first
// occurrence of a property with that name, and so on. Also, there may
// be properties for entities which are no longer in the list of
// images.
void lookupPropertyIndices(std::vector<std::map<std::string, std::string>> const&
                           properties,
                           std::vector<std::string> const& images,
                           std::vector<int> & propertyIndices);

// Filter the input files, keeping only the valid images, shape files, and csv files
void filterImages(std::vector<std::string> & image_files);

// Remove option and vals
void rmOptionVals(int argc, char ** argv, std::string const& opt, int num_vals);

} // namespace asp

#endif  // __ASP_GUI_GUI_ARGS_H__

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


/// \file DiskImagePyramidMultiChannel.h
///
/// Functions for managing multi-channel images and converting
/// them to QImage.
///
#ifndef __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__
#define __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__

#include <asp/GUI/GuiBase.h>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/mpl/or.hpp>
#include <omp.h>

// Qt
#include <QWidget>

// Vision Workbench
#include <vw/Core/Thread.h>
#include <vw/Core/Log.h>
#include <vw/Image/ImageViewRef.h>
#include <vw/Image/ImageView.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/BBox.h>
#include <vw/Math/Vector.h>
#include <vw/Mosaic/DiskImagePyramid.h>
#include <vw/Core/RunOnce.h>
#include <vw/Core/Stopwatch.h>

#include <string>
#include <vector>
#include <list>
#include <set>

namespace vw { namespace gui {

  // The kinds of images we support
  enum ImgType {UNINIT, CH1_DOUBLE, CH2_UINT8, CH3_UINT8, CH4_UINT8};

  /// A global structure to hold all the temporary files we have created
  struct TemporaryFiles {
    std::set<std::string> files;
  };
  /// Access the global list of temporary files
  TemporaryFiles& temporary_files();

  // An image class that supports 1 to 3 channels.  We use
  // DiskImagePyramid<double> to be able to use some of the
  // pre-defined member functions for an image class. This class
  // is not a perfect solution, but there seem to be no easy way
  // in ASP to handle images with variable numbers of channels.
  // TODO: Add the case when multi-channel images also have float or double pixels
  struct DiskImagePyramidMultiChannel {
    vw::GdalWriteOptions m_opt;
    vw::mosaic::DiskImagePyramid<double>               m_img_ch1_double;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 2>> m_img_ch2_uint8;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 3>> m_img_ch3_uint8;
    vw::mosaic::DiskImagePyramid<Vector<vw::uint8, 4>> m_img_ch4_uint8;
    int m_num_channels;
    int m_rows, m_cols;
    ImgType m_type; // keeps track of which of the above images we use

    // Constructor
    DiskImagePyramidMultiChannel(std::string const& image_file = "",
                                 vw::GdalWriteOptions const&
                                 opt = vw::GdalWriteOptions(),
                                 int top_image_max_pix = 1000*1000,
                                 int subsample = 2);

    // This function will return a QImage to be shown on screen.
    // How we create it, depends on the type of image we want to display.
    void get_image_clip(double scale_in, vw::BBox2i region_in, bool highlight_nodata,
                        QImage & qimg, double & scale_out,
                        vw::BBox2i & region_out) const;
    double get_nodata_val() const;

    int32 cols  () const { return m_cols;  }
    int32 rows  () const { return m_rows;  }
    int32 planes() const { return m_num_channels; }

    /// Return the element at this location (at the lowest level) cast to double.
    /// - Only works for single channel pyramids!
    double get_value_as_double(int32 x, int32 y) const;

    // Return value as string
    std::string get_value_as_str(int32 x, int32 y) const;
  };

}} // namespace vw::gui

#endif  // __STEREO_GUI_DISK_IMAGE_PYRAMID_MULTICHANNEL_H__

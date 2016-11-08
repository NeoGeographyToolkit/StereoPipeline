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


/// \file disparitydebug.cc
///

#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <stdlib.h>

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace vw;
using namespace vw::stereo;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::string input_file_name;
  BBox2       normalization_range;
  BBox2       roi;    ///< Only generate output images in this region

  // Output
  std::string output_prefix, output_file_type;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add_options()
    ("normalization", po::value(&opt.normalization_range)->default_value(BBox2(0,0,0,0), "auto"),
     "Normalization range. Specify in format: hmin,vmin,hmax,vmax.")
    ("roi", po::value(&opt.roi)->default_value(BBox2(0,0,0,0), "auto"),
     "Region of interest. Specify in format: xmin,ymin,xmax,ymax.")
    ("output-prefix,o", po::value(&opt.output_prefix), "Specify the output prefix.")
    ("output-filetype,t", po::value(&opt.output_file_type)->default_value("tif"), "Specify the output file type.");
  general_options.add( vw::cartography::GdalWriteOptionsDescription(opt) );

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.input_file_name), "Input disparity map.");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("[options] <input disparity map>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered );

  if ( opt.input_file_name.empty() )
    vw_throw( ArgumentErr() << "Missing input file!\n"
              << usage << general_options );
  if ( opt.output_prefix.empty() )
    opt.output_prefix = vw::prefix_from_filename(opt.input_file_name);
}

template <class PixelT>
void do_disparity_visualization(Options& opt) {
  DiskImageView<PixelT > disk_disparity_map(opt.input_file_name);

  cartography::GeoReference georef;
  bool has_georef  = read_georeference(georef, opt.input_file_name);
  bool has_nodata = false;
  float output_nodata = -32768.0;

  vw_out() << "\t--> Computing disparity range \n";

  // If no ROI passed in, use the full image
  BBox2 roiToUse(opt.roi);
  if ( opt.roi == BBox2(0,0,0,0) )
    roiToUse = BBox2(0,0,disk_disparity_map.cols(),disk_disparity_map.rows());

  if (has_georef)
    georef = crop(georef, roiToUse);

  // We don't want to sample every pixel as the image might be very
  // large. Let's subsample the image so that it is rough 1000x1000 samples.
  float subsample_amt =
    float(roiToUse.height())*float(roiToUse.width()) / ( 1000.f * 1000.f );

  // Compute intensity display range if not passed in
  if ( opt.normalization_range == BBox2(0,0,0,0) )
    opt.normalization_range =
      get_disparity_range(subsample(crop(disk_disparity_map, roiToUse),
                                    subsample_amt > 1 ? subsample_amt : 1));

  vw_out() << "\t    Horizontal: [" << opt.normalization_range.min().x()
           << " " << opt.normalization_range.max().x() << "]    Vertical: ["
           << opt.normalization_range.min().y() << " "
           << opt.normalization_range.max().y() << "]\n";

  // Generate value-normalized copies of the H and V channels
  typedef typename PixelChannelType<PixelT>::type ChannelT;
  ImageViewRef<ChannelT> horizontal =
    apply_mask(copy_mask(clamp(normalize(crop(select_channel(disk_disparity_map,0),
                                              roiToUse),
                                         opt.normalization_range.min().x(),
                                         opt.normalization_range.max().x(),
                                         ChannelRange<ChannelT>::min(),ChannelRange<ChannelT>::max()
                                        )
                              ),
                         crop(disk_disparity_map, roiToUse)
                        )
              );
  ImageViewRef<ChannelT> vertical =
    apply_mask(copy_mask(clamp(normalize(crop(select_channel(disk_disparity_map,1),
                                              roiToUse),
                                         opt.normalization_range.min().y(),
                                         opt.normalization_range.max().y(),
                                         ChannelRange<ChannelT>::min(),ChannelRange<ChannelT>::max()
                                        )
                              ),
                         crop(disk_disparity_map, roiToUse)
                        )
              );

  // Write both images to disk, casting as UINT8
  std::string h_file = opt.output_prefix+"-H."+opt.output_file_type;
  vw_out() << "\t--> Writing horizontal disparity debug image: " << h_file << "\n";
  block_write_gdal_image( h_file,
                          channel_cast_rescale<uint8>(horizontal),
                          has_georef, georef,
                          has_nodata, output_nodata,
                          opt, TerminalProgressCallback("asp","\t    H : "));
  std::string v_file = opt.output_prefix+"-V."+opt.output_file_type;
  vw_out() << "\t--> Writing vertical disparity debug image: " << v_file << "\n";
  block_write_gdal_image( v_file,
                          channel_cast_rescale<uint8>(vertical),
                          has_georef, georef,
                          has_nodata, output_nodata,
                          opt, TerminalProgressCallback("asp","\t    V : "));
}

int main( int argc, char *argv[] ) {

  Options opt;
  try {
    handle_arguments( argc, argv, opt );

    vw_out() << "Opening " << opt.input_file_name << "\n";
    ImageFormat fmt = vw::image_format(opt.input_file_name);

    switch(fmt.pixel_format) {
    case VW_PIXEL_GENERIC_2_CHANNEL:
      switch (fmt.channel_type) {
      case VW_CHANNEL_INT32:
        do_disparity_visualization<Vector2i>(opt); break;
      default:
        do_disparity_visualization<Vector2f>(opt); break;
      } break;
    case VW_PIXEL_RGB:
    case VW_PIXEL_GENERIC_3_CHANNEL:
      switch (fmt.channel_type) {
      case VW_CHANNEL_INT32:
        do_disparity_visualization<PixelMask<Vector2i> >(opt); break;
      default:
        do_disparity_visualization<PixelMask<Vector2f> >(opt); break;
      } break;
    case VW_PIXEL_SCALAR:
      // OpenEXR stores everything as planar, so this allows us to
      // still read that data.
      if ( fmt.planes == 2 ) {
        switch (fmt.channel_type) {
        case VW_CHANNEL_INT32:
          do_disparity_visualization<Vector2i>(opt); break;
        default:
          do_disparity_visualization<Vector2f>(opt); break;
        } break;
      } else if ( fmt.planes == 3 ) {
        switch (fmt.channel_type) {
        case VW_CHANNEL_INT32:
          do_disparity_visualization<PixelMask<Vector2i> >(opt); break;
        default:
          do_disparity_visualization<PixelMask<Vector2f> >(opt); break;
        } break;
      }
    default:
      vw_throw( ArgumentErr() << "Unsupported pixel format. Expected GENERIC 2 or 3 CHANNEL image. Instead got [" << pixel_format_name(fmt.pixel_format) << "]" );
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}

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

#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Image/Filter.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace vw;
using namespace vw::stereo;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Disparity norm
double disparity_norm(PixelMask<Vector2f> const& pix) {
  if (!is_valid(pix)) 
    return 0.0;
  return norm_2(pix.child());
}

// To find the disparity norm of an image, use
// per_pixel_filter(image, DisparityNorm()).
class DisparityNorm: public ReturnFixedType<double> {
  public:
  double operator()(PixelMask<Vector2f> const& pix) const {
    return disparity_norm(pix);
  }
};

// Find the maximum of the norms of differences between a disparity
// at a pixel and its four left, right, top, and bottom neighbors.
// For the disparity to result in a nice transform from left to
// right image the norms better be less than 1.
class DispNormDiff: public ImageViewBase<DispNormDiff> {
  ImageViewRef<PixelMask<Vector2f>> m_img;

public:
  DispNormDiff(ImageViewRef<PixelMask<Vector2f>> const& img):
    m_img(img) {}
  
  typedef float result_type;
  typedef result_type pixel_type;
  
  typedef ProceduralPixelAccessor<DispNormDiff> pixel_accessor;

  inline int32 cols() const { return m_img.cols(); }
  inline int32 rows() const { return m_img.rows(); }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this, 0, 0); }

  inline result_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "DispNormDiff::operator()(...) is not implemented");
    return result_type(0.0);
  }

  typedef CropView<ImageView<result_type>> prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    ImageView<result_type> tile(bbox.width(), bbox.height());

    // Need to see each pixel's neighbors
    int extra = 1;
    BBox2i biased_box = bbox;
    biased_box.expand(extra);
    biased_box.crop(bounding_box(m_img));

    // Bring fully in memory a crop of the input for this tile
    ImageView<PixelMask<Vector2f>> cropped_img = crop(m_img, biased_box);
    
    for (int col = bbox.min().x(); col < bbox.max().x(); col++) {
      for (int row = bbox.min().y(); row < bbox.max().y(); row++) {

        // Find the disparity value at the current pixel and the neighbors.
        // Need to be careful to account for the bias and for indices
        // going out of range.
        // Coordinates of a pixel and its neighbors.
        std::vector<Vector2i> coords = {Vector2i(0, 0), Vector2i(1, 0), Vector2i(0, 1),
                                        Vector2i(-1, 0), Vector2i(0, -1)};
        std::vector<PixelMask<Vector2f>> vals(5);

        for (int it = 0; it < 5; it++) {
          Vector2i coord = Vector2i(col, row) + coords[it]; // pixel in uncropped image
          if (biased_box.contains(coord)) {
            // Take into account the bounds and the crop
            vals[it] = cropped_img(coord.x() - biased_box.min().x(),
                                   coord.y() - biased_box.min().y());
          } else {
            // Out of range
            vals[it].child() = Vector2f(0, 0);
            vals[it].invalidate(); 
          }
        }

        double max_norm_diff = 0.0;
        if (is_valid(vals[0])) {
          // Need to have the center pixel valid
          for (int it = 1; it < 5; it++) {
            if (!is_valid(vals[it])) 
              continue;
            max_norm_diff = std::max(max_norm_diff, norm_2(vals[it].child() - vals[0].child()));
          }
        }
        
        tile(col - bbox.min().x(), row - bbox.min().y()) = max_norm_diff;
      }
    }
    
    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

struct Options : vw::cartography::GdalWriteOptions {
  // Input
  std::string input_file_name;
  BBox2       normalization_range;
  BBox2       roi; ///< Only generate output images in this region
  bool        save_norm, save_norm_diff;
  
  // Output
  std::string output_prefix, output_file_type;
  
  Options(): save_norm(false), save_norm_diff(false) {}
};

void handle_arguments(int argc, char *argv[], Options& opt) {
  po::options_description general_options("");
  general_options.add_options()
    ("normalization", po::value(&opt.normalization_range)->default_value(BBox2(0, 0, 0, 0), "auto"),
     "Normalization range. Specify in format: hmin vmin hmax vmax.")
    ("roi", po::value(&opt.roi)->default_value(BBox2(0,0,0,0), "auto"),
     "Region of interest. Specify in format: xmin ymin xmax ymax.")
    ("save-norm",      po::bool_switch(&opt.save_norm)->default_value(false),
     "Save the norm of the disparity instead of its two bands.")
    ("save-norm-diff",      po::bool_switch(&opt.save_norm_diff)->default_value(false),
     "Save the maximum of norms of differences between a disparity and its four neighbors.")
    ("output-prefix, o", po::value(&opt.output_prefix), "Specify the output prefix.")
    ("output-filetype, t", po::value(&opt.output_file_type)->default_value("tif"),
     "Specify the output file type.");
  
  general_options.add(vw::cartography::GdalWriteOptionsDescription(opt));

  po::options_description positional("");
  positional.add_options()
    ("input-file", po::value(&opt.input_file_name), "Input disparity map.");

  po::positional_options_description positional_desc;
  positional_desc.add("input-file", 1);

  std::string usage("[options] <input disparity map>");
  bool allow_unregistered = false;
  std::vector<std::string> unregistered;
  po::variables_map vm =
    asp::check_command_line(argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage,
                             allow_unregistered, unregistered);

  if (opt.input_file_name.empty())
    vw_throw(ArgumentErr() << "Missing input file!\n"
              << usage << general_options);

  if (opt.output_prefix.empty())
    opt.output_prefix = vw::prefix_from_filename(opt.input_file_name);

  if (opt.save_norm && opt.save_norm_diff)
    vw_throw(ArgumentErr() << "Cannot save both the norm and norm of differences at the same "
             << "time.\n" << usage << general_options);
}

template <class PixelT>
void process_disparity(Options& opt) {

  cartography::GeoReference georef;
  bool has_georef  = read_georeference(georef, opt.input_file_name);
  bool has_nodata = false;
  float output_nodata = -32768.0;

  if (opt.save_norm) {
    DiskImageView<PixelMask<Vector2f>> disk_disparity_map(opt.input_file_name);

    std::string norm_file = opt.output_prefix + "-norm." + opt.output_file_type;
    vw_out() << "\t--> Writing disparity norm: " << norm_file << "\n";
    block_write_gdal_image(norm_file,
                           per_pixel_filter(disk_disparity_map, DisparityNorm()),
                           has_georef, georef,
                           has_nodata, output_nodata,
                           opt, TerminalProgressCallback("asp","\t    norm: "));
    return;
  }
  
  if (opt.save_norm_diff) {
    DiskImageView<PixelMask<Vector2f>> disk_disparity_map(opt.input_file_name);

    std::string norm_file = opt.output_prefix + "-norm-diff." + opt.output_file_type;
    vw_out() << "\t--> Writing norm of disparity diff: " << norm_file << "\n";
    block_write_gdal_image(norm_file,
                           DispNormDiff(disk_disparity_map),
                           has_georef, georef,
                           has_nodata, output_nodata,
                           opt, TerminalProgressCallback("asp","\t    norm: "));
    return;
  }

  DiskImageView<PixelT> disk_disparity_map(opt.input_file_name);

  // If no ROI passed in, use the full image
  BBox2 roiToUse(opt.roi);
  if (opt.roi == BBox2(0,0,0,0))
    roiToUse = BBox2(0,0,disk_disparity_map.cols(),disk_disparity_map.rows());

  if (has_georef)
    georef = crop(georef, roiToUse);

  // Compute intensity display range if not passed in. For this purpose
  // subsample the image.
  vw_out() << "\t--> Computing disparity range.\n";
  if (opt.normalization_range == BBox2(0,0,0,0)) {
    float subsample_amt =
      float(roiToUse.height())*float(roiToUse.width()) / (1000.f * 1000.f);
    subsample_amt = std::max(subsample_amt, 1.0f);
    opt.normalization_range
      = get_disparity_range(subsample(crop(disk_disparity_map, roiToUse), subsample_amt));
  }
  
  vw_out() << "\t    Horizontal: [" << opt.normalization_range.min().x()
           << " " << opt.normalization_range.max().x() << "]    Vertical: ["
           << opt.normalization_range.min().y() << " "
           << opt.normalization_range.max().y() << "]\n";

  // Generate value-normalized copies of the H and V channels
  typedef typename PixelChannelType<PixelT>::type ChannelT;
  ImageViewRef<ChannelT> horizontal =
    apply_mask(copy_mask(clamp(normalize(crop(select_channel(disk_disparity_map, 0),
                                              roiToUse),
                                         opt.normalization_range.min().x(),
                                         opt.normalization_range.max().x(),
                                         ChannelRange<ChannelT>::min(),
                                         ChannelRange<ChannelT>::max()
                                         )),
                         crop(disk_disparity_map, roiToUse)));
  ImageViewRef<ChannelT> vertical =
    apply_mask(copy_mask(clamp(normalize(crop(select_channel(disk_disparity_map, 1),
                                              roiToUse),
                                         opt.normalization_range.min().y(),
                                         opt.normalization_range.max().y(),
                                         ChannelRange<ChannelT>::min(),
                                         ChannelRange<ChannelT>::max()
                                       )),
                         crop(disk_disparity_map, roiToUse)));
  
  // Write both images to disk, casting as UINT8
  std::string h_file = opt.output_prefix + "-H." + opt.output_file_type;
  vw_out() << "\t--> Writing horizontal disparity debug image: " << h_file << "\n";
  block_write_gdal_image(h_file,
                          channel_cast_rescale<uint8>(horizontal),
                          has_georef, georef,
                          has_nodata, output_nodata,
                          opt, TerminalProgressCallback("asp","\t    H : "));
  std::string v_file = opt.output_prefix + "-V." + opt.output_file_type;
  vw_out() << "\t--> Writing vertical disparity debug image: " << v_file << "\n";
  block_write_gdal_image(v_file,
                          channel_cast_rescale<uint8>(vertical),
                          has_georef, georef,
                          has_nodata, output_nodata,
                          opt, TerminalProgressCallback("asp","\t    V : "));
}

int main(int argc, char *argv[]) {

  Options opt;
  try {
    handle_arguments(argc, argv, opt);

    vw_out() << "Opening " << opt.input_file_name << "\n";
    ImageFormat fmt = vw::image_format(opt.input_file_name);

    switch(fmt.pixel_format) {
    case VW_PIXEL_GENERIC_2_CHANNEL:
      switch (fmt.channel_type) {
      case VW_CHANNEL_INT32:
        process_disparity<Vector2i>(opt); break;
      default:
        process_disparity<Vector2f>(opt); break;
      } break;
    case VW_PIXEL_RGB:
    case VW_PIXEL_GENERIC_3_CHANNEL:
      switch (fmt.channel_type) {
      case VW_CHANNEL_INT32:
        process_disparity<PixelMask<Vector2i>>(opt); break;
      default:
        process_disparity<PixelMask<Vector2f>>(opt); break;
      } break;
    case VW_PIXEL_SCALAR:
      // OpenEXR stores everything as planar, so this allows us to
      // still read that data.
      if (fmt.planes == 2) {
        switch (fmt.channel_type) {
        case VW_CHANNEL_INT32:
          process_disparity<Vector2i>(opt); break;
        default:
          process_disparity<Vector2f>(opt); break;
        } break;
      } else if (fmt.planes == 3) {
        switch (fmt.channel_type) {
        case VW_CHANNEL_INT32:
          process_disparity<PixelMask<Vector2i>>(opt); break;
        default:
          process_disparity<PixelMask<Vector2f>>(opt); break;
        } break;
      }
    default:
      vw_throw(ArgumentErr() << "Unsupported pixel format. Expected 2 or 3 channel image. "
                << "Instead got [" << pixel_format_name(fmt.pixel_format) << "].");
    }

  } ASP_STANDARD_CATCHES;

  return 0;
}

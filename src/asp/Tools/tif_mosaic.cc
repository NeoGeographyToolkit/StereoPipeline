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

/// \file tif_mosaic
///

#include <vw/FileIO.h>
#include <vw/Image.h>
#include <vw/Cartography.h>
#include <vw/Math.h>
using std::endl;
using std::string;

using namespace vw;
using namespace vw::cartography;

#include <asp/Core/AntiAliasing.h>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
using namespace asp;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

std::vector<ImageViewRef<float> > g_map;

struct imgData{
  std::string src_file;
  BBox2 src, dst;
  // Transform from src box to dst box.
  boost::shared_ptr<AffineTransform> T;

  imgData(std::string const& src_file_in, BBox2 const& src_in, BBox2 const& dst_in,
          Matrix2x2 const& matrix, Vector2 const& offset){
    src_file = src_file_in;
    src = src_in;
    dst = dst_in;
    // Must use a pointer as the constructor of AffineTransform is
    // protected.
    T = boost::shared_ptr<AffineTransform>(new AffineTransform(matrix, offset));
  }
};

void parseImgData(std::string data, int& dst_cols, int& dst_rows,
                  std::vector<imgData> & img_data){

  // Extract the tif files to concatenate, their dimensions, and for
  // each of them the location to concatenate to in the output image.
  // The input is comma-separated.

  // Replace commas with spaces.
  std::string oldStr = ",", newStr = " ";
  size_t pos = 0;
  while((pos = data.find(oldStr, pos)) != std::string::npos){
      data.replace(pos, oldStr.length(), newStr);
      pos += newStr.length();
  }
  std::istringstream is(data);

  is >> dst_cols >> dst_rows;

  img_data.clear();
  std::string src_file;
  double src_lenx, src_leny, dst_minx, dst_miny, dst_lenx, dst_leny;
  BBox2 src, dst;
  // Transform from src box to dst box
  while( is >> src_file >> src_lenx >> src_leny >> dst_minx >> dst_miny
         >> dst_lenx >> dst_leny){

    src = BBox2(0,        0,        src_lenx, src_leny);
    dst = BBox2(dst_minx, dst_miny, dst_lenx, dst_leny);

    Vector2 offset = dst.min() - src.min();
    Matrix2x2 matrix;
    matrix(0, 0) = dst.width()/src.width();
    matrix(1, 1) = dst.height()/src.height();

    imgData img(src_file, src, dst, matrix, offset);
    img_data.push_back(img);
  }

  for (int k = (int)img_data.size()-1; k >= 0; k--){

    for (int l = k - 1; l >= 0; l--){

      // Later images will be on top of earlier images. For that
      // reason, reduce each image to the part it does not overlap
      // with later images.
      if (img_data[l].dst.max().y() > img_data[k].dst.min().y()){
        img_data[l].dst.max().y() = img_data[k].dst.min().y();
      }

      // Make sure min of box is <= max of box
      if (img_data[l].dst.min().y() > img_data[l].dst.max().y())
        img_data[l].dst.min().y() = img_data[l].dst.max().y();
    }

    // Adjust the source box as well
    img_data[k].src = img_data[k].T->reverse_bbox(img_data[k].dst);
  }

#if 0
  for (int k = 0; k < (int)img_data.size(); k++){
    std::cout << "boxes: " << img_data[k].src << ' '
              << img_data[k].T->reverse_bbox(img_data[k].dst)
              << ' ' << img_data[k].dst << std::endl;
  }
#endif

}

// A class to mosaic and rescale images using bilinear interpolation.

class tifMosaic: public ImageViewBase<tifMosaic >{
  int m_dst_cols, m_dst_rows;
  std::vector<imgData> m_img_data;
  double m_scale;

public:
  tifMosaic(int dst_cols, int dst_rows, std::vector<imgData> & img_data,
              double scale):
    m_img_data(img_data),
    m_scale(scale){
    // To do: Fix here!!!
    m_dst_cols = (int)(scale*dst_cols);
    m_dst_rows = (int)(scale*dst_rows);
  }

  // Image View interface
  typedef PixelMask<float> m_pixel_type;
  typedef float pixel_type;
  typedef pixel_type result_type;
  typedef ProceduralPixelAccessor<tifMosaic> pixel_accessor;

  inline int32 cols() const { return m_dst_cols; }
  inline int32 rows() const { return m_dst_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor( *this, 0, 0 ); }

  inline pixel_type operator()( double/*i*/, double/*j*/, int32/*p*/ = 0 ) const {
    vw_throw(NoImplErr() << "tifMosaic::operator()(...) is not implemented");
    return pixel_type();
  }

  typedef CropView<ImageView<pixel_type> > prerasterize_type;
  inline prerasterize_type prerasterize(BBox2i const& bbox) const {

    double nodata_val = 0;

    // To do: Fix here!!! Must have consistent nodata!
    std::string file = m_img_data[0].src_file;
    //std::cout << "---- reading: " << file << std::endl;
    DiskImageResourceGDAL in_rsrc(file);
    if ( in_rsrc.has_nodata_read() ) {
      nodata_val = in_rsrc.nodata_read();
      vw_out() << "\tFound input nodata value: " << nodata_val << std::endl;
    }

    // Scaled box
    Vector2i b = floor(bbox.min()/m_scale);
    Vector2i e = ceil((bbox.max() - Vector2(1, 1))/m_scale) + Vector2i(1, 1);
    BBox2i scaled_box(b[0], b[1], e[0] - b[0], e[1] - b[1]);

    // The scaled box can potentially intersect several of the images
    // to be mosaicked. So prepare to interpolate into all of them.
    typedef ImageView<m_pixel_type> ImageT;
    typedef InterpolationView<EdgeExtensionView<ImageT, ConstantEdgeExtension>,
      BilinearInterpolation> InterpT;

    std::vector<BBox2>   src_vec(m_img_data.size());
    std::vector<ImageT>  crop_vec(m_img_data.size());
    for (int k = 0; k < (int)m_img_data.size(); k++){
      BBox2 box = m_img_data[k].dst;
      box.crop(scaled_box);
      box = m_img_data[k].T->reverse_bbox(box);
      box.crop(bounding_box(g_map[k]));
      src_vec[k] = box;
      crop_vec[k] = create_mask(crop(g_map[k], box), nodata_val);
    }

    ImageView<pixel_type> tile(bbox.width(), bbox.height());
    for (int col = bbox.min().x(); col < bbox.max().x(); col++){
      for (int row = bbox.min().y(); row < bbox.max().y(); row++){
        tile(col - bbox.min().x(), row - bbox.min().y()) = nodata_val;
      }
    }

    for (int row = bbox.min().y(); row < bbox.max().y(); row++){
      for (int col = bbox.min().x(); col < bbox.max().x(); col++){

        Vector2 pix = Vector2(col, row)/m_scale; // to do: Fix here!!!!
        // See which src image we end up in.
        int good_k = -1;
        Vector2 src_pix;
        for (int k = 0; k < (int)m_img_data.size(); k++){
          src_pix = m_img_data[k].T->reverse(pix);
          if (src_vec[k].contains(src_pix)){
            good_k = k;
            break;
          }
        }
        //std::cout << "good: " << col << ' ' << row << ' ' << good_k << std::endl;

        if (good_k < 0) continue;

        InterpT interp_masked_img
          = interpolate(crop_vec[good_k], BilinearInterpolation(),
                        ConstantEdgeExtension());

        m_pixel_type r = interp_masked_img(src_pix[0] - src_vec[good_k].min().x(),
                                           src_pix[1] - src_vec[good_k].min().y());
        // m_pixel_type r = interp_masked_img(col/m_scale - scaled_box.min().x(),
        // row/m_scale - scaled_box.min().y());
        if (is_valid(r))
          tile(col - bbox.min().x(), row - bbox.min().y()) = r.child();

        //if (is_valid(p) && is_valid(r)){
        //  std::cout << "pixel is " << p << ' ' << r << ' ' << std::abs(p.child() - r.child()) << std::endl;
        //}

      }
    }

    return prerasterize_type(tile, -bbox.min().x(), -bbox.min().y(),
                             cols(), rows() );
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, BBox2i bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }
};

struct Options : asp::BaseOptions {
  string data, out_prefix;
  double percent;
};

void handle_arguments( int argc, char *argv[], Options& opt ) {
  po::options_description general_options("");
  general_options.add( asp::BaseOptionsDescription(opt) );
  general_options.add_options()
    ("data", po::value(&opt.data), "Information on the images to combine.")
    ("output-prefix,o", po::value(&opt.out_prefix), "Specify the output prefix.")
    ("reduce-percent", po::value(&opt.percent)->default_value(100.0),
     "Reduce the size by this percentage.");

  po::options_description positional("");
  //positional.add_options();
  po::positional_options_description positional_desc;

  // To do: Fix here!!!
  std::string usage("[options]");
  po::variables_map vm =
    asp::check_command_line( argc, argv, opt, general_options, general_options,
                             positional, positional_desc, usage );

}

int main( int argc, char *argv[] ) {

  Options opt;

  try {

    handle_arguments( argc, argv, opt );

    std::string prefix = opt.out_prefix;
    double sub_scale = opt.percent/100.0;

    int dst_cols, dst_rows;
    std::vector<imgData> img_data;
    parseImgData(opt.data, dst_cols, dst_rows, img_data);

    for (int k = 0; k < (int)img_data.size(); k++){
      g_map.push_back(DiskImageView<float>(img_data[k].src_file));
    }

    // Get the nodata value. We assume all files have same nodata
    // value.
    double nodata_val = 0.0;
    for (int k = 0; k < (int)img_data.size(); k++){
      DiskImageResourceGDAL in_rsrc(img_data[k].src_file);
      if ( in_rsrc.has_nodata_read() ) {
        nodata_val = in_rsrc.nodata_read();
      }
    }

    std::ostringstream os;
    os << prefix << ".r" << opt.percent << ".tif";
    std::string out_img = os.str();

    std::cout << "Writing: " << out_img << std::endl;
    asp::block_write_gdal_image(out_img,
                                tifMosaic(dst_cols, dst_rows,
                                          img_data, sub_scale),
                                nodata_val, opt,
                                TerminalProgressCallback("asp", "\t    mosaic:"));

  } ASP_STANDARD_CATCHES;
  return 0;
}

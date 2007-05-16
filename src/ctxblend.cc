// __BEGIN_LICENSE__
// 
// Copyright (C) 2006 United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration
// (NASA).  All Rights Reserved.
// 
// Copyright 2006 Carnegie Mellon University. All rights reserved.
// 
// This software is distributed under the NASA Open Source Agreement
// (NOSA), version 1.3.  The NOSA has been approved by the Open Source
// Initiative.  See the file COPYING at the top of the distribution
// directory tree for the complete NOSA document.
// 
// THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY
// KIND, EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT
// LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO
// SPECIFICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
// A PARTICULAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT
// THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT
// DOCUMENTATION, IF PROVIDED, WILL CONFORM TO THE SUBJECT SOFTWARE.
// 
// __END_LICENSE__
#ifdef _MSC_VER
#pragma warning(disable:4244)
#pragma warning(disable:4267)
#pragma warning(disable:4996)
#endif

#include <string>
#include <fstream>
#include <list>
#include <sys/types.h>
#include <sys/stat.h>

#include <boost/operators.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/fstream.hpp>
namespace fs = boost::filesystem;

#include <vw/Core.h>
#include <vw/Image.h>
#include <vw/FileIO.h>
#include <vw/Mosaic.h>

#include <MRO/DiskImageResourceDDD.h>
using namespace vw;

//  mask_zero_pixels()
//
struct MaskZeroPixelFunc: public vw::ReturnFixedType<vw::PixelGrayA<vw::uint16> > {
  
  vw::PixelGrayA<uint16> operator() (vw::PixelGray<uint16> const& pix) const {
    if (pix.v() == 0) 
      return vw::PixelGrayA<uint16>();  // Mask pixel
    else
      return vw::PixelGrayA<uint16>(pix.v());
  }
};

template <class ViewT>
vw::UnaryPerPixelView<ViewT, MaskZeroPixelFunc> 
mask_zero_pixels(vw::ImageViewBase<ViewT> const& view) {
  return vw::per_pixel_filter(view.impl(), MaskZeroPixelFunc());
}

// *******************************************************************
// normalize()
// *******************************************************************

/// \cond INTERNAL
template <class PixelT>
class RescalePixelsWithAlphaFunc: public UnaryReturnSameType {
  typedef typename CompoundChannelType<PixelT>::type channel_type;
  channel_type m_old_min, m_new_min,m_old_max, m_new_max;
  double m_old_to_new_ratio;
public:
  RescalePixelsWithAlphaFunc( channel_type old_min, channel_type old_max, 
                              channel_type new_min, channel_type new_max )
    : m_old_min(old_min), m_new_min(new_min), m_old_max(old_max), m_new_max(new_max)
  {
    if( old_max == old_min ) { m_old_to_new_ratio = 0.0; }
    else { m_old_to_new_ratio = (new_max - new_min)/(double)(old_max - old_min); }
  }
  
  PixelT operator()( PixelT value ) const {
    PixelT result;
    for (int i = 0; i < CompoundNumChannels<PixelT>::value-1; ++i) {
      result[i] = ((value[i] - m_old_min) * m_old_to_new_ratio + m_new_min);
      // Limit to the min and max values
      if (result[i] > m_new_max) result[i] = m_new_max;
      if (result[i] < m_new_min) result[i] = m_new_min;
    }
    // Copy the alpha value
    result[CompoundNumChannels<PixelT>::value-1] = value[CompoundNumChannels<PixelT>::value-1];
    return result;
  }
};
/// \endcond

/// Renormalize the values in an image to fall within the range [low,high).
template <class ImageT, class ValT>
UnaryPerPixelView<ImageT, RescalePixelsWithAlphaFunc<typename ImageT::pixel_type> >
inline rescale_pixels_with_alpha( ImageViewBase<ImageT> const& image, ValT old_low, ValT old_high, ValT new_low, ValT new_high ) {
  typedef RescalePixelsWithAlphaFunc<typename ImageT::pixel_type> func_type;
  func_type func(old_low, old_high, new_low, new_high );
  return UnaryPerPixelView<ImageT, func_type >( image.impl(), func );
}


template <class PixelT>
void do_blend( std::vector<std::string> image_files, std::string const& mosaic_name, std::string const& file_type, bool draft, bool qtree, int patch_size, int patch_overlap ) {
  vw::mosaic::ImageComposite<PixelT> composite;
    
  if( draft ) composite.set_draft_mode( true );
    
  for( int i = 0; i < image_files.size(); ++i) {
    DiskImageResourceDDD file_resource( image_files[i] );
    
    // Pull the relevent metadata out of the image header
    double fullwidth = atol(file_resource.query("projection_fullwidth").c_str());
    double projection_x_offset = atol(file_resource.query("projection_x_offset").c_str());
    double projection_y_offset = atol(file_resource.query("projection_y_offset").c_str());
    
    std::cout << "Importing image file " << image_files[i] << " at offet (" << projection_x_offset << "," << projection_y_offset << ")" << std::endl;

//     ///
//     uint16 min, max;
//     min_max_channel_values(vw::DiskImageView<uint16>( image_files[i] ), min, max);
//     std::cout << "Min: " << min << "  Max: " << max << "\n";
//     ///

    ImageViewRef<PixelT> input_image = channel_cast<typename PixelChannelType<PixelT>::type>(rescale_pixels_with_alpha(mask_zero_pixels(vw::DiskImageView<uint16>( image_files[i] )), 200, 1100, 0, 255));
    composite.insert( input_image, projection_x_offset, projection_y_offset );
  }
    
  vw::vw_out(vw::InfoMessage) << "Preparing the composite..." << std::endl;
  composite.prepare(TerminalProgressCallback());
  if( qtree ) {
    vw::vw_out(vw::InfoMessage) << "Preparing the quadtree..." << std::endl;
    BBox2 ll_bbox( -30,-30,30.0,30.0*(float)(composite.rows())/(float)(composite.cols()) );
    vw::mosaic::KMLQuadTreeGenerator<PixelT > quadtree( mosaic_name, composite, ll_bbox );
    quadtree.set_output_image_file_type( file_type );
    quadtree.set_patch_size( patch_size );
    quadtree.set_patch_overlap( patch_overlap );
    vw::vw_out(vw::InfoMessage) << "Generating..." << std::endl;
    quadtree.generate(TerminalProgressCallback());
    vw::vw_out(vw::InfoMessage) << "Done!" << std::endl;
  }
  else {
    vw::vw_out(vw::InfoMessage) << "Blending..." << std::endl;
    vw::ImageViewRef<PixelT> im = block_rasterize( composite );
    write_image( mosaic_name+".blend."+file_type, im, TerminalProgressCallback());
    vw::vw_out(vw::InfoMessage) << "Done!" << std::endl;
  }
}

int main( int argc, char *argv[] ) {

  vw::DiskImageResource::register_file_type( ".ddd", vw::DiskImageResourceDDD::type_static(), &vw::DiskImageResourceDDD::construct_open, &vw::DiskImageResourceDDD::construct_create );

  try {
    std::vector<std::string> image_files;
    std::string mosaic_name, file_type;
    int patch_size, patch_overlap;
    unsigned cache_size;
    
    po::options_description general_options("Options");
    general_options.add_options()
      ("help", "Display this help message")
      ("output-name,o", po::value<std::string>(&mosaic_name)->default_value("output"), "Explicitly specify the input directory")
      ("file-type", po::value<std::string>(&file_type)->default_value("png"), "Output file type")
      ("size", po::value<int>(&patch_size)->default_value(256), "Patch size, in pixels")
      ("overlap", po::value<int>(&patch_overlap)->default_value(0), "Patch overlap, in pixels (must be even)")
      ("cache", po::value<unsigned>(&cache_size)->default_value(1024), "Cache size, in megabytes")
      ("draft", "Draft mode (no blending)")
      ("qtree", "Output in quadtree format")
      ("verbose", "Verbose output");
    
    po::options_description hidden_options("");
    hidden_options.add_options()("input-file", po::value<std::vector<std::string> >(&image_files));

    po::options_description options("Allowed Options");
    options.add(general_options).add(hidden_options);

    po::positional_options_description p;
    p.add("input-file", -1);

    po::variables_map vm;
    po::store( po::command_line_parser( argc, argv ).options(options).positional(p).run(), vm );
    po::notify( vm );
    
    std::ostringstream usage;
    usage << "Usage: " << argv[0] << " [options] <filename>..." << std::endl << std::endl;
    usage << general_options << std::endl;

    if( vm.count("help") ) {
      std::cout << usage.str();
      return 1;
    }

    if( vm.count("input-file") < 1 ) {
      std::cout << "Error: Must specify at least one input file!" << std::endl << std::endl;
      std::cout << usage.str();
      return 1;
    }
    
    if( vm.count("verbose") ) {
      vw::set_debug_level(vw::VerboseDebugMessage);
    }

    if( patch_size <= 0 ) {
      std::cerr << "Error: The patch size must be a positive number!  (You specified " << patch_size << ".)" << std::endl;
      std::cout << usage.str() << std::endl;
      return 1;
    }
    
    if( patch_overlap<0 || patch_overlap>=patch_size || patch_overlap%2==1 ) {
      std::cerr << "Error: The patch overlap must be an even number nonnegative number" << std::endl;
      std::cerr << "smaller than the patch size!  (You specified " << patch_overlap << ".)" << std::endl;
      std::cout << usage.str() << std::endl;
      return 1;
    }
    
    vw::Cache::system_cache().resize( cache_size*1024*1024 );

    do_blend<vw::PixelGrayA<uint8> >( image_files, mosaic_name, file_type, vm.count("draft"), vm.count("qtree"), patch_size, patch_overlap );

  }
  catch( std::exception &err ) {
    vw::vw_out(vw::ErrorMessage) << "Error: " << err.what() << std::endl << "Aborting!" << std::endl;
    return 1;
  }
  return 0;
}

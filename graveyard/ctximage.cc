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


/// \file ctximage.cc
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

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <vw/vw.h>
using namespace vw;
using namespace vw::cartography;

#include <asp/Core/DiskImageResourceDDD.h>

/// Erases a file suffix if one exists and returns the base string
static std::string prefix_from_filename(std::string const& filename) {
  std::string result = filename;
  int index = result.rfind(".");
  if (index != -1)
    result.erase(index, result.size());
  return result;
}

//  mask_zero_pixels()
//
struct MaskZeroPixelFunc: public vw::UnaryReturnSameType {

  template <class PixelT>
  PixelT operator() (PixelT const& pix) const {
    if (pix.v() == 0)
      return PixelT();  // Mask pixel
    else
      return PixelT(pix.v());
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


int main( int argc, char *argv[] ) {

  // The DiskImageResourceDDD needs to be registered since it is not a
  // built-in VW FileIO driver.
  vw::DiskImageResource::register_file_type( ".ddd", vw::DiskImageResourceDDD::type_static(), &vw::DiskImageResourceDDD::construct_open, &vw::DiskImageResourceDDD::construct_create );


  std::string input_file_name, output_file_name, index_file_name;
  int debug_level;

  po::options_description desc("Options");
  desc.add_options()
    ("help", "Display this help message")
    ("input-file", po::value<std::string>(&input_file_name), "Explicitly specify the input file")
    ("index-file", po::value<std::string>(&index_file_name)->default_value("none"), "Specify the index file")
    ("output-file,o", po::value<std::string>(&output_file_name)->default_value("none"), "Specify the output file")
    ("debug-level", po::value<int>(&debug_level)->default_value(InfoMessage), "Set the level of debugging output.");
  po::positional_options_description p;
  p.add("input-file", 1);
  p.add("index-file", 2);

  po::variables_map vm;
  po::store( po::command_line_parser( argc, argv ).options(desc).positional(p).run(), vm );
  po::notify( vm );

  if( vm.count("help") ) {
    std::cout << "Usage: " << argv[0] << " <ddd file> <catalog file>\n";
    std::cout << desc << "\n";
    return 1;
  }

  if( vm.count("input-file") != 1 ) {
    std::cout << "Error: Must specify exactly one input file!\n\n";
    std::cout << "Usage: " << argv[0] << " <ddd file> <catalog file>\n";
    std::cout << desc << "\n";
    return 1;
  }

  if( vm.count("index-file") != 1 ) {
    std::cout << "Error: Must specify exactly one index file!\n\n";
    std::cout << "Usage: " << argv[0] << " <ddd file> <catalog file>\n";
    std::cout << desc << "\n";
    return 1;
  }

  if (output_file_name == "none") {
    output_file_name = prefix_from_filename(input_file_name) + ".tif";
  }

  // Set the vision workbench debugging output level.
  set_debug_level(debug_level);

  // The following big 'ol chunk of code fetches the sun incidence
  // angle from the ctx catalog for use in photometric calibration.
  double emission, incidence, phase;
  bool m_do_photometric_calibartion = false;
  if( index_file_name != "none" ) {
    std::cout << "Using index file: " << index_file_name << "\n";
    std::string prefix = prefix_from_filename(input_file_name);

    std::ifstream input(index_file_name.c_str());
    if (!(input.good())) {
      std::cout << "Could not open index file: " << index_file_name << "\nExiting.\n\n";
      exit(1);
    }

    char c_line[2048];
    bool found = false;
    while (!input.eof()) {
      input.getline(c_line, 2048);
      std::string line = c_line;
      if (line.find(prefix) == 0) {
        // Split into several strings using whitespace as delimeter
        std::vector<std::string> split_vec, final_vec;
        boost::split(split_vec, line, boost::is_space());
        // Get rid of empty strings from multiple adjacent whitespaces
        for (unsigned int i=0; i < split_vec.size();++i) {
          if (split_vec[i].size() != 0)
            final_vec.push_back(split_vec[i]);
        }
        if (final_vec.size() != 6) {
          std::cout << "Error parsing line: " << line << "\nExiting.\n\n";
          exit(1);
        }
        emission = atof(final_vec[3].c_str());
        incidence = atof(final_vec[4].c_str());
        phase = atof(final_vec[5].c_str());
        std::cout << "Photometric Values -- Emission: " << emission << "  Incidence: " << incidence << "  Phase: " << phase << "\n";
        found = true;
        break;
      }
    }
    if (!found) {
      std::cout << "Could not find index entry for image: " << prefix << "\nExiting.\n\n";
      exit(1);
    }
    input.close();

    m_do_photometric_calibartion = true;
  }

  try {
    ImageView<PixelGrayA<uint16> > disk_image;
    read_image(disk_image, input_file_name);

    // Use the grassfire algorithm to force the black border
    // information to be transparent.
    select_channel(disk_image,1) = clamp(grassfire(select_channel(disk_image,0)),0,1) * 65535;

    // Fetch parameters from the CTX header
    DiskImageResourceDDD file_resource( input_file_name );
    double fullwidth = atol(file_resource.query("projection_fullwidth").c_str());
    double projection_x_offset = atol(file_resource.query("projection_x_offset").c_str());
    double projection_y_offset = atol(file_resource.query("projection_y_offset").c_str());
    double exposure = atof(file_resource.query("exposure").c_str());

    // Set up a georefernce object so that we can embed this
    // information in the file.  Note that we choose a "dummy"
    // mercator projection here simply so that we put something in the
    // geotiff header.  The choice of projection doesn't matter in the
    // end because we pass these images along to geoblend, which only
    // uses the affine transform information anyway.
    GeoReference georef;
    georef.set_mercator(0,0,1); // dummy projection
    Matrix3x3 geotransform = math::identity_matrix<3>();
    geotransform(0,2) = projection_x_offset;
    geotransform(1,2) = projection_y_offset;
    georef.set_transform(geotransform);

    if (m_do_photometric_calibartion) {
      float sun_coeff = cos(incidence*M_PI/180);

      // These normalization factors were chosen based on sampling
      // some representative light and dark images.  They produced
      // reasonable results for this particular mosaic, but there is
      // probably a more principled way to choose these.
      float norm_low = 5.3;
      float norm_high = 7.8;

      if (sun_coeff < 0.1) sun_coeff = 0.1; // Prevent division by a
                                            // number close to zero
                                            // for very larg incidence
                                            // angles
      ImageViewRef<PixelGrayA<uint8> > corrected_image = channel_cast<uint8>(normalize(clamp(log(1+(disk_image / (exposure*sun_coeff))),norm_low,norm_high),norm_low,norm_high,0.0,255.0));
      DiskImageResourceGDAL r = DiskImageResourceGDAL( output_file_name, corrected_image.format(), Vector2i(vw_settings().default_tile_size(),vw_settings().default_tile_size()) );
      write_georeference( r, georef );
      write_image(r, corrected_image, TerminalProgressCallback() );
    } else {

      // See comment above about choosing these normalization factors.
      float norm_low = 6;
      float norm_high = 8;

      ImageViewRef<PixelGrayA<uint8> > corrected_image = channel_cast<uint8>(normalize(clamp(log(disk_image/exposure),norm_low,norm_high),norm_low,norm_high,0.0,255.0));
      DiskImageResourceGDAL r = DiskImageResourceGDAL( output_file_name, corrected_image.format(), Vector2i(vw_settings().default_tile_size(),vw_settings().default_tile_size()) );
      write_georeference( r, georef );
      write_image(r, corrected_image, TerminalProgressCallback() );
    }
  }
  catch( Exception& e ) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  return 0;
}

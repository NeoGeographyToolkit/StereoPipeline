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


/// \file stereo_parse.cc
///
/// This program is to allow python access to stereo settings.

#include <asp/Tools/stereo.h>
#include <vw/Stereo/DisparityMap.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <asp/Sessions/StereoSession.h>
#include <asp/Sessions/StereoSessionFactory.h>
#include <xercesc/util/PlatformUtils.hpp>

using namespace vw;
using namespace asp;
using namespace std;
namespace fs = boost::filesystem;

int main( int argc, char* argv[] ) {

  try {
    xercesc::XMLPlatformUtils::Initialize();
    stereo_register_sessions();

    // Below, TriangulationDescription() would work just as well
    // as anything else. Just need to pass something.
    bool verbose = true;
    vector<ASPGlobalOptions> opt_vec;
    string output_prefix;
    asp::parse_multiview(argc, argv, TriangulationDescription(),
                         verbose, output_prefix, opt_vec);
    if (opt_vec.empty())
      return 1;

    ASPGlobalOptions opt = opt_vec[0];

    vw_out() << "in_file1,"        << opt.in_file1        << endl;
    vw_out() << "in_file2,"        << opt.in_file2        << endl;
    vw_out() << "cam_file1,"       << opt.cam_file1       << endl;
    vw_out() << "cam_file2,"       << opt.cam_file2       << endl;
    vw_out() << "input_dem,"       << opt.input_dem       << endl;
    vw_out() << "extra_argument1," << opt.extra_argument1 << endl;
    vw_out() << "extra_argument2," << opt.extra_argument2 << endl;
    vw_out() << "extra_argument3," << opt.extra_argument3 << endl;

    vw_out() << "stereo_session_string,"   << opt.stereo_session_string         << endl;
    vw_out() << "stereo_default_filename," << opt.stereo_default_filename       << endl;
    vw_out() << "left_image_crop_win,"     << stereo_settings().left_image_crop_win.min().x() << ","
             << stereo_settings().left_image_crop_win.min().y() << ","
             << stereo_settings().left_image_crop_win.width()   << ","
             << stereo_settings().left_image_crop_win.height()  << endl;

    vw_out() << "out_prefix," << output_prefix << endl;

    Vector2i left_image_size  = file_image_size(opt.in_file1),
             right_image_size = file_image_size(opt.in_file2);
    vw_out() << "left_image_size,"  << left_image_size.x()  << "," << left_image_size.y()  << endl;
    vw_out() << "right_image_size," << right_image_size.x() << "," << right_image_size.y() << endl;

    string trans_left_image  = opt.out_prefix+"-L.tif";
    string trans_right_image = opt.out_prefix+"-R.tif";
    vw_out() << "trans_left_image,"  << trans_left_image  << endl;
    vw_out() << "trans_right_image," << trans_right_image << endl;
    Vector2 trans_left_image_size;
    if ( fs::exists(trans_left_image) )
      trans_left_image_size = file_image_size(trans_left_image);
    vw_out() << "trans_left_image_size," << trans_left_image_size.x() << "," << trans_left_image_size.y() << endl;

    cartography::GeoReference georef = opt.session->get_georef();
    vw_out() << "WKT--non-comma-separator--" << georef.get_wkt() << std::endl;

    // Write the geotransform as a string as expected by GDAL's vrt xml format
    // TODO: Not sure if this will be useful as a member function in GeoReference.
    Matrix3x3 T = georef.transform();
    std::ostringstream os;
    os.precision(18);
    os << " "  << T(0, 2) << ",  " << T(0, 0) << ",  " << T(0, 1) << ",  "
       << " "  << T(1, 2) << ",  " << T(1, 0) << ",  " << T(1, 1);
    vw_out() << "GeoTransform--non-comma-separator--" << os.str() << std::endl;

    // Some care is needed below. The transformed_window will be used
    // by parallel_stereo to parallelize stereo on a given user-specified
    // region. If both --left-image-crop-win and --right-image-crop-win
    // is specified, we already chopped the input images to these windows,
    // and created L.tif. Hence, we will work on the bd box on L.tif. However,
    // if just left-image-crop-win was set, we still use the full images,
    // but just the domain of computation is restricted. Hence we take user's
    // crop window, transform it to be in the L.tif coordinates, and use that one.
    bool crop_left = (stereo_settings().left_image_crop_win  != BBox2i(0, 0, 0, 0));
    BBox2i transformed_window;
    if (crop_left) {
      transformed_window.min() = Vector2(0, 0);
      transformed_window.max() = trans_left_image_size;
    }else{
      transformed_window = transformed_crop_win(opt);
    }
    vw_out() << "transformed_window," << transformed_window.min().x() << ","
             << transformed_window.min().y() << ","
             << transformed_window.width()   << ","
             << transformed_window.height()  << endl;

    //vw_out() << "corr_tile_size," << ASPGlobalOptions::corr_tile_size() << endl;
    vw_out() << "corr_tile_size," << stereo_settings().corr_tile_size_ovr << endl;
    vw_out() << "rfne_tile_size," << ASPGlobalOptions::rfne_tile_size() << endl;
    vw_out() << "tri_tile_size,"  << ASPGlobalOptions::tri_tile_size()  << endl;

    vw_out() << "stereo_algorithm," << stereo_settings().stereo_algorithm << endl;
    if (stereo_settings().stereo_algorithm == 0)
      vw_out() << "collar_size," << 0 << endl;
    else
      vw_out() << "collar_size," << stereo_settings().sgm_collar_size << endl;

    // This block of code should be in its own executable but I am
    // reluctant to create one just for it. This functionality will be
    // invoked after low-res disparity is computed, whether done in
    // C++ or in Python. It will attach a georeference to this disparity.
    std::string left_image_file = opt.out_prefix + "-L.tif";
    if (stereo_settings().attach_georeference_to_lowres_disparity &&
        fs::exists(left_image_file) ) {

      cartography::GeoReference left_georef, left_sub_georef;
      bool   has_left_georef = read_georeference(left_georef, left_image_file);
      bool   has_nodata      = false;
      double output_nodata   = -32768.0;
      if (has_left_georef) {

        DiskImageView<float> left_image(left_image_file);
        for (int i = 0; i < 2; i++) {
          std::string d_sub_file = opt.out_prefix + "-D_sub.tif";
          if (i == 1) d_sub_file = opt.out_prefix + "-D_sub_spread.tif";
          if (!fs::exists(d_sub_file)) 
            continue;

          bool has_sub_georef = read_georeference(left_sub_georef, d_sub_file);
          if (has_sub_georef) {
            // If D_sub already has a georef, as with seed-mode 3, don't overwrite it.
            continue;
          }

          ImageView<PixelMask<Vector2f> > d_sub;
          read_image(d_sub, d_sub_file);
          // Account for scale.
          double left_scale = 0.5*( double(d_sub.cols())/left_image.cols() +
                                    double(d_sub.rows())/left_image.rows());
          left_sub_georef = resample(left_georef, left_scale);
          vw::cartography::block_write_gdal_image(d_sub_file, d_sub,
                                      has_left_georef, left_sub_georef,
                                      has_nodata, output_nodata,
                                      opt, TerminalProgressCallback("asp", "\t    D_sub: ")
                                      );
        } // End i loop
      } // End has_left_georef
    } // End georef attach ?

    xercesc::XMLPlatformUtils::Terminate();
  } ASP_STANDARD_CATCHES;

  return 0;
}

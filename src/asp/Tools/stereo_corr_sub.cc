// __BEGIN_LICENSE__
//  Copyright (c) 2009-2012, United States Government as represented by the
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

#include <asp/Tools/stereo.h>
#include <vw/InterestPoint.h>

using namespace vw;
using namespace asp;

void stereo_correlation_sub( Options& opt ) {

  vw_out() << "\n[ " << current_posix_time_string()
           << " ] : Stage 1 --> PRE CORRELATION \n";

  // Working out search range if need be
  if (stereo_settings().is_search_defined()) {
    vw_out() << "\t--> Using user defined search range.\n";
  } else {

    std::string match_filename
      = ip::match_filename(opt.out_prefix, opt.in_file1, opt.in_file2);

    if (!fs::exists(match_filename)) {
      // If there is not any match files for the input image. Let's
      // gather some IP quickly from the low resolution images. This
      // routine should only run for:
      //   Pinhole + Epipolar
      //   Pinhole + None
      //   DG + None
      // Everything else should gather IP's all the time.
      float sub_scale =
        sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-L_sub.tif" )),
                       Vector2f(file_image_size( opt.out_prefix+"-L.tif" ) ) )) +
        sum(elem_quot( Vector2f(file_image_size( opt.out_prefix+"-R_sub.tif" )),
                       Vector2f(file_image_size( opt.out_prefix+"-R.tif" ) ) ));
      sub_scale /= 4.0f;

      stereo_settings().search_range =
        approximate_search_range( opt.out_prefix+"-L_sub.tif",
                                  opt.out_prefix+"-R_sub.tif",
                                  ip::match_filename( opt.out_prefix,
                                                      opt.out_prefix+"-L_sub.tif",
                                                      opt.out_prefix+"-R_sub.tif"),
                                  sub_scale );
    } else {
      // There exists a matchfile out there.
      std::vector<ip::InterestPoint> ip1, ip2;
      ip::read_binary_match_file( match_filename, ip1, ip2 );

      Matrix<double> align_matrix = math::identity_matrix<3>();
      if ( fs::exists( opt.out_prefix+"-align.exr" ) )
        read_matrix(align_matrix, opt.out_prefix + "-align.exr");

      BBox2 search_range;
      for ( size_t i = 0; i < ip1.size(); i++ ) {
        Vector3 r = align_matrix * Vector3(ip2[i].x,ip2[i].y,1);
        r /= r[2];
        search_range.grow( subvector(r,0,2) - Vector2(ip1[i].x,ip1[i].y) );
      }
      stereo_settings().search_range = grow_bbox_to_int( search_range );
    }
    vw_out() << "\t--> Detected search range: " << stereo_settings().search_range << "\n";
  }

  DiskImageView<vw::uint8> Lmask(opt.out_prefix + "-lMask.tif");

  produce_lowres_disparity( Lmask.cols(), Lmask.rows(), opt );
}

int main(int argc, char* argv[]) {

  stereo_register_sessions();
  Options opt;
  try {
    handle_arguments( argc, argv, opt,
                      CorrelationDescription() );

    // Integer correlator requires 1024 px tiles
    //---------------------------------------------------------
    opt.raster_tile_size = Vector2i(1024,1024);

    // Internal Processes
    //---------------------------------------------------------
    stereo_correlation_sub( opt );

    vw_out() << "\n[ " << current_posix_time_string()
             << " ] : PRE CORRELATION FINISHED \n";

  } ASP_STANDARD_CATCHES;

  return 0;
}

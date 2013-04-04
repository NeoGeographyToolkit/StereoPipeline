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


/// \file LocalHomography.cc
///

#include <vw/Image/ImageView.h>
#include <vw/Image/Transform.h>
#include <vw/Core/ThreadPool.h>
#include <vw/FileIO/DiskImageView.h>
#include <vw/Stereo/DisparityMap.h>
#include <asp/Core/LocalHomography.h>
#include <asp/Core/StereoSettings.h>
#include <asp/Core/InterestPointMatching.h>

using namespace vw;

namespace asp {

  // We would like to split the numbers 0, ..., n - 1 into k buckets
  // of approximately equal size. For example, for n = 8 and k = 3,
  // we will have the split {0, 1, 2}, {3, 4, 5}, {6, 7}.

  void split_n_into_k(int n, int k, std::vector<int> & partition){

    VW_ASSERT(n >= k && k > 0,
              ArgumentErr() << "split_n_into_k: Must have n >= k && k > 0.\n");
    int rem = n % k;
    int dx0 = n / k;

    partition.clear();
    int start = 0;
    for (int i = 0; i < k; i++){
      int dx = dx0;
      if (rem > 0){
        dx++;
        rem--;
      }
      partition.push_back(start);
      start += dx;
    }
    partition.push_back(start);

  }

  // Task that computes local homography in a given tile
  class LocalHomTask: public vw::Task, private boost::noncopyable {

    int m_col, m_row;
    BBox2i m_sub_bbox;
    // We on purpose store a local deep copy of sub_disparity_crop to
    // avoid segfault.
    ImageView < PixelMask<Vector2i> > m_sub_disparity_crop;
    ImageView<Matrix3x3> & m_local_hom;
  public:
    LocalHomTask(int col, int row,
                 BBox2i sub_bbox,
                 ImageViewRef < PixelMask<Vector2i> > const& sub_disparity_crop,
                 ImageView<Matrix3x3> & local_hom):
      m_col(col), m_row(row), m_sub_bbox(sub_bbox),
      m_sub_disparity_crop(sub_disparity_crop),
      m_local_hom(local_hom){}

    void operator()() {
      m_local_hom(m_col, m_row)
        = homography_for_disparity(m_sub_bbox, m_sub_disparity_crop);
    }
  };

  // Create a local homography for each correlation tile
  void create_local_homographies(Options const& opt){

    DiskImageView< PixelGray<float> > left_sub (opt.out_prefix + "-L_sub.tif");
    DiskImageView< PixelGray<float> > left_img (opt.out_prefix + "-L.tif");
    DiskImageView< PixelMask<Vector2i> >
      sub_disparity(opt.out_prefix + "-D_sub.tif");

    Vector2 upscale_factor( double(left_img.cols()) / double(left_sub.cols()),
                            double(left_img.rows()) / double(left_sub.rows()) );

    int ts = Options::corr_tile_size();
    int cols = (int)ceil(left_img.cols()/double(ts));
    int rows = (int)ceil(left_img.rows()/double(ts));
    ImageView<Matrix3x3> local_hom(cols, rows);

    // Calculate the local homographies using multiple threads.
    // To do: Each thread calls RANSAC, which calls rand(), which
    // is not thread safe. So for now use just one thread.

    Stopwatch sw;
    sw.start();

    //FifoWorkQueue queue( vw_settings().default_num_threads() );
    for (int col = 0; col < cols; col++){
      for (int row = 0; row < rows; row++){

        BBox2i bbox(col*ts, row*ts, ts, ts);
        bbox.crop(bounding_box(left_img));

        // The low-res version of bbox
        BBox2i sub_bbox( elem_quot(bbox.min(), upscale_factor),
                          elem_quot(bbox.max(), upscale_factor) );

        // Expand the box until square to make sure the local homography
        // calculation does not fail.
        int len = std::max(sub_bbox.width(), sub_bbox.height());
        sub_bbox = BBox2i(sub_bbox.max() - Vector2(len, len), sub_bbox.max());
        sub_bbox.expand(1);
        sub_bbox.crop( bounding_box(sub_disparity) );

        //boost::shared_ptr<LocalHomTask>
        //task(new LocalHomTask(col, row, sub_bbox, crop(sub_disparity, sub_bbox),
        //                        local_hom));
        //queue.add_task(task);
        local_hom(col, row)
          = homography_for_disparity(sub_bbox, crop(sub_disparity, sub_bbox));
      }
    }
    //queue.join_all();

    sw.stop();
    vw_out(DebugMessage,"asp") << "Local homographies elapsed time: "
                               << sw.elapsed_seconds() << std::endl;

    std::string local_hom_file = opt.out_prefix + "-local_hom.txt";
    vw_out() << "Writing: " << local_hom_file << "\n";
    write_local_homographies(local_hom_file, local_hom);

    return;
  }

  void write_local_homographies(std::string const& local_hom_file,
                                ImageView<Matrix3x3> const& local_hom){

    std::ofstream fh(local_hom_file.c_str());
    fh.precision(18);
    fh << local_hom.cols() << " " << local_hom.rows() << std::endl;

    for (int col = 0; col < local_hom.cols(); col++){
      for (int row = 0; row < local_hom.rows(); row++){
        Vector<double> V = matrix_to_vector(local_hom(col, row));
        for (int t = 0; t < int(V.size())-1; t++) fh << V[t] << " ";
        if (V.size() > 0) fh << V[V.size()-1] << std::endl;
      }
    }
    fh.close();

    return;
  }

  void read_local_homographies(std::string const& local_hom_file,
                               ImageView<Matrix3x3> & local_hom){

    std::ifstream fh(local_hom_file.c_str());
    if (!fh.good())
      vw_throw( IOErr() << "read_local_homographies: File does not exist: "
                << local_hom_file << ".\n" );

    int cols, rows;
    if ( !(fh >> cols >> rows) )
      vw_throw( IOErr() << "read_local_homographies: Invalid file: "
                << local_hom_file << ".\n" );

    local_hom.set_size(cols, rows);
    for (int col = 0; col < local_hom.cols(); col++){
      for (int row = 0; row < local_hom.rows(); row++){

        Vector<double, 9> V;
        for (int t = 0; t < int(V.size()); t++){
          if (! (fh >> V[t]) )
            vw_throw( IOErr() << "read_local_homographies: Invalid file: "
                      << local_hom_file << ".\n" );
        }

        local_hom(col, row) = vector_to_matrix(V);
      }
    }
    fh.close();

    return;
  }

} // namespace asp

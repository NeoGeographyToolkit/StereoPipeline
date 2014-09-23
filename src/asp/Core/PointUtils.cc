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


/// \file PointUtils.cc
///

#include <liblas/liblas.hpp>
#include <asp/Core/Macros.h>
#include <asp/Core/Common.h>
#include <asp/Core/PointUtils.h>
#include <vw/Cartography/Chipper.h>

using namespace vw;
using namespace vw::cartography;
using namespace pdal::filters;

// Allows FileIO to correctly read/write these pixel types
namespace vw {
  template<> struct PixelFormatID<Vector3>   { static const PixelFormatEnum value = VW_PIXEL_GENERIC_3_CHANNEL; };
}

// Create a point cloud image from a las file. The image will be
// created block by block, when it needs to be written to disk. It is
// important that the writer invoking this image be single-threaded,
// as we read from the las file sequentially.

template <class ImageT>
class Las2Tif:
  public ImageViewBase< Las2Tif<ImageT> > {
  typedef typename ImageT::pixel_type PixelT;
  liblas::Reader& m_reader;
  int m_rows, m_cols;
  int m_block_size;
  bool m_have_georef;
  GeoReference m_georef;

public:

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef ProceduralPixelAccessor<Las2Tif> pixel_accessor;
  
  Las2Tif(liblas::Reader & reader, int num_rows, int block_size):
    m_reader(reader),
    m_rows( block_size*std::max(1, (int)ceil(double(num_rows)/block_size)) ),
    m_block_size(block_size){
    
    liblas::Header const& header = m_reader.GetHeader();
    
    std::string wkt = header.GetSRS().GetWKT();
    m_have_georef = false;
    if (wkt != ""){
      m_have_georef = true;
      m_georef.set_wkt(wkt);
    }

    int num_points = header.GetPointRecordsCount();
    m_cols = (int)ceil(double(num_points)/m_rows);
    m_cols = m_block_size*std::max(1, (int)ceil(double(m_cols)/m_block_size));
    
  }

  inline int32 cols() const { return m_cols; }
  inline int32 rows() const { return m_rows; }
  inline int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()( size_t i, size_t j, size_t p=0 ) const {

    vw_throw( NoImplErr() << "Las2Tif::operator(...) has not been implemented.\n");
    return result_type();
  }

  typedef CropView<ImageView<PixelT> > prerasterize_type;
  inline prerasterize_type prerasterize( BBox2i const& bbox ) const{

    // Read a chunk of the las file, and store it in the current tile.
    
    int num_cols = bbox.width();
    int num_rows = bbox.height();
    
    VW_ASSERT(num_rows % m_block_size == 0 && num_cols % m_block_size == 0,
              ArgumentErr() << "Expecting the number of rows "
              << "to be a multiple of the block size.\n");
    
    int num_pts = num_cols*num_rows;
    int count = 0;
    PointBuffer in(num_pts); in.clear();
    std::vector<PointBuffer> outVec;
    while (m_reader.ReadNextPoint()){
      liblas::Point const& p = m_reader.GetPoint();
      in.push_back(Vector3(p.GetX(), p.GetY(), p.GetZ()));
      count++;
      if (count >= num_pts) break;
    }
    num_pts = in.size();
    
    ImageView<Vector3> Img(num_cols, num_rows);
    Chipper(in, m_block_size, m_have_georef, m_georef, Img);

    return crop( Img, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
    
  }
  
  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  
};

std::string asp::read_las_file(std::string const& file, int num_rows, int block_size){
    
  std::ifstream ifs;
  ifs.open(file.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  // We will fetch a chunk of the las file of area tile_size, split it
  // into bins of spatially close points, and write it to disk.
  // the bigger the tile size, the more likely the binning will be
  // more efficient.
  Vector2 tile_size(2048, 2048);
  int num_threads = 1; // Must use a thread only, as we read the las file serially.
  ImageViewRef<Vector3> Img = block_cache
    (Las2Tif< ImageView<Vector3> > (reader, num_rows, block_size),
     tile_size, num_threads);

  // Writing with single-thread
  std::string outfile = "pc.tif";
  asp::BaseOptions opt;
  std::cout << "--Writing: " << outfile << std::endl;
  vw::write_image(outfile, Img, TerminalProgressCallback
                              ("asp", "\t--> Good Pxl Map: ") );
  return outfile;
}
  

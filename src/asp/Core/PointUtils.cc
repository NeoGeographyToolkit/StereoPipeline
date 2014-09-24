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
  
  Las2Tif(liblas::Reader & reader, int num_rows, int tile_len, int block_size):
    m_reader(reader), m_block_size(block_size){
    
    liblas::Header const& header = m_reader.GetHeader();
    int num_points = header.GetPointRecordsCount();
    m_rows = tile_len*std::max(1, (int)ceil(double(num_rows)/tile_len));
    m_cols = (int)ceil(double(num_points)/m_rows);
    m_cols = tile_len*std::max(1, (int)ceil(double(m_cols)/tile_len));
    
    std::string wkt = header.GetSRS().GetWKT();
    m_have_georef = false;
    if (wkt != ""){
      m_have_georef = true;
      m_georef.set_wkt(wkt);
    }

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
              ArgumentErr() << "Las2Tif: Expecting the number of rows "
              << "to be a multiple of the block size.\n");
    
    int max_num_pts_to_read = num_cols*num_rows;
    int count = 0;
    PointBuffer in;
    std::vector<PointBuffer> outVec;
    while (m_reader.ReadNextPoint()){
      liblas::Point const& p = m_reader.GetPoint();
      in.push_back(Vector3(p.GetX(), p.GetY(), p.GetZ()));
      count++;
      if (count >= max_num_pts_to_read) break;
    }
    
    ImageView<Vector3> Img;
    Chipper(in, m_block_size, m_have_georef, m_georef, num_cols, num_rows, Img);
    
    VW_ASSERT(num_cols == Img.cols() && num_rows == Img.rows(),
              ArgumentErr() << "Las2Tif: Size mis-match.\n");
    
    return crop( Img, -bbox.min().x(), -bbox.min().y(), cols(), rows() );
    
  }
  
  template <class DestT>
  inline void rasterize( DestT const& dest, BBox2i const& bbox ) const {
    vw::rasterize( prerasterize(bbox), dest, bbox );
  }
  
};

void asp::las_to_tif(std::string const& las_file,
                     std::string const& pc_file,
                     int num_rows, int block_size){
  
  // We will fetch a chunk of the las file of area tile_len x
  // tile_len, split it into bins of spatially close points, and write
  // it to disk as a tile in a vector tif image. The bigger the tile
  // size, the more likely the binning will be more efficient. But big
  // tiles use a lot of memory.
  int tile_len = 2048;
  Vector2 tile_size(tile_len, tile_len);
  
  std::ifstream ifs;
  ifs.open(las_file.c_str(), std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  // Must use a thread only, as we read the las file serially.
  int num_threads = 1;
  ImageViewRef<Vector3> Img
    = Las2Tif< ImageView<Vector3> > (reader, num_rows, tile_len, block_size);
  
  asp::BaseOptions opt;
  opt.gdal_options["COMPRESS"] = "LZW";  
  opt.raster_tile_size = tile_size; // Force tiles of this size
  vw_out() << "Writing temporary file: " << pc_file << std::endl;
  // This writer will use a single thread only
  asp::write_gdal_image(pc_file, Img, opt, TerminalProgressCallback("asp", "\t--> ") );
}
  
bool asp::read_user_datum(double semi_major, double semi_minor,
                          std::string const& reference_spheroid,
                          cartography::Datum& datum ) {
  // Select a cartographic datum. There are several hard coded datums
  // that can be used here, or the user can specify their own.
  if ( reference_spheroid != "" ) {
    if (reference_spheroid == "mars") {
      datum.set_well_known_datum("D_MARS");
      vw_out() << "\t--> Re-referencing altitude values using a Mars spheroid\n";
    } else if (reference_spheroid == "moon") {
      datum.set_well_known_datum("D_MOON");
      vw_out() << "\t--> Re-referencing altitude values using a Lunar spheroid\n";
    } else if (reference_spheroid == "earth") {
      vw_out() << "\t--> Re-referencing altitude values using the Earth WGS84 spheroid\n";
    } else {
      vw_throw( ArgumentErr() << "\t--> Unknown reference spheriod: "
                << reference_spheroid
                << ". Current options are [ earth, moon, mars ]\nExiting." );
    }
    vw_out() << "\t    Axes [" << datum.semi_major_axis() << " " << datum.semi_minor_axis() << "] meters\n";
  } else if (semi_major != 0 && semi_minor != 0) {
    vw_out() << "\t--> Re-referencing altitude values to user supplied datum.\n"
             << "\t    Semi-major: " << semi_major << "  Semi-minor: " << semi_minor << "\n";
    datum = cartography::Datum("User Specified Datum",
                               "User Specified Spheroid",
                               "Reference Meridian",
                               semi_major, semi_minor, 0.0);
  } else {
    return false;
  }
  return true;
}
  
// Erases a file suffix if one exists and returns the base string
std::string asp::prefix_from_pointcloud_filename(std::string const& filename) {
  std::string result = filename;
  
  // First case: filenames that match <prefix>-PC.<suffix>
  int index = result.rfind("-PC.");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }
    
  // Second case: filenames that match <prefix>.<suffix>
  index = result.rfind(".");
  if (index != -1) {
    result.erase(index, result.size());
    return result;
  }

  // No match
  return result;
}


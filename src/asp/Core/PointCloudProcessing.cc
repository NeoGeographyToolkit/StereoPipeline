// __BEGIN_LICENSE__
//  Copyright (c) 2009-2025, United States Government as represented by the
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

// Process point clouds. This makes use of PDAL and ASP's CSV logic.

/// \file PointCloudProcessing.cc
#include <asp/Core/PointCloudProcessing.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/PdalUtils.h>

#include <vw/Cartography/Chipper.h>

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/Reader.hpp>
#include <io/LasHeader.hpp>
#include <io/CopcReader.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/SrsBounds.hpp>

// Read through las points in streaming fashion. When a given amount is collected,
// write a chip to disk.
namespace asp {

class PDAL_DLL ChipMaker: public pdal::Writer, public pdal::Streamable {

public:

std::string getName() const { return "chip maker"; }

  // Go through a LAS file and write to disk spatially organized tiles.
  // Also converts along the way from projected coordinates (if applicable)
  // to ECEF.
  // tile_len is big, but chip_size is small.
  ChipMaker(std::int64_t tile_len, std::int64_t chip_size,
            bool has_georef, vw::cartography::GeoReference const& georef,
            vw::GdalWriteOptions & opt, // will change
            std::string const& out_prefix,
            std::vector<std::string> & out_files):
    m_tile_len(tile_len), m_chip_size(chip_size),
    m_has_georef(has_georef), m_georef(georef), m_opt(opt),
    m_out_prefix(out_prefix), m_out_files(out_files),
    m_tile_count(0), m_buf(vw::PointBuffer()) {}

~ChipMaker() {}

private:

  std::int64_t m_tile_len;
  std::int64_t m_chip_size;
  bool m_has_georef;
  vw::cartography::GeoReference m_georef;
  vw::GdalWriteOptions & m_opt;
  std::string m_out_prefix; // output files will start with this prefix
  std::vector<std::string> & m_out_files; // alias, used for output
  std::int64_t m_tile_count;
  vw::PointBuffer m_buf;

  // Call this when the buffer is full or when we are done reading.
  // Organize the points in small chips, with points in each chip
  // being spatially close together. Then write each chip to disk.
  void processBuf() {

    if (m_buf.empty())
      return;

    // Make the chips
    vw::ImageView<vw::Vector3> Img;
    vw::Chipper(m_buf, m_chip_size, m_has_georef, m_georef,
                m_tile_len, m_tile_len, Img);

    // Create a file of the form tile with index m_tile_count
    std::ostringstream os;
    os << m_out_prefix << "-" << m_tile_count << ".tif";
    std::string out_file = os.str();
    m_out_files.push_back(out_file);

    vw::vw_out() << "Writing temporary file: " << out_file << "\n";
    bool has_nodata = false;
    double nodata = -std::numeric_limits<double>::max();
    vw::TerminalProgressCallback tpc("asp", "\t--> ");
    vw::cartography::block_write_gdal_image(out_file, Img, m_has_georef, m_georef,
                                            has_nodata, nodata, m_opt, tpc);

    // Wipe the buf when done and increment the tile count
    m_buf.clear();
    m_tile_count++;
  }

  // This will be called for each point in the cloud.
  virtual bool processOne(pdal::PointRef& point) {

    // Current point
    vw::Vector3 pt(point.getFieldAs<double>(pdal::Dimension::Id::X),
                   point.getFieldAs<double>(pdal::Dimension::Id::Y),
                   point.getFieldAs<double>(pdal::Dimension::Id::Z));
    m_buf.push_back(pt);
    // If the buffer is full, process it
    std::int64_t max_num_pts_to_read = m_tile_len * m_tile_len;
    if (m_buf.size() >= max_num_pts_to_read)
      processBuf();

    return true;
  }

  // To be called after all the points are read.
  virtual void done(pdal::PointTableRef table) {
    // Process the rest of the points
    processBuf();
  }

  // Part of the API, not used here.
  virtual void writeView(const pdal::PointViewPtr view) {
    throw pdal::pdal_error("The writeView() function must not be called in streaming mode.");
  }
  virtual void addArgs(pdal::ProgramArgs& args) {}
  virtual void initialize() {}
  ChipMaker& operator=(const ChipMaker&) = delete;
  ChipMaker(const ChipMaker&) = delete;
  ChipMaker(const ChipMaker&&) = delete;

};

class CsvReader: public BaseReader {
  std::string  m_csv_file;
  asp::CsvConv m_csv_conv;
  bool         m_is_first_line;
  bool         m_has_valid_point;
  vw::Vector3  m_curr_point;
  std::ifstream * m_ifs;

public:

  CsvReader(std::string const & csv_file,
            asp::CsvConv const& csv_conv,
            vw::cartography::GeoReference const& georef):
      m_csv_file(csv_file), m_csv_conv(csv_conv),
      m_is_first_line(true), m_has_valid_point(false) {

    // We will convert from projected space to xyz, unless points
    // are already in this format.
    m_has_georef = (m_csv_conv.format != asp::CsvConv::XYZ);

    m_georef      = georef;
    m_num_points  = asp::csv_file_size(m_csv_file);

    m_ifs = new std::ifstream (m_csv_file.c_str());
    if (!*m_ifs)
      vw::vw_throw(vw::IOErr() << "Unable to open file: " << m_csv_file << "\n");

    VW_ASSERT(m_csv_conv.csv_format_str != "",
              vw::ArgumentErr() << "CsvReader: The CSV format was not specified.\n");
  }

  virtual bool ReadNextPoint() {

    std::string line;
    asp::CsvConv::CsvRecord vals;

    // Keep on reading, until a valid point is hit or the end of the file
    // is reached.
    while (1) {
      m_has_valid_point = static_cast<bool>(getline(*m_ifs, line, '\n'));
      if (!m_has_valid_point)
        return m_has_valid_point; // reached end of file

      vals = m_csv_conv.parse_csv_line(m_is_first_line, m_has_valid_point, line);

      // Will return projected point and height or xyz. We really
      // prefer projected points, as then the chipper will have an
      // easier time grouping spatially points close together, as it
      // operates the first two coordinates.
      bool return_point_height = true;
      try {
        m_curr_point
          = m_csv_conv.csv_to_cartesian_or_point_height(vals, m_georef, return_point_height);
      } catch (...) {
        // This is a bug fix. Skip points out of bounds.
        continue;
      }

      // Found a valid point
      if (m_has_valid_point)
        break;
    }

    return m_has_valid_point;
  }

  virtual vw::Vector3 GetPoint() {
    return m_curr_point;
  }

  virtual ~CsvReader() {
    delete m_ifs;
    m_ifs = NULL;
  }

}; // End class CsvReader

// Read unordered points from a Tif image. They will be organized into chips.
class TifReader: public BaseReader {
  vw::ImageViewRef<vw::Vector3> m_img;
  bool m_has_valid_point;
  vw::Vector3 m_curr_point;
  std::vector<vw::BBox2i> m_blocks;
  vw::ImageView<vw::Vector3> m_block_img;
  // Use int64_t to avoid overflow
  std::int64_t m_block_count;
  std::int64_t m_block_len;
  std::int64_t m_point_count_in_block;
  std::int64_t m_total_point_count;

public:

  TifReader(std::string const & tif_file):
    m_has_valid_point(false), m_block_count(0), m_point_count_in_block(0), m_block_len(0),
    m_total_point_count(0) {

    // Must ensure there are at least 3 channels in the image
    {
      vw::DiskImageResourceGDAL rsrc(tif_file);
      int num_channels = rsrc.channels();
      if (num_channels < 3)
        vw::vw_throw(vw::ArgumentErr() << "TifReader: Expecting at least 3 channels "
                  << "in the image.\n");
    }

    // Read the first 3 channels
    std::vector<std::string> tif_files;
    tif_files.push_back(tif_file);
    m_img = asp::form_point_cloud_composite<vw::Vector3>(tif_files, ASP_MAX_SUBBLOCK_SIZE);

    // Compute the number of points in the image. To avoid integer overflow,
    // cast first to int64_t.
    m_num_points = std::int64_t(m_img.cols()) * std::int64_t(m_img.rows());

    // Reading points one by one is very inefficient. We will read them in blocks.
    int block_size = 1024;
    m_blocks = vw::subdivide_bbox(vw::bounding_box(m_img), block_size, block_size);
  }

  virtual bool ReadNextPoint() {

    if (m_total_point_count >= m_num_points) {
      // No more points to read
      m_curr_point = vw::Vector3();
      m_has_valid_point = false;
      return m_has_valid_point;
    }

    if (m_block_img.cols() == 0 || m_point_count_in_block >= m_block_len) {

      // Sanity checks
      if (m_block_count >= (int64_t)m_blocks.size())
        vw::vw_throw(vw::ArgumentErr() << "TifReader: Unexpected end of file.\n");

      // Read the next block
      vw::BBox2i block = m_blocks[m_block_count];

      // The block must be contained within the bounding box and be non-empty
      if (block.empty() || !vw::bounding_box(m_img).contains(block))
        vw::vw_throw(vw::ArgumentErr() << "TifReader: Invalid block.\n");

      m_block_img = crop(m_img, block);
      m_block_len = std::int64_t(block.width()) * std::int64_t(block.height());
      m_point_count_in_block = 0;
      m_block_count++;
    }

    // Find the row and column of the current point
    int col = m_point_count_in_block % m_block_img.cols();
    int row = m_point_count_in_block / m_block_img.cols();

    // Get the element
    m_curr_point = m_block_img(col, row);

    // Set the zero point to nan. Usually a scanner uses this value
    // if it cannot get a valid reading.
    if (m_curr_point == vw::Vector3()) {
      double nan = std::numeric_limits<double>::quiet_NaN();
      m_curr_point = vw::Vector3(nan, nan, nan);
    }

    // Increment the counters
    m_point_count_in_block++;
    m_total_point_count++;

    m_has_valid_point = true;
    return m_has_valid_point;
  }

  virtual vw::Vector3 GetPoint() {
    return m_curr_point;
  }

  virtual ~TifReader() {
  }

}; // End class TifReader

// TODO(oalexan1): Consider using PDAL for PCD files too.
void PcdReader::read_header() {
  // Open the file as text
  std::ifstream handle;
  handle.open(m_pcd_file.c_str());
  if (handle.fail()) {
    vw_throw(vw::IOErr() << "Unable to open file \"" << m_pcd_file << "\"");
  }
  // Start checking all of the header elements
  bool valid = true;
  std::string line, dummy, value;
  std::getline(handle, line);
  while (line[0] == '#') // Skip initial comment lines
    std::getline(handle, line);
  // Check the header version - we only support one kind for now.
  boost::to_lower(line);
  if (line.find("version 0.7") == std::string::npos) {
    vw::vw_out() << "Error: Unsupported PCD file version: " << line << "\n";
    valid = false;
  }
  // Verify the fields
  std::getline(handle, line);
  boost::to_lower(line);
  if (line.find("fields x y z") == std::string::npos) {
    vw::vw_out() << "Error: Unsupported PCD fields: " << line << "\n";
    valid = false;
  }
  // Get some other information, no checking here...
  handle >> dummy >> m_size_bytes;
  std::getline(handle, line); // Go to the next line
  if ((m_size_bytes != 4) && (m_size_bytes != 8)) {
    vw::vw_out() << "Error: Unsupported byte size: " << m_size_bytes << "\n";
    valid = false;
  }
  handle >> dummy >> m_type;
  std::getline(handle, line); // Go to the next line
  if (m_type == 'F')
    m_type = 'f';
  if (m_type != 'f') {
    vw::vw_out() << "Error: Currently only Float type PCD files are supported.\n";
    valid = false;
  }

  // Get size info
  int width, height, count;
  handle >> dummy >> count;
  std::getline(handle, line); // Go to the next line
  if (count != 1) {
    vw::vw_out() << "Error: Unsupported PCD count: " << count << "\n";
    valid = false;
  }
  handle >> dummy >> width >> dummy >> height;
  std::getline(handle, line); // Skip viewpoint line
  std::getline(handle, line);
  handle >> dummy >> m_num_points;
  if (m_num_points != (static_cast<size_t>(width*height))) {
    vw::vw_out() << "Error: PCD point count error!\n";
    valid = false;
  }
  // Get the type of file, ascii or binary
  handle >> dummy >> value;
  boost::to_lower(value);
  m_binary_format = (value != "ascii");

  if (handle.fail()) {
    vw::vw_out() << "Error: PCD read error!\n";
    valid = false;
  }

  m_header_length_bytes = handle.tellg();

  // Stop reading the header file
  handle.close();
  if (!valid)
    vw::vw_throw(vw::ArgumentErr() << "Fatal error reading PCD file: " << m_pcd_file);
}

PcdReader::PcdReader(std::string const & pcd_file)
  : m_pcd_file(pcd_file), m_has_valid_point(false) {

  // For now PCD files are required to be in XYZ GCC format.
  m_has_georef = false;

  read_header();

  // Open the file for data reading in the proper format then skip past the header
  if (m_binary_format)
    m_ifs = new std::ifstream (m_pcd_file.c_str(), std::ios_base::binary);
  else
    m_ifs = new std::ifstream (m_pcd_file.c_str());

  m_ifs->seekg(m_header_length_bytes);
}

bool PcdReader::ReadNextPoint() {

  // Check if there is more data
  if (!m_ifs->good()) {
    m_has_valid_point = false;
    return false;
  }

  if (m_binary_format) {

    if (m_size_bytes == 4) { // -> float
      float x, y, z;
      m_ifs->read(reinterpret_cast<char*>(&x), m_size_bytes);
      m_ifs->read(reinterpret_cast<char*>(&y), m_size_bytes);
      m_ifs->read(reinterpret_cast<char*>(&z), m_size_bytes);
      m_curr_point = vw::Vector3(x, y, z);
    } else { // 8 bytes -> double
      double x, y, z;
      m_ifs->read(reinterpret_cast<char*>(&x), m_size_bytes);
      m_ifs->read(reinterpret_cast<char*>(&y), m_size_bytes);
      m_ifs->read(reinterpret_cast<char*>(&z), m_size_bytes);
      m_curr_point = vw::Vector3(x, y, z);
    }

  } else { // Text format

    // Read in the next point
    double x, y, z;
    (*m_ifs) >> x >> y >> z;
    m_curr_point = vw::Vector3(x, y, z);
  }

  // Make sure the reads succeeded
  if (m_ifs->fail()) {
    m_has_valid_point = false;
    return false;
  }

  return true;
}

vw::Vector3 PcdReader::GetPoint() {
  return m_curr_point;
}

PcdReader::~PcdReader() {
  delete m_ifs;
  m_ifs = NULL;
}

std::int64_t pcd_file_size(std::string const& file) {
  PcdReader reader(file);
  return reader.m_num_points;
}

/// Create a Tif file from a point cloud. The file will be created block by
/// block, when it needs to be written to disk. In each block the points will
/// be organized into chips by location. It is important that the writer
/// invoking this image be single-threaded, as we read from the cloud file
/// sequentially. The LAS file is handled separately, as PDAL does not have a
/// function for getting a point. The point cloud is in CSV or PCL format,
/// with PCL not tested yet.
class CloudToTif: public vw::ImageViewBase<CloudToTif> {

  typedef vw::Vector3 PixelT;

  asp::BaseReader * m_reader;
  int m_rows, m_cols; // These are pixel sizes, not tile counts.
  int m_block_size;

public:

  typedef PixelT pixel_type;
  typedef PixelT result_type;
  typedef vw::ProceduralPixelAccessor<CloudToTif> pixel_accessor;

  CloudToTif(asp::BaseReader * reader, int image_rows,
                int image_cols, int block_size):
    m_reader(reader), m_rows(image_rows), m_cols(image_cols),
    m_block_size(block_size) {}

  inline vw::int32 cols  () const { return m_cols; }
  inline vw::int32 rows  () const { return m_rows; }
  inline vw::int32 planes() const { return 1; }

  inline pixel_accessor origin() const { return pixel_accessor(*this); }

  inline result_type operator()(size_t i, size_t j, size_t p=0) const {
    vw::vw_throw(vw::NoImplErr()
                 << "CloudToTif::operator(...) has not been implemented.\n");
    return result_type();
  }

  typedef vw::CropView<vw::ImageView<PixelT>> prerasterize_type;
  inline prerasterize_type prerasterize(vw::BBox2i const& bbox) const {

    // Read a chunk of the las file, and store it in the current tile.

    std::int64_t num_cols = bbox.width();
    std::int64_t num_rows = bbox.height();

    VW_ASSERT((num_rows % m_block_size == 0) && (num_cols % m_block_size == 0),
              vw::ArgumentErr() << "CloudToTif: Expecting the number of rows and columns "
                            << "to be a multiple of the block size.\n");

    // Read the specified number of points from the file
    std::int64_t max_num_pts_to_read = num_cols * num_rows;
    std::int64_t count = 0;
    vw::PointBuffer in;
    while (m_reader->ReadNextPoint()) {
      in.push_back(m_reader->GetPoint());
      count++;
      if (count >= max_num_pts_to_read)
        break;
    }

    // Take the points just read, and put them in groups by spatial
    // location, so that later point2dem does not need to read every
    // input point when writing a given tile, but only certain groups.
    vw::ImageView<vw::Vector3> Img;
    vw::Chipper(in, m_block_size, m_reader->m_has_georef, m_reader->m_georef,
            num_cols, num_rows, Img);

    VW_ASSERT(num_cols == Img.cols() && num_rows == Img.rows(),
              vw::ArgumentErr() << "CloudToTif: Size mis-match.\n");

    return crop(Img, -bbox.min().x(), -bbox.min().y(), cols(), rows());
  }

  template <class DestT>
  inline void rasterize(DestT const& dest, vw::BBox2i const& bbox) const {
    vw::rasterize(prerasterize(bbox), dest, bbox);
  }

}; // End class CloudToTif

// We will fetch a chunk of the las file of area ASP_POINT_CLOUD_TILE_LEN x
// ASP_POINT_CLOUD_TILE_LEN, split it into bins of spatially close points, and write
// it to disk as a tile in a vector tif image. The bigger the tile
// size, the more likely the binning will be more efficient. But big
// tiles use a lot of memory. Each tile will be partitioned in chips
// of size block_size x block_size, which is always 128, consistent
// with what point2dem later uses.
void las_or_csv_to_tif(std::string const& in_file,
                       std::string const& out_prefix,
                       int num_rows, int block_size,
                       vw::BBox2 const& copc_win, bool copc_read_all,
                       vw::GdalWriteOptions & opt, // will change
                       vw::cartography::GeoReference const& csv_georef,
                       asp::CsvConv const& csv_conv,
                       std::vector<std::string> & out_files) {

  // Wipe the output
  out_files.clear();

  // To do: Study performance for large files when this number changes
  vw::Vector2i tile_size(ASP_POINT_CLOUD_TILE_LEN, ASP_POINT_CLOUD_TILE_LEN);

  // Temporarily change the raster tile size
  vw::Vector2 original_tile_size = opt.raster_tile_size;
  opt.raster_tile_size = tile_size;

  boost::shared_ptr<asp::BaseReader> reader_ptr;
  if (asp::is_csv(in_file)) // CSV
    reader_ptr = boost::shared_ptr<asp::CsvReader>
      (new asp::CsvReader(in_file, csv_conv, csv_georef));
  else if (asp::is_pcd(in_file)) // PCD
    reader_ptr = boost::shared_ptr<asp::PcdReader>(new asp::PcdReader(in_file));
  else if (asp::is_tif(in_file))
    reader_ptr = boost::shared_ptr<asp::TifReader>(new asp::TifReader(in_file));
  else if (asp::is_las(in_file))
     reader_ptr = boost::shared_ptr<asp::BaseReader>(); // see below for las
  else
    vw::vw_throw(vw::ArgumentErr() << "Unknown file type: " << in_file << "\n");

  // LAS files have a totally different interface, so have to be handled separately.
  if (asp::is_las(in_file)) { // LAS
                             
    // Set up the options
    pdal::Options read_options;
    read_options.add("filename", in_file);

    // Set the input point cloud. COPC is a streaming format, and need to fetch
    // the data in a box
    boost::shared_ptr<pdal::Reader> pdal_reader;
    if (asp::isCopc(in_file)) {
      
      pdal_reader.reset(new pdal::CopcReader());
      if (copc_win == vw::BBox2() && !copc_read_all)
        vw::vw_throw(vw::ArgumentErr() << "Must set either --copc-win or --copc-read-all.\n");
      if (!copc_read_all) {
        pdal::BOX2D bounds(copc_win.min().x(), copc_win.min().y(),
                           copc_win.max().x(), copc_win.max().y());
        read_options.add("bounds", bounds);
      }
      
    } else {
      pdal_reader.reset(new pdal::LasReader());
    }
    pdal_reader->setOptions(read_options);
    
    // buf_size is the number of points that will be processed and kept in this
    // table at the same time. A somewhat bigger value may result in some
    // efficiencies.
    vw::vw_out() << "Breaking up the LAS file into spatially-organized files.\n";
    int buf_size = 100;
    pdal::FixedPointTable t(buf_size);
    pdal_reader->prepare(t);
    vw::cartography::GeoReference las_georef;
    bool has_georef = asp::georef_from_las(in_file, las_georef);
    asp::ChipMaker writer(ASP_POINT_CLOUD_TILE_LEN, block_size, has_georef, las_georef,
                          opt, out_prefix, out_files);
    pdal::Options write_options;
    writer.setOptions(write_options);
    writer.setInput(*pdal_reader);
    writer.prepare(t);
    writer.execute(t);

  } else { // CSV or PCD

    // Compute the dimensions of the temporary image we are about to create. 
    std::int64_t num_points = reader_ptr->m_num_points;
    int num_row_tiles  = std::max(1, (int)ceil(double(num_rows)/ASP_POINT_CLOUD_TILE_LEN));
    int image_rows     = ASP_POINT_CLOUD_TILE_LEN * num_row_tiles;
    int points_per_row = (int)ceil(double(num_points)/image_rows);
    int num_col_tiles  = (int)ceil(double(points_per_row)/ASP_POINT_CLOUD_TILE_LEN);
    num_col_tiles      = std::max(1, num_col_tiles);
    int image_cols     = ASP_POINT_CLOUD_TILE_LEN * num_col_tiles;

    // For small CSV files, an image of dimensions ASP_POINT_CLOUD_TILE_LEN is
    // way too large and gridding is slow rather than instant. We will make the
    // size smaller while respecting all assumptions.
    if (num_points < ASP_POINT_CLOUD_TILE_LEN * ASP_POINT_CLOUD_TILE_LEN) {
      // Ensure multiple of block size
      int len = ceil(sqrt(num_points * 1.0) / block_size) * block_size;
      len = std::max(len, 1);
      len = std::min(len, ASP_POINT_CLOUD_TILE_LEN);
      image_rows = len;
      image_cols = len;
    }

    // Create the image. This will also separate it into chips, with each
    // chip having points that are spatially close together.
    vw::ImageViewRef<vw::Vector3> Img
      = asp::CloudToTif(reader_ptr.get(), image_rows, image_cols, block_size);
    // Must use a thread only, as we read the input file serially
    std::string out_file = out_prefix + ".tif";
    vw::vw_out() << "Writing spatially ordered data: " << out_file << "\n";
    auto tpc = vw::TerminalProgressCallback("asp", "\t--> ");
    vw::cartography::write_gdal_image(out_file, Img, opt, tpc);
    out_files.push_back(out_file);
  }

  // Restore the original tile size
  opt.raster_tile_size = original_tile_size;

  return;
}

/// Builds a GeoReference from the first cloud having a georeference in the list
bool georef_from_pc_files(std::vector<std::string> const& files,
                          vw::cartography::GeoReference & georef) {

  // Initialize
  georef = vw::cartography::GeoReference();

  for (int i = 0; i < (int)files.size(); i++) {
    vw::cartography::GeoReference local_georef;

    // Sometimes ASP PC files can have georef, written there by stereo
    try {
      if (!is_las(files[i]) && read_georeference(local_georef, files[i])) {
        georef = local_georef;
        return true;
      }
    } catch(...) {}

    // Sometimes las files can have georef
    if (is_las(files[i]) && asp::georef_from_las(files[i], local_georef)) {
      georef = local_georef;
      return true;
    }
  }

  return false;
}

} // end namespace asp

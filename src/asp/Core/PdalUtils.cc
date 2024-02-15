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

// TODO(oalexan1): Move all LAS logic to here from PointUtils.cc, 
// as that file is too big and very slow to compile.

/// \file PdalUtils.cc
///

#include <asp/Core/PdalUtils.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Core/ProgressCallback.h>   // for TerminalProgressCallback

#include <pdal/PointView.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/Reader.hpp>
#include <io/LasHeader.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <pdal/SpatialReference.hpp>

namespace pdal {
    
// A class to produce a point cloud point-by-point, rather than
// having it all in memory at the same time. It will be streamed to
// disk. See the GDALReader class for how to add more fields
// and read from disk.
class PDAL_DLL StreamedCloud: public Reader, public Streamable {
  
public:
  std::string getName() const;
  StreamedCloud(bool has_georef, 
                vw::ImageViewRef<vw::Vector3> point_image,
                vw::ImageViewRef<double> error_image,
                double max_valid_triangulation_error,
                double triangulation_error_factor);
  ~StreamedCloud();

private:
  virtual void initialize();
  virtual void addDimensions(PointLayoutPtr layout);
  virtual void ready(PointTableRef table);
  virtual point_count_t read(PointViewPtr view, point_count_t num);
  virtual void done(PointTableRef table);
  virtual bool processOne(PointRef& point);
  virtual void addArgs(ProgramArgs& args);

  bool m_has_georef;
  vw::ImageViewRef<vw::Vector3> m_point_image;
  vw::ImageViewRef<double> m_error_image;
  double m_max_valid_triangulation_error;
  double m_triangulation_error_factor;

  // These are of type uint64_t
  point_count_t m_col_count, m_row_count, m_cols, m_rows;
  point_count_t m_count, m_size, m_num_valid_points, m_num_saved_points;

  vw::TerminalProgressCallback m_tpc;
};
    
std::string StreamedCloud::getName() const {
  return "Ames Stereo Pipeline point cloud";
}

StreamedCloud::StreamedCloud(bool has_georef,
                             vw::ImageViewRef<vw::Vector3> point_image,
                             vw::ImageViewRef<double> error_image,
                             double max_valid_triangulation_error,
                             double triangulation_error_factor):
  m_has_georef(has_georef),
  m_point_image(point_image), m_error_image(error_image),
  m_max_valid_triangulation_error(max_valid_triangulation_error),
  m_triangulation_error_factor(triangulation_error_factor), 
  m_col_count(0), m_row_count(0),
  m_cols(m_point_image.cols()), m_rows(m_point_image.rows()),
  m_size(m_cols * m_rows), // careful here to avoid integer overflow
  m_count(0), m_num_valid_points(0), m_num_saved_points(0),
  m_tpc(vw::TerminalProgressCallback("asp", "\t--> ")) {

  // Sanity check, if the error image is nonempty, it must have the 
  // same dimensions as the point image.
  if (m_error_image.cols() > 0 && m_error_image.rows() > 0 &&
      m_error_image.cols() != m_point_image.cols() &&
      m_error_image.rows() != m_point_image.rows()) {
    vw::vw_throw(vw::ArgumentErr() 
                  << "Expecting the error image to have the same dimensions "
                  << "as the point cloud image.\n");
    }
}

StreamedCloud::~StreamedCloud() {}

void StreamedCloud::initialize() {}

// Set the cloud dimensions.
void StreamedCloud::addDimensions(PointLayoutPtr layout) {
  layout->registerDim(pdal::Dimension::Id::X);
  layout->registerDim(pdal::Dimension::Id::Y);
  layout->registerDim(pdal::Dimension::Id::Z);
  if (m_triangulation_error_factor > 0.0)
    layout->registerDim(Dimension::Id::Intensity);
}

void StreamedCloud::addArgs(ProgramArgs& args) {
}

void StreamedCloud::ready(PointTableRef table) {
  m_count = 0;
}

// This function is used when a point cloud is formed fully in memory.
// Not applicable here.
point_count_t StreamedCloud::read(PointViewPtr view, point_count_t numPts) {
  throw pdal_error("The read() function must not be called in streaming mode.");
  return -1;
}

// Create one point at a time. Will ask for a point till the counter
// reaches m_size.
bool StreamedCloud::processOne(PointRef& point) {
  
  // Keep on going through the input cloud until a valid point
  // is found or until we run out of points.
  while (1) {
    
    // Break the loop if no more points are available
    if (m_count >= m_size)
        return false; 

    // Note how we access in col, row order, per ASP conventions
    vw::Vector3 xyz = m_point_image(m_col_count, m_row_count);
    
    // Skip no-data points and point above the max valid triangulation error
    bool is_good1 = ((!m_has_georef && xyz != vw::Vector3()) ||
                    (m_has_georef  && !boost::math::isnan(xyz.z())));
    bool is_good2 = (m_max_valid_triangulation_error <= 0 ||
                    m_error_image(m_col_count, m_row_count) <= 
                    m_max_valid_triangulation_error);

    if (is_good1)
      m_num_valid_points++;

    if (is_good1 && is_good2) {
      
      point.setField(Dimension::Id::X, xyz[0]);
      point.setField(Dimension::Id::Y, xyz[1]);
      point.setField(Dimension::Id::Z, xyz[2]);
      m_num_saved_points++;
      
      if (m_triangulation_error_factor > 0.0) {
        // Scale the triangulation error, clamp it, and save it as
        // uint16.  The LAS 1.2 format has no fields (apart from the
        // taken already x, y, and z) with 32-bit values, so uint16
        // is all one can do.
        double scaled_error = m_triangulation_error_factor * 
                              m_error_image(m_col_count, m_row_count);
        scaled_error = round(scaled_error); // round to int32
        scaled_error = std::max(scaled_error, 0.0); // should not be necessary
        double max_int16 = std::numeric_limits<std::uint16_t>::max();
        scaled_error = std::min(scaled_error, double(max_int16)); // clamp
        point.setField(Dimension::Id::Intensity, (std::uint16_t)scaled_error);
      }
    }
      
    // Adjust the counters whether the point is good or not
    m_col_count++;
    if (m_col_count >= m_cols) {
      m_col_count = 0;
      m_row_count++;
      m_tpc.report_fractional_progress(m_row_count, m_rows);
    }
    m_count++;
    
    // Break the loop if a good point was found
    if (is_good1 && is_good2)
      return true;
  } // end while loop
  
  // This should not be reached
  return false;
}

void StreamedCloud::done(PointTableRef table) {
  m_tpc.report_finished();
  
  vw::vw_out () << "Wrote: " << m_num_saved_points << " points." << std::endl;
  if (m_max_valid_triangulation_error > 0.0) {
    point_count_t num_excluded = m_num_valid_points - m_num_saved_points;
    double percent = 100.0 * double(num_excluded)/double(m_num_valid_points);
    percent = round(percent * 100.0)/100.0; // don't keep too many digits
    vw::vw_out() << "Excluded based on triangulation error: " << num_excluded 
                 << " points (" << percent << "%)." << std::endl;
  }
  
} // End function done

// A class to read a point cloud from a file point by point, without
// loading it fully in memory. The points are printed to the screen.
// This can be used as a template for other readers.
class PDAL_DLL StreamProcessor: public Writer, public Streamable {

public:
  std::string getName() const;
  StreamProcessor();
  ~StreamProcessor();

private:

  virtual void addArgs(ProgramArgs& args);
  virtual void initialize();
  virtual void writeView(const PointViewPtr view);
  virtual bool processOne(PointRef& point);
  virtual void done(PointTableRef table);

  StreamProcessor& operator=(const StreamProcessor&) = delete;
  StreamProcessor(const StreamProcessor&) = delete;
  StreamProcessor(const StreamProcessor&&) = delete;
};

std::string StreamProcessor::getName() const { return "sample streamer"; }

StreamProcessor::StreamProcessor() {}

StreamProcessor::~StreamProcessor() {}

void StreamProcessor::addArgs(ProgramArgs& args){}

void StreamProcessor::initialize(){}

// This will be called for each point in the cloud.
bool StreamProcessor::processOne(PointRef& point) {
  // Print the point coordinates
  std::cout << "Process point: " 
            << point.getFieldAs<double>(Dimension::Id::X) <<  ", " 
            << point.getFieldAs<double>(Dimension::Id::Y) << ", " 
            << point.getFieldAs<double>(Dimension::Id::Z) << std::endl;
  return true;  
}

void StreamProcessor::done(PointTableRef table) {
}

void StreamProcessor::writeView(const PointViewPtr view) {
  throw pdal_error("The writeView() function must not be called in streaming mode.");
}

} // end namespace pdal

namespace asp {

bool georef_from_las(std::string const& las_file,
                     vw::cartography::GeoReference & georef) {
  
  pdal::Options read_options;
  read_options.add("filename", las_file);

  pdal::LasReader reader;
  reader.setOptions(read_options);
  pdal::QuickInfo qi = reader.preview();

  std::string wkt = qi.m_srs.getWKT();
  if (wkt.empty()) 
    return false;
  
  georef.set_wkt(wkt);
  
  return true;
}

// Read the number of points in the LAS file
std::int64_t las_file_size(std::string const& las_file) {
  
  pdal::Options read_options;
  read_options.add("filename", las_file);

  pdal::LasReader reader;
  reader.setOptions(read_options);
  pdal::QuickInfo qi = reader.preview();

  return qi.m_pointCount;
}

// Read a LAS file point-by-point, and print the points to the screen. This is
// an example that shows how to read LAS files.
void read_las() {

  // Set the input point cloud    
  pdal::Options read_options;
  read_options.add("filename", "input.las");
  pdal::LasReader reader;
  reader.setOptions(read_options);

  // buf_size is the number of points that will be
  // processed and kept in this table at the same time. 
  // A somewhat bigger value may result in some efficiencies.
  int buf_size = 100;
  pdal::FixedPointTable t(buf_size);
  reader.prepare(t);

  // Read each point and print it to the screen
  pdal::StreamProcessor writer;
  pdal::Options write_options;
  writer.setOptions(write_options);
  writer.setInput(reader);
  writer.prepare(t);
  writer.execute(t);
}

// Save a point cloud and triangulation error to the LAS format
void write_las(bool has_georef, vw::cartography::GeoReference const& georef,
               vw::ImageViewRef<vw::Vector3> point_image,
               vw::ImageViewRef<double> error_image,
               vw::Vector3 const& offset, vw::Vector3 const& scale,
               bool compressed,
               double max_valid_triangulation_error,
               double triangulation_error_factor,
               std::string const& out_prefix) {

  // Streamed cloud structure
  pdal::StreamedCloud stream_cloud(has_georef, point_image, error_image,
                                   max_valid_triangulation_error,
                                   triangulation_error_factor);

  // buf_size is the number of points that will be
  // processed and kept in this table at the same time. 
  // A somewhat bigger value may result in some efficiencies.
  int buf_size = 1000;
  pdal::FixedPointTable t(buf_size);
  stream_cloud.prepare(t);

  // Set the output filename. The writer will compress the las file if the .laz
  // extension is used.
  std::string lasFile;
  if (compressed)
    lasFile = out_prefix + ".laz";
  else
    lasFile = out_prefix + ".las";
  vw::vw_out() << "Writing LAS file: " << lasFile + "\n";

  pdal::Options write_options;
  write_options.add("filename", lasFile);
  write_options.add("offset_x", offset[0]);
  write_options.add("offset_y", offset[1]);
  write_options.add("offset_z", offset[2]);
  write_options.add("scale_x",  scale[0]);
  write_options.add("scale_y",  scale[1]);
  write_options.add("scale_z",  scale[2]);
  if (has_georef)     
    write_options.add("a_srs", georef.get_wkt());

  pdal::LasWriter writer;
  writer.setOptions(write_options);
  writer.setInput(stream_cloud);
  writer.prepare(t);
  writer.execute(t);
}

} // End namespace asp


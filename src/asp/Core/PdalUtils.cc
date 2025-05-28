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

// References:
// https://www.asprs.org/wp-content/uploads/2019/07/LAS_1_4_r15.pdf
// https://pdal.io/en/2.7.2/project/docs.html
// https://github.com/PDAL/PDAL/blob/master/pdal/Dimension.json

/// \file PdalUtils.cc
///

#include <asp/Core/PdalUtils.h>
#include <asp/Core/PointUtils.h>
#include <asp/Core/EigenUtils.h>

#include <vw/Cartography/GeoReference.h>
#include <vw/Core/ProgressCallback.h>

#include <io/CopcReader.hpp>
#include <io/LasHeader.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Reader.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal {
    
// A class to produce a point cloud point-by-point, rather than
// having it all in memory at the same time. It will be streamed to
// disk. See the GDALReader in PDAL class for how to add more fields
// and read from disk.
class PDAL_DLL StreamedCloud: public Reader, public Streamable {
  
public:
  std::string getName() const;
  StreamedCloud(bool has_georef, 
                vw::ImageViewRef<vw::Vector3> point_image,
                vw::ImageViewRef<double> error_image,
                vw::ImageViewRef<float> intensity,
                vw::ImageViewRef<double> horizontal_stddev,
                vw::ImageViewRef<double> vertical_stddev,                
                bool save_triangulation_error,
                double max_valid_triangulation_error);
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
  vw::ImageViewRef<float> m_intensity;
  vw::ImageViewRef<double> m_horizontal_stddev;
  vw::ImageViewRef<double> m_vertical_stddev;
  bool m_save_triangulation_error;
  double m_max_valid_triangulation_error;

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
                             vw::ImageViewRef<float> intensity,
                             vw::ImageViewRef<double> horizontal_stddev,
                             vw::ImageViewRef<double> vertical_stddev,
                             bool save_triangulation_error,
                             double max_valid_triangulation_error):
  m_has_georef(has_georef),
  m_point_image(point_image), m_error_image(error_image), m_intensity(intensity),
  m_horizontal_stddev(horizontal_stddev), m_vertical_stddev(vertical_stddev),
  m_save_triangulation_error(save_triangulation_error),
  m_max_valid_triangulation_error(max_valid_triangulation_error),
  m_col_count(0), m_row_count(0),
  m_cols(m_point_image.cols()), m_rows(m_point_image.rows()),
  m_size(m_cols * m_rows), // careful here to avoid integer overflow
  m_count(0), m_num_valid_points(0), m_num_saved_points(0),
  m_tpc(vw::TerminalProgressCallback("asp", "\t--> ")) {}

StreamedCloud::~StreamedCloud() {}

void StreamedCloud::initialize() {}

// Set the cloud dimensions.
void StreamedCloud::addDimensions(PointLayoutPtr layout) {
  layout->registerDim(pdal::Dimension::Id::X);
  layout->registerDim(pdal::Dimension::Id::Y);
  layout->registerDim(pdal::Dimension::Id::Z);
  
  // Co-opt the W dimension for the intensity
  if (m_intensity.cols() != 0 || m_intensity.rows() != 0)
    layout->registerDim(pdal::Dimension::Id::W); // intensity  

  // Co-opt the TextureU dimension for the triangulation error
  if (m_save_triangulation_error)
    layout->registerDim(pdal::Dimension::Id::TextureU);

  // Co-opt the TextureV and TextureW dimensions for the horizontal and vertical stddev
  if (m_horizontal_stddev.cols() != 0 || m_horizontal_stddev.rows() != 0)
    layout->registerDim(pdal::Dimension::Id::TextureV); // horizontal stddev
  if (m_vertical_stddev.cols() != 0 || m_vertical_stddev.rows() != 0)
    layout->registerDim(pdal::Dimension::Id::TextureW); // vertical stddev    
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
  
  bool save_intensity = (m_intensity.cols() != 0 || m_intensity.rows() != 0);
  bool save_horizontal_stddev = (m_horizontal_stddev.cols() != 0 ||
                                 m_horizontal_stddev.rows() != 0);
  bool save_vertical_stddev = (m_vertical_stddev.cols() != 0 ||
                               m_vertical_stddev.rows() != 0);
  
  while (1) {
    
    // Break the loop if no more points are available
    if (m_count >= m_size)
        return false; 

    // Note how we access in col, row order, per ASP conventions
    vw::Vector3 xyz = m_point_image(m_col_count, m_row_count);
    
    // Skip no-data points and point above the max valid triangulation error
    bool valid_xyz = ((!m_has_georef && xyz != vw::Vector3()) ||
                    (m_has_georef  && !boost::math::isnan(xyz.z())));
    bool valid_tri_err = (m_max_valid_triangulation_error <= 0 ||
                    m_error_image(m_col_count, m_row_count) <= 
                    m_max_valid_triangulation_error);

    if (valid_xyz)
      m_num_valid_points++;

    if (valid_xyz && valid_tri_err) {
      
      point.setField(Dimension::Id::X, xyz[0]);
      point.setField(Dimension::Id::Y, xyz[1]);
      point.setField(Dimension::Id::Z, xyz[2]);
      m_num_saved_points++;
      
      // Save the intensity as a double
      if (save_intensity)
        point.setField(Dimension::Id::W, m_intensity(m_col_count, m_row_count));
        
      // Save the triangulation error as a double
      if (m_save_triangulation_error)
        point.setField(Dimension::Id::TextureU, m_error_image(m_col_count, m_row_count));
        
      // Save the horizontal and vertical stddev as doubles
      if (save_horizontal_stddev)
        point.setField(Dimension::Id::TextureV, m_horizontal_stddev(m_col_count, m_row_count));
      if (save_vertical_stddev)
        point.setField(Dimension::Id::TextureW, m_vertical_stddev(m_col_count, m_row_count));
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
    if (valid_xyz && valid_tri_err)
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
// This can be used as a template for other readers, such as the 
// LasLoader class.
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

std::string StreamProcessor::getName() const { return "StreamProcessor"; }

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

// Read the number of points in the LAS file. For COPC files, we usually
// care not for this, but for the number of points in a region.
std::int64_t las_file_size(std::string const& las_file) {
  
  pdal::Options read_options;
  read_options.add("filename", las_file);

  pdal::LasReader reader;
  reader.setOptions(read_options);
  pdal::QuickInfo qi = reader.preview();

  return qi.m_pointCount;
}

// Check if a file is in the LAS COPC format
bool isCopc(std::string const& file) {

  bool is_copc = false;
  
  if (!asp::is_las(file))
    return false;
    
  try {
    pdal::Options options;
    options.add("filename", file);

    pdal::CopcReader reader;
    reader.setOptions(options);
    pdal::QuickInfo qi(reader.preview());
    is_copc = qi.valid();
  } catch (pdal::pdal_error const& e) {
    is_copc = false;
  }

  return is_copc;
} 

// Save a point cloud and triangulation error to the LAS format.
void write_las(bool has_georef, vw::cartography::GeoReference const& georef,
               vw::ImageViewRef<vw::Vector3> point_image,
               vw::ImageViewRef<double> error_image,
               vw::ImageViewRef<float> intensity,
               vw::ImageViewRef<double> horizontal_stddev,
               vw::ImageViewRef<double> vertical_stddev,
               vw::Vector3 const& offset, vw::Vector3 const& scale,
               bool compressed, bool save_triangulation_error,
               double max_valid_triangulation_error,
               std::string const& out_prefix) {

  // The point image and error image must have the same dimensions
  if ((error_image.cols() != 0 || error_image.rows() != 0) &&
      (point_image.cols() != error_image.cols() ||
      point_image.rows() != error_image.rows()))
    vw::vw_throw(vw::ArgumentErr() 
                  << "Expecting the point cloud image and the error image "
                  << "to have the same dimensions.\n");

  // If the intensity image is present, it must have the same dimensions
  // as the point image.
  if ((intensity.cols() != 0 || intensity.rows() != 0) &&
      (point_image.cols() != intensity.cols() ||
      point_image.rows() != intensity.rows()))
    vw::vw_throw(vw::ArgumentErr() 
                  << "Expecting the point cloud image and the intensity image (L.tif) "
                  << "to have the same dimensions.\n");
    
  // Sanity checks for horizontal and vertical stddev images
  if ((horizontal_stddev.cols() != 0 || horizontal_stddev.rows() != 0) &&
      (point_image.cols() != horizontal_stddev.cols() ||
      point_image.rows() != horizontal_stddev.rows()))
    vw::vw_throw(vw::ArgumentErr() 
                  << "Expecting the point cloud image and the horizontal stddev image "
                  << "to have the same dimensions.\n");
  if ((vertical_stddev.cols() != 0 || vertical_stddev.rows() != 0) &&
      (point_image.cols() != vertical_stddev.cols() ||
      point_image.rows() != vertical_stddev.rows()))
    vw::vw_throw(vw::ArgumentErr() 
                  << "Expecting the point cloud image and the vertical stddev image "
                  << "to have the same dimensions.\n");
      
  // Streamed cloud structure
  pdal::StreamedCloud stream_cloud(has_georef, point_image, error_image, intensity,
                                   horizontal_stddev, vertical_stddev,
                                   save_triangulation_error,
                                   max_valid_triangulation_error);

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
  
  // LAS 1.4 instead of default LAS 1.2 is needed for advanced fields
  if (save_triangulation_error || 
      intensity.cols() != 0 || intensity.rows() != 0 ||
      horizontal_stddev.cols() != 0 || horizontal_stddev.rows() != 0 ||
      vertical_stddev.cols() != 0 || vertical_stddev.rows() != 0) {
    write_options.add("minor_version", 4); 
    write_options.add("extra_dims", "all");
  }

  if (has_georef)     
    write_options.add("a_srs", georef.get_wkt());

  pdal::LasWriter writer;
  writer.setOptions(write_options);
  writer.setInput(stream_cloud);
  writer.prepare(t);
  writer.execute(t);
}

// Read a LAS cloud and return a subset of it. This inherits from pdal::Writer
// because it is a processing class. It brings in the data via a pdal::Reader
// handle that is set when an instance of this is configured.
class PDAL_DLL LasProcessor: public pdal::Writer, public pdal::Streamable {

public:
  LasProcessor(std::string const& file_name, std::int64_t num_points_to_load,
              vw::BBox2 const& lonlat_box,
              vw::cartography::GeoReference const& input_georef,
              bool verbose, bool calc_shift,
              std::int64_t num_total_points, 
              // Outputs
              vw::Vector3 & shift, 
              Eigen::MatrixXd & data):
  m_file_name(file_name),
  m_num_points_to_load(num_points_to_load),
  m_lonlat_box(lonlat_box),
  m_input_georef(input_georef),
  m_verbose(verbose),
  m_calc_shift(calc_shift),
  m_tpc(vw::TerminalProgressCallback("asp", "\t--> ")),
  m_num_total_points(num_total_points), 
  // Outputs
  m_shift(shift), m_data(data) {
    
    m_data.conservativeResize(asp::DIM + 1, m_num_points_to_load);
    m_has_las_georef = asp::georef_from_las(m_file_name, m_las_georef);
    m_shift_was_calc = false;
    m_points_count = 0;
    
    // We will randomly pick or not a point with probability load_ratio
    m_load_ratio = (double)m_num_points_to_load/std::max(1.0, (double)m_num_total_points);

    std::int64_t hundred = 100;
    m_spacing = std::max(m_num_total_points/hundred, std::int64_t(1));
    m_inc_amount = 1.0 / hundred;
    if (m_verbose) 
      m_tpc.report_progress(0);
  }
  
  ~LasProcessor() {}

  virtual std::string getName() const { return "sample streamer"; }

private:

  std::string m_file_name;
  std::int64_t m_num_points_to_load;
  vw::BBox2 m_lonlat_box;
  vw::cartography::GeoReference m_input_georef;
  bool m_verbose;
  bool m_calc_shift;
  bool m_has_las_georef;
  vw::cartography::GeoReference m_las_georef;
  double m_load_ratio;
  bool m_shift_was_calc;
  std::int64_t m_points_count;
  vw::TerminalProgressCallback m_tpc;
  std::int64_t m_spacing;
  double m_inc_amount;
  std::int64_t m_num_total_points;
  
  // Aliases, to be returned to the caller
  vw::Vector3 & m_shift;
  Eigen::MatrixXd & m_data;
  
  virtual void addArgs(pdal::ProgramArgs& args) {}
  virtual void initialize() {}

  // This will be called for each point in the cloud.
  virtual bool processOne(pdal::PointRef& point) {

    if (m_points_count >= m_num_points_to_load)
      return false; // done with reading points

    // try next time is above the load ratio
    double r = (double)std::rand()/(double)RAND_MAX;
    if (r > m_load_ratio)
      return true;
    
    // Current point
    vw::Vector3 xyz(point.getFieldAs<double>(pdal::Dimension::Id::X),
                    point.getFieldAs<double>(pdal::Dimension::Id::Y),
                    point.getFieldAs<double>(pdal::Dimension::Id::Z));
    
    if (m_has_las_georef) {
      // This is a projected point, convert to cartesian
      vw::Vector2 ll = m_las_georef.point_to_lonlat(subvector(xyz, 0, 2));
      xyz = m_las_georef.datum().geodetic_to_cartesian(vw::Vector3(ll[0], ll[1], xyz[2]));
    }
    
    if (m_calc_shift && !m_shift_was_calc) {
      m_shift = xyz;
      m_shift_was_calc = true;
    }
    
    // Skip points outside the given box. Here we use the input georef.
    // It is assumed that if the box is non-empty then this georef is valid.
    if (!m_lonlat_box.empty()) {
      vw::Vector3 llh = m_input_georef.datum().cartesian_to_geodetic(xyz);
      if (!m_lonlat_box.contains(subvector(llh, 0, 2)))
        return true;
    }
    
    // Save this point
    for (int row = 0; row < asp::DIM; row++)
      m_data(row, m_points_count) = xyz[row] - m_shift[row];
    m_data(asp::DIM, m_points_count) = 1; // last field

    if (m_verbose && m_points_count % m_spacing == 0) 
      m_tpc.report_incremental_progress(m_inc_amount);

    m_points_count++;
    
    return true;  
  }

  virtual void writeView(const pdal::PointViewPtr view) {
    throw pdal::pdal_error("The writeView() function must not be called in streaming mode.");
  }

  // To be called after all the points are read.
  virtual void done(pdal::PointTableRef table) {
    m_data.conservativeResize(Eigen::NoChange, m_points_count);

    if (m_verbose) 
      m_tpc.report_finished();
  }
  
  LasProcessor& operator=(const LasProcessor&) = delete;
  LasProcessor(const LasProcessor&) = delete;
  LasProcessor(const LasProcessor&&) = delete;
};

// A filter to apply a transform to a cloud. Each point is processed in
// streaming mode, without loading the entire point cloud into memory. May
// adjust the scale and offset in the header of the output file.
      
class PDAL_DLL TransformFilter: public pdal::Filter, public pdal::Streamable {

public:

  std::string getName() const {
      return "transform_filter";
  }

  TransformFilter(std::int64_t num_total_points, 
                  bool has_georef, 
                  vw::cartography::GeoReference const& georef,
                  Eigen::MatrixXd const& T): 
        m_has_georef(has_georef), m_georef(georef), m_T(T), 
        m_tpc(vw::TerminalProgressCallback("asp", "\t--> ")) {
    
    int hundred = 100;
    m_spacing = std::max(num_total_points/hundred, std::int64_t(1));
    m_inc_amount = 1.0 / double(hundred);
    m_count = 0;
  }

  ~TransformFilter() {}

private:

  // Apply a transform to each point
  virtual bool processOne(pdal::PointRef& point) {
    
    // Initial point
    vw::Vector3 P(point.getFieldAs<double>(pdal::Dimension::Id::X),
                  point.getFieldAs<double>(pdal::Dimension::Id::Y),
                  point.getFieldAs<double>(pdal::Dimension::Id::Z));
    
    if (m_has_georef) {
      // This is a projected point, convert to cartesian
      vw::Vector2 ll = m_georef.point_to_lonlat(subvector(P, 0, 2));
      P = m_georef.datum().geodetic_to_cartesian(vw::Vector3(ll[0], ll[1], P[2]));
    }
    
    // Apply the transform
    P = asp::apply_transform_to_vec(m_T, P);
    
    if (m_has_georef) {
      // Go back to projected space
      vw::Vector3 llh = m_georef.datum().cartesian_to_geodetic(P);
      subvector(P, 0, 2) = m_georef.lonlat_to_point(subvector(llh, 0, 2));
      P[2] = llh[2];
    }
    
    // Put the point back
    point.setField(pdal::Dimension::Id::X, P[0]);
    point.setField(pdal::Dimension::Id::Y, P[1]);
    point.setField(pdal::Dimension::Id::Z, P[2]);

    // Update the progress and the counter
    if (m_count % m_spacing == 0) 
      m_tpc.report_incremental_progress(m_inc_amount);
    m_count++;  
    
    return true;
  }
  
  virtual void done(pdal::PointTableRef table) {
    m_tpc.report_finished();
  }
    
  bool m_has_georef;
  vw::cartography::GeoReference m_georef;
  Eigen::MatrixXd m_T;
  std::int64_t m_spacing;
  double m_inc_amount;
  std::int64_t m_count;
  vw::TerminalProgressCallback m_tpc;
  
};

// Set up a reader for a LAS or COPC file 
void setupLasOrCopcReader(std::string const& in_file,
                          vw::BBox2 const& copc_win, bool copc_read_all,
                          boost::shared_ptr<pdal::Reader>& pdal_reader,
                          pdal::Options& read_options,
                          std::int64_t & num_total_points) {

  read_options.add("filename", in_file);

  if (asp::isCopc(in_file)) {
    
    // Set the input point cloud. COPC is a streaming format, and need to fetch
    // the data in a box.
    pdal_reader.reset(new pdal::CopcReader());
    if (copc_win == vw::BBox2() && !copc_read_all)
      vw::vw_throw(vw::ArgumentErr() 
         << "Detected COPC file: " << in_file << ".\n"
         << "Set either the copc-win or copc-read-all option (the precise names "
         << "depends the invoked tool).\n");
    if (!copc_read_all) {
      pdal::BOX2D bounds(copc_win.min().x(), copc_win.min().y(),
                         copc_win.max().x(), copc_win.max().y());
      read_options.add("bounds", bounds);
    }
    
  } else {
    pdal_reader.reset(new pdal::LasReader());
  }
  pdal_reader->setOptions(read_options);
  
  // Note: For COPC files, the number of total points in the desired region
  // is a very rough estimate, and can be off by up to a factor of 10.
  
  // These are necessary to avoid a segfault in PDAL. It is not clear if it is
  // important to "execute" over a large table, or what exactly it is doing.
  pdal::PointTable table;
  pdal_reader->prepare(table);
  const auto set(pdal_reader->execute(table));
  
  pdal::QuickInfo qi(pdal_reader->preview());
  num_total_points = qi.m_pointCount;
}

// This is a helper function. Use instead load_las(). This function attempts to
// load a given number of points but does no no checks on how many are loaded.
// This returns the total number of points in the file, not the number of loaded 
// points.
std::int64_t load_las_aux(std::string const& file_name,
                          std::int64_t num_points_to_load,
                          vw::BBox2 const& lonlat_box,
                          vw::cartography::GeoReference const& geo,
                          bool verbose,
                          vw::BBox2 const& copc_win, bool copc_read_all,
                          bool calc_shift,
                          // Outputs
                          vw::Vector3 & shift,
                          Eigen::MatrixXd & data) {
  
  // Set the input point cloud    
  boost::shared_ptr<pdal::Reader> pdal_reader;
  pdal::Options read_options;
  std::int64_t num_total_points = 0; // will change
  setupLasOrCopcReader(file_name, copc_win, copc_read_all,
                       pdal_reader, read_options, num_total_points);

  // buf_size is the number of points that will be processed and kept in this
  // table at the same time. A somewhat bigger value may result in some
  // efficiencies.
  int buf_size = 100;
  pdal::FixedPointTable t(buf_size);
  pdal_reader->prepare(t);

  // Read the data
  asp::LasProcessor las_proc(file_name, num_points_to_load, lonlat_box, geo,
                             verbose, calc_shift, num_total_points, 
                             // Outputs
                             shift, data);
  pdal::Options proc_options;
  // proc_options.add("filename", file_name); // will be needed when upgrading PDAL
  las_proc.setOptions(proc_options);
  las_proc.setInput(*pdal_reader);
  las_proc.prepare(t);
  las_proc.execute(t);

  return num_total_points;
}

// Try to load at least this many points from the LAS file. 
// TODO(oalexan1): This function should reduce the number of points
// if they are too many.
void load_las(std::string const& file_name,
              std::int64_t num_points_to_load,
              vw::BBox2 const& lonlat_box,
              vw::BBox2 const& copc_win, bool copc_read_all,
              bool calc_shift,
              vw::Vector3 & shift,
              vw::cartography::GeoReference const& geo,
              bool verbose, Eigen::MatrixXd & data) {

  std::int64_t num_total_points 
    = load_las_aux(file_name, num_points_to_load, lonlat_box, geo, verbose, 
                   copc_win, copc_read_all,
                   calc_shift, 
                   shift, data); // outputs

  int num_loaded_points = data.cols();
  if (!lonlat_box.empty()                    &&
      num_loaded_points < num_points_to_load &&
      num_loaded_points < num_total_points) {

    // We loaded too few points. Try harder. Need some care here as to not run
    // out of memory.
    num_points_to_load = std::max(4*num_points_to_load, std::int64_t(10000000));
    if (verbose)
      vw::vw_out() << "Too few points were loaded. Trying again." << std::endl;
    load_las_aux(file_name, num_points_to_load, lonlat_box, geo, verbose,
                 copc_win, copc_read_all,
                 calc_shift,
                 shift, data); // outputs
  }

}

// Apply a given transform to a LAS file and save it.
void apply_transform_to_las(std::string const& input_file,
                            std::string const& output_file,
                            vw::BBox2 const& copc_win, bool copc_read_all,
                            Eigen::MatrixXd const& T) {

  // Set the input point cloud    
  boost::shared_ptr<pdal::Reader> pdal_reader;
  pdal::Options read_options;
  std::int64_t num_total_points = 0; // will change
  setupLasOrCopcReader(input_file, copc_win, copc_read_all,
                       pdal_reader, read_options, num_total_points);
  
  // buf_size is the number of points that will be
  // processed and kept in this table at the same time. 
  // A somewhat bigger value may result in some efficiencies.
  int buf_size = 100;
  pdal::FixedPointTable t(buf_size);
  pdal_reader->prepare(t); 
  
  // Get the scale and offset. Must be run after the table is prepared.
  vw::Vector3 offset, scale;
  pdal::CopcReader *copc_reader = dynamic_cast<pdal::CopcReader*>(pdal_reader.get());
  pdal::LasReader *las_reader = dynamic_cast<pdal::LasReader*>(pdal_reader.get());
  if (copc_reader != NULL) {
  
    pdal::QuickInfo qi(copc_reader->preview());
    pdal::BOX3D bounds = qi.m_bounds;
  
    // The offset is the center point
    offset = vw::Vector3((bounds.minx + bounds.maxx)/2.0,
                         (bounds.miny + bounds.maxy)/2.0,
                         (bounds.minz + bounds.maxz)/2.0);
    
    // Let the scale be about 1 mm. Ensure it won't result in integer overflow,
    // with a margin.
    double max_len = std::max(bounds.maxx - bounds.minx, 
                              bounds.maxy - bounds.miny);
    max_len = std::max(max_len, bounds.maxz - bounds.minz);
    double s = std::max(1e-3, max_len / 1e+9);
    scale = vw::Vector3(s, s, s);
  
  } else if (las_reader != NULL) {
  
    pdal::LasHeader const& header = las_reader->header();
    offset = vw::Vector3(header.offsetX(), header.offsetY(), header.offsetZ());
    scale  = vw::Vector3(header.scaleX(),  header.scaleY(),  header.scaleZ());
  } else {
    vw::vw_throw(vw::IOErr() << "Unknown LAS file type: " << input_file);
  }
  
  vw::cartography::GeoReference las_georef;
  bool has_georef = asp::georef_from_las(input_file, las_georef);

  // Set up the filter
  asp::TransformFilter transform_filter(num_total_points, has_georef, las_georef, T);
  transform_filter.setInput(*pdal_reader);
  transform_filter.prepare(t);

  // If the data is in ECEF, apply the same transform to the offset and scale as
  // to the data. This way the internal representation of the data changes very
  // little, and the data is still well-normalized.
  if (!has_georef) {
    offset = asp::apply_transform_to_vec(T, offset);
    scale = asp::apply_transform_to_vec(T, scale);
  }
    
  // Set up the output file
  pdal::Options write_options;
  write_options.add("filename", output_file);
  
  // Set up the scale and offset for the output
  write_options.add("offset_x", offset[0]);
  write_options.add("offset_y", offset[1]);
  write_options.add("offset_z", offset[2]);
  write_options.add("scale_x",  scale[0]);
  write_options.add("scale_y",  scale[1]);
  write_options.add("scale_z",  scale[2]);
  
  // Write the output file
  pdal::LasWriter writer;
  writer.setOptions(write_options);
  writer.setInput(transform_filter);
  writer.prepare(t);
  writer.execute(t);
}

} // End namespace asp

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

// Utilities used in cloud alignment that make use of PDAL. 

/// \file PointCloudAlignment.cc

#include <asp/Core/PointCloudAlignment.h>
#include <asp/Core/PdalUtils.h>
#include <asp/Core/EigenUtils.h>

#include <vw/Cartography/GeoReference.h>

// PDAL includes
#include <io/LasReader.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <io/LasWriter.hpp>
#include <io/LasHeader.hpp>
#include <pdal/Options.hpp>

namespace asp {

// Read a LAS cloud and return a subset of it. This inherits from pdal::Writer
// because it is a processing class. It brings in the data via a pdal::Reader
// handle that is set when an instance of this is configured.
class PDAL_DLL LasProcessor: public pdal::Writer, public pdal::Streamable {

public:
  LasProcessor(std::string const& file_name, std::int64_t num_points_to_load,
              vw::BBox2 const& lonlat_box,
              vw::cartography::GeoReference const& input_georef,
              bool verbose, bool calc_shift,
              // Outputs
              std::int64_t & num_total_points, vw::Vector3 & shift, 
              Eigen::MatrixXd & data):
  m_file_name(file_name),
  m_num_points_to_load(num_points_to_load),
  m_lonlat_box(lonlat_box),
  m_input_georef(input_georef),
  m_verbose(verbose),
  m_calc_shift(calc_shift),
  m_tpc(vw::TerminalProgressCallback("asp", "\t--> ")),
  // Outputs
  m_num_total_points(num_total_points), m_shift(shift), m_data(data) {
    
    m_data.conservativeResize(asp::DIM + 1, m_num_points_to_load);
    m_has_las_georef = asp::georef_from_las(m_file_name, m_las_georef);
    m_shift_was_calc = false;
    m_points_count = 0;
    
    // We will randomly pick or not a point with probability load_ratio
    m_num_total_points = asp::las_file_size(m_file_name);
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
  
  // Aliases, to be returned to the caller
  std::int64_t & m_num_total_points;
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

std::int64_t load_las(std::string const& file_name,
                      std::int64_t num_points_to_load,
                      vw::BBox2 const& lonlat_box,
                      vw::cartography::GeoReference const& geo,
                      bool verbose,
                      bool calc_shift,
                      // Outputs
                      vw::Vector3 & shift,
                      Eigen::MatrixXd & data) {
  
  // Set the input point cloud    
  pdal::Options read_options;
  read_options.add("filename", file_name);
  pdal::LasReader pdal_reader;
  pdal_reader.setOptions(read_options);

  // buf_size is the number of points that will be
  // processed and kept in this table at the same time. 
  // A somewhat bigger value may result in some efficiencies.
  int buf_size = 100;
  pdal::FixedPointTable t(buf_size);
  pdal_reader.prepare(t);

  // Read the data
  std::int64_t num_total_points = 0;
  asp::LasProcessor las_proc(file_name, num_points_to_load, lonlat_box, geo,
                             verbose, calc_shift, 
                             // Outputs
                             num_total_points, shift, data);
  pdal::Options proc_options;
  las_proc.setOptions(proc_options);
  las_proc.setInput(pdal_reader);
  las_proc.prepare(t);
  las_proc.execute(t);

  return num_total_points;
}

// Apply a given transform to a LAS file and save it.
void apply_transform_to_las(std::string const& input_file,
                            std::string const& output_file,
                            Eigen::MatrixXd const& T) {

  // buf_size is the number of points that will be
  // processed and kept in this table at the same time. 
  // A somewhat bigger value may result in some efficiencies.
  int buf_size = 500;
  pdal::FixedPointTable t(buf_size);

  // Set the input point cloud    
  pdal::Options read_options;
  read_options.add("filename", input_file);
  pdal::LasReader reader;
  reader.setOptions(read_options);
  reader.prepare(t); 
    
  // Get the scale and offset from the input cloud header
  // Must be run after the table is prepared
  const pdal::LasHeader & header = reader.header();
  vw::Vector3 offset(header.offsetX(), header.offsetY(), header.offsetZ());
  vw::Vector3 scale (header.scaleX(),  header.scaleY(),  header.scaleZ());

  std::int64_t num_total_points = asp::las_file_size(input_file);
  vw::cartography::GeoReference las_georef;
  bool has_georef = asp::georef_from_las(input_file, las_georef);

  // Set up the filter
  asp::TransformFilter transform_filter(num_total_points, has_georef, las_georef, T);
  transform_filter.setInput(reader);
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

} // end namespace asp
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

// This tool uses libpointmatcher for alignment,
// https://github.com/ethz-asl/libpointmatcher
// Copyright (c) 2010--2012,
// Francois Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
// You can contact the authors at <f dot pomerleau at gmail dot com> and
// <stephane at magnenat dot net>

// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __PC_ALIGN_UTILS_H__
#define __PC_ALIGN_UTILS_H__

#include <vw/FileIO/DiskImageView.h>
#include <vw/Math.h>
#include <vw/Image.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <asp/Core/Common.h>
#include <asp/Core/Macros.h>
#include <asp/Core/PointUtils.h>
#include <liblas/liblas.hpp>

#include <limits>
#include <cstring>

#include <pointmatcher/PointMatcher.h>

/*
  This file contains helper functions for the pc_align tool.

  Some of these could probably be moved elsewhere, but many of them depend on
  libpointmatcher object types.
*/


typedef double RealT; // We will use doubles in libpointmatcher.

// This stuff is from the libpointmatcher library
typedef PointMatcher<RealT> PM;
typedef PM::DataPoints DP;

// Note: Just changing 3 to 2 below won't be enough to make the code
// work with 2D point clouds. There are some Vector3's all over the place.
const int DIM = 3;

std::string UNSPECIFIED_DATUM = "unspecified_datum";

//======================================================================

/// Analyze a file name to determine the file type
std::string get_file_type(std::string const& file_name);

/// cartesian_to_geodetic() returns longitude between -180 and 180.
/// Sometimes this is 360 degrees less than what desired,
/// so here we do an adjustment.
/// To do: This may not be perfectly fool-proof.
vw::Vector3 cartesian_to_geodetic_adj(vw::cartography::GeoReference const&
                                      geo, vw::Vector3 xyz);

/// Generate libpointmatcher compatible labels.
template<typename T>
typename PointMatcher<T>::DataPoints::Labels form_labels(int dim);

/// Out of the elements 0, 1,..., n - 1, pick m unique
/// random elements and sort them in increasing order.
void pick_at_most_m_unique_elems_from_n_elems(int m, int n, std::vector<int>& elems);

/// Return at most m random points out of the input point cloud.
template<typename T>
void random_pc_subsample(int m, typename PointMatcher<T>::DataPoints& points);

/// Loads a helper file associated with the CSV files.
template<typename T>
int load_csv_aux(std::string const& file_name, int num_points_to_load,
                 vw::BBox2 const& lonlat_box, bool verbose,
                 bool calc_shift, vw::Vector3 & shift,
                 vw::cartography::GeoReference const& geo, asp::CsvConv const& csv_conv,
                 bool & is_lola_rdr_format, double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data);

/// Load a csv file
template<typename T>
void load_csv(std::string const& file_name,
                 int num_points_to_load,
                 vw::BBox2 const& lonlat_box,
                 bool verbose,
                 bool calc_shift,
                 vw::Vector3 & shift,
                 vw::cartography::GeoReference const& geo,
                 asp::CsvConv const& csv_conv,
                 bool & is_lola_rdr_format,
                 double & mean_longitude,
                 typename PointMatcher<T>::DataPoints & data);

/// Load a DEM file
/// - The points are stored in GCC coordinates.  These coordinates are either
///   shifted by the "shift" argument or (if calc_shift) so that the top left
///   coordinate becomes (0,0,0).
/// - If provided, only points in the lonlat_box will be loaded.
template<typename T>
void load_dem(bool verbose, std::string const& file_name,
              int num_points_to_load, vw::BBox2 const& lonlat_box,
              bool calc_shift, vw::Vector3 & shift,
              typename PointMatcher<T>::DataPoints & data);

/// Helper function to load a Stereo Pipeline point cloud files.
template<typename T>
vw::int64 load_pc_aux(bool verbose,
                  std::string const& file_name,
                  int num_points_to_load,
                  vw::BBox2 const& lonlat_box,
                  bool calc_shift,
                  vw::Vector3 & shift,
                  vw::cartography::GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data);

/// Helper function to load a LAS file.
template<typename T>
vw::int64 load_las_aux(bool verbose,
                  std::string const& file_name,
                  int num_points_to_load,
                  vw::BBox2 const& lonlat_box,
                  bool calc_shift,
                  vw::Vector3 & shift,
                  vw::cartography::GeoReference const& geo,
                  typename PointMatcher<T>::DataPoints & data);

/// Load one of the Stereo Pipeline Point Cloud files with additional options.
template<typename T>
void load_pc(bool verbose,
             std::string const& file_name,
             int num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             );

/// Load an LAS file with additional options.
template<typename T>
void load_las(bool verbose,
             std::string const& file_name,
             int num_points_to_load,
             vw::BBox2 const& lonlat_box,
             bool calc_shift,
             vw::Vector3 & shift,
             vw::cartography::GeoReference const& geo,
             typename PointMatcher<T>::DataPoints & data
             );

/// Load a file from disk and convert to libpointmatcher's format
template<typename T>
void load_file(std::string const& file_name,
               int num_points_to_load,
               vw::BBox2 const& lonlat_box,
               bool calc_shift,
               vw::Vector3 & shift,
               vw::cartography::GeoReference const& geo,
               asp::CsvConv const& csv_conv,
               bool & is_lola_rdr_format,
               double & mean_longitude,
               bool verbose,
               typename PointMatcher<T>::DataPoints & data);

/// Calculate the lon-lat bounding box of the points and bias it based
/// on max displacement (which is in meters). This is used to throw
/// away points in the other cloud which are not within this box.
vw::BBox2 calc_extended_lonlat_bbox(vw::cartography::GeoReference const& geo,
                                int num_sample_pts,
                                asp::CsvConv const& csv_conv,
                                std::string const& file_name,
                                double max_disp);

/// Compute the mean value of an std::vector out to a length
double calc_mean(std::vector<double> const& errs, int len);

/// Compute the standard deviation of an std::vector out to a length
double calc_stddev(std::vector<double> const& errs, double mean);


/// Consider a 4x4 matrix T which implements a rotation + translation
/// y = A*x + b. Consider a point s in space close to the points
/// x. We want to make that the new origin, so the points x get
/// closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
/// transform becomes y2 + s = A*(x2 + s) + b, or
/// y2 = A*x2 + b + A*s - s. Encode the obtained transform into another 4x4 matrix T2.
PointMatcher<RealT>::Matrix apply_shift(PointMatcher<RealT>::Matrix const& T,
                                        vw::Vector3 const& shift);


/// Calculate translation vector between the centers two point clouds
void calc_translation_vec(DP const& source, DP const& trans_source,
                          vw::Vector3 & shift, // from planet center to current origin
                          vw::cartography::Datum const& datum,
                          vw::Vector3 & source_ctr_vec,
                          vw::Vector3 & source_ctr_llh,
                          vw::Vector3 & trans_xyz,
                          vw::Vector3 & trans_ned,
                          vw::Vector3 & trans_llh);

/// Calculate max distance between any two points of two point clouds.
void calc_max_displacment(DP const& source, DP const& trans_source);


namespace asp{
  /// Apply a transformation matrix to a Vector3 in homogenous coordinates
  vw::Vector3 apply_transform(PointMatcher<RealT>::Matrix const& T, vw::Vector3 const& P);
}


/// Apply a transform to the first three coordinates of the cloud
struct TransformPC: public vw::UnaryReturnSameType {
  PointMatcher<RealT>::Matrix m_T;
  TransformPC(PointMatcher<RealT>::Matrix const& T):m_T(T){}
  inline vw::Vector<double> operator()(vw::Vector<double> const& pt) const {

    vw::Vector<double> P = pt; // local copy
    vw::Vector3 xyz = subvector(P, 0, 3);

    if (xyz == vw::Vector3())
      return P; // invalid point

    vw::Vector3 Q = asp::apply_transform(m_T, xyz);
    subvector(P, 0, 3) = Q;

    return P;
  }
};


/// Apply a given transform to the point cloud in input file, and save it.
/// - Note: We transform the entire point cloud, not just the resampled
///         version used in alignment.
void save_trans_point_cloud(vw::cartography::GdalWriteOptions const& opt,
                            std::string input_file,
                            std::string out_prefix,
                            vw::cartography::GeoReference const& geo,
                            asp::CsvConv const& csv_conv,
                            PointMatcher<RealT>::Matrix const& T);

/// Save a transformed point cloud with N bands
template<int n> // Number of bands
void save_trans_point_cloud_n(vw::cartography::GdalWriteOptions const& opt,
                              vw::cartography::GeoReference const& geo,
                              std::string input_file,
                              std::string output_file,
                              PointMatcher<RealT>::Matrix const& T){

  // We will try to save the transformed cloud with a georef. Try to get it from
  // the input cloud, or otherwise from the "global" georef.
  vw::cartography::GeoReference curr_geo;
  bool has_georef = vw::cartography::read_georeference(curr_geo, input_file);
  if (!has_georef && geo.datum().name() != UNSPECIFIED_DATUM){
    has_georef = true;
    curr_geo = geo;
  }

  // There is no nodata
  bool has_nodata = false;
  double nodata = -std::numeric_limits<float>::max(); // smallest float

  vw::ImageViewRef< vw::Vector<double, n> > point_cloud = asp::read_asp_point_cloud<n>(input_file);
  vw::cartography::block_write_gdal_image(output_file,
                              per_pixel_filter(point_cloud, TransformPC(T)),
                              has_georef, curr_geo,
                              has_nodata, nodata,
                              opt, vw::TerminalProgressCallback("asp", "\t--> "));
}


//=======================================================================================
// Stuff pulled up from point_to_dem_dist in the Tools repository.


/// A type for interpolation from a masked DEM object.
typedef vw::InterpolationView< vw::EdgeExtensionView< vw::ImageViewRef< vw::PixelMask<float> >,
                                                      vw::ConstantEdgeExtension>,
                               vw::BilinearInterpolation> InterpolationReadyDem;

/// Get ready to interpolate points on a DEM existing on disk.
InterpolationReadyDem load_interpolation_ready_dem(std::string                  const& dem_path,
                                                   vw::cartography::GeoReference     & georef);

/// Interpolates the DEM height at the input coordinate.
/// - Returns false if the coordinate falls outside the valid DEM area.
bool interp_dem_height(vw::ImageViewRef< vw::PixelMask<float> > const& dem,
                       vw::cartography::GeoReference const & georef,
                       vw::Vector3                   const & lonlat,
                       double                              & dem_height);

#include <asp/Tools/pc_align_utils.tcc>

#endif // #define __PC_ALIGN_UTILS_H__

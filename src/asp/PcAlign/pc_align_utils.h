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

#ifndef __PC_ALIGN_UTILS_H__
#define __PC_ALIGN_UTILS_H__

#include <asp/Core/EigenUtils.h>

#include <vw/FileIO/DiskImageView.h>
#include <vw/Cartography/Datum.h>
#include <vw/Cartography/GeoReference.h>
#include <vw/Cartography/PointImageManipulation.h>
#include <vw/FileIO/DiskImageUtils.h>
#include <vw/Math/Quaternion.h>
#include <vw/FileIO/GdalWriteOptions.h>
#include <vw/Image/Interpolation.h>

#include <limits>
#include <cstring>

#include <pointmatcher/PointMatcher.h>

namespace asp {
/*
  This file contains helper functions for the pc_align tool.

  Some of these could probably be moved elsewhere, but many of them depend on
  libpointmatcher object types.
*/

// This stuff is from the libpointmatcher library
typedef PointMatcher<double> PM;
typedef PM::DataPoints DP;

const std::string UNSPECIFIED_DATUM = "unspecified_datum";

//======================================================================

/// Generate libpointmatcher compatible labels.
template<typename T>
typename PointMatcher<T>::DataPoints::Labels form_labels(int dim) {

  typedef typename PointMatcher<T>::DataPoints::Label Label;
  typedef typename PointMatcher<T>::DataPoints::Labels Labels;

  Labels labels;
  for (int i=0; i < dim; i++){
    std::string text;
    text += char('x' + i);
    labels.push_back(Label(text, 1));
  }
  labels.push_back(Label("pad", 1));

  return labels;
}

// Load xyz points from disk into a matrix with 4 columns. Last column is just ones.
void load_cloud_as_mat(std::string const& file_name,
                       std::int64_t num_points_to_load,
                       vw::BBox2 const& lonlat_box,
                       bool calc_shift,
                       vw::Vector3 & shift,
                       vw::cartography::GeoReference const& geo,
                       CsvConv const& csv_conv,
                       bool   & is_lola_rdr_format,
                       double & median_longitude,
                       bool verbose,
                       Eigen::MatrixXd & data);

/// Load a file from disk and convert to libpointmatcher's format
void load_cloud(std::string const& file_name,
		std::int64_t num_points_to_load,
		vw::BBox2 const& lonlat_box,
		bool calc_shift,
		vw::Vector3 & shift,
		vw::cartography::GeoReference const& geo,
		CsvConv const& csv_conv,
		bool   & is_lola_rdr_format,
		double & median_longitude,
		bool verbose,
		typename PointMatcher<double>::DataPoints & data);

/// Calculate the lon-lat bounding box of the points and bias it based
/// on max displacement (which is in meters). This is used to throw
/// away points in the other cloud which are not within this box.
/// Return a version of it with given transform applied to it
void calc_extended_lonlat_bbox(vw::cartography::GeoReference const& geo,
                               int num_sample_pts,
                               CsvConv const& csv_conv,
                               std::string const& file_name,
                               double max_disp,
                               Eigen::MatrixXd const transform,
                               vw::BBox2 & out_box, 
                               vw::BBox2 & trans_out_box);
  
/// Compute the mean value of an std::vector out to a length
double calc_mean(std::vector<double> const& errs, int len);

/// Compute the standard deviation of an std::vector out to a length
double calc_stddev(std::vector<double> const& errs, double mean);

/// Calculate translation vector between the centers two point clouds
void calc_translation_vec(Eigen::MatrixXd const& initT,
                          DP const& source, DP const& trans_source,
                          vw::Vector3 & shift, // from planet center to current origin
                          vw::cartography::Datum const& datum,
                          vw::Vector3 & source_ctr_vec,
                          vw::Vector3 & source_ctr_llh,
                          vw::Vector3 & trans_xyz,
                          vw::Vector3 & trans_ned,
                          vw::Vector3 & trans_llh,
                          vw::Matrix3x3 & NedToEcef);

/// Calculate max distance between any two points of two point clouds.
double calc_max_displacement(DP const& source, DP const& trans_source);

/// Apply a transformation matrix to a Vector3 in homogenous coordinates
vw::Vector3 apply_transform(Eigen::MatrixXd const& T, vw::Vector3 const& P);

/// Apply a transform to the first three coordinates of the cloud
struct TransformPC: public vw::UnaryReturnSameType {
  Eigen::MatrixXd m_T;
  TransformPC(Eigen::MatrixXd const& T): m_T(T){}
  inline vw::Vector<double> operator()(vw::Vector<double> const& pt) const {

    vw::Vector<double> P = pt; // local copy
    vw::Vector3 xyz = subvector(P, 0, 3);

    if (xyz == vw::Vector3())
      return P; // invalid point

    vw::Vector3 Q = apply_transform_to_vec(m_T, xyz);
    subvector(P, 0, 3) = Q;

    return P;
  }
};

/// Apply a given transform to the point cloud in input file, and save it.
/// - Note: We transform the entire point cloud, not just the resampled
///         version used in alignment.
void save_trans_point_cloud(vw::GdalWriteOptions const& opt,
                            std::string input_file,
                            std::string out_prefix,
                            vw::cartography::GeoReference const& geo,
                            CsvConv const& csv_conv,
                            Eigen::MatrixXd const& T);

/// A type for interpolation from a masked DEM object.
typedef vw::InterpolationView<vw::EdgeExtensionView<vw::ImageViewRef<vw::PixelMask<float>>,
                              vw::ConstantEdgeExtension>, vw::BilinearInterpolation>
                              InterpolationReadyDem;

/// Get ready to interpolate points on a DEM existing on disk.
InterpolationReadyDem load_interpolation_ready_dem(std::string const& dem_path,
                                                   vw::cartography::GeoReference & georef);

// Extract rotation and translation from a vector of 6 elements
void extract_rotation_translation(const double * transform, vw::Quat & rotation, 
                                  vw::Vector3 & translation);


vw::Vector3 get_cloud_gcc_coord(DP const& point_cloud, 
                                vw::Vector3 const& shift, int index);

/// Interpolates the DEM height at the input coordinate.
/// - Returns false if the coordinate falls outside the valid DEM area.
bool interp_dem_height(vw::ImageViewRef<vw::PixelMask<float>> const& dem,
                       vw::cartography::GeoReference          const& georef,
                       vw::Vector3                            const& lonlat,
                       double                                      & dem_height);

/// Consider a 4x4 matrix T which implements a rotation + translation
/// y = A*x + b. Consider a point s in space close to the points
/// x. We want to make that the new origin, so the points x get
/// closer to origin. In the coordinates (x2 = x - s, y2 = y - s) the
/// transform becomes y2 + s = A*(x2 + s) + b, or
/// y2 = A*x2 + b + A*s - s. Encode the obtained transform into another 4x4 matrix T2.
Eigen::MatrixXd apply_shift(Eigen::MatrixXd const& T, vw::Vector3 const& shift);

// Sometime the box we computed with cartesian_to_geodetic is offset
// from the box computed with pixel_to_lonlat by 360 degrees.
// Fix that.
void adjust_lonlat_bbox(std::string const& file_name, vw::BBox2 & box);

/// Try to read the georef/datum info, need it to read CSV files.
void read_georef(std::vector<std::string> const& clouds,
                 std::string const& datum_str,
                 std::string const& csv_srs, 
                 double semi_major_axis,
                 double semi_minor_axis,
                 std::string & csv_format_str,
                 asp::CsvConv& csv_conv, vw::cartography::GeoReference& geo);

}

#endif // #define __PC_ALIGN_UTILS_H__

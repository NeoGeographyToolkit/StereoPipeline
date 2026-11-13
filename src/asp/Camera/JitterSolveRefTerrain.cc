// __BEGIN_LICENSE__
//  Copyright (c) 2009-2026, United States Government as represented by the
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

// Cost functions for the reference-terrain (disparity-based) reprojection error
// used by jitter solving, the option --reference-terrain. Stored in the Camera
// folder, as they need access to the camera models.

#include <asp/Camera/BundleAdjustCamera.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/JitterSolveCostFuns.h>
#include <asp/Camera/JitterSolveRigCostFuns.h>
#include <asp/Camera/BundleAdjustCostFuns.h> // for get_loss_function
#include <asp/Rig/TransformUtils.h>
#include <asp/Rig/RigConfig.h>
#include <asp/Core/EigenTransformUtils.h>
#include <asp/Core/CameraTransforms.h>
#include <asp/Core/CamPoseUtils.h>
#include <asp/Core/BundleAdjustUtils.h>
#include <asp/Core/DataLoader.h>

#include <vw/Camera/CameraImage.h>
#include <vw/Cartography/GeoReferenceBaseUtils.h>
#include <vw/Cartography/GeoReferenceUtils.h>
#include <vw/Cartography/CameraBBox.h>
#include <vw/Core/Exception.h>
#include <vw/Core/Stopwatch.h>
#include <vw/Cartography/DatumUtils.h>
#include <vw/Math/Transform.h>
#include <vw/Math/Statistics.h>
#include <vw/Math/RandomSet.h>
#include <vw/Image/Interpolation.h>
#include <vw/Image/ImageChannels.h>
#include <vw/Cartography/Map2CamTrans.h>

#include <ceres/rotation.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <vector>
#include <unordered_map>

namespace asp {

// The disparity-based error measured by --reference-terrain. It is the
// difference between left_pix + disp and right_pix, given the current ground
// pixel and camera models.
bool calcDispBasedError(UsgsAstroLsSensorModel *left_ls,
                        UsgsAstroLsSensorModel *right_ls,
                        vw::TransformPtr left_tx,
                        vw::TransformPtr right_tx,
                        DispPtr disp,
                        csm::EcefCoord const& P,
                        vw::Vector2 & error) {

  // Project into the images
  double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;
  csm::ImageCoord left_csm_pix = left_ls->groundToImage(P, desired_precision);
  csm::ImageCoord right_csm_pix = right_ls->groundToImage(P, desired_precision);

  // Covert to ASP pixels    
  vw::Vector2 left_raw_pix, right_raw_pix;
  asp::fromCsmPixel(left_raw_pix, left_csm_pix);
  asp::fromCsmPixel(right_raw_pix, right_csm_pix);

  // Convert to aligned pixel
  vw::Vector2 left_trans_pix;
  try {
    left_trans_pix = left_tx->forward(left_raw_pix);
  } catch (...) {
    return false;
  }

  // Typedefs to avoid long lines      
  typedef vw::ValueEdgeExtension<vw::PixelMask<vw::Vector2f>> NoDataType;
  typedef vw::DiskImageView<vw::PixelMask<vw::Vector2f>> DispDiskView;

  // Compute the interpolated aligned disparity. Use no-data when out of bounds.
  vw::PixelMask<vw::Vector2f> no_data;
  no_data.invalidate();
  NoDataType no_data_ext(no_data);
  vw::InterpolationView<vw::EdgeExtensionView<DispDiskView, NoDataType>,
                        vw::BicubicInterpolation>
    interp_disp = interpolate(*disp.get(), vw::BicubicInterpolation(), no_data_ext);
  vw::PixelMask<vw::Vector2f> disp_pix = interp_disp(left_trans_pix[0], left_trans_pix[1]);

  // Flag invalid pixels
  if (!is_valid(disp_pix))
    return false;

  // Discrepancy between right pixel and left pixel + disparity
  vw::Vector2 right_trans_pix = left_trans_pix + disp_pix.child();
  vw::Vector2 right_raw_derived_pix;
  try {
    right_raw_derived_pix = right_tx->reverse(right_trans_pix);
  } catch (...) {
    return false;
  }
  error = right_raw_derived_pix - right_raw_pix;

  return true;
}

// Fallback error value for --reference-terrain. Do not make this big, as when
// the calculation succeeds, the errors are small.
double g_ref_terrain_pix_val = 5.0;

// Reference terrain error. See calcDispBasedError() for the implementation.
struct RefTerrainReprojErr {
  RefTerrainReprojErr(UsgsAstroLsSensorModel *left_ls,
                      UsgsAstroLsSensorModel *right_ls,
                      vw::TransformPtr left_tx,
                      vw::TransformPtr right_tx,
                      DispPtr disp,
                      csm::EcefCoord const& P,
                      double weight,
                      int begLeftPosIndex, int endLeftPosIndex,
                      int begLeftQuatIndex, int endLeftQuatIndex,
                      int begRightPosIndex, int endRightPosIndex,
                      int begRightQuatIndex, int endRightQuatIndex):
    m_left_ls(left_ls), m_right_ls(right_ls),
    m_left_tx(left_tx), m_right_tx(right_tx), m_disp(disp), m_P(P), m_weight(weight),
    m_begLeftPosIndex(begLeftPosIndex), m_endLeftPosIndex(endLeftPosIndex),
    m_begLeftQuatIndex(begLeftQuatIndex), m_endLeftQuatIndex(endLeftQuatIndex),
    m_begRightPosIndex(begRightPosIndex), m_endRightPosIndex(endRightPosIndex),
    m_begRightQuatIndex(begRightQuatIndex), m_endRightQuatIndex(endRightQuatIndex) {}

  // The implementation is further down
  bool operator()(double const * const * parameters, double * residuals) const;

  // Factory to hide the construction of the CostFunction object from the client code.
  static ceres::CostFunction* Create(
                      UsgsAstroLsSensorModel *left_ls,
                      UsgsAstroLsSensorModel *right_ls,
                      vw::TransformPtr left_tx,
                      vw::TransformPtr right_tx,
                      DispPtr disp,
                      csm::EcefCoord const& P,
                      double weight,
                      int begLeftPosIndex, int endLeftPosIndex,
                      int begLeftQuatIndex, int endLeftQuatIndex,
                      int begRightPosIndex, int endRightPosIndex,
                      int begRightQuatIndex, int endRightQuatIndex) {

    ceres::DynamicNumericDiffCostFunction<RefTerrainReprojErr>* cost_function =
      new ceres::DynamicNumericDiffCostFunction<RefTerrainReprojErr>
      (new RefTerrainReprojErr(left_ls, right_ls, left_tx, right_tx, disp, P, weight,
                               begLeftPosIndex, endLeftPosIndex,
                               begLeftQuatIndex, endLeftQuatIndex,
                               begRightPosIndex, endRightPosIndex,
                               begRightQuatIndex, endRightQuatIndex));

    // The residual size is the pixel size
    cost_function->SetNumResiduals(PIXEL_SIZE);

    // Add a parameter block for each position and quaternion
    for (int it = begLeftPosIndex; it < endLeftPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    for (int it = begLeftQuatIndex; it < endLeftQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);
    for (int it = begRightPosIndex; it < endRightPosIndex; it++)
      cost_function->AddParameterBlock(NUM_XYZ_PARAMS);
    for (int it = begRightQuatIndex; it < endRightQuatIndex; it++)
      cost_function->AddParameterBlock(NUM_QUAT_PARAMS);

    return cost_function;
  }

private:
  UsgsAstroLsSensorModel *m_left_ls, *m_right_ls;
  vw::TransformPtr m_left_tx, m_right_tx;
  DispPtr m_disp;
  csm::EcefCoord m_P;
  double m_weight;
  int m_begLeftPosIndex, m_endLeftPosIndex;
  int m_begLeftQuatIndex, m_endLeftQuatIndex;
  int m_begRightPosIndex, m_endRightPosIndex;
  int m_begRightQuatIndex, m_endRightQuatIndex;

}; // End class RefTerrainReprojErr

// See the documentation higher up in the file.
bool RefTerrainReprojErr::operator()(double const * const * parameters,
                                     double * residuals) const {

  try {
    // Make a copy of the models
    UsgsAstroLsSensorModel local_left_ls = *m_left_ls;
    UsgsAstroLsSensorModel local_right_ls = *m_right_ls;

    // Update position and quaternion parameters with the current values
    int param_count = 0;
    for (int it = m_begLeftPosIndex; it < m_endLeftPosIndex; it++) {
      for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        local_left_ls.m_positions[it * NUM_XYZ_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    for (int it = m_begLeftQuatIndex; it < m_endLeftQuatIndex; it++) {
      for (int c = 0; c < NUM_QUAT_PARAMS; c++)
        local_left_ls.m_quaternions[it * NUM_QUAT_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    // Same for the right camera
    for (int it = m_begRightPosIndex; it < m_endRightPosIndex; it++) {
      for (int c = 0; c < NUM_XYZ_PARAMS; c++)
        local_right_ls.m_positions[it * NUM_XYZ_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }
    for (int it = m_begRightQuatIndex; it < m_endRightQuatIndex; it++) {
      for (int c = 0; c < NUM_QUAT_PARAMS; c++)
        local_right_ls.m_quaternions[it * NUM_QUAT_PARAMS + c] = parameters[param_count][c];
      param_count++;
    }

    vw::TransformPtr local_left_tx = m_left_tx;
    vw::TransformPtr local_right_tx = m_right_tx;

    vw::Vector2 error;
    bool success = calcDispBasedError(&local_left_ls, &local_right_ls,
                                      local_left_tx, local_right_tx,
                                      m_disp, m_P, error);

    if (success) {
      residuals[0] = m_weight * error[0];
      residuals[1] = m_weight * error[1];
    } else {
      residuals[0] = m_weight * g_ref_terrain_pix_val;
      residuals[1] = m_weight * g_ref_terrain_pix_val;
    }

    // Accept the solution in either case, as it is too late to back out,
    // because that will mess up the bookkeeping.
    return true;

  } catch (...) {
    residuals[0] = m_weight * g_ref_terrain_pix_val;
    residuals[1] = m_weight * g_ref_terrain_pix_val;
    return true; // accept the solution anyway
  }

  return true;
}

bool addRefTerrainReprojectionErr(asp::BaBaseOptions const& opt,
                                  UsgsAstroLsSensorModel *left_ls,
                                  UsgsAstroLsSensorModel *right_ls,
                                  vw::TransformPtr left_tx,
                                  vw::TransformPtr right_tx,
                                  vw::BBox2i const& left_bbox,
                                  vw::BBox2i const& right_bbox,
                                  DispPtr disp,
                                  csm::EcefCoord const& P,
                                  double weight,
                                  ceres::Problem & problem) {

  double line_extra = opt.max_init_reproj_error + 5.0; // add some more just in case
  double desired_precision = asp::DEFAULT_CSM_DESIRED_PRECISION;

  // Find all positions and quaternions that can affect the current pixel.
  csm::ImageCoord left_csm_pix = left_ls->groundToImage(P, desired_precision);
  vw::Vector2 left_pix;
  asp::fromCsmPixel(left_pix, left_csm_pix);
  if (!left_bbox.contains(left_pix))
    return false; // the pixel is outside the left image

  int leftBegPosIndex = -1, leftEndPosIndex = -1;
  int leftBegQuatIndex = -1, leftEndQuatIndex = -1;
  calcPosQuatIndexBounds(line_extra, left_ls, left_pix,
                         // Outputs
                         leftBegPosIndex, leftEndPosIndex,
                         leftBegQuatIndex, leftEndQuatIndex);

  // Same for the right camera
  csm::ImageCoord right_csm_pix = right_ls->groundToImage(P, desired_precision);
  vw::Vector2 right_pix;
  asp::fromCsmPixel(right_pix, right_csm_pix);
  if (!right_bbox.contains(right_pix))
    return false; // the pixel is outside the right image

  int rightBegPosIndex = -1, rightEndPosIndex = -1;
  int rightBegQuatIndex = -1, rightEndQuatIndex = -1;
  calcPosQuatIndexBounds(line_extra, right_ls, right_pix,
                         // Outputs
                         rightBegPosIndex, rightEndPosIndex,
                         rightBegQuatIndex, rightEndQuatIndex);

  ceres::CostFunction* pixel_cost_function =
    RefTerrainReprojErr::Create(left_ls, right_ls, left_tx, right_tx, disp, P, weight,
                                leftBegPosIndex, leftEndPosIndex,
                                leftBegQuatIndex, leftEndQuatIndex,
                                rightBegPosIndex, rightEndPosIndex,
                                rightBegQuatIndex, rightEndQuatIndex);

  ceres::LossFunction* pixel_loss_function
    = new ceres::CauchyLoss(opt.reference_terrain_robust_threshold);

  // The variable of optimization are the relevant camera positions and quaternions.
  std::vector<double*> vars;
  for (int it = leftBegPosIndex; it < leftEndPosIndex; it++)
    vars.push_back(&left_ls->m_positions[it * NUM_XYZ_PARAMS]);
  for (int it = leftBegQuatIndex; it < leftEndQuatIndex; it++)
    vars.push_back(&left_ls->m_quaternions[it * NUM_QUAT_PARAMS]);

  // Same for the right camera
  for (int it = rightBegPosIndex; it < rightEndPosIndex; it++)
    vars.push_back(&right_ls->m_positions[it * NUM_XYZ_PARAMS]);
  for (int it = rightBegQuatIndex; it < rightEndQuatIndex; it++)
    vars.push_back(&right_ls->m_quaternions[it * NUM_QUAT_PARAMS]);

  problem.AddResidualBlock(pixel_cost_function, pixel_loss_function, vars);

  return true;
}

// Read the reference terrain. Ensure enough reference points are loaded that
// project in the camera boxes. Do two attempts.
// TODO(oalexan1): This will need adjustment if there is more than one stereo pair.
void loadReferenceTerrain(asp::BaBaseOptions            const& opt,
                          std::vector<asp::CsmModel*>   const& csm_models,
                          std::vector<int>              const& left_indices,
                          std::vector<int>              const& right_indices,
                          std::vector<vw::TransformPtr> const& left_trans,
                          std::vector<vw::TransformPtr> const& right_trans,
                          std::vector<vw::BBox2i>       const& image_boxes,
                          // Output
                          std::vector<vw::Vector3> & reference_vec) {

  vw::vw_out() << "Loading reference terrain points and then filtering them.\n";

  // Set up a GeoReference object using the datum, it may get modified later
  vw::GeoReference geo;
  geo.set_datum(opt.datum); // We checked for a datum earlier
  // Ensure the read georef lives on the same planet
  bool warn_only = false;
  vw::checkDatumConsistency(opt.datum, geo.datum(), warn_only);

  for (size_t i = 0; i < left_indices.size(); i++) {

    auto * left_cam = csm_models[left_indices[i]];
    auto * right_cam = csm_models[right_indices[i]];
    auto left_bbox = image_boxes[left_indices[i]];
    auto right_bbox = image_boxes[right_indices[i]];

    int num_try_load = 5 * opt.max_num_reference_points;
    for (int attempt = 0; attempt < 2; attempt++) {

      reference_vec.clear();
      std::vector<vw::Vector3> local_reference_vec;
      asp::load_csv_or_dem(opt.csv_format_str, opt.csv_srs, opt.reference_terrain,
                            num_try_load,
                            // Outputs
                            geo, local_reference_vec);

      // TODO(oalexan1): For non-ISIS cameras this could be done in parallel.
      for (size_t data_col = 0; data_col < local_reference_vec.size(); data_col++) {
        vw::Vector3 reference_xyz = local_reference_vec[data_col];
        vw::Vector2 left_pix, right_pix;
        try {
          left_pix = left_cam->point_to_pixel(reference_xyz);
          right_pix = right_cam->point_to_pixel(reference_xyz);
        } catch (...) {
          continue;
        }
        if (!left_bbox.contains(left_pix) || !right_bbox.contains(right_pix))
          continue;

        reference_vec.push_back(reference_xyz);
      }

      int num_loaded = reference_vec.size();
      double ratio = double(num_loaded) / opt.max_num_reference_points;
      if (ratio >= 1.0)
        break; // loaded enough points

       // Try again
       num_try_load = 1.5 * num_try_load / std::max(ratio, 1e-3); // avoid division by 0
       vw::vw_out() << "Loaded too few points after filtering. Try again.\n";

    } // end attempt loop

    // If there are too few points, print a warning
    if (reference_vec.size() < opt.max_num_reference_points)
      vw::vw_out(vw::WarningMessage) << "Loaded only " << reference_vec.size()
        << " reference terrain points after filtering by projection into camera.\n";

    // If they are too many, pick a subset
    if (reference_vec.size() > opt.max_num_reference_points) {
      // Deterministic shuffle
      unsigned seed = 0;
      std::mt19937 gen(seed);
      std::shuffle(reference_vec.begin(), reference_vec.end(), gen);
      reference_vec.resize(opt.max_num_reference_points);
    }

    vw::vw_out() << "Read " << reference_vec.size()
      << " reference terrain points (after filtering by projection into camera).\n";

  } // end loop through stereo pairs

} // end function loadReferenceTerrain()

// Option --reference-terrain
// TODO(oalexan1): Must add the uncertainty logic.
// TODO(oalexan1): Filter out points outside the box and load more if needed.
void addRefTerrainCostFun(asp::BaBaseOptions            const& opt,
                                     std::vector<asp::CsmModel*>   const& csm_models,
                                     std::vector<int>              const& left_indices,
                                     std::vector<int>              const& right_indices,
                                     std::vector<vw::TransformPtr> const& left_trans,
                                     std::vector<vw::TransformPtr> const& right_trans,
                                     std::vector<std::string>      const& disp_files,
                                     // Outputs
                                     ceres::Problem                 & problem,
                                     std::vector<DispPtr>           & disp_vec,
                                     vw::ImageView<float>           & mapproj_dem,
                                     std::vector<double>            & weight_per_residual,
                                     std::vector<vw::Vector3>       & reference_vec,
                                     std::vector<std::vector<int>>  & ref_indices) {

  // For now, only one stereo pair is supported, so size of left_indices must be 1
  if (left_indices.size() != 1)
    vw::vw_throw(vw::ArgumentErr() << "Expecting only one stereo pair.\n");

  // Read the image boxes
  std::vector<vw::BBox2i> image_boxes;
  for (size_t icam = 0; icam < opt.image_files.size(); icam++) {
    vw::DiskImageView<float> img(opt.image_files[icam]);
    vw::BBox2i bbox = vw::bounding_box(img);
    image_boxes.push_back(bbox);
  }

  // Read the reference terrain. Ensure enough reference points are loaded that
  // project in the camera boxes. 
  loadReferenceTerrain(opt, csm_models, left_indices, right_indices,
                      left_trans, right_trans, image_boxes,
                      reference_vec); // output

  // Load the disparity files. Keep a pointer to DiskImageView to avoid loading
  // them fully in memory, as they can be huge.
  disp_vec.resize(disp_files.size());
  for (size_t i = 0; i < disp_files.size(); i++) {
    vw::vw_out() << "Reading disparity file: " << disp_files[i] << "\n";
    disp_vec[i].reset(new vw::DiskImageView<vw::PixelMask<vw::Vector2f>>(disp_files[i]));
  }

  // Record the indices of the residuals of the reference points  
  ref_indices.clear();
  ref_indices.resize(reference_vec.size());

  int num_kept_ref_points = 0;
  vw::vw_out() << "Setting up the error to the reference terrain.\n";

  // For Map2CamTrans, must turn off caching to ensure thread safety, and read
  // the DEM in memory.
  for (size_t i = 0; i < left_indices.size(); i++) {

    auto & left_tx = left_trans[i];
    auto & right_tx = right_trans[i];

    vw::Map2CamTrans* left_map2cam =
      dynamic_cast<vw::Map2CamTrans*>(left_tx.get());
    vw::Map2CamTrans* right_map2cam =
      dynamic_cast<vw::Map2CamTrans*>(right_tx.get());

    // Either both must be null or both must be non-null.
    if ((left_map2cam == NULL) != (right_map2cam == NULL))
      vw::vw_throw(vw::ArgumentErr() << "Both images in a stereo pair must be "
                   << "mapprojected, or neither of them.\n");

    if (left_map2cam == NULL || right_map2cam == NULL)
      continue; // nothing to do

    // Both must be projected onto the same DEM
    if (left_map2cam->m_dem_file != right_map2cam->m_dem_file)
      vw::vw_throw(vw::ArgumentErr() << "The two images in a stereo pair must be "
                   << "projected onto the same DEM.\n");

    // Print a warning if the left_map2cam->m_dem width and height are no less than 
    // 5000 pixels, as that would be slow to load.
    int k = 5000;
    if (left_map2cam->m_dem.cols() >= k || left_map2cam->m_dem.rows() >= k)
      vw::vw_out(vw::WarningMessage)
        << "The mapprojection DEM " << left_map2cam->m_dem_file
        << " is very large, it may take a long time to load. Dimensions: "
        << left_map2cam->m_dem.cols() << " x " << left_map2cam->m_dem.rows()
        << " pixels.\n";

    // Load the DEM in memory and set it for each transform. This is a fix
    // for very slow performance. Note that the disparity is still loaded
    // on demand only, as that can be large.
    mapproj_dem = copy(left_map2cam->m_dem);

    // TODO(oalexan1): This must be a member function. It uses very detailed
    // info from the class.
    left_map2cam->set_use_cache(false);
    left_map2cam->m_masked_dem = vw::create_mask(mapproj_dem,
                                                 left_map2cam->m_nodata);
    left_map2cam->m_interp_dem = vw::interpolate(left_map2cam->m_masked_dem,
                                                  vw::BicubicInterpolation(),
                                                  vw::ZeroEdgeExtension());

    right_map2cam->set_use_cache(false);
    right_map2cam->m_masked_dem = vw::create_mask(mapproj_dem,
                                                  right_map2cam->m_nodata);
    right_map2cam->m_interp_dem = vw::interpolate(right_map2cam->m_masked_dem,
                                                  vw::BicubicInterpolation(),
                                                  vw::ZeroEdgeExtension());
  } // end loop through stereo pairs

  // Need a progress bar as this can be slow
  vw::TerminalProgressCallback tpc("asp", "Filter ref points with disparity: --> ");
  // Find the spacing for progress. Avoid int32 overflow.
  double total = double(reference_vec.size()) * double(left_indices.size());
  double hundred = 100.0;
  long long spacing = std::max(round(total / hundred), 1.0);
  long long data_count = 0;

  // Loop through the reference points
  for (size_t data_col = 0; data_col < reference_vec.size(); data_col++) {

    vw::Vector3 reference_xyz = reference_vec[data_col];

    // Iterate over left indices
    for (size_t i = 0; i < left_indices.size(); i++) {

      data_count++;
      if (data_count % spacing == 0)
        tpc.report_incremental_progress(1.0/hundred);

      auto & left_tx = left_trans[i];
      auto & right_tx = right_trans[i];
      int left_index = left_indices[i];
      int right_index = right_indices[i];
      vw::BBox2i left_bbox = image_boxes[left_index];
      vw::BBox2i right_bbox = image_boxes[right_index];

      // Get the underlying linescan models      
      csm::RasterGM * left_csm = csm_models[left_index]->m_gm_model.get();
      UsgsAstroLsSensorModel * left_ls
        = dynamic_cast<UsgsAstroLsSensorModel*>(left_csm);
      csm::RasterGM * right_csm = csm_models[right_index]->m_gm_model.get();
      UsgsAstroLsSensorModel * right_ls
        = dynamic_cast<UsgsAstroLsSensorModel*>(right_csm);
      if (left_ls == NULL || right_ls == NULL)
        vw::vw_throw(vw::ArgumentErr() << "The option --reference-terrain works only "
                      << "with linescan cameras.\n");

      // Convert to CSM points
      csm::EcefCoord P(reference_xyz[0], reference_xyz[1], reference_xyz[2]);

      // If the error cannot be measured even with nominal values of the parameters,
      // there is no point in adding the error.
      vw::Vector2 error;
      bool success = calcDispBasedError(left_ls, right_ls, left_tx, right_tx, disp_vec[i], P,
                                        error); // output

      if (!success)
        continue; // skip this point

      // Find the weight to use with the camera constraint
      double gsd = 0.0;
      try {
        vw::Vector2 pix_obs = csm_models[left_index]->point_to_pixel(reference_xyz);
        gsd = vw::estimatedGSD(csm_models[left_index], image_boxes[left_index],
                                pix_obs, reference_xyz);
      } catch (...) {
        continue;
      }
      if (gsd <= 0.0)
        continue;

      // Bigger uncertainty means a smaller weight for the residual. Also 
      // convert from meters to pixels.
      double weight = gsd / opt.reference_terrain_uncertainty;

      // Add the error cost function for this point. Skip points whose pixel
      // falls outside the camera ephemeris range.
      try {
        success = addRefTerrainReprojectionErr(opt, left_ls, right_ls,
                                               left_tx, right_tx,
                                               left_bbox, right_bbox,
                                               disp_vec[i], P, weight, problem);
      } catch (...) { success = false; }
      if (!success)
        continue; // skip this point

      // Record where the residuals for this point are stored
      ref_indices[data_col].push_back(weight_per_residual.size());

      // Two residuals were added. Save the corresponding weights. The saved residuals
      // will be divided back by the weight.
      for (int c = 0; c < asp::PIXEL_SIZE; c++)
        weight_per_residual.push_back(weight);

    } // end loop through stereo pairs

    // See if this point is kept
    if (ref_indices[data_col].size() > 0)
      num_kept_ref_points++;
  }

  tpc.report_finished();

  vw::vw_out() << "Read " << num_kept_ref_points
    << " reference terrain points (after filtering by disparity).\n";

}

} // end namespace asp

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

// Orbit constraint for jitter_solve. See JitterSolveOrbit.h.

#include <asp/Camera/JitterSolveOrbit.h>
#include <asp/Camera/BaseCostFuns.h>
#include <asp/Camera/CsmModel.h>
#include <asp/Core/FileUtils.h>

#include <vw/Core/Exception.h>
#include <vw/Core/Log.h>
#include <vw/Math/Statistics.h>

#include <usgscsm/UsgsAstroLsSensorModel.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace asp {

const int NUM_XYZ = 3;

// The linescan model of a camera, or throw
UsgsAstroLsSensorModel * orbitLsModel(asp::CsmModel * csm_model) {
  UsgsAstroLsSensorModel * ls_model
    = dynamic_cast<UsgsAstroLsSensorModel*>((csm_model->m_gm_model).get());
  if (ls_model == NULL)
    vw::vw_throw(vw::ArgumentErr() << "The orbit constraint (--orbital-group-list) "
             << "applies only to CSM linescan cameras.\n");
  return ls_model;
}

vw::Vector3 orbitFrameCoords(OrbitGroup const& group, vw::Vector3 const& ecef) {
  vw::Vector3 p = ecef - group.origin;
  return vw::Vector3(vw::dot_prod(p, group.along),
                     vw::dot_prod(p, group.across),
                     vw::dot_prod(p, group.up));
}

// Evaluate a polynomial with the given coefficients at t
double orbitPoly(double const* c, int degree, double t) {
  double val = 0.0, tk = 1.0;
  for (int k = 0; k <= degree; k++) {
    val += c[k] * tk;
    tk *= t;
  }
  return val;
}

// Derivative of the polynomial with respect to t
double orbitPolyDeriv(double const* c, int degree, double t) {
  double val = 0.0, tk = 1.0;
  for (int k = 1; k <= degree; k++) {
    val += k * c[k] * tk;
    tk *= t;
  }
  return val;
}

vw::Vector2 orbitCurve(OrbitGroup const& group, double along) {
  double t = along / group.half_len;
  int n = group.degree + 1;
  return vw::Vector2(orbitPoly(&group.coeffs[0], group.degree, t),
                     orbitPoly(&group.coeffs[n], group.degree, t));
}

// Least squares fit of a polynomial of given degree to (t, y) values
void fitOrbitPoly(std::vector<double> const& t, std::vector<double> const& y,
                  int degree, double * coeffs) {
  int n = degree + 1;
  Eigen::MatrixXd A(t.size(), n);
  Eigen::VectorXd b(t.size());
  for (size_t i = 0; i < t.size(); i++) {
    double tk = 1.0;
    for (int k = 0; k < n; k++) {
      A(i, k) = tk;
      tk *= t[i];
    }
    b(i) = y[i];
  }
  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
  for (int k = 0; k < n; k++)
    coeffs[k] = x(k);
}

void buildOrbitGroups(std::string const& orbital_group_list,
                      int degree,
                      std::vector<std::string> const& image_files,
                      std::vector<asp::CsmModel*> const& csm_models,
                      // Outputs
                      std::vector<OrbitGroup> & groups) {

  groups.clear();
  if (orbital_group_list.empty())
    return;

  if (degree < 1 || degree > 3)
    vw::vw_throw(vw::ArgumentErr() << "The orbit polynomial degree must be "
                 << "1, 2, or 3.\n");

  std::map<std::string, int> image2cam;
  for (size_t i = 0; i < image_files.size(); i++)
    image2cam[image_files[i]] = i;

  std::vector<std::string> group_files;
  boost::split(group_files, orbital_group_list, boost::is_any_of(","));
  std::vector<int> cam2group(image_files.size(), -1);

  for (size_t g = 0; g < group_files.size(); g++) {
    std::string group_file = group_files[g];
    boost::trim(group_file);
    if (group_file.empty())
      continue;

    std::vector<std::string> images;
    asp::read_list(group_file, images);
    OrbitGroup group;
    group.degree = degree;
    for (size_t j = 0; j < images.size(); j++) {
      auto it = image2cam.find(images[j]);
      if (it == image2cam.end())
        vw::vw_throw(vw::ArgumentErr() << "Orbital group file '" << group_file
                     << "' has image '" << images[j]
                     << "' that is not among the input images.\n");
      int icam = it->second;
      if (cam2group[icam] >= 0)
        vw::vw_throw(vw::ArgumentErr() << "Image '" << images[j]
                     << "' is listed in more than one orbital group.\n");
      cam2group[icam] = groups.size();
      orbitLsModel(csm_models[icam]); // check it is linescan
      group.cams.push_back(icam);
    }
    if (group.cams.empty())
      vw::vw_throw(vw::ArgumentErr() << "Orbital group file '" << group_file
                   << "' has no valid images.\n");
    groups.push_back(group);
  }

  for (size_t g = 0; g < groups.size(); g++) {
    OrbitGroup & group = groups[g];

    // All position samples of the group
    std::vector<vw::Vector3> samples;
    for (int icam: group.cams) {
      UsgsAstroLsSensorModel * ls = orbitLsModel(csm_models[icam]);
      int num = ls->m_positions.size() / NUM_XYZ;
      for (int s = 0; s < num; s++)
        samples.push_back(vw::Vector3(ls->m_positions[NUM_XYZ * s + 0],
                                      ls->m_positions[NUM_XYZ * s + 1],
                                      ls->m_positions[NUM_XYZ * s + 2]));
    }
    if (samples.size() < 2)
      vw::vw_throw(vw::ArgumentErr() << "An orbital group needs at least two "
                   << "position samples.\n");

    // The frame origin is the mean of the samples. The along-track direction is
    // the main direction of the samples, up is the radial direction at the origin
    // made perpendicular to it, and across-track is perpendicular to both.
    group.origin = vw::Vector3();
    for (auto const& s: samples)
      group.origin += s;
    group.origin /= double(samples.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto const& s: samples) {
      Eigen::Vector3d d(s[0] - group.origin[0], s[1] - group.origin[1],
                        s[2] - group.origin[2]);
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d main_dir = es.eigenvectors().col(2); // largest eigenvalue
    group.along = vw::Vector3(main_dir(0), main_dir(1), main_dir(2));
    vw::Vector3 radial = vw::normalize(group.origin);
    group.up = vw::normalize(radial
                                   - vw::dot_prod(radial, group.along) * group.along);
    group.across = vw::cross_prod(group.up, group.along);

    // Initial curve, fit to the samples
    std::vector<double> t(samples.size()), a(samples.size()), h(samples.size());
    group.half_len = 0.0;
    std::vector<vw::Vector3> frame(samples.size());
    for (size_t i = 0; i < samples.size(); i++) {
      frame[i] = orbitFrameCoords(group, samples[i]);
      group.half_len = std::max(group.half_len, std::abs(frame[i][0]));
    }
    if (group.half_len <= 0.0)
      vw::vw_throw(vw::ArgumentErr() << "The position samples of an orbital group "
                   << "do not extend along track.\n");
    for (size_t i = 0; i < samples.size(); i++) {
      t[i] = frame[i][0] / group.half_len;
      a[i] = frame[i][1];
      h[i] = frame[i][2];
    }
    int n = degree + 1;
    group.coeffs.assign(2 * n, 0.0);
    fitOrbitPoly(t, a, degree, &group.coeffs[0]);
    fitOrbitPoly(t, h, degree, &group.coeffs[n]);

    vw::vw_out() << "Orbital group " << g << ": " << group.cams.size()
                 << " cameras, " << samples.size() << " position samples, "
                 << "along-track extent " << 2.0 * group.half_len / 1000.0
                 << " km, polynomial degree " << degree << ".\n";
  }
}

// Pull a position sample to the orbit curve of its group. Two residuals, the
// across-track and vertical deviations normalized by their uncertainties. The map
// from ECEF to the orbit frame is linear. The Jacobian is analytic, as numerical
// differences on large ECEF values are inaccurate.
class OrbitDevError: public ceres::CostFunction {
public:
  OrbitDevError(OrbitGroup const& group, vw::Vector3 const& init_ecef,
                vw::Vector2 const& uncertainty, double power):
    m_degree(group.degree), m_half_len(group.half_len),
    m_init_ecef(init_ecef), m_uncertainty(uncertainty), m_power(power) {

    set_num_residuals(2);
    mutable_parameter_block_sizes()->push_back(NUM_XYZ);
    mutable_parameter_block_sizes()->push_back(2 * (m_degree + 1));

    // The frame coordinates at the sample, and the rows of the linear map
    m_init_frame = orbitFrameCoords(group, init_ecef);
    for (int col = 0; col < NUM_XYZ; col++) {
      m_jac(0, col) = group.along[col];
      m_jac(1, col) = group.across[col];
      m_jac(2, col) = group.up[col];
    }
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {

    double const* pos = parameters[0];
    double const* coeffs = parameters[1];
    int n = m_degree + 1;

    // Frame coordinates
    vw::Vector3 dx(pos[0] - m_init_ecef[0], pos[1] - m_init_ecef[1],
                   pos[2] - m_init_ecef[2]);
    vw::Vector3 q = m_init_frame + m_jac * dx;
    double t = q[0] / m_half_len;

    // Normalized deviations from the curve
    double xa = (q[1] - orbitPoly(&coeffs[0], m_degree, t)) / m_uncertainty[0];
    double xh = (q[2] - orbitPoly(&coeffs[n], m_degree, t)) / m_uncertainty[1];
    residuals[0] = posUncResidual(xa, m_power);
    residuals[1] = posUncResidual(xh, m_power);

    if (jacobians == NULL)
      return true;

    double fa = posUncDerivative(xa, m_power) / m_uncertainty[0];
    double fh = posUncDerivative(xh, m_power) / m_uncertainty[1];
    double da = orbitPolyDeriv(&coeffs[0], m_degree, t) / m_half_len;
    double dh = orbitPolyDeriv(&coeffs[n], m_degree, t) / m_half_len;

    if (jacobians[0] != NULL) {
      for (int col = 0; col < NUM_XYZ; col++) {
        jacobians[0][0 * NUM_XYZ + col] = fa * (m_jac(1, col) - da * m_jac(0, col));
        jacobians[0][1 * NUM_XYZ + col] = fh * (m_jac(2, col) - dh * m_jac(0, col));
      }
    }

    if (jacobians[1] != NULL) {
      int len = 2 * n;
      for (int k = 0; k < len; k++) {
        jacobians[1][0 * len + k] = 0.0;
        jacobians[1][1 * len + k] = 0.0;
      }
      double tk = 1.0;
      for (int k = 0; k < n; k++) {
        jacobians[1][0 * len + k]     = -fa * tk;
        jacobians[1][1 * len + n + k] = -fh * tk;
        tk *= t;
      }
    }

    return true;
  }

private:
  int m_degree;
  double m_half_len;
  vw::Vector3 m_init_ecef, m_init_frame;
  vw::Matrix3x3 m_jac;
  vw::Vector2 m_uncertainty;
  double m_power;
};

ceres::CostFunction * createOrbitDevError(OrbitGroup const& group,
                                          vw::Vector3 const& init_ecef,
                                          vw::Vector2 const& uncertainty,
                                          double power) {
  return new OrbitDevError(group, init_ecef, uncertainty, power);
}

void addOrbitConstraint(std::vector<asp::CsmModel*> const& csm_models,
                        vw::Vector2 const& uncertainty, double power,
                        // Outputs
                        std::vector<OrbitGroup> & groups,
                        std::vector<double> & weight_per_residual,
                        ceres::Problem & problem) {

  if (uncertainty[0] <= 0 || uncertainty[1] <= 0)
    vw::vw_throw(vw::ArgumentErr() << "The orbit uncertainty must be positive.\n");

  for (size_t g = 0; g < groups.size(); g++) {
    OrbitGroup & group = groups[g];
    for (int icam: group.cams) {
      UsgsAstroLsSensorModel * ls = orbitLsModel(csm_models[icam]);
      int num = ls->m_positions.size() / NUM_XYZ;
      for (int s = 0; s < num; s++) {
        double * pos = &ls->m_positions[NUM_XYZ * s];
        vw::Vector3 init_ecef(pos[0], pos[1], pos[2]);
        ceres::CostFunction * cost
          = createOrbitDevError(group, init_ecef, uncertainty, power);
        ceres::LossFunction * loss = new ceres::TrivialLoss();
        problem.AddResidualBlock(cost, loss, pos, &group.coeffs[0]);
        for (int c = 0; c < 2; c++)
          weight_per_residual.push_back(1.0);
      }
    }
  }
}

void orbitDeviations(std::vector<OrbitGroup> const& groups,
                     std::vector<asp::CsmModel*> const& csm_models,
                     // Outputs
                     std::vector<std::vector<OrbitSampleDev>> & devs) {

  devs.clear();
  devs.resize(groups.size());
  for (size_t g = 0; g < groups.size(); g++) {
    OrbitGroup const& group = groups[g];
    for (int icam: group.cams) {
      UsgsAstroLsSensorModel * ls = orbitLsModel(csm_models[icam]);
      int num = ls->m_positions.size() / NUM_XYZ;
      for (int s = 0; s < num; s++) {
        double const* pos = &ls->m_positions[NUM_XYZ * s];
        vw::Vector3 frame = orbitFrameCoords(group, vw::Vector3(pos[0], pos[1], pos[2]));
        vw::Vector2 curve = orbitCurve(group, frame[0]);
        OrbitSampleDev dev;
        dev.cam = icam;
        dev.sample = s;
        dev.along = frame[0];
        dev.across_dev = frame[1] - curve[0];
        dev.vert_dev = frame[2] - curve[1];
        devs[g].push_back(dev);
      }
    }
  }
}

// Root mean square and max absolute value
void orbitRmsMax(std::vector<double> const& vals, double & rms, double & max_abs) {
  rms = 0.0;
  max_abs = 0.0;
  for (double v: vals) {
    rms += v * v;
    max_abs = std::max(max_abs, std::abs(v));
  }
  if (!vals.empty())
    rms = sqrt(rms / vals.size());
}

void printOrbitDeviations(std::string const& tag,
                          std::vector<std::vector<OrbitSampleDev>> const& devs) {
  for (size_t g = 0; g < devs.size(); g++) {
    std::vector<double> a, h;
    for (auto const& d: devs[g]) {
      a.push_back(d.across_dev);
      h.push_back(d.vert_dev);
    }
    double a_rms = 0.0, a_max = 0.0, h_rms = 0.0, h_max = 0.0;
    orbitRmsMax(a, a_rms, a_max);
    orbitRmsMax(h, h_rms, h_max);
    std::ostringstream os;
    os << std::fixed << std::setprecision(2)
       << tag << " deviation from orbit, group " << g
       << " (meters): across-track rms " << a_rms << ", max " << a_max
       << ", vertical rms " << h_rms << ", max " << h_max << ".\n";
    vw::vw_out() << os.str();
  }
}

void saveOrbitDeviations(std::string const& out_prefix,
                         std::vector<std::string> const& image_files,
                         std::vector<OrbitGroup> const& groups,
                         std::vector<std::vector<OrbitSampleDev>> const& init_devs,
                         std::vector<std::vector<OrbitSampleDev>> const& final_devs) {

  std::string sample_file = out_prefix + "-orbit_deviations.csv";
  std::string stats_file = out_prefix + "-orbit_deviation_stats.txt";
  vw::vw_out() << "Writing: " << sample_file << "\n";
  vw::vw_out() << "Writing: " << stats_file << "\n";

  std::ofstream ofs(sample_file.c_str());
  ofs << "# Deviation of each position sample from the orbit curve of its group, in "
      << "meters, before and after the solve. The along-track coordinate is after "
      << "the solve.\n";
  ofs << "# group, image, sample, along_track, init_across_track, init_vertical, "
      << "final_across_track, final_vertical\n";
  ofs << std::fixed << std::setprecision(3);

  std::ofstream sfs(stats_file.c_str());
  sfs << "# Per camera deviation of the position samples from the orbit curve, in "
      << "meters. rms and max absolute value, before and after the solve.\n";
  sfs << "# group, image, num_samples, init_across_rms, init_vert_rms, "
      << "final_across_rms, final_across_max, final_vert_rms, final_vert_max\n";
  sfs << std::fixed << std::setprecision(3);

  for (size_t g = 0; g < groups.size(); g++) {
    if (init_devs[g].size() != final_devs[g].size())
      vw::vw_throw(vw::ArgumentErr() << "Book-keeping failure in orbit deviations.\n");

    // Per camera stats
    std::map<int, std::vector<int>> cam2rows;
    for (size_t i = 0; i < final_devs[g].size(); i++) {
      OrbitSampleDev const& fd = final_devs[g][i];
      OrbitSampleDev const& id = init_devs[g][i];
      ofs << g << ", " << image_files[fd.cam] << ", " << fd.sample << ", "
          << fd.along << ", " << id.across_dev << ", " << id.vert_dev << ", "
          << fd.across_dev << ", " << fd.vert_dev << "\n";
      cam2rows[fd.cam].push_back(i);
    }

    for (int icam: groups[g].cams) {
      std::vector<double> ia, ih, fa, fh;
      for (int i: cam2rows[icam]) {
        ia.push_back(init_devs[g][i].across_dev);
        ih.push_back(init_devs[g][i].vert_dev);
        fa.push_back(final_devs[g][i].across_dev);
        fh.push_back(final_devs[g][i].vert_dev);
      }
      double ia_rms = 0, ia_max = 0, ih_rms = 0, ih_max = 0;
      double fa_rms = 0, fa_max = 0, fh_rms = 0, fh_max = 0;
      orbitRmsMax(ia, ia_rms, ia_max);
      orbitRmsMax(ih, ih_rms, ih_max);
      orbitRmsMax(fa, fa_rms, fa_max);
      orbitRmsMax(fh, fh_rms, fh_max);
      sfs << g << ", " << image_files[icam] << ", " << fa.size() << ", "
          << ia_rms << ", " << ih_rms << ", " << fa_rms << ", " << fa_max << ", "
          << fh_rms << ", " << fh_max << "\n";
    }
  }
}

} // end namespace asp

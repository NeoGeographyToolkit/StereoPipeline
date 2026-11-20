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

// Check the orbit constraint of jitter_solve: the frame coordinates, and the
// analytic Jacobian of the cost function against numerical differences.

#include <asp/Camera/JitterSolveOrbit.h>

#include <vw/Cartography/Datum.h>
#include <vw/Math/Matrix.h>
#include <test/Helpers.h>

#include <ceres/ceres.h>

#include <cmath>
#include <vector>

// An orbit group at 170 km height over Arizona, heading north-east, with a curve
// that is slightly off the samples
asp::OrbitGroup makeGroup(int degree) {
  asp::OrbitGroup group;
  vw::Datum datum("WGS84");
  vw::Vector3 llh(-111.7, 32.9, 170000.0);
  group.origin = datum.geodetic_to_cartesian(llh);
  vw::Matrix3x3 ned2ecef = datum.lonlat_to_ned_matrix(llh);
  double theta = M_PI / 5.0;
  vw::Vector3 north = vw::select_col(ned2ecef, 0);
  vw::Vector3 east  = vw::select_col(ned2ecef, 1);
  group.along  = cos(theta) * north + sin(theta) * east;
  group.up     = -vw::select_col(ned2ecef, 2);
  group.across = vw::cross_prod(group.up, group.along);
  group.half_len = 120000.0;
  group.degree = degree;
  int n = degree + 1;
  group.coeffs.assign(2 * n, 0.0);
  group.coeffs[0] = 3.0;
  group.coeffs[1] = -12.0;
  group.coeffs[n] = -5.0;
  group.coeffs[n + 1] = 20.0;
  if (degree == 2) {
    group.coeffs[2] = 7.0;
    group.coeffs[n + 2] = -1100.0; // the orbit drops below the chord
  }
  return group;
}

// The ECEF point at given frame coordinates (along, across, up)
vw::Vector3 frameToEcef(asp::OrbitGroup const& group, vw::Vector3 const& f) {
  return group.origin + f[0] * group.along + f[1] * group.across + f[2] * group.up;
}

TEST(JitterSolveOrbit, FrameCoords) {
  asp::OrbitGroup group = makeGroup(1);
  vw::Vector3 f(50000.0, -80.0, 35.0);
  vw::Vector3 g = asp::orbitFrameCoords(group, frameToEcef(group, f));
  for (int i = 0; i < 3; i++)
    EXPECT_NEAR(f[i], g[i], 1e-3);
}

TEST(JitterSolveOrbit, Jacobian) {
  for (int degree = 1; degree <= 2; degree++) {
    for (double power: {2.0, 4.0}) {
      asp::OrbitGroup group = makeGroup(degree);
      vw::Vector3 init = frameToEcef(group, vw::Vector3(-70000.0, 60.0, -40.0));
      vw::Vector2 unc(10.0, 5.0);
      ceres::CostFunction * cost = asp::createOrbitDevError(group, init, unc, power);

      // Evaluate away from the initial position, as the solver would
      std::vector<double> pos = {init[0] + 25.0, init[1] - 40.0, init[2] + 15.0};
      std::vector<double> coeffs = group.coeffs;
      coeffs[0] += 2.0;
      std::vector<double const*> params = {&pos[0], &coeffs[0]};
      int nc = coeffs.size();

      double res[2];
      std::vector<double> jpos(2 * 3), jcoef(2 * nc);
      double * jac[2] = {&jpos[0], &jcoef[0]};
      ASSERT_TRUE(cost->Evaluate(&params[0], res, jac));

      // Central differences, for each parameter of each block
      std::vector<double*> blocks = {&pos[0], &coeffs[0]};
      std::vector<int> sizes = {3, nc};
      std::vector<double> steps = {1e-2, 1e-4};
      for (int b = 0; b < 2; b++) {
        for (int k = 0; k < sizes[b]; k++) {
          double orig = blocks[b][k], rp[2], rm[2];
          blocks[b][k] = orig + steps[b];
          cost->Evaluate(&params[0], rp, NULL);
          blocks[b][k] = orig - steps[b];
          cost->Evaluate(&params[0], rm, NULL);
          blocks[b][k] = orig;
          for (int r = 0; r < 2; r++) {
            double num = (rp[r] - rm[r]) / (2.0 * steps[b]);
            double ana = jac[b][r * sizes[b] + k];
            EXPECT_NEAR(num, ana, 1e-6 + 1e-5 * std::abs(num));
          }
        }
      }
      delete cost;
    }
  }
}

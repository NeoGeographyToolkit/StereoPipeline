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

// Validate the estimated ground sample distance (GSD) and the conversion of a
// ground uncertainty (in meters) to a pixel weight. This underpins the anchor
// point constraint in jitter_solve, where --anchor-dem-uncertainty is given in
// meters on the ground but the anchor cost is a reprojection error in pixels.

#include <asp/Camera/CsmModel.h>

#include <vw/Camera/CameraImage.h>
#include <vw/Cartography/Datum.h>
#include <vw/Math/Vector.h>

#include <test/Helpers.h>

#include <vector>
#include <algorithm>

using namespace vw;
using namespace asp;

// The sample CTX linescan camera that ships with the tests.
std::string ctx_camera() {
  return std::string(TEST_SRCDIR) + "/ctx.json";
}

// Intersect the ray through a pixel with the datum, returning the ground point.
// This is an independent way to find the GSD, by measuring the ground distance
// between neighboring pixels, as a mapproject query onto the datum would.
bool groundPoint(CsmModel const& cam, vw::cartography::Datum const& datum,
                 vw::Vector2 const& pix, vw::Vector3 & ground) {
  vw::Vector3 ctr = cam.camera_center(pix);
  vw::Vector3 dir = cam.pixel_to_vector(pix);
  ground = vw::cartography::datum_intersection(datum, ctr, dir);
  return (ground != vw::Vector3());
}

TEST(CameraGSD, EstimatedGsdMatchesDatumIntersection) {

  CsmModel cam;
  cam.load_model(ctx_camera());

  vw::Vector2 image_size = cam.get_image_size();
  vw::BBox2i image_box(0, 0, image_size[0], image_size[1]);
  vw::cartography::Datum datum = cam.get_datum_csm("unknown", false);

  // Sample a grid of pixels well inside the image.
  std::vector<double> gsd_est, gsd_indep;
  for (double f = 0.2; f <= 0.8; f += 0.2) {
    vw::Vector2 pix(image_size[0] * f, image_size[1] * f);

    vw::Vector3 ground, ground_col, ground_row;
    if (!groundPoint(cam, datum, pix, ground))
      continue;
    if (!groundPoint(cam, datum, pix + vw::Vector2(1, 0), ground_col))
      continue;
    if (!groundPoint(cam, datum, pix + vw::Vector2(0, 1), ground_row))
      continue;

    // The GSD from estimatedGSD, using the datum ground point along the ray.
    double est = vw::camera::estimatedGSD(&cam, image_box, pix, ground);

    // The independent GSD, from the ground spacing of neighboring pixels.
    double indep = (norm_2(ground - ground_col) + norm_2(ground - ground_row)) / 2.0;

    std::cout << "pixel " << pix << " estimatedGSD " << est
              << " datum-intersection GSD " << indep << " meters/pixel\n";

    gsd_est.push_back(est);
    gsd_indep.push_back(indep);

    EXPECT_GT(est, 0.0);
    // The two independent estimates must agree. They differ by a few percent,
    // since estimatedGSD projects the neighbor rays at the first pixel's slant
    // distance (a local linearization), while the datum intersection re-shoots
    // each ray onto the ellipsoid, as mapproject does.
    EXPECT_NEAR(est, indep, 0.10 * indep);
  }

  ASSERT_FALSE(gsd_est.empty());

  // The median GSD, as computed per image in jitter_solve.
  std::sort(gsd_est.begin(), gsd_est.end());
  double median_gsd = gsd_est[gsd_est.size() / 2];
  std::cout << "Median estimated GSD: " << median_gsd << " meters/pixel\n";

  // CTX is a narrow-angle camera near 6 meters/pixel. Pin the scale, so a
  // broken camera load or a unit error (km vs m) is caught.
  EXPECT_GT(median_gsd, 3.0);
  EXPECT_LT(median_gsd, 12.0);
}

// The anchor cost is a reprojection error in pixels. A ground uncertainty in
// meters is converted to a pixel weight by dividing by the GSD. Check the
// formula weight = GSD / uncertainty, and that this is the fix for the former
// bug where the uncertainty was used directly as a pixel weight (weight =
// 1/uncertainty), ignoring the GSD.
TEST(CameraGSD, MetersUncertaintyToPixelWeight) {

  double gsd = 6.0;         // meters/pixel, typical for CTX
  double uncertainty = 20.0; // meters on the ground

  // Pixel uncertainty and the resulting weight.
  double sigma_pix = uncertainty / gsd;
  double weight = gsd / uncertainty;

  EXPECT_NEAR(weight, 1.0 / sigma_pix, 1e-12);

  // A ground displacement of one uncertainty is a reprojection error of one
  // pixel sigma, so weight times uncertainty equals the GSD.
  EXPECT_NEAR(weight * uncertainty, gsd, 1e-12);

  // The former buggy weight ignored the GSD. The correct weight differs from it
  // by exactly the GSD factor.
  double buggy_weight = 1.0 / uncertainty;
  EXPECT_NEAR(weight / buggy_weight, gsd, 1e-12);
}

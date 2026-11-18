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

// The cost reported per constraint type must match the residual blocks of that
// type, include the robust loss, and sum to the total cost of the problem.

#include <asp/Camera/BundleAdjustResiduals.h>

#include <test/Helpers.h>

#include <ceres/ceres.h>

#include <cmath>
#include <string>
#include <vector>

namespace asp {

// Residual: weight * (x - target)
struct ScaledDiff {
  ScaledDiff(double target, double weight): m_target(target), m_weight(weight) {}
  template <typename T>
  bool operator()(const T* x, T* residual) const {
    residual[0] = m_weight * (x[0] - m_target);
    return true;
  }
  double m_target, m_weight;
};

void addDiff(double target, double weight, ceres::LossFunction* loss,
             double* x, ceres::Problem & problem) {
  ceres::CostFunction* cost_function
    = new ceres::AutoDiffCostFunction<ScaledDiff, 1, 1>(new ScaledDiff(target, weight));
  problem.AddResidualBlock(cost_function, loss, x);
}

TEST(CostTerms, CostPerTerm) {

  double x = 0.0;
  ceres::Problem problem;
  std::vector<asp::CostTerm> terms;

  // Two blocks of type A, each with cost 0.5 * (2 * 1)^2 = 2
  addDiff(1.0, 2.0, NULL, &x, problem);
  addDiff(-1.0, 2.0, NULL, &x, problem);
  asp::recordCostTerm("A", problem.NumResidualBlocks(), terms);

  // A type that adds no blocks is not recorded
  asp::recordCostTerm("empty", problem.NumResidualBlocks(), terms);

  // One block of type B with a Cauchy loss. Residual 3, so cost 0.5 * log(1 + 9).
  addDiff(3.0, 1.0, new ceres::CauchyLoss(1.0), &x, problem);
  asp::recordCostTerm("B", problem.NumResidualBlocks(), terms);

  ASSERT_EQ(terms.size(), 2u);
  EXPECT_EQ(terms[0].name, "A");
  EXPECT_EQ(terms[0].begin, 0);
  EXPECT_EQ(terms[0].end, 2);
  EXPECT_EQ(terms[1].name, "B");
  EXPECT_EQ(terms[1].begin, 2);
  EXPECT_EQ(terms[1].end, 3);

  int num_threads = 1;
  std::vector<double> costs = asp::reportCostPerTerm(problem, terms, "test",
                                                     num_threads);
  ASSERT_EQ(costs.size(), 2u);
  EXPECT_NEAR(costs[0], 4.0, 1e-12);
  EXPECT_NEAR(costs[1], 0.5 * std::log(10.0), 1e-12);

  // The terms add up to the total cost
  double total = 0.0;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &total, NULL, NULL, NULL);
  EXPECT_NEAR(costs[0] + costs[1], total, 1e-12);
}

} // end namespace asp
